//! [`Ewkt<G>`] and the behaviours of reading and writing it.
//!
//! Every read goes through one private step: scan the `SRID=` prefix,
//! normalise the glued dimension suffixes of the body, hand the body to
//! the WKT crate, and — on failure — rebase the reported byte offset by
//! where the body started before wrapping it. That is the crate's only
//! wrapping site, which is why there is no `From<WktError>` impl: a `?`
//! would let a caller skip the rebasing.
//!
//! Every write is the prefix `SRID=<n>;` (omitted for `None`) followed
//! immediately by the WKT crate's canonical output, with no whitespace
//! between them.
//!
//! Reference: the `PostGIS` manual §4.2.1 ("`PostGIS` EWKB and EWKT") for
//! the format `ST_AsEWKT` emits and `ST_GeomFromEWKT` accepts.

use alloc::string::String;
use core::fmt::Write as _;

use geometry_cs::Cartesian;
use geometry_io_wkt::{WktError, WriteWkt};
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon,
};
use geometry_trait::{Geometry, Point as PointTrait, Polygon as PolygonTrait};

use crate::dimension_suffix;
use crate::ewkt_error::EwktError;
use crate::srid::Srid;
use crate::srid_prefix::{self, Scanned};

/// A concrete 2D Cartesian point — the coordinate type every parsed
/// geometry is built from, as in the WKT crate's parser.
type Pt = Point2D<f64, Cartesian>;

/// A geometry together with the spatial-reference id that prefixed it.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{Ewkt, Srid, from_ewkt};
///
/// let e = from_ewkt("SRID=4326;POINT(1 2)").unwrap();
/// assert_eq!(e.srid, Some(Srid::new(4326)));
/// assert_eq!(e.to_string(), "SRID=4326;POINT(1 2)");
///
/// let bare: Ewkt<_> = from_ewkt("POINT(1 2)").unwrap();
/// assert_eq!(bare.srid, None);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Ewkt<G> {
    /// `None` when the input carried no `SRID=` prefix.
    pub srid: Option<Srid>,
    /// The geometry the WKT body parsed to, or the one to write.
    pub geometry: G,
}

impl<G: WriteWkt> core::fmt::Display for Ewkt<G> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write_ewkt(&self.geometry, self.srid, f)
    }
}

/// Shift the one WKT error variant that carries a byte offset so it
/// indexes the caller's original string rather than the body slice.
///
/// The normaliser is a same-length rewrite, so the shifted offset is
/// exact whether or not it ran.
fn rebase(e: WktError, body_start: usize) -> WktError {
    match e {
        WktError::UnexpectedChar { pos, ch } => WktError::UnexpectedChar {
            pos: pos + body_start,
            ch,
        },
        other => other,
    }
}

/// Scan the prefix, normalise the body, delegate to `parse_body`, and
/// rebase any offset before wrapping the failure.
///
/// The crate's single `EwktError::Wkt` construction site.
fn read<T>(
    input: &str,
    parse_body: impl FnOnce(&str) -> Result<T, WktError>,
) -> Result<Ewkt<T>, EwktError> {
    let Scanned { srid, body_start } = srid_prefix::scan(input)?;
    let body = dimension_suffix::normalise(&input[body_start..]);
    match parse_body(&body) {
        Ok(geometry) => Ok(Ewkt { srid, geometry }),
        Err(e) => Err(EwktError::Wkt(rebase(e, body_start))),
    }
}

/// Read EWKT of any kind into a [`DynGeometry`] and its SRID.
///
/// # Errors
///
/// [`EwktError::InvalidSrid`] when the input is claimed as prefixed but
/// the prefix is malformed, and [`EwktError::Wkt`] for anything the WKT
/// crate rejects in the body, with byte offsets rebased onto the input.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{Srid, from_ewkt};
/// use geometry_model::DynKind;
///
/// let e = from_ewkt("SRID=4326;POINTM(1 2 3)").unwrap();
/// assert_eq!(e.srid, Some(Srid::new(4326)));
/// assert_eq!(e.geometry.kind(), DynKind::Point);
/// ```
pub fn from_ewkt<S: AsRef<str>>(s: S) -> Result<Ewkt<DynGeometry<f64, Cartesian>>, EwktError> {
    // A closure, not the bare fn item: `from_wkt` is generic over
    // `S: AsRef<str>` and so does not coerce to `impl FnOnce(&str) -> …`.
    read(s.as_ref(), |body| geometry_io_wkt::from_wkt(body))
}

/// Read EWKT known to be a `POINT`.
///
/// # Errors
///
/// [`EwktError::InvalidSrid`] for a malformed prefix, and
/// [`EwktError::Wkt`] for a body the WKT crate rejects — including
/// `WktError::TypeMismatch` when the body is a different kind.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{Srid, parse_point, to_ewkt};
///
/// let e = parse_point("SRID=4326;POINTM(1 2 3)").unwrap();
/// assert_eq!(to_ewkt(&e.geometry, e.srid), "SRID=4326;POINT(1 2)");
/// ```
pub fn parse_point(s: &str) -> Result<Ewkt<Pt>, EwktError> {
    read(s, geometry_io_wkt::parse_point)
}

/// Read EWKT known to be a `LINESTRING`.
///
/// # Errors
///
/// As [`parse_point`], including `WktError::TypeMismatch` for a body of a
/// different kind.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{parse_linestring, to_ewkt};
///
/// let e = parse_linestring("SRID=4326;LINESTRINGM(1 2 3,4 5 6)").unwrap();
/// assert_eq!(to_ewkt(&e.geometry, e.srid), "SRID=4326;LINESTRING(1 2,4 5)");
/// ```
pub fn parse_linestring(s: &str) -> Result<Ewkt<Linestring<Pt>>, EwktError> {
    read(s, geometry_io_wkt::parse_linestring)
}

/// Read EWKT known to be a `POLYGON`.
///
/// # Errors
///
/// As [`parse_point`], including `WktError::TypeMismatch` for a body of a
/// different kind.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{parse_polygon, to_ewkt};
///
/// let e = parse_polygon("SRID=4326;POLYGONM((0 0 1,1 0 1,0 1 1,0 0 1))").unwrap();
/// assert_eq!(
///     to_ewkt(&e.geometry, e.srid),
///     "SRID=4326;POLYGON((0 0,1 0,0 1,0 0))"
/// );
/// ```
pub fn parse_polygon(s: &str) -> Result<Ewkt<Polygon<Pt>>, EwktError> {
    read(s, geometry_io_wkt::parse_polygon)
}

/// Read EWKT known to be a `MULTIPOINT`.
///
/// # Errors
///
/// As [`parse_point`], including `WktError::TypeMismatch` for a body of a
/// different kind.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{parse_multi_point, to_ewkt};
///
/// let e = parse_multi_point("SRID=4326;MULTIPOINTM(1 2 3,4 5 6)").unwrap();
/// assert_eq!(to_ewkt(&e.geometry, e.srid), "SRID=4326;MULTIPOINT((1 2),(4 5))");
/// ```
pub fn parse_multi_point(s: &str) -> Result<Ewkt<MultiPoint<Pt>>, EwktError> {
    read(s, geometry_io_wkt::parse_multi_point)
}

/// Read EWKT known to be a `MULTILINESTRING`.
///
/// # Errors
///
/// As [`parse_point`], including `WktError::TypeMismatch` for a body of a
/// different kind.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{parse_multi_linestring, to_ewkt};
///
/// let e = parse_multi_linestring("SRID=4326;MULTILINESTRINGM((1 2 3,4 5 6))").unwrap();
/// assert_eq!(
///     to_ewkt(&e.geometry, e.srid),
///     "SRID=4326;MULTILINESTRING((1 2,4 5))"
/// );
/// ```
pub fn parse_multi_linestring(s: &str) -> Result<Ewkt<MultiLinestring<Linestring<Pt>>>, EwktError> {
    read(s, geometry_io_wkt::parse_multi_linestring)
}

/// Read EWKT known to be a `MULTIPOLYGON`.
///
/// # Errors
///
/// As [`parse_point`], including `WktError::TypeMismatch` for a body of a
/// different kind.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{parse_multi_polygon, to_ewkt};
///
/// let e = parse_multi_polygon("SRID=4326;MULTIPOLYGONM(((0 0 1,1 0 1,0 1 1,0 0 1)))").unwrap();
/// assert_eq!(
///     to_ewkt(&e.geometry, e.srid),
///     "SRID=4326;MULTIPOLYGON(((0 0,1 0,0 1,0 0)))"
/// );
/// ```
pub fn parse_multi_polygon(s: &str) -> Result<Ewkt<MultiPolygon<Polygon<Pt>>>, EwktError> {
    read(s, geometry_io_wkt::parse_multi_polygon)
}

/// Serialise a geometry and its SRID to canonical EWKT.
///
/// With `srid` of `None` the output is byte-identical to the WKT crate's
/// `to_wkt`.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_ewkt::{Srid, to_ewkt};
/// use geometry_model::Point2D;
///
/// let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
/// assert_eq!(to_ewkt(&p, Some(Srid::new(4326))), "SRID=4326;POINT(1 2)");
/// assert_eq!(to_ewkt(&p, None), "POINT(1 2)");
/// ```
#[must_use]
pub fn to_ewkt<G: Geometry + WriteWkt>(g: &G, srid: Option<Srid>) -> String {
    let mut out = String::with_capacity(g.wkt_capacity_hint().unwrap_or(0));
    if let Some(srid) = srid {
        // Writing into a `String` never fails, so the `Result` is discarded.
        let _ = write!(out, "SRID={srid};");
    }
    let _ = g.write_wkt_string(&mut out);
    out
}

/// Serialise a geometry and its SRID into any [`core::fmt::Write`] sink.
///
/// The streaming counterpart to [`to_ewkt`].
///
/// # Errors
///
/// Propagates any [`core::fmt::Error`] the sink returns.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_ewkt::{Srid, write_ewkt};
/// use geometry_model::Point2D;
///
/// let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
/// let mut s = String::new();
/// write_ewkt(&p, Some(Srid::UNKNOWN), &mut s).unwrap();
/// assert_eq!(s, "SRID=0;POINT(1 2)");
/// ```
pub fn write_ewkt<G: WriteWkt, W: core::fmt::Write>(
    g: &G,
    srid: Option<Srid>,
    out: &mut W,
) -> core::fmt::Result {
    if let Some(srid) = srid {
        write!(out, "SRID={srid};")?;
    }
    geometry_io_wkt::write_wkt(g, out)
}

/// Serialise any polygon implementing the geometry traits, plus its SRID,
/// to canonical EWKT.
///
/// The bring-your-own-type counterpart to [`to_ewkt`]. There is no
/// streaming variant, because the WKT crate's polygon writer is private.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_ewkt::{Srid, to_ewkt_polygon};
/// use geometry_tag::{PointTag, PolygonTag, RingTag};
/// use geometry_trait::{Geometry, Point, Polygon, Ring};
///
/// struct Coordinate(f64, f64);
///
/// impl Geometry for Coordinate {
///     type Kind = PointTag;
///     type Point = Self;
/// }
///
/// impl Point for Coordinate {
///     type Scalar = f64;
///     type Cs = Cartesian;
///     const DIM: usize = 2;
///
///     fn get<const D: usize>(&self) -> f64 {
///         match D {
///             0 => self.0,
///             1 => self.1,
///             _ => unreachable!("a Coordinate has two dimensions"),
///         }
///     }
/// }
///
/// struct Boundary(Vec<Coordinate>);
///
/// impl Geometry for Boundary {
///     type Kind = RingTag;
///     type Point = Coordinate;
/// }
///
/// impl Ring for Boundary {
///     fn points(&self) -> impl ExactSizeIterator<Item = &Coordinate> + Clone {
///         self.0.iter()
///     }
/// }
///
/// struct Parcel(Boundary);
///
/// impl Geometry for Parcel {
///     type Kind = PolygonTag;
///     type Point = Coordinate;
/// }
///
/// impl Polygon for Parcel {
///     type Ring = Boundary;
///
///     fn exterior(&self) -> &Boundary {
///         &self.0
///     }
///
///     fn interiors(&self) -> impl ExactSizeIterator<Item = &Boundary> {
///         [].iter()
///     }
/// }
///
/// let parcel = Parcel(Boundary(vec![
///     Coordinate(0.0, 0.0),
///     Coordinate(0.0, 2.0),
///     Coordinate(2.0, 2.0),
///     Coordinate(0.0, 0.0),
/// ]));
///
/// assert_eq!(
///     to_ewkt_polygon(&parcel, Some(Srid::new(4326))),
///     "SRID=4326;POLYGON((0 0,0 2,2 2,0 0))"
/// );
/// ```
#[must_use]
pub fn to_ewkt_polygon<Pg>(polygon: &Pg, srid: Option<Srid>) -> String
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    let body = geometry_io_wkt::to_wkt_polygon(polygon);
    match srid {
        None => body,
        Some(srid) => {
            let mut out = String::new();
            // Writing into a `String` never fails, so the `Result` is discarded.
            let _ = write!(out, "SRID={srid};");
            out.push_str(&body);
            out
        }
    }
}

#[cfg(test)]
mod tests {
    use alloc::format;
    use alloc::string::String;

    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D};

    use super::{Ewkt, to_ewkt, write_ewkt};
    use crate::srid::Srid;

    /// The point every writer row is written from.
    fn point() -> Point2D<f64, Cartesian> {
        Point2D::new(1.0, 2.0)
    }

    #[test]
    fn writes_no_prefix_for_none() {
        assert_eq!(to_ewkt(&point(), None), "POINT(1 2)");
    }

    #[test]
    fn writes_the_prefix_for_some() {
        assert_eq!(
            to_ewkt(&point(), Some(Srid::new(4326))),
            "SRID=4326;POINT(1 2)"
        );
    }

    #[test]
    fn writes_zero_for_unknown() {
        assert_eq!(to_ewkt(&point(), Some(Srid::UNKNOWN)), "SRID=0;POINT(1 2)");
    }

    #[test]
    fn writes_an_empty_linestring() {
        let empty: Linestring<Point2D<f64, Cartesian>> = Linestring::new();
        assert_eq!(
            to_ewkt(&empty, Some(Srid::new(4326))),
            "SRID=4326;LINESTRING EMPTY"
        );
    }

    /// A sink that accepts `budget` bytes and then refuses everything.
    struct Sink {
        budget: usize,
        written: String,
    }

    impl core::fmt::Write for Sink {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            if s.len() > self.budget {
                return Err(core::fmt::Error);
            }
            self.budget -= s.len();
            self.written.push_str(s);
            Ok(())
        }
    }

    /// `write_ewkt` streams into a caller's sink, so a sink that fails must
    /// surface as an error rather than a silently truncated geometry. Both
    /// halves of the write can fail independently: the `SRID=` prefix this
    /// crate emits, and the WKT body the inner writer emits. A zero budget
    /// fails the prefix; a budget that covers only the prefix gets past it
    /// and fails inside the body.
    #[test]
    fn a_failing_sink_is_reported_from_either_half() {
        let mut refuses_the_prefix = Sink {
            budget: 0,
            written: String::new(),
        };
        assert!(write_ewkt(&point(), Some(Srid::new(4326)), &mut refuses_the_prefix).is_err());
        assert_eq!(refuses_the_prefix.written, "");

        let mut refuses_the_body = Sink {
            budget: "SRID=4326;".len(),
            written: String::new(),
        };
        assert!(write_ewkt(&point(), Some(Srid::new(4326)), &mut refuses_the_body).is_err());
        assert_eq!(refuses_the_body.written, "SRID=4326;");
    }

    /// With no SRID there is no prefix to write, so the sink sees only the
    /// body and a generous budget succeeds.
    #[test]
    fn a_sink_with_room_receives_the_whole_geometry() {
        let mut sink = Sink {
            budget: 64,
            written: String::new(),
        };
        write_ewkt(&point(), None, &mut sink).unwrap();
        assert_eq!(sink.written, "POINT(1 2)");
    }

    #[test]
    fn display_equals_to_ewkt() {
        let srid = Some(Srid::new(4326));
        let e = Ewkt {
            srid,
            geometry: point(),
        };
        assert_eq!(format!("{e}"), "SRID=4326;POINT(1 2)");
        assert_eq!(format!("{e}"), to_ewkt(&e.geometry, srid));
    }
}
