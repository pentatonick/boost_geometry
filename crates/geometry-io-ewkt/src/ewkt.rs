//! EWKT values and entry points over the checked XY text codec.
//!
//! Prefix parsing retains effective `PostGIS` SRIDs. Body errors are rebased
//! onto the original input without rewriting or erasing any geometry tokens.

use alloc::string::String;

use geometry_cs::Cartesian;
use geometry_io_wkt::{WktError, WriteWkt};
use geometry_model::{
    DynGeometry, GeometryValue, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D,
    Polygon,
};
use geometry_trait::{Point as PointTrait, Polygon as PolygonTrait};

use crate::ewkt_error::EwktError;
use crate::ewkt_write_error::EwktWriteError;
use crate::srid_prefix::{self, Scanned};
use geometry_srid::Srid;

/// A concrete 2D Cartesian point — the coordinate type every parsed
/// geometry is built from, as in the WKT crate's parser.
type Pt = Point2D<f64, Cartesian>;

/// A geometry together with the spatial-reference id that prefixed it.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{Ewkt, Srid, from_ewkt, to_ewkt};
///
/// let e = from_ewkt("SRID=4326;POINT(1 2)").unwrap();
/// assert_eq!(e.srid, Some(Srid::new(4326)));
/// assert_eq!(to_ewkt(&e.geometry, e.srid).unwrap(), "SRID=4326;POINT(1 2)");
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

/// Scan the prefix and delegate without changing the body text.
fn read<T>(
    input: &str,
    parse_body: impl FnOnce(&str) -> Result<T, WktError>,
) -> Result<Ewkt<T>, EwktError> {
    let Scanned { srid, body_start } = srid_prefix::scan(input)?;
    match parse_body(&input[body_start..]) {
        Ok(geometry) => Ok(Ewkt { srid, geometry }),
        Err(e) => Err(EwktError::Wkt(e.with_offset(body_start))),
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
/// let e = from_ewkt("SRID=4326;POINT(1 2)").unwrap();
/// assert_eq!(e.srid, Some(Srid::new(4326)));
/// assert_eq!(e.geometry.kind(), DynKind::Point);
/// ```
pub fn from_ewkt<S: AsRef<str>>(s: S) -> Result<Ewkt<DynGeometry<f64, Cartesian>>, EwktError> {
    // A closure, not the bare fn item: `from_wkt` is generic over
    // `S: AsRef<str>` and so does not coerce to `impl FnOnce(&str) -> …`.
    read(s.as_ref(), |body| geometry_io_wkt::from_wkt(body))
}

/// Read EWKT while retaining empty points and multipoint members.
///
/// # Errors
///
/// Returns prefix errors or the errors of [`geometry_io_wkt::from_wkt_2d`],
/// with positions referring to the original EWKT input.
///
/// ```
/// use geometry_io_ewkt::{from_ewkt_2d, Srid};
/// use geometry_model::GeometryValue;
/// let value = from_ewkt_2d("SRID=4326;POINT EMPTY").unwrap();
/// assert_eq!(value.geometry, GeometryValue::Point(None));
/// assert_eq!(value.srid, Some(Srid::new(4326)));
/// ```
pub fn from_ewkt_2d<S: AsRef<str>>(
    input: S,
) -> Result<Ewkt<GeometryValue<Point2D<f64, Cartesian>>>, EwktError> {
    read(input.as_ref(), |body| geometry_io_wkt::from_wkt_2d(body))
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
/// let e = parse_point("SRID=4326;POINT(1 2)").unwrap();
/// assert_eq!(to_ewkt(&e.geometry, e.srid).unwrap(), "SRID=4326;POINT(1 2)");
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
/// let e = parse_linestring("SRID=4326;LINESTRING(1 2,4 5)").unwrap();
/// assert_eq!(to_ewkt(&e.geometry, e.srid).unwrap(), "SRID=4326;LINESTRING(1 2,4 5)");
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
/// let e = parse_polygon("SRID=4326;POLYGON((0 0,1 0,0 1,0 0))").unwrap();
/// assert_eq!(
///     to_ewkt(&e.geometry, e.srid).unwrap(),
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
/// let e = parse_multi_point("SRID=4326;MULTIPOINT(1 2,4 5)").unwrap();
/// assert_eq!(to_ewkt(&e.geometry, e.srid).unwrap(), "SRID=4326;MULTIPOINT((1 2),(4 5))");
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
/// let e = parse_multi_linestring("SRID=4326;MULTILINESTRING((1 2,4 5))").unwrap();
/// assert_eq!(
///     to_ewkt(&e.geometry, e.srid).unwrap(),
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
/// let e = parse_multi_polygon("SRID=4326;MULTIPOLYGON(((0 0,1 0,0 1,0 0)))").unwrap();
/// assert_eq!(
///     to_ewkt(&e.geometry, e.srid).unwrap(),
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
/// assert_eq!(to_ewkt(&p, Some(Srid::new(4326))).unwrap(), "SRID=4326;POINT(1 2)");
/// assert_eq!(to_ewkt(&p, None).unwrap(), "POINT(1 2)");
/// ```
///
/// # Errors
///
/// Returns an error for values outside the supported XY profile or a failing
/// custom writer. See the crate documentation for validation and migration rules.
pub fn to_ewkt<G: WriteWkt + ?Sized>(g: &G, srid: Option<Srid>) -> Result<String, EwktWriteError> {
    let mut out = String::with_capacity(g.wkt_capacity_hint().unwrap_or(0));
    srid_prefix::write(srid, &mut out)?;
    g.write_wkt_string(&mut out)?;
    Ok(out)
}

/// Serialise a geometry and its SRID into any [`core::fmt::Write`] sink.
///
/// The streaming counterpart to [`to_ewkt`].
///
/// # Errors
///
/// Returns SRID or geometry validation errors and wraps sink failures.
/// On error the sink may contain partial output; discard it.
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
) -> Result<(), EwktWriteError> {
    srid_prefix::write(srid, out)?;
    Ok(geometry_io_wkt::write_wkt(g, out)?)
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
///     to_ewkt_polygon(&parcel, Some(Srid::new(4326))).unwrap(),
///     "SRID=4326;POLYGON((0 0,0 2,2 2,0 0))"
/// );
/// ```
///
/// # Errors
///
/// Returns an error for values outside the supported XY profile or a failing
/// custom writer. See the crate documentation for validation and migration rules.
pub fn to_ewkt_polygon<Pg>(polygon: &Pg, srid: Option<Srid>) -> Result<String, EwktWriteError>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    let mut out = String::new();
    write_ewkt_polygon(polygon, srid, &mut out)?;
    Ok(out)
}

/// Write a caller-defined XY polygon and optional SRID into a text sink.
///
/// # Errors
///
/// Returns SRID, geometry or sink errors. An error can leave partial output.
///
/// ```
/// use geometry_io_ewkt::{write_ewkt_polygon, Srid};
/// use geometry_model::{Point2D, Polygon};
/// let mut text = String::new();
/// write_ewkt_polygon(&Polygon::<Point2D<f64>>::default(), Some(Srid::new(4326)), &mut text).unwrap();
/// assert_eq!(text, "SRID=4326;POLYGON EMPTY");
/// ```
pub fn write_ewkt_polygon<Pg, W>(
    polygon: &Pg,
    srid: Option<Srid>,
    out: &mut W,
) -> Result<(), EwktWriteError>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
    W: core::fmt::Write,
{
    srid_prefix::write(srid, out)?;
    Ok(geometry_io_wkt::write_wkt_polygon(polygon, out)?)
}

#[cfg(test)]
mod tests {
    use alloc::string::String;

    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D};

    use super::{Ewkt, to_ewkt, write_ewkt};
    use geometry_srid::Srid;

    /// The point every writer row is written from.
    fn point() -> Point2D<f64, Cartesian> {
        Point2D::new(1.0, 2.0)
    }

    #[test]
    fn writes_no_prefix_for_none() {
        assert_eq!(to_ewkt(&point(), None).unwrap(), "POINT(1 2)");
    }

    #[test]
    fn writes_the_prefix_for_some() {
        assert_eq!(
            to_ewkt(&point(), Some(Srid::new(4326))).unwrap(),
            "SRID=4326;POINT(1 2)"
        );
    }

    #[test]
    fn writes_zero_for_unknown() {
        assert_eq!(
            to_ewkt(&point(), Some(Srid::UNKNOWN)).unwrap(),
            "SRID=0;POINT(1 2)"
        );
    }

    #[test]
    fn writes_an_empty_linestring() {
        let empty: Linestring<Point2D<f64, Cartesian>> = Linestring::new();
        assert_eq!(
            to_ewkt(&empty, Some(Srid::new(4326))).unwrap(),
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
    fn wrapper_uses_checked_writer() {
        let srid = Some(Srid::new(4326));
        let e = Ewkt {
            srid,
            geometry: point(),
        };
        assert_eq!(
            to_ewkt(&e.geometry, e.srid).unwrap(),
            "SRID=4326;POINT(1 2)"
        );
    }
}
