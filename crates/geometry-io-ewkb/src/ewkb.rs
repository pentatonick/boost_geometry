//! The EWKB value and the crate's entry points.

use alloc::vec::Vec;

use geometry_cs::Cartesian;
use geometry_io_wkb::{ByteOrder, WriteWkb, from_wkb_parts, polygon_wkb_len, write_wkb_polygon};
use geometry_model::DynGeometry;
use geometry_srid::Srid;
use geometry_trait::{Geometry, Point, Polygon};

use crate::ewkb_error::EwkbError;
use crate::record_header::{self, Header};

/// A geometry, the spatial-reference id its record carried, and the
/// byte order its record declared.
///
/// Returned by [`from_ewkb`]. To serialize a geometry, pass it and the
/// desired SRID and byte order directly to [`to_ewkb`].
///
/// # Comparison carries the byte order
///
/// The derived `PartialEq` includes [`Ewkb::byte_order`], so **two
/// readings of the same geometry with the same SRID compare unequal if
/// the two records declared different byte orders**. That is a
/// difference of pure transport. A caller diffing a big-endian row
/// against a little-endian one should compare `srid` and `geometry`
/// field-wise rather than comparing whole `Ewkb` values.
///
/// # There is deliberately no `Display`
///
/// A geometry has no single display form. Hex is one encoding among
/// several and a caller asks for it by name, through [`to_ewkb_hex`]:
///
/// ```compile_fail,E0277
/// use geometry_io_ewkb::{ByteOrder, Ewkb};
///
/// // `Ewkb` has no `Display`, so this does not compile:
/// let value = Ewkb { srid: None, geometry: 0u8, byte_order: ByteOrder::LittleEndian };
/// let s = format!("{value}");
/// ```
///
/// [`to_ewkb_hex`]: crate::to_ewkb_hex
#[derive(Debug, Clone, PartialEq)]
pub struct Ewkb<G> {
    /// `None` when the record's type word carried no SRID flag.
    /// `Some(Srid::UNKNOWN)` is a record that carried an explicit 0,
    /// which is a different fact.
    pub srid: Option<Srid>,
    /// The geometry the record's body parsed to.
    pub geometry: G,
    /// The order the outermost record declared.
    pub byte_order: ByteOrder,
}

/// Read an EWKB record: strip the `PostGIS` header, parse the OGC
/// record underneath it, and return both with the byte order the record
/// declared.
///
/// Allocates nothing of its own: the body is parsed in place, borrowed
/// from `bytes`.
///
/// # Errors
///
/// Returns [`EwkbError::BoundingBoxFlag`], [`EwkbError::DimensionFlag`]
/// or [`EwkbError::TruncatedSrid`] for a header this reader refuses, and
/// [`EwkbError::Wkb`] for anything `geometry-io-wkb` rejects in the
/// record underneath.
///
/// Only the **outermost** record is inspected for EWKB flags. A flag on
/// a nested member is the WKB reader's to refuse, and it reports its own
/// error for it — `PostGIS`'s writer never emits one.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkb::{ByteOrder, Srid, from_ewkb};
/// use geometry_model::DynKind;
///
/// // Little-endian POINT(1 2) with SRID 4326.
/// let bytes = [
///     0x01, // little-endian
///     0x01, 0x00, 0x00, 0x20, // type 1 | SRID flag
///     0xE6, 0x10, 0x00, 0x00, // SRID 4326
///     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, // 1.0
///     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, // 2.0
/// ];
/// let e = from_ewkb(&bytes).unwrap();
/// assert_eq!(e.srid, Some(Srid::new(4326)));
/// assert_eq!(e.byte_order, ByteOrder::LittleEndian);
/// assert_eq!(e.geometry.kind(), DynKind::Point);
/// ```
pub fn from_ewkb(bytes: &[u8]) -> Result<Ewkb<DynGeometry<f64, Cartesian>>, EwkbError> {
    let Header { wkb, srid } = record_header::read(bytes)?;
    let geometry = from_wkb_parts(wkb.byte_order, wkb.type_word, wkb.body)?;
    Ok(Ewkb {
        srid,
        geometry,
        byte_order: wkb.byte_order,
    })
}

/// Write a geometry to EWKB, with or without an SRID.
///
/// With `srid: None` the output is byte-identical to
/// `geometry_io_wkb::to_wkb` — plain OGC WKB, no EWKB flag set. With
/// `Some`, the SRID flag is set on the **outermost** type word only and
/// four bytes carry the id; every member record keeps a plain OGC
/// header, which is what `PostGIS` emits.
///
/// Empty polygons are encoded as zero rings. Other geometry structure
/// is written unchanged without validation: a consumer may reject
/// short or unclosed rings, empty interiors, or holes without an exterior.
///
/// # Panics
///
/// Panics if a custom [`WriteWkb`] implementation supplies an excessive
/// capacity hint or fails to append a WKB header when an SRID is requested.
/// Geometry accessors and writers must also honor their own contracts,
/// including providing the two ordinates this 2D codec writes.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_ewkb::{ByteOrder, Srid, to_ewkb};
/// use geometry_io_wkb::to_wkb;
/// use geometry_model::Point2D;
///
/// let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
/// // No SRID: byte-identical to plain WKB.
/// assert_eq!(
///     to_ewkb(&p, None, ByteOrder::LittleEndian),
///     to_wkb(&p, ByteOrder::LittleEndian),
/// );
/// // With one: four more bytes, and the flag on the type word.
/// let with = to_ewkb(&p, Some(Srid::new(4326)), ByteOrder::LittleEndian);
/// assert_eq!(with.len(), to_wkb(&p, ByteOrder::LittleEndian).len() + 4);
/// assert_eq!(&with[..9], &[0x01, 0x01, 0x00, 0x00, 0x20, 0xE6, 0x10, 0x00, 0x00]);
/// ```
#[must_use]
pub fn to_ewkb<G: Geometry + WriteWkb>(g: &G, srid: Option<Srid>, order: ByteOrder) -> Vec<u8> {
    let hint = g.wkb_len().unwrap_or(0);
    match srid {
        None => {
            let mut out = Vec::with_capacity(hint);
            g.write_wkb(order, &mut out); // byte-identical to to_wkb
            out
        }
        Some(srid) => record_header::record_with_srid(hint, srid, |buf| g.write_wkb(order, buf)),
    }
}

/// Write a caller's own polygon type to EWKB, through the geometry
/// traits.
///
/// The EWKB counterpart to `geometry_io_wkb::to_wkb_polygon`, for a
/// caller whose polygon is not a `geometry-model` type. Writes into one
/// output buffer with no intermediate body copy. Empty polygons use zero rings; other
/// ring structures are preserved without validation, as in [`to_ewkb`].
///
/// # Panics
///
/// Panics if custom polygon accessors violate their contracts, for
/// example by reporting iterator lengths that imply an excessive allocation
/// or by failing to supply the first two point ordinates.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_ewkb::{ByteOrder, Srid, from_ewkb, to_ewkb_polygon};
/// use geometry_model::{Point2D, Polygon, Ring};
///
/// type Pt = Point2D<f64, Cartesian>;
/// let pg = Polygon::<Pt>::new(Ring::from_vec(vec![
///     Pt::new(0.0, 0.0),
///     Pt::new(0.0, 1.0),
///     Pt::new(1.0, 1.0),
///     Pt::new(0.0, 0.0),
/// ]));
///
/// let bytes = to_ewkb_polygon(&pg, Some(Srid::new(4326)), ByteOrder::LittleEndian);
/// assert_eq!(from_ewkb(&bytes).unwrap().srid, Some(Srid::new(4326)));
/// ```
#[must_use]
pub fn to_ewkb_polygon<Pg>(polygon: &Pg, srid: Option<Srid>, order: ByteOrder) -> Vec<u8>
where
    Pg: Polygon,
    Pg::Point: Point<Scalar = f64>,
{
    let hint = polygon_wkb_len(polygon).unwrap_or(0);
    match srid {
        None => {
            let mut out = Vec::with_capacity(hint);
            write_wkb_polygon(polygon, order, &mut out);
            out
        }
        Some(srid) => record_header::record_with_srid(hint, srid, |buf| {
            write_wkb_polygon(polygon, order, buf);
        }),
    }
}
