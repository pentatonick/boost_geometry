//! The WKB recursive-descent parser.
//!
//! Reads a byte buffer laid out per OGC Simple Feature Access 06-103r4
//! §8.2 and emits a [`DynGeometry`]. Each record opens with an
//! endianness flag and a 32-bit type tag (§8.2.3–8.2.4); the multi and
//! collection kinds nest *complete* WKB records (each with its own
//! byte-order flag and header), so the parser recurses through
//! [`Parser::parse_geometry`] for every member. This port emits a
//! [`DynGeometry`] because WKB is heterogeneous by construction (a
//! `GeometryCollection` mixes kinds), matching the sibling WKT reader.
//!
//! # Dimension handling (2D only)
//!
//! This is a strictly-2D reader. The 32-bit type tag is inspected for
//! `Z`/`M` markers in both encodings the wild uses: the EWKB/OGC high
//! bits (`0x8000_0000` = Z, `0x4000_0000` = M) and the ISO SQL/MM
//! ranges (`1000`+ = Z, `2000`+ = M, `3000`+ = ZM). Any such marker is
//! **rejected** with [`WkbError::UnsupportedDimension`] rather than
//! silently dropping the extra ordinates — a WKB reader cannot re-emit
//! bytes it discarded, so a lossy read would break round-trip parity.
//! Only the seven base codes `1..=7` are accepted.
//!
//! Reference: OGC 06-103r4 §8.2.

use alloc::vec::Vec;

use geometry_cs::Cartesian;
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};

use crate::header::{ByteOrder, Cursor, WkbError};

/// A concrete 2D Cartesian point — the coordinate type every parsed
/// geometry is built from.
type Pt = Point2D<f64, Cartesian>;

/// OGC base type code for `Point` (06-103r4 §8.2.4).
const WKB_POINT: u32 = 1;
/// OGC base type code for `LineString`.
const WKB_LINESTRING: u32 = 2;
/// OGC base type code for `Polygon`.
const WKB_POLYGON: u32 = 3;
/// OGC base type code for `MultiPoint`.
const WKB_MULTIPOINT: u32 = 4;
/// OGC base type code for `MultiLineString`.
const WKB_MULTILINESTRING: u32 = 5;
/// OGC base type code for `MultiPolygon`.
const WKB_MULTIPOLYGON: u32 = 6;
/// OGC base type code for `GeometryCollection`.
const WKB_GEOMETRYCOLLECTION: u32 = 7;

/// EWKB high bit marking a `Z` (3D) geometry-type tag.
const EWKB_Z: u32 = 0x8000_0000;
/// EWKB high bit marking an `M` (measured) geometry-type tag.
const EWKB_M: u32 = 0x4000_0000;
/// EWKB high bit marking a spatial-reference id in the tag.
const EWKB_SRID: u32 = 0x2000_0000;

/// The OGC base type code of an already-parsed geometry — used to
/// report the *found* kind when a multi-geometry member has the wrong
/// type.
fn dyn_code(g: &DynGeometry<f64, Cartesian>) -> u32 {
    match g {
        DynGeometry::Point(_) => WKB_POINT,
        DynGeometry::LineString(_) => WKB_LINESTRING,
        DynGeometry::Polygon(_) => WKB_POLYGON,
        DynGeometry::MultiPoint(_) => WKB_MULTIPOINT,
        DynGeometry::MultiLineString(_) => WKB_MULTILINESTRING,
        DynGeometry::MultiPolygon(_) => WKB_MULTIPOLYGON,
        DynGeometry::GeometryCollection(_) => WKB_GEOMETRYCOLLECTION,
    }
}

/// Reduce a raw 32-bit type tag to its base OGC code (`1..=7`),
/// rejecting any `Z`/`M`/`ZM` dimension marker.
///
/// Handles both encodings seen in real WKB:
/// * EWKB flag bits (`0x8000_0000` Z, `0x4000_0000` M, `0x2000_0000`
///   SRID), and
/// * ISO SQL/MM code ranges (`1000`+ Z, `2000`+ M, `3000`+ ZM).
fn base_type_code(tag: u32) -> Result<u32, WkbError> {
    // Any EWKB Z/M flag → higher dimension, unsupported. An SRID flag
    // would also prefix extra bytes this 2D reader does not consume.
    if tag & (EWKB_Z | EWKB_M | EWKB_SRID) != 0 {
        return Err(WkbError::UnsupportedDimension);
    }
    // ISO SQL/MM: strip the thousands digit; anything above the 2D
    // range (base + 1000/2000/3000) carries Z and/or M.
    let base = tag % 1000;
    if tag != base {
        return Err(WkbError::UnsupportedDimension);
    }
    Ok(base)
}

/// Maximum WKB record nesting depth accepted while parsing. The multi
/// and collection kinds recurse through [`Parser::parse_geometry`] per
/// member, so an adversarial buffer of tens of thousands of nested
/// `GeometryCollection` headers would otherwise overflow the native
/// stack and **abort the process** (a stack overflow is not catchable).
/// A bounded depth turns that denial-of-service into a recoverable
/// error. `128` mirrors the sibling `GeoJSON` reader's cap and the
/// `serde_json` default; real WKB nests only a few levels (a `Point`
/// inside a `MultiPoint` inside a `GeometryCollection`).
const MAX_DEPTH: usize = 128;

/// Smallest number of bytes a single point body occupies: two `f64`
/// ordinates. A `numPoints`/`numRings`-style count cannot describe more
/// points than `remaining / 16`, so a run's capacity is clamped to that.
const MIN_POINT_BYTES: usize = 16;

/// Smallest number of bytes a nested WKB record occupies: a one-byte
/// byte-order flag plus a 4-byte type tag (§8.2.3–8.2.4). A multi /
/// collection count cannot describe more members than `remaining / 5`.
const MIN_RECORD_BYTES: usize = 5;

/// Pre-reserve capacity for `count` elements, but never more than the
/// remaining buffer could actually contain (`remaining / min_elem_bytes`).
///
/// A raw WKB count is an untrusted `u32`; reserving `count` directly lets
/// a tiny corrupt buffer (e.g. a `MultiPolygon` header claiming
/// `0xFFFF_FFFF` members with no body) drive a multi-gigabyte
/// `Vec::with_capacity`, which aborts the process under a non-overcommit
/// allocator (this crate is `no_std`-capable). Clamping to what the
/// buffer can hold makes the reservation self-limiting; the read loop
/// then errors with [`WkbError::UnexpectedEof`] on the missing bytes.
fn reserve_bounded<T>(count: u32, remaining: usize, min_elem_bytes: usize) -> Vec<T> {
    let cap = (count as usize).min(remaining / min_elem_bytes);
    Vec::with_capacity(cap)
}

/// A cursor over a WKB buffer plus the recursive-descent readers.
struct Parser<'a> {
    cursor: Cursor<'a>,
}

impl<'a> Parser<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self {
            cursor: Cursor::new(bytes),
        }
    }

    /// Read one point body: two `f64` ordinates in `order`.
    fn read_point(&mut self, order: ByteOrder) -> Result<Pt, WkbError> {
        let x = self.cursor.read_f64(order)?;
        let y = self.cursor.read_f64(order)?;
        Ok(Point2D::new(x, y))
    }

    /// Read a `uint32` count followed by that many point bodies.
    /// Used by `LineString` and by each ring of a `Polygon`.
    fn read_point_run(&mut self, order: ByteOrder) -> Result<Vec<Pt>, WkbError> {
        let n = self.cursor.read_u32(order)?;
        let mut pts = reserve_bounded(n, self.cursor.remaining(), MIN_POINT_BYTES);
        for _ in 0..n {
            pts.push(self.read_point(order)?);
        }
        Ok(pts)
    }

    /// `Point` body.
    fn read_point_body(
        &mut self,
        order: ByteOrder,
    ) -> Result<DynGeometry<f64, Cartesian>, WkbError> {
        Ok(DynGeometry::Point(self.read_point(order)?))
    }

    /// `LineString` body: `uint32` numPoints, then the points.
    fn read_linestring_body(
        &mut self,
        order: ByteOrder,
    ) -> Result<DynGeometry<f64, Cartesian>, WkbError> {
        Ok(DynGeometry::LineString(Linestring(
            self.read_point_run(order)?,
        )))
    }

    /// Shared `Polygon` value: `uint32` numRings, then each ring is a
    /// `uint32` numPoints + points. The first ring is the exterior.
    fn read_polygon_value(&mut self, order: ByteOrder) -> Result<Polygon<Pt>, WkbError> {
        let ring_count = self.cursor.read_u32(order)?;
        let mut rings = (0..ring_count).map(|_| self.read_point_run(order).map(Ring::from_vec));
        let outer = match rings.next() {
            Some(r) => r?,
            None => Ring::new(),
        };
        let mut poly = Polygon::new(outer);
        for ring in rings {
            poly.inners.push(ring?);
        }
        Ok(poly)
    }

    /// `Polygon` body.
    fn read_polygon_body(
        &mut self,
        order: ByteOrder,
    ) -> Result<DynGeometry<f64, Cartesian>, WkbError> {
        Ok(DynGeometry::Polygon(self.read_polygon_value(order)?))
    }

    /// `MultiPoint` body: `uint32` numGeoms, then each member is a full
    /// nested WKB `Point` record (its own byte-order flag + header).
    fn read_multipoint_body(
        &mut self,
        order: ByteOrder,
        depth: usize,
    ) -> Result<DynGeometry<f64, Cartesian>, WkbError> {
        let n = self.cursor.read_u32(order)?;
        let mut pts = reserve_bounded(n, self.cursor.remaining(), MIN_RECORD_BYTES);
        for _ in 0..n {
            match self.parse_geometry(depth + 1)? {
                DynGeometry::Point(p) => pts.push(p),
                other => {
                    return Err(WkbError::MismatchedMemberType {
                        expected: WKB_POINT,
                        found: dyn_code(&other),
                    });
                }
            }
        }
        Ok(DynGeometry::MultiPoint(MultiPoint(pts)))
    }

    /// `MultiLineString` body: `uint32` count, then nested WKB
    /// `LineString` records.
    fn read_multilinestring_body(
        &mut self,
        order: ByteOrder,
        depth: usize,
    ) -> Result<DynGeometry<f64, Cartesian>, WkbError> {
        let n = self.cursor.read_u32(order)?;
        let mut lines = reserve_bounded(n, self.cursor.remaining(), MIN_RECORD_BYTES);
        for _ in 0..n {
            match self.parse_geometry(depth + 1)? {
                DynGeometry::LineString(ls) => lines.push(ls),
                other => {
                    return Err(WkbError::MismatchedMemberType {
                        expected: WKB_LINESTRING,
                        found: dyn_code(&other),
                    });
                }
            }
        }
        Ok(DynGeometry::MultiLineString(MultiLinestring(lines)))
    }

    /// `MultiPolygon` body: `uint32` count, then nested WKB `Polygon`
    /// records.
    fn read_multipolygon_body(
        &mut self,
        order: ByteOrder,
        depth: usize,
    ) -> Result<DynGeometry<f64, Cartesian>, WkbError> {
        let n = self.cursor.read_u32(order)?;
        let mut polys = reserve_bounded(n, self.cursor.remaining(), MIN_RECORD_BYTES);
        for _ in 0..n {
            match self.parse_geometry(depth + 1)? {
                DynGeometry::Polygon(pg) => polys.push(pg),
                other => {
                    return Err(WkbError::MismatchedMemberType {
                        expected: WKB_POLYGON,
                        found: dyn_code(&other),
                    });
                }
            }
        }
        Ok(DynGeometry::MultiPolygon(MultiPolygon(polys)))
    }

    /// `GeometryCollection` body: `uint32` count, then that many full
    /// nested WKB records of any kind. Recurses into
    /// [`Parser::parse_geometry`].
    fn read_collection_body(
        &mut self,
        order: ByteOrder,
        depth: usize,
    ) -> Result<DynGeometry<f64, Cartesian>, WkbError> {
        let n = self.cursor.read_u32(order)?;
        let mut items = reserve_bounded(n, self.cursor.remaining(), MIN_RECORD_BYTES);
        for _ in 0..n {
            items.push(self.parse_geometry(depth + 1)?);
        }
        Ok(DynGeometry::GeometryCollection(items))
    }

    /// Parse one complete WKB record: byte-order flag, 32-bit type tag
    /// (read in that order), then the kind-specific body. Mirrors the
    /// header dispatch of OGC 06-103r4 §8.2. `depth` bounds the multi /
    /// collection recursion against [`MAX_DEPTH`] so adversarial nesting
    /// fails with a recoverable error instead of overflowing the stack.
    fn parse_geometry(&mut self, depth: usize) -> Result<DynGeometry<f64, Cartesian>, WkbError> {
        if depth >= MAX_DEPTH {
            return Err(WkbError::NestingTooDeep);
        }
        let order = self.cursor.read_byte_order()?;
        let tag = self.cursor.read_u32(order)?;
        let code = base_type_code(tag)?;
        match code {
            WKB_POINT => self.read_point_body(order),
            WKB_LINESTRING => self.read_linestring_body(order),
            WKB_POLYGON => self.read_polygon_body(order),
            WKB_MULTIPOINT => self.read_multipoint_body(order, depth),
            WKB_MULTILINESTRING => self.read_multilinestring_body(order, depth),
            WKB_MULTIPOLYGON => self.read_multipolygon_body(order, depth),
            WKB_GEOMETRYCOLLECTION => self.read_collection_body(order, depth),
            other => Err(WkbError::UnknownGeometryType(other)),
        }
    }
}

/// Parse an OGC Well-Known Binary buffer into a runtime-tagged
/// [`DynGeometry`].
///
/// Implements the OGC kinds `Point` (1), `LineString` (2), `Polygon`
/// (3), `MultiPoint` (4), `MultiLineString` (5), `MultiPolygon` (6), and
/// `GeometryCollection` (7). The multi and collection kinds nest
/// *complete* WKB records — each nested member carries its own
/// byte-order flag and type header — and are read recursively. Mirrors
/// the read path implied by OGC 06-103r4 §8.2.
///
/// # Dimension handling
///
/// This reader is strictly 2D. A type tag carrying a `Z`/`M`/`ZM`
/// marker (EWKB flag bits or ISO SQL/MM `1000`+ codes) is **rejected**
/// with [`WkbError::UnsupportedDimension`]; extra ordinates are never
/// silently dropped.
///
/// # Errors
///
/// Returns a [`WkbError`] on a truncated buffer
/// ([`WkbError::UnexpectedEof`]), an invalid byte-order flag
/// ([`WkbError::InvalidByteOrder`]), an unknown or higher-dimension type
/// tag ([`WkbError::UnknownGeometryType`] /
/// [`WkbError::UnsupportedDimension`]), trailing bytes after the
/// top-level geometry ([`WkbError::TrailingBytes`]), or multi/collection
/// nesting past the recursion limit ([`WkbError::NestingTooDeep`]).
///
/// # Examples
///
/// ```
/// use geometry_io_wkb::from_wkb;
/// use geometry_model::DynKind;
///
/// // Little-endian POINT(1 2): 0x01, type 1, then x=1.0, y=2.0.
/// let bytes = [
///     0x01, // little-endian
///     0x01, 0x00, 0x00, 0x00, // type 1 = Point
///     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, // 1.0
///     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, // 2.0
/// ];
/// let g = from_wkb(&bytes).unwrap();
/// assert_eq!(g.kind(), DynKind::Point);
/// ```
pub fn from_wkb(bytes: &[u8]) -> Result<DynGeometry<f64, Cartesian>, WkbError> {
    let mut parser = Parser::new(bytes);
    let g = parser.parse_geometry(0)?;
    if parser.cursor.is_empty() {
        Ok(g)
    } else {
        Err(WkbError::TrailingBytes)
    }
}

#[cfg(test)]
mod tests {
    //! Hand-crafted little-endian byte fixtures per OGC 06-103r4 §8.2.
    #![allow(
        clippy::float_cmp,
        reason = "coordinate values come from exact integer byte literals"
    )]

    use super::*;
    use alloc::vec;
    use geometry_model::DynKind;
    use geometry_trait::{Linestring as _, Point as _, Polygon as _, Ring as _};

    /// The 8 little-endian bytes of the f64 `1.0`.
    const F1: [u8; 8] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F];
    /// The 8 little-endian bytes of the f64 `2.0`.
    const F2: [u8; 8] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40];
    /// The 8 little-endian bytes of the f64 `3.0`.
    const F3: [u8; 8] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x40];

    #[test]
    fn le_point() {
        // 0x01 LE, type 1, x=1.0, y=2.0.
        let mut b = vec![0x01, 0x01, 0x00, 0x00, 0x00];
        b.extend_from_slice(&F1);
        b.extend_from_slice(&F2);
        let g = from_wkb(&b).unwrap();
        assert_eq!(g.kind(), DynKind::Point);
        let DynGeometry::Point(p) = g else {
            unreachable!()
        };
        assert_eq!(p.get::<0>(), 1.0);
        assert_eq!(p.get::<1>(), 2.0);
    }

    #[test]
    fn le_linestring_two_points() {
        // 0x01 LE, type 2, numPoints=2, (1,2), (3,1).
        let mut b = vec![0x01, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00];
        b.extend_from_slice(&F1);
        b.extend_from_slice(&F2);
        b.extend_from_slice(&F3);
        b.extend_from_slice(&F1);
        let g = from_wkb(&b).unwrap();
        assert_eq!(g.kind(), DynKind::LineString);
        let DynGeometry::LineString(ls) = g else {
            unreachable!()
        };
        assert_eq!(ls.points().len(), 2);
        let last = ls.points().last().unwrap();
        assert_eq!(last.get::<0>(), 3.0);
        assert_eq!(last.get::<1>(), 1.0);
    }

    #[test]
    fn le_polygon_one_ring() {
        // 0x01 LE, type 3, numRings=1, numPoints=3, (1,2),(3,1),(1,2).
        let mut b = vec![
            0x01, 0x03, 0x00, 0x00, 0x00, // header
            0x01, 0x00, 0x00, 0x00, // numRings = 1
            0x03, 0x00, 0x00, 0x00, // numPoints = 3
        ];
        b.extend_from_slice(&F1);
        b.extend_from_slice(&F2);
        b.extend_from_slice(&F3);
        b.extend_from_slice(&F1);
        b.extend_from_slice(&F1);
        b.extend_from_slice(&F2);
        let g = from_wkb(&b).unwrap();
        assert_eq!(g.kind(), DynKind::Polygon);
        let DynGeometry::Polygon(pg) = g else {
            unreachable!()
        };
        assert_eq!(pg.exterior().points().len(), 3);
        assert_eq!(pg.interiors().count(), 0);
    }

    #[test]
    fn trailing_bytes_rejected() {
        let mut b = vec![0x01, 0x01, 0x00, 0x00, 0x00];
        b.extend_from_slice(&F1);
        b.extend_from_slice(&F2);
        b.push(0xFF); // one byte too many
        assert_eq!(from_wkb(&b).unwrap_err(), WkbError::TrailingBytes);
    }

    #[test]
    fn z_dimension_rejected() {
        // Type tag 0x8000_0001 (EWKB Point Z).
        let b = vec![0x01, 0x01, 0x00, 0x00, 0x80];
        assert_eq!(from_wkb(&b).unwrap_err(), WkbError::UnsupportedDimension);
    }

    #[test]
    fn iso_z_dimension_rejected() {
        // ISO SQL/MM Point Z = 1001.
        let b = vec![0x01, 0xE9, 0x03, 0x00, 0x00];
        assert_eq!(from_wkb(&b).unwrap_err(), WkbError::UnsupportedDimension);
    }

    #[test]
    fn truncated_point_is_eof() {
        // Header says Point but no coordinate bytes follow.
        let b = vec![0x01, 0x01, 0x00, 0x00, 0x00];
        assert_eq!(from_wkb(&b).unwrap_err(), WkbError::UnexpectedEof);
    }

    #[test]
    fn hostile_count_does_not_over_reserve() {
        // Regression: a header claiming a `u32::MAX` element count with no
        // body must error gracefully, NOT drive a multi-gigabyte
        // `Vec::with_capacity` (which aborts under a non-overcommit
        // allocator). The reserve is clamped to `remaining / min_elem`,
        // and the read loop then hits EOF on the first missing element.
        // LineString with numPoints = 0xFFFF_FFFF, no points.
        let ls = vec![0x01, 0x02, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF];
        assert_eq!(from_wkb(&ls).unwrap_err(), WkbError::UnexpectedEof);
        // MultiPolygon with numGeoms = 0xFFFF_FFFF, no members.
        let mpg = vec![0x01, 0x06, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF];
        assert_eq!(from_wkb(&mpg).unwrap_err(), WkbError::UnexpectedEof);
        // GeometryCollection with the same hostile count.
        let gc = vec![0x01, 0x07, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF];
        assert_eq!(from_wkb(&gc).unwrap_err(), WkbError::UnexpectedEof);
    }

    #[test]
    fn deeply_nested_collections_are_rejected_without_overflow() {
        // Regression: a chain of nested GeometryCollection headers used to
        // recurse until the native stack overflowed (uncatchable SIGABRT).
        // Now the reader caps recursion and returns `NestingTooDeep`.
        // Each level is: LE flag, type 7 (GC), count = 1 (9 bytes).
        let mut b = vec![];
        for _ in 0..10_000 {
            b.push(0x01);
            b.extend_from_slice(&7u32.to_le_bytes());
            b.extend_from_slice(&1u32.to_le_bytes());
        }
        assert_eq!(from_wkb(&b).unwrap_err(), WkbError::NestingTooDeep);
    }

    #[test]
    fn multipoint_member_of_wrong_kind_reports_both_codes() {
        // MULTIPOINT header claiming 1 member, whose nested record is a
        // (valid, empty) LineString. The old reader reported
        // `UnknownGeometryType(1)` — the EXPECTED code, as if type 1 were
        // unknown. It must name both sides.
        let mut b = vec![0x01, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00];
        // nested record: LE, type 2 (LineString), numPoints = 0
        b.extend_from_slice(&[0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(
            from_wkb(&b).unwrap_err(),
            WkbError::MismatchedMemberType {
                expected: 1,
                found: 2
            }
        );
    }
}
