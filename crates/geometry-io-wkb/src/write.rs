//! The WKB serializer.
//!
//! Emits a byte buffer laid out per OGC Simple Feature Access 06-103r4
//! §8.2: each record is a one-byte endianness flag, a 32-bit type tag
//! written in that order, then the kind-specific body. The multi and
//! collection kinds nest *complete* WKB records (each with its own flag
//! byte and header). Every concrete model type (and [`DynGeometry`])
//! routes through the [`WriteWkb`] trait so one implementation per kind backs
//! the public [`to_wkb`] entry point, mirroring the tag-dispatched
//! design of the sibling WKT writer.
//!
//! # Dimension
//!
//! This is a 2D writer: only the base OGC type codes (`1..=7`) and the
//! first two ordinates of each point are written. No `Z`/`M` flag bits
//! are ever set, so output re-parses cleanly through the strictly-2D
//! [`crate::from_wkb`].
//!
//! Reference: OGC 06-103r4 §8.2.

use alloc::vec::Vec;

use geometry_cs::CoordinateSystem;
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon, Ring,
};
use geometry_trait::{
    Geometry, Linestring as LinestringTrait, MultiLinestring as MultiLinestringTrait,
    MultiPoint as MultiPointTrait, MultiPolygon as MultiPolygonTrait, Point as PointTrait,
    Polygon as PolygonTrait, Ring as RingTrait,
};

use crate::header::ByteOrder;

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

/// Serialise a geometry to an OGC Well-Known Binary byte vector in the
/// caller-chosen [`ByteOrder`].
///
/// Writes the seven base OGC kinds; the multi and collection kinds nest
/// *complete* WKB records (each carrying its own byte-order flag and
/// header), all in the same requested order. Only the first two
/// ordinates of each point are written — this is a 2D writer, so no
/// `Z`/`M` flag bits are set and the output re-parses through
/// [`crate::from_wkb`]. Mirrors the write path implied by OGC 06-103r4
/// §8.2.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_wkb::{from_wkb, to_wkb, ByteOrder};
/// use geometry_model::Point2D;
///
/// let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
/// let bytes = to_wkb(&p, ByteOrder::LittleEndian);
/// assert_eq!(bytes[0], 0x01); // little-endian flag
/// assert!(from_wkb(&bytes).is_ok());
/// ```
#[must_use]
pub fn to_wkb<G: Geometry + WriteWkb>(g: &G, order: ByteOrder) -> Vec<u8> {
    let mut out = Vec::new();
    g.write_wkb(order, &mut out);
    out
}

/// The per-kind WKB emitter, implemented for every concrete model type
/// and for [`DynGeometry`].
///
/// Hidden from the public docs: callers use [`to_wkb`], which bounds on
/// this trait. It exists so one implementation per geometry kind backs
/// the entry point, mirroring the tag-dispatched stream inserters of the
/// sibling WKT writer.
#[doc(hidden)]
pub trait WriteWkb {
    /// Append `self` as a complete WKB record (byte-order flag + type
    /// tag + body) to `out`, in the given [`ByteOrder`].
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>);
}

/// Append the one-byte endianness flag (OGC 06-103r4 §8.2.3).
fn write_byte_order(order: ByteOrder, out: &mut Vec<u8>) {
    out.push(match order {
        ByteOrder::LittleEndian => 0x01,
        ByteOrder::BigEndian => 0x00,
    });
}

/// Append a `uint32` in the given byte order.
fn write_u32(v: u32, order: ByteOrder, out: &mut Vec<u8>) {
    let b = match order {
        ByteOrder::LittleEndian => v.to_le_bytes(),
        ByteOrder::BigEndian => v.to_be_bytes(),
    };
    out.extend_from_slice(&b);
}

/// Append an `f64` in the given byte order.
fn write_f64(v: f64, order: ByteOrder, out: &mut Vec<u8>) {
    let b = match order {
        ByteOrder::LittleEndian => v.to_le_bytes(),
        ByteOrder::BigEndian => v.to_be_bytes(),
    };
    out.extend_from_slice(&b);
}

/// Append the record header: byte-order flag + 32-bit type tag.
fn write_header(type_code: u32, order: ByteOrder, out: &mut Vec<u8>) {
    write_byte_order(order, out);
    write_u32(type_code, order, out);
}

/// Append one point's first two ordinates as two `f64` (no header). This
/// is a 2D writer, so only dimensions 0 and 1 are emitted.
fn write_point_body<P: PointTrait<Scalar = f64>>(p: &P, order: ByteOrder, out: &mut Vec<u8>) {
    write_f64(p.get::<0>(), order, out);
    write_f64(p.get::<1>(), order, out);
}

/// Append a `uint32` count followed by that many point bodies. Shared by
/// `LineString` and by each ring of a `Polygon`.
fn write_point_run<'a, P, I>(points: I, order: ByteOrder, out: &mut Vec<u8>)
where
    P: PointTrait<Scalar = f64> + 'a,
    I: ExactSizeIterator<Item = &'a P>,
{
    #[allow(
        clippy::cast_possible_truncation,
        reason = "WKB counts are 32-bit per OGC 06-103r4 §8.2; a WKB buffer never holds 2^32 points"
    )]
    let n = points.len() as u32;
    write_u32(n, order, out);
    for p in points {
        write_point_body(p, order, out);
    }
}

/// Append a polygon body (no header): `uint32` numRings, then each ring
/// as a `uint32` numPoints + points. Shared by `Polygon` and each
/// member of `MultiPolygon`.
fn write_polygon_body<Pg>(pg: &Pg, order: ByteOrder, out: &mut Vec<u8>)
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    #[allow(
        clippy::cast_possible_truncation,
        reason = "WKB counts are 32-bit per OGC 06-103r4 §8.2"
    )]
    let ring_count = (1 + pg.interiors().count()) as u32;
    write_u32(ring_count, order, out);
    write_point_run(pg.exterior().points(), order, out);
    for ring in pg.interiors() {
        write_point_run(ring.points(), order, out);
    }
}

impl<Cs: CoordinateSystem> WriteWkb for Point<f64, 2, Cs> {
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        write_header(WKB_POINT, order, out);
        write_point_body(self, order, out);
    }
}

impl<P: PointTrait<Scalar = f64>> WriteWkb for Linestring<P> {
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        write_header(WKB_LINESTRING, order, out);
        write_point_run(self.points(), order, out);
    }
}

// `Ring` / `Polygon` carry two const-generic booleans (clockwise,
// closed). Pinning the impls to Boost's defaults (`true, true`) — the
// shape every `DynGeometry` variant is built from — keeps const-generic
// inference unambiguous at the call site, matching the sibling WKT
// writer. A ring's serialisation does not depend on those flags.
impl<P: PointTrait<Scalar = f64>> WriteWkb for Ring<P, true, true> {
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        // A bare ring serialises as a single-ring polygon — WKB has no
        // standalone ring type code.
        write_header(WKB_POLYGON, order, out);
        write_u32(1, order, out);
        write_point_run(self.points(), order, out);
    }
}

impl<P: PointTrait<Scalar = f64>> WriteWkb for Polygon<P, true, true> {
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        write_header(WKB_POLYGON, order, out);
        write_polygon_body(self, order, out);
    }
}

impl<P: PointTrait<Scalar = f64>> WriteWkb for MultiPoint<P> {
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        write_header(WKB_MULTIPOINT, order, out);
        #[allow(
            clippy::cast_possible_truncation,
            reason = "WKB counts are 32-bit per OGC 06-103r4 §8.2"
        )]
        let n = self.points().len() as u32;
        write_u32(n, order, out);
        for p in self.points() {
            // Each member is a full nested WKB Point record.
            write_header(WKB_POINT, order, out);
            write_point_body(p, order, out);
        }
    }
}

impl<L> WriteWkb for MultiLinestring<L>
where
    L: LinestringTrait,
    L::Point: PointTrait<Scalar = f64>,
{
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        write_header(WKB_MULTILINESTRING, order, out);
        #[allow(
            clippy::cast_possible_truncation,
            reason = "WKB counts are 32-bit per OGC 06-103r4 §8.2"
        )]
        let n = self.linestrings().len() as u32;
        write_u32(n, order, out);
        for ls in self.linestrings() {
            // Each member is a full nested WKB LineString record.
            write_header(WKB_LINESTRING, order, out);
            write_point_run(ls.points(), order, out);
        }
    }
}

impl<Pg> WriteWkb for MultiPolygon<Pg>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        write_header(WKB_MULTIPOLYGON, order, out);
        #[allow(
            clippy::cast_possible_truncation,
            reason = "WKB counts are 32-bit per OGC 06-103r4 §8.2"
        )]
        let n = self.polygons().len() as u32;
        write_u32(n, order, out);
        for pg in self.polygons() {
            // Each member is a full nested WKB Polygon record.
            write_header(WKB_POLYGON, order, out);
            write_polygon_body(pg, order, out);
        }
    }
}

impl<Cs: CoordinateSystem> WriteWkb for DynGeometry<f64, Cs> {
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        // Only the `GeometryCollection` arm nests; the leaf/multi arms
        // delegate to their own non-recursive writers. WKB is
        // length-prefixed (no trailing delimiter), so a collection just
        // emits its header + member count, then its members in order.
        // Walk the nesting with an explicit stack rather than recursion so
        // a deeply nested `DynGeometry` cannot overflow the native stack
        // (an uncatchable process abort).
        let mut stack = alloc::vec![self];
        while let Some(g) = stack.pop() {
            match g {
                DynGeometry::Point(p) => p.write_wkb(order, out),
                DynGeometry::LineString(ls) => ls.write_wkb(order, out),
                DynGeometry::Polygon(pg) => pg.write_wkb(order, out),
                DynGeometry::MultiPoint(mp) => mp.write_wkb(order, out),
                DynGeometry::MultiLineString(mls) => mls.write_wkb(order, out),
                DynGeometry::MultiPolygon(mpg) => mpg.write_wkb(order, out),
                DynGeometry::GeometryCollection(items) => {
                    write_header(WKB_GEOMETRYCOLLECTION, order, out);
                    #[allow(
                        clippy::cast_possible_truncation,
                        reason = "WKB counts are 32-bit per OGC 06-103r4 §8.2"
                    )]
                    let n = items.len() as u32;
                    write_u32(n, order, out);
                    // Push members in reverse so they are emitted in source
                    // order (the stack is LIFO). All members are written
                    // contiguously right after the header+count, exactly as
                    // the recursive version did.
                    stack.extend(items.iter().rev());
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    //! Byte-exact witnesses against hand-crafted little-endian fixtures,
    //! per OGC 06-103r4 §8.2.

    use super::*;
    use alloc::vec;
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn nested_collection_writer_is_iterative() {
        use geometry_model::DynGeometry;
        // The DynGeometry writer walks nesting with an explicit stack, not
        // recursion. A shallow GC[Point, GC[Point]] emits the header+count
        // for the outer collection first (byte-exact round-trip parity is
        // checked in the reader crate); a deeply nested value must not
        // overflow the stack.
        let g = DynGeometry::<f64, Cartesian>::GeometryCollection(vec![
            DynGeometry::Point(Pt::new(1.0, 1.0)),
            DynGeometry::GeometryCollection(vec![DynGeometry::Point(Pt::new(2.0, 2.0))]),
        ]);
        let mut bytes = alloc::vec::Vec::new();
        g.write_wkb(ByteOrder::LittleEndian, &mut bytes);
        // Outer GC header: LE flag 0x01, type 7, count 2.
        assert_eq!(&bytes[..9], &[0x01, 0x07, 0, 0, 0, 0x02, 0, 0, 0]);

        let mut deep = DynGeometry::<f64, Cartesian>::Point(Pt::new(0.0, 0.0));
        for _ in 0..200_000 {
            deep = DynGeometry::GeometryCollection(vec![deep]);
        }
        let mut out = alloc::vec::Vec::new();
        deep.write_wkb(ByteOrder::LittleEndian, &mut out);
        assert!(!out.is_empty());
        core::mem::forget(deep); // avoid the still-recursive value Drop
    }

    #[test]
    fn point_little_endian() {
        // 0x01 LE, type 1, x=1.0, y=2.0.
        let expected = vec![
            0x01, // little-endian
            0x01, 0x00, 0x00, 0x00, // type 1 = Point
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, // 1.0
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, // 2.0
        ];
        assert_eq!(
            to_wkb(&Pt::new(1.0, 2.0), ByteOrder::LittleEndian),
            expected
        );
    }

    #[test]
    fn point_big_endian_flag_and_type() {
        let bytes = to_wkb(&Pt::new(1.0, 2.0), ByteOrder::BigEndian);
        // 0x00 flag, then type 1 in big-endian.
        assert_eq!(&bytes[0..5], &[0x00, 0x00, 0x00, 0x00, 0x01]);
    }
}
