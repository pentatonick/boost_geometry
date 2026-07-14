//! M-IO2 — WKB round-trip: build each OGC kind, serialise, re-parse, and
//! assert structural equality on the [`DynGeometry`]. Exercised in both
//! byte orders. Mirrors the WKT round-trip milestone (M-IO1).
//!
//! Reference: OGC Simple Feature Access 06-103r4 §8.2.

use geometry_cs::Cartesian;
use geometry_io_wkb::{ByteOrder, WkbError, WriteWkb, from_wkb, to_wkb, to_wkb_polygon};
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};
use geometry_tag::PointTag;
use geometry_trait::Geometry;

type Pt = Point2D<f64, Cartesian>;
type Dyn = DynGeometry<f64, Cartesian>;

/// Serialise `g` in `order`, re-parse, and assert it is unchanged.
fn assert_round_trip(g: &Dyn, order: ByteOrder) {
    let bytes = to_wkb(g, order);
    let back = from_wkb(&bytes).expect("re-parse must succeed");
    assert_eq!(&back, g, "round-trip mismatch in {order:?}");
}

/// Round-trip `g` in both byte orders.
fn assert_both_orders(g: &Dyn) {
    assert_round_trip(g, ByteOrder::LittleEndian);
    assert_round_trip(g, ByteOrder::BigEndian);
}

fn sample_ring() -> Ring<Pt> {
    Ring::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(0.0, 10.0),
        Pt::new(10.0, 10.0),
        Pt::new(10.0, 0.0),
        Pt::new(0.0, 0.0),
    ])
}

fn sample_hole() -> Ring<Pt> {
    Ring::from_vec(vec![
        Pt::new(2.0, 2.0),
        Pt::new(2.0, 4.0),
        Pt::new(4.0, 4.0),
        Pt::new(4.0, 2.0),
        Pt::new(2.0, 2.0),
    ])
}

fn sample_polygon() -> Polygon<Pt> {
    Polygon::with_inners(sample_ring(), vec![sample_hole()])
}

#[test]
fn point_round_trips() {
    assert_both_orders(&DynGeometry::Point(Pt::new(1.5, -2.25)));
}

#[test]
fn linestring_round_trips() {
    let ls = Linestring(vec![
        Pt::new(10.0, 10.0),
        Pt::new(20.0, 20.0),
        Pt::new(30.0, 40.0),
    ]);
    assert_both_orders(&DynGeometry::LineString(ls));
}

#[test]
fn polygon_with_hole_round_trips() {
    assert_both_orders(&DynGeometry::Polygon(sample_polygon()));
}

#[test]
fn multipoint_round_trips() {
    let mp = MultiPoint(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)]);
    assert_both_orders(&DynGeometry::MultiPoint(mp));
}

#[test]
fn multilinestring_round_trips() {
    let mls = MultiLinestring(vec![
        Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)]),
        Linestring(vec![Pt::new(15.0, 15.0), Pt::new(30.0, 15.0)]),
    ]);
    assert_both_orders(&DynGeometry::MultiLineString(mls));
}

#[test]
fn multipolygon_round_trips() {
    let mpg = MultiPolygon(vec![sample_polygon(), Polygon::new(sample_ring())]);
    assert_both_orders(&DynGeometry::MultiPolygon(mpg));
}

#[test]
fn geometry_collection_round_trips() {
    let g = DynGeometry::GeometryCollection(vec![
        DynGeometry::Point(Pt::new(10.0, 10.0)),
        DynGeometry::LineString(Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)])),
        DynGeometry::Polygon(sample_polygon()),
        // Nested collection.
        DynGeometry::GeometryCollection(vec![DynGeometry::Point(Pt::new(3.0, 4.0))]),
    ]);
    assert_both_orders(&g);
}

#[test]
fn byte_for_byte_stable_across_reparse() {
    // A serialised buffer that survives parse → re-emit unchanged
    // (the PostGIS-parity property of M-IO2).
    let g = DynGeometry::Polygon(sample_polygon());
    for order in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
        let bytes = to_wkb(&g, order);
        let reparsed = from_wkb(&bytes).unwrap();
        let reemitted = to_wkb(&reparsed, order);
        assert_eq!(bytes, reemitted, "byte-for-byte parity failed in {order:?}");
    }
}

#[test]
fn bare_ring_and_public_polygon_writer_round_trip() {
    for order in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
        let ring = sample_ring();
        let parsed_ring = from_wkb(&to_wkb(&ring, order)).unwrap();
        assert_eq!(parsed_ring, Dyn::Polygon(Polygon::new(ring)));

        let polygon = sample_polygon();
        let parsed_polygon = from_wkb(&to_wkb_polygon(&polygon, order)).unwrap();
        assert_eq!(parsed_polygon, Dyn::Polygon(polygon));
    }
}

#[test]
fn collection_with_every_geometry_kind_round_trips() {
    let polygon = sample_polygon();
    let collection = Dyn::GeometryCollection(vec![
        Dyn::Point(Pt::new(1.0, 2.0)),
        Dyn::LineString(Linestring(vec![Pt::new(3.0, 4.0), Pt::new(5.0, 6.0)])),
        Dyn::Polygon(polygon.clone()),
        Dyn::MultiPoint(MultiPoint(vec![Pt::new(7.0, 8.0)])),
        Dyn::MultiLineString(MultiLinestring(vec![Linestring(vec![Pt::new(9.0, 10.0)])])),
        Dyn::MultiPolygon(MultiPolygon(vec![polygon])),
        Dyn::GeometryCollection(vec![Dyn::Point(Pt::new(11.0, 12.0))]),
    ]);
    assert_both_orders(&collection);
}

fn little_endian_container(type_code: u32, member: &[u8]) -> Vec<u8> {
    let mut bytes = vec![1];
    bytes.extend_from_slice(&type_code.to_le_bytes());
    bytes.extend_from_slice(&1_u32.to_le_bytes());
    bytes.extend_from_slice(member);
    bytes
}

#[test]
fn malformed_documents_cover_the_public_error_contract() {
    assert_eq!(from_wkb(&[]).unwrap_err(), WkbError::UnexpectedEof);
    assert_eq!(from_wkb(&[2]).unwrap_err(), WkbError::InvalidByteOrder(2));

    let mut unknown = vec![1];
    unknown.extend_from_slice(&8_u32.to_le_bytes());
    assert_eq!(
        from_wkb(&unknown).unwrap_err(),
        WkbError::UnknownGeometryType(8)
    );

    for type_code in [0x8000_0001_u32, 0x4000_0001, 0x2000_0001, 1001, 2001, 3001] {
        let mut dimensional = vec![1];
        dimensional.extend_from_slice(&type_code.to_le_bytes());
        assert_eq!(
            from_wkb(&dimensional).unwrap_err(),
            WkbError::UnsupportedDimension
        );
    }

    let mut trailing = to_wkb(&Pt::new(1.0, 2.0), ByteOrder::LittleEndian);
    trailing.push(0xff);
    assert_eq!(from_wkb(&trailing).unwrap_err(), WkbError::TrailingBytes);

    let wrong_members = [
        Dyn::LineString(Linestring(vec![])),
        Dyn::Polygon(Polygon::new(Ring::new())),
        Dyn::MultiPoint(MultiPoint(vec![])),
        Dyn::MultiLineString(MultiLinestring(vec![])),
        Dyn::MultiPolygon(MultiPolygon(vec![])),
        Dyn::GeometryCollection(vec![]),
    ];
    for (offset, member) in wrong_members.into_iter().enumerate() {
        let member_code = u32::try_from(offset + 2).unwrap();
        let member = to_wkb(&member, ByteOrder::LittleEndian);
        assert_eq!(
            from_wkb(&little_endian_container(4, &member)).unwrap_err(),
            WkbError::MismatchedMemberType {
                expected: 1,
                found: member_code,
            }
        );
    }

    for container_code in [2_u32, 3, 4, 5, 6, 7] {
        let mut truncated = vec![1];
        truncated.extend_from_slice(&container_code.to_le_bytes());
        truncated.extend_from_slice(&u32::MAX.to_le_bytes());
        assert_eq!(from_wkb(&truncated).unwrap_err(), WkbError::UnexpectedEof);
    }

    let mut deep = to_wkb(&Pt::new(0.0, 0.0), ByteOrder::LittleEndian);
    for _ in 0..129 {
        deep = little_endian_container(7, &deep);
    }
    assert_eq!(from_wkb(&deep).unwrap_err(), WkbError::NestingTooDeep);

    let errors = [
        WkbError::UnexpectedEof,
        WkbError::InvalidByteOrder(2),
        WkbError::UnknownGeometryType(8),
        WkbError::UnsupportedDimension,
        WkbError::TrailingBytes,
        WkbError::NestingTooDeep,
        WkbError::MismatchedMemberType {
            expected: 1,
            found: 2,
        },
    ];
    for error in errors {
        assert!(!error.to_string().is_empty());
    }
}

struct ExternalPointWriter;

impl Geometry for ExternalPointWriter {
    type Kind = PointTag;
    type Point = Pt;
}

impl WriteWkb for ExternalPointWriter {
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        let point = Pt::new(3.0, 4.0);
        out.extend(to_wkb(&point, order));
    }
}

#[test]
fn external_writer_uses_the_public_default_length_hint() {
    let bytes = to_wkb(&ExternalPointWriter, ByteOrder::LittleEndian);
    assert_eq!(from_wkb(&bytes), Ok(Dyn::Point(Pt::new(3.0, 4.0))));
}
