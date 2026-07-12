//! M-IO2 — WKB round-trip: build each OGC kind, serialise, re-parse, and
//! assert structural equality on the [`DynGeometry`]. Exercised in both
//! byte orders. Mirrors the WKT round-trip milestone (M-IO1).
//!
//! Reference: OGC Simple Feature Access 06-103r4 §8.2.

use geometry_cs::Cartesian;
use geometry_io_wkb::{ByteOrder, from_wkb, to_wkb};
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};

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
