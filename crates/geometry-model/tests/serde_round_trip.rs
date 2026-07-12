//! M-AD3 — serde round-trip for every model type and every
//! `DynGeometry` variant. Requires `--features serde`.
#![cfg(feature = "serde")]

use geometry_cs::Cartesian;
use geometry_model::{
    Box, DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon,
    Ring, Segment, linestring, polygon,
};

type Pt = Point2D<f64, Cartesian>;

/// Serialise `value` to JSON, deserialise it back, and assert the
/// round-trip is exact.
fn round_trip<T>(value: &T)
where
    T: serde::Serialize + serde::de::DeserializeOwned + PartialEq + core::fmt::Debug,
{
    let json = serde_json::to_string(value).expect("serialize");
    let back: T = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(*value, back, "round-trip mismatch via {json}");
}

#[test]
fn point_round_trip() {
    round_trip(&Pt::new(1.5, -2.25));
}

#[test]
fn point_serializes_as_coordinate_tuple() {
    let json = serde_json::to_string(&Pt::new(3.0, 4.0)).unwrap();
    assert_eq!(json, "[3.0,4.0]");
}

#[test]
fn linestring_round_trip() {
    let ls: Linestring<Pt> = linestring![(0., 0.), (1., 1.), (2., 3.)];
    round_trip(&ls);
}

#[test]
fn ring_round_trip() {
    let r: Ring<Pt> = Ring::from_vec(vec![
        Pt::new(0., 0.),
        Pt::new(1., 0.),
        Pt::new(1., 1.),
        Pt::new(0., 0.),
    ]);
    round_trip(&r);
}

#[test]
fn polygon_with_hole_round_trip() {
    let pg: Polygon<Pt> = polygon![
        [(0., 0.), (5., 0.), (5., 5.), (0., 5.), (0., 0.)],
        [(1., 1.), (2., 1.), (2., 2.), (1., 1.)]
    ];
    round_trip(&pg);
}

#[test]
fn multi_point_round_trip() {
    round_trip(&MultiPoint(vec![Pt::new(0., 0.), Pt::new(1., 1.)]));
}

#[test]
fn multi_linestring_round_trip() {
    let ml: MultiLinestring<Linestring<Pt>> =
        MultiLinestring(vec![linestring![(0., 0.), (1., 1.)]]);
    round_trip(&ml);
}

#[test]
fn multi_polygon_round_trip() {
    let mp: MultiPolygon<Polygon<Pt>> =
        MultiPolygon(vec![polygon![[(0., 0.), (1., 0.), (1., 1.), (0., 0.)]]]);
    round_trip(&mp);
}

#[test]
fn box_round_trip() {
    round_trip(&Box::from_corners(Pt::new(0., 0.), Pt::new(4., 2.)));
}

#[test]
fn segment_round_trip() {
    round_trip(&Segment::new(Pt::new(0., 0.), Pt::new(3., 4.)));
}

#[test]
fn dyn_geometry_each_variant_round_trips() {
    let cases: Vec<DynGeometry<f64, Cartesian>> = vec![
        DynGeometry::Point(Pt::new(1., 2.)),
        DynGeometry::LineString(linestring![(0., 0.), (1., 1.)]),
        DynGeometry::Polygon(polygon![[(0., 0.), (1., 0.), (1., 1.), (0., 0.)]]),
        DynGeometry::MultiPoint(MultiPoint(vec![Pt::new(0., 0.)])),
        DynGeometry::MultiLineString(MultiLinestring(vec![linestring![(0., 0.), (1., 1.)]])),
        DynGeometry::MultiPolygon(MultiPolygon(vec![polygon![[
            (0., 0.),
            (1., 0.),
            (1., 1.),
            (0., 0.)
        ]]])),
        DynGeometry::GeometryCollection(vec![DynGeometry::Point(Pt::new(9., 9.))]),
    ];
    for g in &cases {
        round_trip(g);
    }
}
