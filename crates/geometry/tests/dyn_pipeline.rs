//! M-KC2 — heterogeneous-pipeline smoke test.
//!
//! Construction order (indices into `build_collection`):
//! 0. Point(0, 0)                      ─ length: Err; area: Err; envelope: {(0,0),(0,0)}
//! 1. LineString[(0,0)→(3,4)]          ─ length: 5;   area: Err; envelope: {(0,0),(3,4)}
//! 2. Polygon[(0,0),(2,0),(2,2),(0,2)] ─ length: Err; area: 4;   envelope: {(0,0),(2,2)}
//! 3. MultiPoint{(1,1),(5,5)}          ─ length: Err; area: Err; envelope: {(1,1),(5,5)}
//! 4. MultiLineString of the (0,0)→(3,4) line ─ length: 5; area: Err
//! 5. MultiPolygon of two 2×2 squares  ─ length: Err; area: 8
//! 6. GeometryCollection{ … }          ─ all wrappers return Err (no recursion in v1)
//!
//! Per-pair distance assertions:
//!   distance_dyn(items[0], Point(3,4)) == 5
//!   distance_dyn(items[0], items[1])   == 0  (point on linestring's first vertex)
//!   distance_dyn(items[2], items[2])   -> Err (Polygon × Polygon unsupported in v1)
#![allow(
    clippy::float_cmp,
    reason = "Reference values are exact integer-valued literals."
)]
#![allow(
    clippy::doc_markdown,
    reason = "The construction-order table names OGC kinds as prose, not code spans."
)]

use boost_geometry::algorithm::{area_dyn, distance_dyn, envelope_dyn, length_dyn, within_dyn};
use boost_geometry::cs::Cartesian;
use boost_geometry::model::{
    DynGeometry, DynGeometryCollection, DynKind, MultiLinestring, MultiPoint, MultiPolygon, Point2D,
};
use boost_geometry::trait_::GeometryCollection;
use geometry_model::{linestring, polygon};

type S = f64;
type Pt = Point2D<S, Cartesian>;

fn build_collection() -> Vec<DynGeometry<S, Cartesian>> {
    vec![
        DynGeometry::Point(Pt::new(0.0, 0.0)),
        DynGeometry::LineString(linestring![(0.0, 0.0), (3.0, 4.0)]),
        // Clockwise ring (the model default) so its signed area is
        // positive under Boost's clockwise-positive convention.
        DynGeometry::Polygon(polygon![[
            (0.0, 0.0),
            (0.0, 2.0),
            (2.0, 2.0),
            (2.0, 0.0),
            (0.0, 0.0)
        ]]),
        DynGeometry::MultiPoint(MultiPoint(vec![Pt::new(1.0, 1.0), Pt::new(5.0, 5.0)])),
        DynGeometry::MultiLineString(MultiLinestring(vec![linestring![(0.0, 0.0), (3.0, 4.0)]])),
        DynGeometry::MultiPolygon(MultiPolygon(vec![
            polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]],
            polygon![[(3.0, 0.0), (3.0, 2.0), (5.0, 2.0), (5.0, 0.0), (3.0, 0.0)]],
        ])),
        DynGeometry::GeometryCollection(vec![
            DynGeometry::Point(Pt::new(0.0, 0.0)),
            DynGeometry::LineString(linestring![(0.0, 0.0), (1.0, 1.0)]),
        ]),
    ]
}

#[test]
fn collection_is_a_geometry_collection() {
    let xs = DynGeometryCollection::<S, Cartesian>(build_collection());
    assert_eq!(xs.items().count(), 7);

    let expected_kinds = [
        DynKind::Point,
        DynKind::LineString,
        DynKind::Polygon,
        DynKind::MultiPoint,
        DynKind::MultiLineString,
        DynKind::MultiPolygon,
        DynKind::GeometryCollection,
    ];
    for (item, expected) in xs.items().zip(expected_kinds.iter()) {
        assert_eq!(item.kind(), *expected);
    }
}

#[test]
fn length_dyn_matches_hand_computation() {
    let xs = build_collection();

    // Point → 0 (non-linear, Boost length.hpp:75-80)
    assert_eq!(length_dyn(&xs[0]), 0.0);
    // LineString → 5
    assert_eq!(length_dyn(&xs[1]), 5.0);
    // Polygon → 0 (non-linear)
    assert_eq!(length_dyn(&xs[2]), 0.0);
    // MultiLineString of one LineString → same as length(inner)
    assert_eq!(length_dyn(&xs[4]), 5.0);
}

#[test]
fn area_dyn_matches_hand_computation() {
    let xs = build_collection();

    // Point → 0 (non-areal, Boost area.hpp)
    assert_eq!(area_dyn(&xs[0]), 0.0);
    // 2×2 square (clockwise ring) → +4
    assert!((area_dyn(&xs[2]) - 4.0).abs() < 1e-12);
    // MultiPolygon of two 2×2 squares → 8
    assert!((area_dyn(&xs[5]) - 8.0).abs() < 1e-12);
}

#[test]
fn envelope_dyn_is_total_for_single_and_multi_variants() {
    let xs = build_collection();
    // Indices 0..=5 are single/multi kinds — every one has an envelope.
    // The `GeometryCollection` arm (index 6) is deliberately Err in v1
    // (it needs a box-merge primitive; see `dyn_envelope`).
    for item in &xs[0..=5] {
        let _ = envelope_dyn(item).expect("envelope_dyn supports every single/multi kind");
    }
    assert!(envelope_dyn(&xs[6]).is_err());
}

#[test]
fn distance_dyn_supported_and_unsupported() {
    let xs = build_collection();

    // Supported: Point × Point — hand-computed 3-4-5.
    let three_four_five = DynGeometry::<S, Cartesian>::Point(Pt::new(3.0, 4.0));
    assert_eq!(distance_dyn(&xs[0], &three_four_five).unwrap(), 5.0);

    // Point (0,0) to the linestring starting at (0,0) → 0 (on first vertex).
    assert_eq!(distance_dyn(&xs[0], &xs[1]).unwrap(), 0.0);

    // Unsupported in v1: Polygon × Polygon.
    let err = distance_dyn(&xs[2], &xs[2]).unwrap_err();
    assert_eq!(err.got, vec![DynKind::Polygon, DynKind::Polygon]);
    assert!(!err.expected.is_empty());
}

#[test]
fn within_dyn_supported_and_unsupported() {
    let xs = build_collection();

    // Supported: (Point, Polygon).
    let p = DynGeometry::<S, Cartesian>::Point(Pt::new(1.0, 1.0));
    assert!(within_dyn(&p, &xs[2]).unwrap());

    // Unsupported: (LineString, LineString).
    let err = within_dyn(&xs[1], &xs[1]).unwrap_err();
    assert_eq!(err.got, vec![DynKind::LineString, DynKind::LineString]);
}

#[test]
fn mismatch_error_renders_human_readably() {
    // `within_dyn(Point, Point)` is unsupported → a DynKindMismatch
    // whose Display names both the received kinds and the supported
    // combination.
    let a = DynGeometry::<S, Cartesian>::Point(Pt::new(0.0, 0.0));
    let b = DynGeometry::<S, Cartesian>::Point(Pt::new(1.0, 1.0));
    let err = within_dyn(&a, &b).unwrap_err();
    let s = format!("{err}");
    assert!(s.contains("Point")); // in `got`
    assert!(s.contains("Polygon")); // in `expected`
}
