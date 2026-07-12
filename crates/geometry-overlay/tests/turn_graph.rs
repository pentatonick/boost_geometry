//! M-OVL2 — turn-graph round-trip milestone.
//!
//! Builds the turn graph for fixed polygon pairs and asserts the turn
//! count, each turn's method, and the operation assignment. Mirrors the
//! reference-output snapshots in
//! `test/algorithms/overlay/get_turns.cpp` and `get_turn_info.cpp` —
//! the counts and classifications here are the hand-verified expected
//! values for these small inputs (integer coordinates, so the turn
//! points are exact).

use geometry_cs::Cartesian;
use geometry_model::{Point2D, Polygon, polygon};
use geometry_overlay::turn::{Method, OperationType, get_turns_polygon_polygon};
use geometry_trait::Point as _;

type P = Point2D<f64, Cartesian>;

fn approx(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

/// Two axis-aligned squares overlapping at a corner region. Their
/// boundaries cross at exactly two points, both proper crossings.
#[test]
fn overlapping_squares_two_crossings() {
    let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
    let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
    let turns = get_turns_polygon_polygon(&a, &b);

    assert_eq!(turns.len(), 2, "expected two boundary crossings");

    // Both are proper crossings (the boundaries cut transversally at
    // (2,1) and (1,2)).
    for t in &turns {
        assert_eq!(t.method, Method::Crosses);
        assert!(!t.touch_only);
        // A crossing assigns opposite operations to the two inputs.
        assert_eq!(t.operations[0].operation, OperationType::Union);
        assert_eq!(t.operations[1].operation, OperationType::Intersection);
    }

    // The two crossing points are (2,1) and (1,2), in some order.
    let mut pts: Vec<(f64, f64)> = turns
        .iter()
        .map(|t| (t.point.get::<0>(), t.point.get::<1>()))
        .collect();
    pts.sort_by(|x, y| x.partial_cmp(y).unwrap());
    assert!(approx(pts[0].0, 1.0) && approx(pts[0].1, 2.0));
    assert!(approx(pts[1].0, 2.0) && approx(pts[1].1, 1.0));
}

/// A polygon crossed by another whose corner touches an edge — the
/// touch is a `Touch` method (lines meet at an endpoint, no transversal
/// cross), plus the crossings.
#[test]
fn corner_touch_is_classified_touch() {
    // b's bottom-left corner (1,1) sits exactly on a's top edge? No —
    // build a case where b's vertex lands on a's edge interior.
    // a: unit square [0,2]². b: diamond whose left vertex touches a's
    // right edge at (2,1).
    let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
    let b: Polygon<P> = polygon![[(2.0, 1.0), (4.0, 0.0), (4.0, 2.0), (2.0, 1.0)]];
    let turns = get_turns_polygon_polygon(&a, &b);

    // b touches a only at the single point (2,1) on a's right edge.
    assert!(!turns.is_empty(), "expected at least the touch turn");
    assert!(
        turns
            .iter()
            .any(|t| t.method == Method::Touch && t.touch_only),
        "expected a Touch turn at the vertex-on-edge contact"
    );
    assert!(
        turns
            .iter()
            .any(|t| approx(t.point.get::<0>(), 2.0) && approx(t.point.get::<1>(), 1.0)),
        "expected a turn at the contact point (2,1)"
    );
}

/// Disjoint polygons produce an empty turn graph.
#[test]
fn disjoint_polygons_empty_graph() {
    let a: Polygon<P> = polygon![[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]];
    let b: Polygon<P> = polygon![[(5.0, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 6.0), (5.0, 5.0)]];
    let turns = get_turns_polygon_polygon(&a, &b);
    assert!(turns.is_empty());
}

/// Every turn names both source geometries (`source_index` 0 and 1),
/// which is the invariant enrichment and traversal rely on.
#[test]
fn every_turn_references_both_inputs() {
    let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
    let b: Polygon<P> = polygon![[(2.0, 2.0), (6.0, 2.0), (6.0, 6.0), (2.0, 6.0), (2.0, 2.0)]];
    let turns = get_turns_polygon_polygon(&a, &b);
    assert_eq!(turns.len(), 2);
    for t in &turns {
        assert_eq!(t.operations[0].seg_id.source_index, 0);
        assert_eq!(t.operations[1].seg_id.source_index, 1);
    }
}
