//! M-OVL3 — traversal-only intersection milestone.
//!
//! A hand-picked pair of polygons whose intersection is a single ring;
//! assert the ring the traversal emits has the expected shape and area.
//! Mirrors `test/algorithms/overlay/traversal.cpp`. Area is checked
//! against the exact value at Boost's `BOOST_CHECK_CLOSE(0.001%)`
//! tolerance (these integer-vertex overlaps have exact rational areas).

use geometry_algorithm::ring_area;
use geometry_cs::Cartesian;
use geometry_model::{Point2D, Ring};
use geometry_overlay::traverse::{OverlayOp, enrich, traverse};
use geometry_overlay::turn::{RingKind, get_turns_ring_ring};
use geometry_trait::{Point as _, Ring as _};

type P = Point2D<f64, Cartesian>;

fn square(x: f64, y: f64, s: f64) -> Ring<P> {
    Ring::from_vec(vec![
        P::new(x, y),
        P::new(x + s, y),
        P::new(x + s, y + s),
        P::new(x, y + s),
        P::new(x, y),
    ])
}

fn intersection_rings(a: &Ring<P>, b: &Ring<P>) -> Vec<Ring<P>> {
    let turns = get_turns_ring_ring(a, 0, RingKind::Exterior, b, 1, RingKind::Exterior);
    let enriched = enrich(a, b, &turns);
    traverse(&enriched, &turns, OverlayOp::Intersection).expect("traversal supported")
}

/// Relative tolerance matching Boost's `BOOST_CHECK_CLOSE(0.001%)`.
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-5 * a.abs().max(b.abs()).max(1.0)
}

#[test]
fn offset_unit_squares_intersection_is_unit_square() {
    // [0,2]² ∩ [1,3]² = [1,2]², area 1.
    let a = square(0.0, 0.0, 2.0);
    let b = square(1.0, 1.0, 2.0);
    let rings = intersection_rings(&a, &b);
    assert_eq!(rings.len(), 1);
    assert!(
        close(ring_area(&rings[0]).abs(), 1.0),
        "area {}",
        ring_area(&rings[0])
    );
}

#[test]
fn rectangle_overlap_intersection_area() {
    // [0,4]×[0,3] ∩ [2,6]×[1,5] = [2,4]×[1,3], area 2·2 = 4.
    // Offset in both axes so the boundaries cross transversally with no
    // shared collinear edges (the clean areal case).
    let a = Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(4.0, 0.0),
        P::new(4.0, 3.0),
        P::new(0.0, 3.0),
        P::new(0.0, 0.0),
    ]);
    let b = Ring::from_vec(vec![
        P::new(2.0, 1.0),
        P::new(6.0, 1.0),
        P::new(6.0, 5.0),
        P::new(2.0, 5.0),
        P::new(2.0, 1.0),
    ]);
    let rings = intersection_rings(&a, &b);
    assert_eq!(rings.len(), 1);
    assert!(
        close(ring_area(&rings[0]).abs(), 4.0),
        "area {}",
        ring_area(&rings[0])
    );
}

#[test]
fn intersection_ring_vertices_are_corners_of_overlap() {
    let a = square(0.0, 0.0, 2.0);
    let b = square(1.0, 1.0, 2.0);
    let rings = intersection_rings(&a, &b);
    let ring = &rings[0];
    // Every vertex is a corner of [1,2]².
    for p in ring.points() {
        let (x, y) = (p.get::<0>(), p.get::<1>());
        assert!((x - 1.0).abs() < 1e-9 || (x - 2.0).abs() < 1e-9, "x={x}");
        assert!((y - 1.0).abs() < 1e-9 || (y - 2.0).abs() < 1e-9, "y={y}");
    }
    assert_eq!(ring.points().count(), 5); // 4 corners + closing repeat
}

#[test]
fn disjoint_squares_intersection_is_empty() {
    let a = square(0.0, 0.0, 1.0);
    let b = square(5.0, 5.0, 1.0);
    let rings = intersection_rings(&a, &b);
    assert!(rings.is_empty());
}
