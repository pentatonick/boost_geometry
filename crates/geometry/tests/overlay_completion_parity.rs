//! Public-facade regression tests for degenerate and holed areal overlay.

use boost_geometry::model::{MultiPolygon, Point2D, Polygon, Ring, polygon};
use boost_geometry::prelude::{Cartesian, area, difference, intersection, sym_difference, r#union};
use boost_geometry::trait_::{MultiPolygon as _, Polygon as _};

type P = Point2D<f64, Cartesian>;

fn total_area(result: &MultiPolygon<Polygon<P>>) -> f64 {
    result.polygons().map(|polygon| area(polygon).abs()).sum()
}

fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<P> {
    polygon![[(x0, y0), (x0, y1), (x1, y1), (x1, y0), (x0, y0)]]
}

fn donut() -> Polygon<P> {
    polygon![
        [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0)
        ],
        [(3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0), (3.0, 3.0)]
    ]
}

/// `test/algorithms/overlay/overlay.cpp:400-415` — difference/symmetric
/// difference preserve contained regions as interior rings.
#[test]
fn contained_difference_and_symmetric_difference_emit_a_hole() {
    let big = square(0.0, 0.0, 10.0, 10.0);
    let small = square(3.0, 3.0, 5.0, 5.0);

    let difference = difference(&big, &small).unwrap();
    assert_eq!(difference.polygons().count(), 1);
    assert_eq!(difference.polygons().next().unwrap().interiors().count(), 1);
    assert!((total_area(&difference) - 96.0).abs() < 1e-9);

    let symmetric = sym_difference(&big, &small).unwrap();
    assert_eq!(symmetric.polygons().count(), 1);
    assert!((total_area(&symmetric) - 96.0).abs() < 1e-9);
}

/// `test/algorithms/overlay/overlay.cpp:376-401` — colocated turns, including
/// shared edges, are resolved by the Boolean-operation driver.
#[test]
fn shared_edge_polygons_are_not_rejected() {
    let left = square(0.0, 0.0, 2.0, 2.0);
    let right = square(2.0, 0.0, 4.0, 2.0);

    assert_eq!(intersection(&left, &right).unwrap().polygons().count(), 0);
    let union = r#union(&left, &right).unwrap();
    assert_eq!(union.polygons().count(), 1);
    assert!((total_area(&union) - 8.0).abs() < 1e-9);
    assert!((total_area(&difference(&left, &right).unwrap()) - 4.0).abs() < 1e-9);
    assert!((total_area(&sym_difference(&left, &right).unwrap()) - 8.0).abs() < 1e-9);
}

/// `test/algorithms/overlay/overlay.cpp:376-384` — equal boundaries exercise a
/// maximal colocation cluster with canonical Boolean results.
#[test]
fn identical_polygons_have_canonical_boolean_results() {
    let polygon = square(0.0, 0.0, 2.0, 2.0);
    assert!((total_area(&intersection(&polygon, &polygon).unwrap()) - 4.0).abs() < 1e-9);
    assert!((total_area(&r#union(&polygon, &polygon).unwrap()) - 4.0).abs() < 1e-9);
    assert_eq!(
        difference(&polygon, &polygon).unwrap().polygons().count(),
        0
    );
    assert_eq!(
        sym_difference(&polygon, &polygon)
            .unwrap()
            .polygons()
            .count(),
        0
    );
}

/// Empty and one-vertex rings have no areal boundary. Set-theoretic Boolean
/// identities still hold through the public polygon operations, matching the
/// degenerate-input families in Boost's overlay suite.
#[test]
fn degenerate_polygons_obey_boolean_identities() {
    let empty = Polygon::new(Ring::<P>::from_vec(Vec::new()));
    let singleton = Polygon::new(Ring::from_vec(vec![P::new(1.0, 1.0)]));
    let filled = square(0.0, 0.0, 2.0, 2.0);

    for degenerate in [&empty, &singleton] {
        assert_eq!(
            intersection(degenerate, &filled)
                .unwrap()
                .polygons()
                .count(),
            0
        );
        assert_eq!(
            difference(degenerate, &filled).unwrap().polygons().count(),
            0
        );
        assert!((total_area(&r#union(degenerate, &filled).unwrap()) - 4.0).abs() < 1e-9);
        assert!((total_area(&difference(&filled, degenerate).unwrap()) - 4.0).abs() < 1e-9);
        assert!((total_area(&sym_difference(degenerate, &filled).unwrap()) - 4.0).abs() < 1e-9);
    }
}

/// Consecutive duplicate vertices do not contribute an edge. Boost's overlay
/// case corpus contains spike and redundant-vertex inputs; this public case
/// isolates the adjacent-duplicate boundary before Boolean graph assembly.
#[test]
fn adjacent_duplicate_vertices_do_not_change_boolean_results() {
    let redundant: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(0.0, 2.0),
        P::new(0.0, 2.0),
        P::new(2.0, 2.0),
        P::new(2.0, 0.0),
        P::new(0.0, 0.0),
    ]));
    let canonical = square(0.0, 0.0, 2.0, 2.0);

    assert!((total_area(&intersection(&redundant, &canonical).unwrap()) - 4.0).abs() < 1e-9);
    assert!((total_area(&r#union(&redundant, &canonical).unwrap()) - 4.0).abs() < 1e-9);
    assert_eq!(difference(&redundant, &canonical).unwrap().0.len(), 0);
}

/// `tests/xmltester/tests/general/TestNGOverlayA.xml`, "AA - repeated
/// points" — GEOS removes a repeated run on one boundary and preserves the
/// canonical union and difference areas.
#[test]
fn geos_repeated_boundary_points_preserve_areal_overlay() {
    let repeated: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(100.0, 200.0),
        P::new(200.0, 200.0),
        P::new(200.0, 100.0),
        P::new(100.0, 100.0),
        P::new(100.0, 151.0),
        P::new(100.0, 151.0),
        P::new(100.0, 151.0),
        P::new(100.0, 151.0),
        P::new(100.0, 200.0),
    ]));
    let adjacent: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(300.0, 200.0),
        P::new(300.0, 100.0),
        P::new(200.0, 100.0),
        P::new(200.0, 200.0),
        P::new(200.0, 200.0),
        P::new(300.0, 200.0),
    ]));

    assert_eq!(intersection(&repeated, &adjacent).unwrap().0.len(), 0);
    assert!((total_area(&r#union(&repeated, &adjacent).unwrap()) - 20_000.0).abs() < 1e-9);
    assert!((total_area(&difference(&repeated, &adjacent).unwrap()) - 10_000.0).abs() < 1e-9);
    assert!((total_area(&sym_difference(&repeated, &adjacent).unwrap()) - 20_000.0).abs() < 1e-9);
}

/// `test/algorithms/overlay/overlay.cpp:380-402` — hole boundaries participate
/// in clipping and assembly just like exterior boundaries.
#[test]
fn holed_polygon_overlay_preserves_topology() {
    let donut = donut();
    let middle = square(2.0, 2.0, 8.0, 8.0);

    let clipped = intersection(&donut, &middle).unwrap();
    assert_eq!(clipped.polygons().count(), 1);
    assert_eq!(clipped.polygons().next().unwrap().interiors().count(), 1);
    assert!((total_area(&clipped) - 20.0).abs() < 1e-9);

    let filled = r#union(&donut, &middle).unwrap();
    assert_eq!(filled.polygons().count(), 1);
    assert_eq!(filled.polygons().next().unwrap().interiors().count(), 0);
    assert!((total_area(&filled) - 100.0).abs() < 1e-9);

    let frame = difference(&donut, &middle).unwrap();
    assert_eq!(frame.polygons().count(), 1);
    assert!((total_area(&frame) - 64.0).abs() < 1e-9);

    let symmetric = sym_difference(&donut, &middle).unwrap();
    assert!((total_area(&symmetric) - 80.0).abs() < 1e-9);
}
