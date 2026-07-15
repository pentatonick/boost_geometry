//! Public-facade tests for polygon and multipolygon topology validity.

use boost_geometry::model::{MultiPolygon, Point2D, Polygon, polygon};
use boost_geometry::prelude::{Cartesian, ValidityFailure, is_valid};

type P = Point2D<f64, Cartesian>;

fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<P> {
    polygon![[(x0, y0), (x0, y1), (x1, y1), (x1, y0), (x0, y0)]]
}

/// `test/algorithms/is_valid.cpp:650-705` — nested and intersecting interior
/// rings are rejected after each ring passes its own validity rules.
#[test]
fn polygon_rejects_crossing_and_nested_interior_rings() {
    let crossing_exterior: Polygon<P> = polygon![
        [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0)
        ],
        [(8.0, 2.0), (12.0, 2.0), (12.0, 6.0), (8.0, 6.0), (8.0, 2.0)]
    ];
    assert_eq!(
        is_valid(&crossing_exterior),
        Err(ValidityFailure::SelfIntersection)
    );

    let overlapping_holes: Polygon<P> = polygon![
        [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0)
        ],
        [(2.0, 2.0), (6.0, 2.0), (6.0, 6.0), (2.0, 6.0), (2.0, 2.0)],
        [(4.0, 4.0), (8.0, 4.0), (8.0, 8.0), (4.0, 8.0), (4.0, 4.0)]
    ];
    assert_eq!(
        is_valid(&overlapping_holes),
        Err(ValidityFailure::SelfIntersection)
    );

    let nested_holes: Polygon<P> = polygon![
        [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0)
        ],
        [(2.0, 2.0), (8.0, 2.0), (8.0, 8.0), (2.0, 8.0), (2.0, 2.0)],
        [(3.0, 3.0), (4.0, 3.0), (4.0, 4.0), (3.0, 4.0), (3.0, 3.0)]
    ];
    assert_eq!(
        is_valid(&nested_holes),
        Err(ValidityFailure::NestedInteriorRings)
    );
}

/// `test/algorithms/is_valid.cpp:612-628,680-687` — an exterior-edge contact
/// can disconnect the polygon interior while isolated boundary points remain
/// admissible.
#[test]
fn polygon_detects_disconnected_interior() {
    let edge_touch: Polygon<P> = polygon![
        [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0)
        ],
        [(0.0, 3.0), (3.0, 3.0), (3.0, 7.0), (0.0, 7.0), (0.0, 3.0)]
    ];
    assert_eq!(
        is_valid(&edge_touch),
        Err(ValidityFailure::DisconnectedInterior)
    );

    let holes_share_edge: Polygon<P> = polygon![
        [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0)
        ],
        [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0), (2.0, 2.0)],
        [(4.0, 2.0), (6.0, 2.0), (6.0, 4.0), (4.0, 4.0), (4.0, 2.0)]
    ];
    assert_eq!(
        is_valid(&holes_share_edge),
        Err(ValidityFailure::SelfIntersection)
    );

    // `test/algorithms/is_valid.cpp` case pg067: two otherwise disjoint
    // holes touch at two isolated points, disconnecting the polygon interior.
    let holes_touch_twice: Polygon<P> = polygon![
        [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0)
        ],
        [
            (1.0, 1.0),
            (2.0, 1.0),
            (2.0, 8.0),
            (9.0, 8.0),
            (9.0, 9.0),
            (1.0, 9.0),
            (1.0, 1.0)
        ],
        [(2.0, 5.0), (5.0, 5.0), (5.0, 8.0), (2.0, 5.0)]
    ];
    assert_eq!(
        is_valid(&holes_touch_twice),
        Err(ValidityFailure::DisconnectedInterior)
    );
}

/// `test/algorithms/is_valid.cpp:929-970` — multi-polygons may touch at an
/// isolated point or place a member in another member's hole, but filled
/// interiors may not overlap/share an edge.
#[test]
fn multipolygon_checks_inter_member_topology() {
    let invalid_member: MultiPolygon<Polygon<P>> = MultiPolygon::from_vec(vec![Polygon::new(
        boost_geometry::model::Ring::from_vec(vec![P::new(0.0, 0.0)]),
    )]);
    assert_eq!(is_valid(&invalid_member), Err(ValidityFailure::FewPoints));

    let overlapping =
        MultiPolygon::from_vec(vec![square(0.0, 0.0, 4.0, 4.0), square(2.0, 2.0, 6.0, 6.0)]);
    assert_eq!(
        is_valid(&overlapping),
        Err(ValidityFailure::IntersectingInteriors)
    );

    let shared_edge =
        MultiPolygon::from_vec(vec![square(0.0, 0.0, 2.0, 2.0), square(2.0, 0.0, 4.0, 2.0)]);
    assert_eq!(
        is_valid(&shared_edge),
        Err(ValidityFailure::IntersectingInteriors)
    );

    let vertex_touch =
        MultiPolygon::from_vec(vec![square(0.0, 0.0, 2.0, 2.0), square(2.0, 2.0, 4.0, 4.0)]);
    assert!(is_valid(&vertex_touch).is_ok());

    let donut: Polygon<P> = polygon![
        [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0)
        ],
        [(3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0), (3.0, 3.0)]
    ];
    let in_hole = MultiPolygon::from_vec(vec![donut, square(4.0, 4.0, 6.0, 6.0)]);
    assert!(is_valid(&in_hole).is_ok());
}
