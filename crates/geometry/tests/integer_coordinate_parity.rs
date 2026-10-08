//! Public-facade parity tests for integer coordinates.
//!
//! Boost computes a measure of integer coordinates — a distance, a length,
//! an area, a centroid — in `double` (`promote_floating_point`), and a side
//! test exactly in a wider integer (`promote_integral`). Reference values
//! from Boost (`aed7bc3`) on the same input.

use boost_geometry::algorithm::{
    area, centroid, comparable_distance, convex_hull, covered_by, distance, intersects, is_convex,
    length, perimeter, within,
};
use boost_geometry::model::{Linestring, MultiPoint, Point2D, Polygon, Ring, Segment};
use boost_geometry::overlay::is_valid;
use boost_geometry::prelude::Cartesian;
use boost_geometry::trait_::{Point as _, Ring as _};

type P = Point2D<i32, Cartesian>;

fn polygon(points: &[(i32, i32)]) -> Polygon<P> {
    Polygon::new(Ring::from_vec(
        points.iter().map(|&(x, y)| P::new(x, y)).collect(),
    ))
}

fn coordinates(point: P) -> (i32, i32) {
    (point.get::<0>(), point.get::<1>())
}

/// Distances and lengths of integer points are `f64`: there is no integer
/// square root to fall back on.
#[test]
fn integer_points_are_measured_in_f64() {
    let (origin, corner) = (P::new(0, 0), P::new(3, 4));
    let line = Linestring::from_vec(vec![origin, corner]);
    let measured: f64 = distance(&origin, &corner);
    assert_eq!(measured, 5.0);
    assert_eq!(comparable_distance(&origin, &corner), 25.0);
    assert_eq!(length(&line), 5.0);
    let square = polygon(&[
        (0, 0),
        (0, 100_000),
        (100_000, 100_000),
        (100_000, 0),
        (0, 0),
    ]);
    assert_eq!(perimeter(&square), 400_000.0);

    let int64 = Point2D::<i64, Cartesian>::new(3, 4);
    assert_eq!(distance(&Point2D::<i64, Cartesian>::new(0, 0), &int64), 5.0);
}

/// An integer area neither truncates a half unit nor overflows the
/// coordinate type.
#[test]
fn integer_areas_keep_their_half_units_and_do_not_overflow() {
    assert_eq!(area(&polygon(&[(0, 0), (0, 1), (1, 0), (0, 0)])), 0.5);
    let square = polygon(&[
        (0, 0),
        (0, 100_000),
        (100_000, 100_000),
        (100_000, 0),
        (0, 0),
    ]);
    assert_eq!(area(&square), 1e10);
}

/// An integer centroid is computed in `f64` and converted back the way
/// Boost's `numeric_cast` does — truncated toward zero. An areal centroid is
/// computed relative to the first point and moved back after that
/// conversion, so the square below lands on `6`, not on `trunc(5.5)`.
#[test]
fn integer_centroids_truncate_where_boost_does() {
    let square = polygon(&[(11, 11), (11, 0), (0, 0), (0, 11), (11, 11)]);
    assert_eq!(coordinates(centroid(&square)), (6, 6));
    let line = Linestring::from_vec(vec![P::new(0, 0), P::new(3, 4)]);
    assert_eq!(coordinates(centroid(&line)), (1, 2));

    // The sum of these coordinates overflows `i32`; the mean does not.
    let points = MultiPoint::from_vec(vec![
        P::new(600_000_000, 1),
        P::new(700_000_000, 2),
        P::new(800_000_000, 3),
        P::new(900_000_000, 4),
    ]);
    assert_eq!(coordinates(centroid(&points)), (750_000_000, 2));
}

/// Side tests on integer coordinates are exact. The third point sits one
/// unit-area off the segment, a difference `f64` products of this size
/// cannot hold: computed in `f64`, both cross terms round to the same value
/// and the point lands on the segment.
#[test]
fn integer_side_tests_are_exact_at_any_coordinate() {
    let n = 1_073_741_822;
    let segment = Segment::new(P::new(0, 0), P::new(n + 1, n));
    assert!(!intersects(&segment, &P::new(n, n - 1)));
    assert!(intersects(&segment, &P::new(0, 0)));
}

/// Predicates and constructions whose products overflow `i32` at these
/// coordinates.
#[test]
fn integer_predicates_hold_beyond_the_square_root_of_the_type() {
    let square = polygon(&[
        (0, 0),
        (0, 300_000),
        (300_000, 300_000),
        (300_000, 0),
        (0, 0),
    ]);
    assert!(within(&P::new(150_000, 150_000), &square));
    assert!(covered_by(&P::new(0, 150_000), &square));
    assert!(is_convex(&square));
    assert_eq!(is_valid(&square), Ok(()));

    let points = MultiPoint::from_vec(vec![
        P::new(0, 0),
        P::new(300_000, 0),
        P::new(300_000, 300_000),
        P::new(0, 300_000),
        P::new(150_000, 150_000),
    ]);
    let hull: Vec<(i32, i32)> = convex_hull(&points)
        .points()
        .copied()
        .map(coordinates)
        .collect();
    assert_eq!(
        hull,
        [
            (0, 0),
            (0, 300_000),
            (300_000, 300_000),
            (300_000, 0),
            (0, 0)
        ]
    );
}

/// A polygon's centroid is computed relative to its first point, so it
/// stays exact far from the origin, where products of absolute coordinates
/// cancel to nothing.
#[test]
fn a_polygon_centroid_is_exact_far_from_the_origin() {
    type F = Point2D<f64, Cartesian>;
    let offset = 1e10;
    let square: Polygon<F> = Polygon::new(Ring::from_vec(vec![
        F::new(offset, offset),
        F::new(offset, offset + 1.0),
        F::new(offset + 1.0, offset + 1.0),
        F::new(offset + 1.0, offset),
        F::new(offset, offset),
    ]));
    let centre = centroid(&square);
    assert_eq!(
        (centre.get::<0>(), centre.get::<1>()),
        (offset + 0.5, offset + 0.5)
    );
}
