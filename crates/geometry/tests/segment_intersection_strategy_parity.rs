//! Public-facade parity tests for the Cartesian segment-intersection strategy.

use boost_geometry::cs::Cartesian;
use boost_geometry::model::{Point2D, Segment};
use boost_geometry::overlay::predicate::{SegmentIntersection, segment_intersection};
use boost_geometry::trait_::Point as _;

type P = Point2D<f64, Cartesian>;
type S = Segment<P>;

/// `strategy/cartesian/intersection.hpp` and
/// `test/strategies/segment_intersection.cpp`: a proper crossing returns its
/// intersection point through the public facade.
#[test]
fn proper_crossing_returns_one_point() {
    let first = S::new(P::new(0.0, 0.0), P::new(4.0, 4.0));
    let second = S::new(P::new(0.0, 4.0), P::new(4.0, 0.0));
    let SegmentIntersection::Single(point) = segment_intersection(&first, &second) else {
        panic!("expected a single intersection point");
    };
    assert!((point.get::<0>() - 2.0).abs() < 1e-15);
    assert!((point.get::<1>() - 2.0).abs() < 1e-15);
}

/// Boost's `segment_intersection_points` count-two case: a collinear overlap
/// returns both ends of the shared stretch.
#[test]
fn collinear_overlap_returns_two_points() {
    let first = S::new(P::new(0.0, 0.0), P::new(5.0, 0.0));
    let second = S::new(P::new(2.0, 0.0), P::new(8.0, 0.0));
    let SegmentIntersection::Collinear { from, to } = segment_intersection(&first, &second) else {
        panic!("expected a two-point collinear overlap");
    };
    assert!((from.get::<0>() - 2.0).abs() < 1e-15);
    assert!((to.get::<0>() - 5.0).abs() < 1e-15);
}
