//! Public-facade parity tests for argument-order reversal.
//!
//! Mirrors the ordering semantics in
//! `test/core/reverse_dispatch.cpp:24-41` at the strategy boundary.

use boost_geometry::model::{Linestring, Point2D, Polygon, Ring, Segment};
use boost_geometry::prelude::{Cartesian, distance_with};
use boost_geometry::strategy::{
    CartesianIntersects, IntersectsStrategy, PointToSegment, Pythagoras, Reversed,
};

type P = Point2D<f64, Cartesian>;

/// `test/core/reverse_dispatch.cpp:36-41` — one public reversal adapter works
/// for different symmetric algorithm concepts, rather than each concept
/// exposing an unrelated wrapper type.
#[test]
fn one_reversed_adapter_serves_distance_and_intersects() {
    let segment = Segment::new(P::new(0.0, 0.0), P::new(4.0, 0.0));
    let point = P::new(2.0, 2.0);
    let distance = distance_with(
        &segment,
        &point,
        Reversed(PointToSegment::<Pythagoras>::default()),
    );
    assert!((distance - 2.0).abs() < 1e-12);

    let linestring = Linestring::from_vec(vec![P::new(1.0, 1.0), P::new(2.0, 2.0)]);
    let polygon: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(10.0, 0.0),
        P::new(10.0, 10.0),
        P::new(0.0, 10.0),
        P::new(0.0, 0.0),
    ]));
    let forward = CartesianIntersects.intersects(&linestring, &polygon);
    let reversed = Reversed(CartesianIntersects).intersects(&polygon, &linestring);
    assert_eq!(forward, reversed);
}
