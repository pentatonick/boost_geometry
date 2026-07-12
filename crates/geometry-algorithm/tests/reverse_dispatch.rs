//! Mirrors the semantics of `test/core/reverse_dispatch.cpp` —
//! "if you wrote (A, B), you get (B, A) for free" — but tested
//! against the algorithm layer because in our design the reversal
//! lives at the strategy layer (proposal §4.6), not at the
//! algorithm-dispatch layer.
//!
//! # Why the (Point, Segment) cases call `distance_with`, not `distance`
//!
//! `geometry-strategy::distance::DefaultDistance` is keyed on the
//! *coordinate-system family pair* (per `boost::geometry::strategy::
//! distance::services::default_strategy` in
//! `boost/geometry/strategies/distance/services.hpp`), not on the
//! geometry-kind tag pair. For Cartesian × Cartesian the default
//! resolves to `Pythagoras`, which only implements
//! `DistanceStrategy<Point, Point>` — there is no
//! `(Point, Segment)` blanket via the default machinery in T22/T23.
//! The point-to-segment routing through `PointToSegment` (T24) is
//! therefore exercised via the explicit-strategy entry point
//! `distance_with`, which is precisely what this test must
//! validate: that the *strategy-layer* reverse-dispatch blanket
//! (`Reversed<S>` from T21) lifts a `DistanceStrategy<Point, Segment>`
//! into a `DistanceStrategy<Segment, Point>` with no manual
//! `(Segment, Point)` impl in sight.
//!
//! The point-point self-symmetry case (`distance(&p1, &p2)` vs
//! `distance(&p2, &p1)`) does go through the strategy-less
//! `distance` entry point because `Pythagoras` does implement
//! `DistanceStrategy<Point, Point>` directly, so both argument
//! orders resolve to the same `(Cartesian, Cartesian) → Pythagoras`
//! default.

use geometry_algorithm::{distance, distance_with};
use geometry_cs::Cartesian;
use geometry_model::{Point2D, Segment};
use geometry_strategy::cartesian::{PointToSegment, Pythagoras};
use geometry_strategy::distance::Reversed;

// reverse_dispatch.cpp:36-41 spirit: (Point, Linestring) is NOT
// reversed in Boost; (Linestring, Point) IS reversed. In our scheme
// every (B, A) → (A, B) reversal happens via the same `Reversed<S>`
// blanket — there is no "canonical orientation" flag. The test
// below verifies the symmetry property, which is what users observe
// regardless of which side Boost considered canonical.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "Reversed<S>::distance(b, a) literally forwards to S::distance(a, b) — \
              the two values are bit-identical by construction, not by numerical \
              proximity, so strict equality is the right assertion."
)]
fn distance_segment_to_point_matches_point_to_segment() {
    let s = Segment::new(
        Point2D::<f64, Cartesian>::new(0.0, 0.0),
        Point2D::<f64, Cartesian>::new(4.0, 0.0),
    );
    let p = Point2D::<f64, Cartesian>::new(2.0, 2.0);

    let pt_to_seg = distance_with(&p, &s, PointToSegment::<Pythagoras>::default());
    // `Reversed<…>` is the blanket impl from T21 (proposal §4.6).
    // It is the *only* way `(Segment, Point)` reaches the
    // `PointToSegment` kernel — no manual `(Segment, Point)` impl
    // exists in `geometry-strategy::cartesian::distance_projected_point`.
    let seg_to_pt = distance_with(&s, &p, Reversed(PointToSegment::<Pythagoras>::default()));

    assert_eq!(pt_to_seg, seg_to_pt);
    // Perpendicular drop to a horizontal segment: foot at (2, 0),
    // distance = |y| = 2.
    assert!((pt_to_seg - 2.0).abs() < 1e-12);
}

// `projected_point.cpp:30` — `POINT(6 1)` projects beyond the
// `(4 1)` endpoint of `SEGMENT(1 4, 4 1)`, distance = 2. Same
// fixture as the T24 `proj_clamps_to_end` test, exercised here in
// both argument orders to confirm reverse dispatch preserves the
// numerical answer.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "Reversed<S>::distance(b, a) literally forwards to S::distance(a, b) — \
              the two values are bit-identical by construction, not by numerical \
              proximity, so strict equality is the right assertion."
)]
fn reversed_proj_case() {
    let s = Segment::new(
        Point2D::<f64, Cartesian>::new(1.0, 4.0),
        Point2D::<f64, Cartesian>::new(4.0, 1.0),
    );
    let p = Point2D::<f64, Cartesian>::new(6.0, 1.0);

    let pt_to_seg = distance_with(&p, &s, PointToSegment::<Pythagoras>::default());
    let seg_to_pt = distance_with(&s, &p, Reversed(PointToSegment::<Pythagoras>::default()));

    assert_eq!(pt_to_seg, seg_to_pt);
    assert!((pt_to_seg - 2.0).abs() < 1e-12);
}

// Explicit `Reversed<…>` works on its own — useful when a user
// wants to commit to a particular argument order at a call site
// without relying on default-strategy machinery to do the swap.
#[test]
fn explicit_reversed_strategy_works() {
    let s = Segment::new(
        Point2D::<f64, Cartesian>::new(0.0, 0.0),
        Point2D::<f64, Cartesian>::new(10.0, 0.0),
    );
    let p = Point2D::<f64, Cartesian>::new(5.0, 7.0);

    let strat = Reversed(PointToSegment::<Pythagoras>::default());
    let d = distance_with(&s, &p, strat);

    assert!((d - 7.0).abs() < 1e-12);
}

// Point-Point is its own reverse — exercise that the symmetry holds
// even when both sides resolve to the same impl (`Pythagoras`). This
// case *does* go through the strategy-less `distance` entry point
// because `Pythagoras` directly implements
// `DistanceStrategy<Point, Point>`.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "Pythagoras is symmetric by construction over IEEE-754 floats: \
              the sum of squared differences is commutative in the inputs."
)]
fn point_point_is_self_symmetric() {
    let a = Point2D::<f64, Cartesian>::new(1.0, 2.0);
    let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
    assert_eq!(distance(&a, &b), distance(&b, &a));
}
