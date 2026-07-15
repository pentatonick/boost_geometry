//! `closest_points(&a, &b) -> (Point, Point)` — nearest-point pair.
//!
//! Mirrors `boost::geometry::closest_points(g1, g2, segment_out)` from
//! `boost/geometry/algorithms/closest_points.hpp`. Boost returns the
//! closest pair as a `Segment`; the Rust port returns a `(Point, Point)`
//! tuple — same information, no `Segment::new` boilerplate at the call
//! site. The first returned point lies on `a`, the second on `b`.
//!
//! v1 ships the Cartesian pairs the Boost test fixtures cover:
//! point↔point, point↔segment, segment↔segment, and
//! linestring↔linestring. The areal (polygon) pairs depend on overlay
//! machinery and land in `phase_03`.

use geometry_strategy::{CartesianClosestPoints, ClosestPointsStrategy};

/// Return the pair of nearest points on `(a, b)` — `(pa, pb)` where
/// `pa` lies on `a`, `pb` on `b`, and `|pa − pb|` is minimal.
///
/// Mirrors `boost::geometry::closest_points` from
/// `boost/geometry/algorithms/closest_points.hpp`. The distance between
/// the returned points equals the geometry-pair distance
/// `distance(a, b)`.
///
/// # Panics
///
/// Panics if a linestring operand has fewer than 2 points — Boost
/// treats empty input as an error (`empty_input_exception`); the
/// Rust port panics with a clear message. Point and segment
/// operands cannot be empty and never panic.
#[inline]
#[must_use]
pub fn closest_points<A, B>(
    a: &A,
    b: &B,
) -> (
    <CartesianClosestPoints as ClosestPointsStrategy<A, B>>::Out,
    <CartesianClosestPoints as ClosestPointsStrategy<A, B>>::Out,
)
where
    CartesianClosestPoints: ClosestPointsStrategy<A, B>,
{
    CartesianClosestPoints.closest_points(a, b)
}

/// Return the nearest-point pair using an explicitly supplied strategy.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "closest-point strategies are zero-sized or small Copy values, matching other _with entries"
)]
pub fn closest_points_with<A, B, S>(a: &A, b: &B, strategy: S) -> (S::Out, S::Out)
where
    S: ClosestPointsStrategy<A, B>,
{
    strategy.closest_points(a, b)
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Closest-point coordinates are exact for these inputs."
)]
mod tests {
    //! Reference values mirror the point↔segment cases in
    //! `boost/geometry/test/algorithms/closest_points/pl_l.cpp` and the
    //! v1 `PointToSegment` distances (`test/strategies/projected_point.cpp`):
    //! the distance between the returned closest points equals the
    //! geometry-pair distance.

    use super::closest_points;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, Segment};
    use geometry_strategy::{DistanceStrategy, Pythagoras};
    use geometry_trait::Point as _;

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn point_above_segment_drops_perpendicular() {
        // (0,5) to segment (0,0)-(10,0) → closest pair ((0,5), (0,0)).
        let p = Pt::new(0., 5.);
        let s = Segment::new(Pt::new(0., 0.), Pt::new(10., 0.));
        let (a, b) = closest_points(&p, &s);
        assert_eq!((a.get::<0>(), a.get::<1>()), (0., 5.));
        assert_eq!((b.get::<0>(), b.get::<1>()), (0., 0.));
        assert!((Pythagoras.distance(&a, &b) - 5.0).abs() < 1e-12);
    }

    #[test]
    fn point_on_segment_returns_input() {
        let p = Pt::new(1., 1.);
        let s = Segment::new(Pt::new(0., 0.), Pt::new(3., 3.));
        let (a, b) = closest_points(&p, &s);
        assert!(Pythagoras.distance(&a, &b) < 1e-12);
    }

    #[test]
    fn crossing_segments_share_intersection_point() {
        // Two crossing segments → the intersection point (1,1) on both.
        let a = Segment::new(Pt::new(0., 0.), Pt::new(2., 2.));
        let b = Segment::new(Pt::new(0., 2.), Pt::new(2., 0.));
        let (ca, cb) = closest_points(&a, &b);
        assert!((ca.get::<0>() - 1.0).abs() < 1e-12 && (ca.get::<1>() - 1.0).abs() < 1e-12);
        assert!(Pythagoras.distance(&ca, &cb) < 1e-12);
    }

    #[test]
    fn parallel_linestrings_closest_pair() {
        // Two horizontal tracks 3 apart; nearest pair is vertically
        // aligned, distance 3.
        let a: Linestring<Pt> =
            Linestring::from_vec(alloc::vec![Pt::new(0., 0.), Pt::new(10., 0.),]);
        let b: Linestring<Pt> =
            Linestring::from_vec(alloc::vec![Pt::new(2., 3.), Pt::new(8., 3.),]);
        let (ca, cb) = closest_points(&a, &b);
        assert!((Pythagoras.distance(&ca, &cb) - 3.0).abs() < 1e-9);
    }

    #[test]
    #[should_panic(expected = "empty or degenerate linestring in closest_points")]
    fn degenerate_linestring_panics() {
        let a: Linestring<Pt> = Linestring::from_vec(alloc::vec![Pt::new(0., 0.)]);
        let b: Linestring<Pt> =
            Linestring::from_vec(alloc::vec![Pt::new(0., 0.), Pt::new(1., 0.),]);
        let _ = closest_points(&a, &b);
    }

    /// Point × Point: the closest pair is trivially the two inputs.
    #[test]
    fn point_point_returns_the_inputs() {
        let a = Pt::new(1., 2.);
        let b = Pt::new(4., 6.);
        let (ca, cb) = closest_points(&a, &b);
        assert_eq!((ca.get::<0>(), ca.get::<1>()), (1., 2.));
        assert_eq!((cb.get::<0>(), cb.get::<1>()), (4., 6.));
        assert!((Pythagoras.distance(&ca, &cb) - 5.0).abs() < 1e-12);
    }

    /// Point × Point works in 3D too, exercising the `get_dim`/`set_dim`
    /// (and `copy_point`) third-ordinate arm.
    #[test]
    fn point_point_in_three_dimensions() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let a = P3::new(0., 0., 0.);
        let b = P3::new(1., 2., 2.);
        let (ca, cb) = closest_points(&a, &b);
        assert_eq!((ca.get::<0>(), ca.get::<1>(), ca.get::<2>()), (0., 0., 0.));
        assert_eq!((cb.get::<0>(), cb.get::<1>(), cb.get::<2>()), (1., 2., 2.));
    }

    /// Two parallel, non-crossing segments: the intersection test
    /// returns `None`, so the closest pair falls out of the four endpoint
    /// projections. Here the parallel offset is a constant 1 apart.
    #[test]
    fn parallel_segments_closest_pair_via_endpoint_projection() {
        let a = Segment::new(Pt::new(0., 0.), Pt::new(4., 0.));
        let b = Segment::new(Pt::new(0., 1.), Pt::new(4., 1.));
        let (ca, cb) = closest_points(&a, &b);
        assert!((Pythagoras.distance(&ca, &cb) - 1.0).abs() < 1e-12);
    }

    /// A degenerate (zero-length) segment drives the projection down its
    /// `denominator <= 0` branch: the closest point is the segment's
    /// start.
    #[test]
    fn degenerate_segment_projects_to_its_point() {
        let p = Pt::new(3., 4.);
        // start == end: a zero-length segment at the origin.
        let s = Segment::new(Pt::new(0., 0.), Pt::new(0., 0.));
        let (a, b) = closest_points(&p, &s);
        assert_eq!((a.get::<0>(), a.get::<1>()), (3., 4.));
        assert_eq!((b.get::<0>(), b.get::<1>()), (0., 0.));
        assert!((Pythagoras.distance(&a, &b) - 5.0).abs() < 1e-12);
    }

    /// Linestring × Linestring: two disjoint parallel polylines. The
    /// windowed double-loop keeps the minimum (the first-iteration seed
    /// then the running-minimum update).
    #[test]
    fn linestring_linestring_closest_over_all_segment_pairs() {
        use geometry_model::linestring;
        let a: Linestring<Pt> = linestring![(0., 0.), (4., 0.)];
        let b: Linestring<Pt> = linestring![(0., 3.), (4., 3.)];
        let (ca, cb) = closest_points(&a, &b);
        assert!((Pythagoras.distance(&ca, &cb) - 3.0).abs() < 1e-12);
    }
}
