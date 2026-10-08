//! `ClosestPointsStrategy<A, B>` — pair of nearest points on
//! `(A, B)`.
//!
//! Mirrors `boost::geometry::strategy::closest_points::*` from
//! `boost/geometry/strategies/closest_points/` and
//! `boost/geometry/algorithms/detail/closest_points/`. The Cartesian
//! implementations reuse the clamped-projection kernel that
//! [`crate::PointToSegment`] is built on for every point↔segment step,
//! and the segment-intersection kernel behind `intersects` for segments
//! that meet, so each answers the pair Boost answers.
//!
//! ## Coherence note
//!
//! Same workaround as [`crate::intersects`] / [`crate::within`]: the
//! impls key off the concrete `geometry-model` structs (`Point`,
//! `Segment`, `Linestring`) rather than the open geometry traits, so a
//! downstream type implementing several geometry traits at once cannot
//! trigger overlapping-impl (E0119) errors.
//!
//! ## Asymmetry
//!
//! `closest_points` is *not* symmetric in the output tuple order — the
//! first returned point lives on `a`, the second on `b`. Each pair is
//! written in its canonical `(A, B)` direction here; there is no
//! `Reversed` blanket.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{Linestring, Point as ModelPoint, Segment};
use geometry_tag::SameAs;
use geometry_trait::{Linestring as LinestringTrait, Point, PointMut, fold_dims, ordinate};

use crate::cartesian::distance_projected_point::closest_point_to_segment;
use crate::segment_intersection::{meeting_point, segment_meeting};

/// A strategy for the pair of nearest points on `(A, B)`.
///
/// Mirrors `boost::geometry::strategy::closest_points::*` from
/// `boost/geometry/strategies/closest_points/`. Boost returns the pair
/// as a `Segment`; the Rust port returns a `(Out, Out)` tuple — same
/// information, no `Segment::new` boilerplate at the call site.
pub trait ClosestPointsStrategy<A, B> {
    /// The point type the closest-pair is returned as.
    type Out: PointMut + Default;

    /// Return `(pa, pb)` where `pa` lies on `a`, `pb` lies on `b`, and
    /// the distance `|pa − pb|` is minimal over the two geometries.
    ///
    /// Mirrors `apply(g1, g2, closest_pair)` on Boost's closest-points
    /// strategy structs, returning the pair by value.
    fn closest_points(&self, a: &A, b: &B) -> (Self::Out, Self::Out);
}

/// The Cartesian closest-points kernel.
///
/// Mirrors the registration in
/// `boost/geometry/strategies/cartesian/closest_points_*.hpp`. Carries
/// no state — every per-pair computation is parameter-less.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianClosestPoints;

// ---- Point × Point ---------------------------------------------------
//
// The two closest points are trivially the two inputs. Mirrors the
// pointlike/pointlike arm at `strategies/cartesian/closest_points_pt_pt.hpp`.

impl<T, const D: usize, Cs> ClosestPointsStrategy<ModelPoint<T, D, Cs>, ModelPoint<T, D, Cs>>
    for CartesianClosestPoints
where
    T: CoordinateScalar,
    Cs: CoordinateSystem,
    Cs::Family: SameAs<CartesianFamily>,
    ModelPoint<T, D, Cs>: PointMut + Default + Copy,
{
    type Out = ModelPoint<T, D, Cs>;

    #[inline]
    fn closest_points(
        &self,
        a: &ModelPoint<T, D, Cs>,
        b: &ModelPoint<T, D, Cs>,
    ) -> (Self::Out, Self::Out) {
        (*a, *b)
    }
}

// ---- Point × Segment -------------------------------------------------
//
// The closest point on the segment is the clamped foot of the
// perpendicular from the point. Mirrors
// `strategies/cartesian/closest_points_pt_seg.hpp`.

impl<P> ClosestPointsStrategy<P, Segment<P>> for CartesianClosestPoints
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = P;

    #[inline]
    fn closest_points(&self, p: &P, s: &Segment<P>) -> (Self::Out, Self::Out) {
        (*p, closest_point_to_segment(p, *s.start(), *s.end()))
    }
}

// ---- Segment × Segment -----------------------------------------------
//
// If the two segments cross, the closest pair is the shared point
// (distance 0). Otherwise the minimum is attained by one of the four
// endpoint-to-opposite-segment projections. Mirrors
// `strategies/cartesian/closest_points_seg_seg.hpp` reduced to the
// candidate-projection form.

impl<P> ClosestPointsStrategy<Segment<P>, Segment<P>> for CartesianClosestPoints
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = P;

    fn closest_points(&self, a: &Segment<P>, b: &Segment<P>) -> (Self::Out, Self::Out) {
        segment_segment_closest(a.start(), a.end(), b.start(), b.end())
    }
}

// ---- Linestring × Linestring -----------------------------------------
//
// Mirrors `detail::closest_points::linear_to_linear`
// (`algorithms/detail/closest_points/linear_to_linear.hpp`): a one-point
// operand is a point; otherwise each segment of the operand with fewer
// segments — the second on a tie — finds its nearest segment of the
// other, and the first nearest pair wins, the search ending at a pair
// that meets.
//
// Boost finds each nearest segment through an R-tree packed from the
// other operand, which keeps a leaf's segments in input order: up to its
// eight-segment leaf capacity, two segments equally near one query are
// told apart the same way here — the first wins. Past it, the packing can
// reorder segments, and a tie between equally near segments can then be
// resolved to a different, equally near pair than Boost's.
//
// Panics on an empty linestring (mirrors Boost's empty_input_exception;
// see the algorithm-layer rustdoc).

impl<P> ClosestPointsStrategy<Linestring<P>, Linestring<P>> for CartesianClosestPoints
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = P;

    fn closest_points(&self, a: &Linestring<P>, b: &Linestring<P>) -> (Self::Out, Self::Out) {
        let pa: Vec<P> = a.points().copied().collect();
        let pb: Vec<P> = b.points().copied().collect();
        assert!(
            !pa.is_empty() && !pb.is_empty(),
            "empty linestring in closest_points"
        );
        if let [point] = pa.as_slice() {
            return point_range_closest(point, &pb);
        }
        if let [point] = pb.as_slice() {
            let (on_b, on_a) = point_range_closest(point, &pa);
            return (on_a, on_b);
        }
        if pa.len() < pb.len() {
            let (on_b, on_a) = range_range_closest(&pb, &pa);
            (on_a, on_b)
        } else {
            range_range_closest(&pa, &pb)
        }
    }
}

// ---- Kernels ---------------------------------------------------------

/// The point of `range` closest to `p`, paired after `p`.
///
/// C++: `closest_points::detail::point_to_range` with
/// `closest_feature::point_to_point_range`: the first segment as near as
/// any, unless a later segment `p` lies on ends the search there.
fn point_range_closest<P>(p: &P, range: &[P]) -> (P, P)
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
{
    let segment = |i: usize| closest_point_to_segment(p, range[i], range[i + 1]);
    let Some(last_segment) = range.len().checked_sub(2) else {
        return (*p, range[0]);
    };
    let mut nearest = 0;
    let mut nearest_distance = squared_distance(p, &segment(0));
    for i in 1..=last_segment {
        let distance = squared_distance(p, &segment(i));
        if distance.tolerant_eq(0.0) {
            nearest = i;
            break;
        }
        if distance < nearest_distance {
            nearest = i;
            nearest_distance = distance;
        }
    }
    (*p, segment(nearest))
}

/// The closest pair between the segments of `indexed` and of `queries`,
/// the point on `indexed` first.
///
/// C++: `closest_feature::range_to_range_rtree` then
/// `segment_to_segment`: each query segment in turn finds its nearest
/// indexed segment, the first strictly nearer pair is kept, and a pair at
/// distance zero (by `math::equals`) ends the search.
fn range_range_closest<P>(indexed: &[P], queries: &[P]) -> (P, P)
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
{
    let mut best: Option<((P, P), f64)> = None;
    for query in queries.windows(2) {
        let mut nearest: Option<((P, P), f64)> = None;
        for segment in indexed.windows(2) {
            let pair = segment_segment_closest(&segment[0], &segment[1], &query[0], &query[1]);
            let distance = squared_distance(&pair.0, &pair.1);
            if nearest.is_none_or(|(_, nearest_distance)| distance < nearest_distance) {
                nearest = Some((pair, distance));
            }
        }
        let Some((pair, squared)) = nearest else {
            continue;
        };
        let distance = squared.sqrt();
        if best.is_none_or(|(_, best_distance)| distance < best_distance) {
            best = Some((pair, distance));
            if distance.tolerant_eq(0.0) {
                break;
            }
        }
    }
    best.expect("both linestrings have a segment").0
}

/// Closest pair between two segments `(a0,a1)` and `(b0,b1)`, the point
/// on `a` first.
///
/// C++: `detail::closest_points::segment_to_segment`
/// (`algorithms/detail/closest_points/segment_to_segment.hpp`). Segments
/// that meet — by the kernel behind `intersects` — are paired at the
/// point Boost reports first. Otherwise each endpoint finds its closest
/// point on the other segment, `b`'s endpoints before `a`'s, and the first
/// of the nearest of those four pairs wins. The meeting test is planar, so
/// it only applies to 2-D points; higher dimensions take the endpoint
/// projections.
fn segment_segment_closest<P>(a0: &P, a1: &P, b0: &P, b1: &P) -> (P, P)
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
{
    if P::DIM == 2 {
        let xy = |q: &P| (q.get::<0>(), q.get::<1>());
        let (p1, p2, q1, q2) = (xy(a0), xy(a1), xy(b0), xy(b1));
        if let Some((x, y)) = meeting_point(p1, p2, q1, q2, segment_meeting(p1, p2, q1, q2)) {
            let mut point = P::default();
            point.set::<0>(x);
            point.set::<1>(y);
            return (point, point);
        }
    }

    let candidates = [
        (closest_point_to_segment(b0, *a0, *a1), *b0),
        (closest_point_to_segment(b1, *a0, *a1), *b1),
        (*a0, closest_point_to_segment(a0, *b0, *b1)),
        (*a1, closest_point_to_segment(a1, *b0, *b1)),
    ];
    // C++: `std::min_element` over the comparable distances.
    let mut best = candidates[0];
    let mut best_distance = squared_distance(&best.0, &best.1);
    for candidate in &candidates[1..] {
        let distance = squared_distance(&candidate.0, &candidate.1);
        if distance < best_distance {
            best = *candidate;
            best_distance = distance;
        }
    }
    best
}

/// The comparable (squared) Pythagorean distance between `a` and `b`.
#[inline]
fn squared_distance<P: Point<Scalar = f64>>(a: &P, b: &P) -> f64 {
    fold_dims(0.0, a, |sum, a, d| {
        let delta = ordinate(a, d) - ordinate(b, d);
        sum + delta * delta
    })
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Closest-point coordinates are exact for these inputs."
)]
mod tests {
    //! Reference values mirror the point↔segment cases in
    //! `boost/geometry/test/algorithms/closest_points/pl_l.cpp` and the
    //! v1 `PointToSegment` distances from
    //! `test/strategies/projected_point.cpp`.

    use super::{CartesianClosestPoints, ClosestPointsStrategy};
    use crate::cartesian::Pythagoras;
    use crate::distance::DistanceStrategy;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Segment};
    use geometry_trait::Point as _;

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn point_above_segment_drops_perpendicular() {
        let p = Pt::new(0., 5.);
        let s = Segment::new(Pt::new(0., 0.), Pt::new(10., 0.));
        let (a, b) = CartesianClosestPoints.closest_points(&p, &s);
        assert_eq!((a.get::<0>(), a.get::<1>()), (0., 5.));
        assert_eq!((b.get::<0>(), b.get::<1>()), (0., 0.));
        assert!((Pythagoras.distance(&a, &b) - 5.0).abs() < 1e-12);
    }

    #[test]
    fn point_on_segment_returns_input() {
        let p = Pt::new(1., 1.);
        let s = Segment::new(Pt::new(0., 0.), Pt::new(3., 3.));
        let (a, b) = CartesianClosestPoints.closest_points(&p, &s);
        assert!((a.get::<0>() - 1.0).abs() < 1e-12);
        assert!((b.get::<0>() - 1.0).abs() < 1e-12);
        assert!(Pythagoras.distance(&a, &b) < 1e-12);
    }

    #[test]
    fn point_beyond_segment_clamps_to_endpoint() {
        // POINT(6 1) to segment (1 4)-(4 1): projects past (4 1), so the
        // closest point on the segment is that endpoint; distance 2.
        let p = Pt::new(6., 1.);
        let s = Segment::new(Pt::new(1., 4.), Pt::new(4., 1.));
        let (a, b) = CartesianClosestPoints.closest_points(&p, &s);
        assert_eq!((b.get::<0>(), b.get::<1>()), (4., 1.));
        assert!((Pythagoras.distance(&a, &b) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn crossing_segments_share_intersection_point() {
        let a = Segment::new(Pt::new(0., 0.), Pt::new(2., 2.));
        let b = Segment::new(Pt::new(0., 2.), Pt::new(2., 0.));
        let (ca, cb) = CartesianClosestPoints.closest_points(&a, &b);
        assert!((ca.get::<0>() - 1.0).abs() < 1e-12);
        assert!((ca.get::<1>() - 1.0).abs() < 1e-12);
        assert!(Pythagoras.distance(&ca, &cb) < 1e-12);
    }

    /// The point ↔ segment pair walks every dimension: a point on a
    /// vertical 3-D segment is its own foot, and the pair's distance
    /// agrees with `PointToSegment`, which already folds all dimensions.
    #[test]
    fn three_dimensional_point_on_vertical_segment_is_its_own_foot() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let p = P3::new(0., 0., 5.);
        let s = Segment::new(P3::new(0., 0., 0.), P3::new(0., 0., 10.));
        let (a, b) = CartesianClosestPoints.closest_points(&p, &s);
        assert_eq!((a.get::<0>(), a.get::<1>(), a.get::<2>()), (0., 0., 5.));
        assert_eq!((b.get::<0>(), b.get::<1>(), b.get::<2>()), (0., 0., 5.));
        let via_distance = crate::PointToSegment::<Pythagoras>::default().distance(&p, &s);
        assert!((Pythagoras.distance(&a, &b) - via_distance).abs() < 1e-12);
    }
}
