//! `ClosestPointsStrategy<A, B>` — pair of nearest points on
//! `(A, B)`.
//!
//! Mirrors `boost::geometry::strategy::closest_points::*` from
//! `boost/geometry/strategies/closest_points/`. The Cartesian
//! implementations reuse the clamped-projection kernel that
//! [`crate::PointToSegment`] is built on for the point↔segment case.
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
use geometry_trait::{Linestring as LinestringTrait, Point, PointMut};

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
        (*p, foot_on_segment(p, s.start(), s.end()))
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
// Walk every sub-segment pair and keep the closest. Mirrors the
// linear/linear arm at `strategies/cartesian/closest_points_l_l.hpp`.
//
// Panics on an empty or single-point linestring (mirrors Boost's
// empty_input_exception; see the algorithm-layer rustdoc).

impl<P> ClosestPointsStrategy<Linestring<P>, Linestring<P>> for CartesianClosestPoints
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = P;

    fn closest_points(&self, a: &Linestring<P>, b: &Linestring<P>) -> (Self::Out, Self::Out) {
        let pa: Vec<&P> = a.points().collect();
        let pb: Vec<&P> = b.points().collect();
        assert!(
            pa.len() >= 2 && pb.len() >= 2,
            "empty or degenerate linestring in closest_points"
        );

        let mut best: Option<((P, P), f64)> = None;
        for wa in pa.windows(2) {
            for wb in pb.windows(2) {
                let (ca, cb) = segment_segment_closest(wa[0], wa[1], wb[0], wb[1]);
                let d = squared_distance(&ca, &cb);
                if best.is_none_or(|(_, bd)| d < bd) {
                    best = Some(((ca, cb), d));
                }
            }
        }
        best.unwrap().0
    }
}

// ---- Kernels ---------------------------------------------------------

/// Closest point on segment `a`-`b` to `p`: the clamped foot of the
/// perpendicular. Mirrors
/// `closest_points::detail::compute_closest_point_to_segment` in
/// `strategies/cartesian/closest_points_pt_seg.hpp`.
fn foot_on_segment<P>(p: &P, a: &P, b: &P) -> P
where
    P: Point<Scalar = f64> + PointMut + Default,
{
    let (numerator, denominator) = dots(p, a, b);
    if denominator <= 0.0 {
        return copy_point(a);
    }
    let t = (numerator / denominator).clamp(0.0, 1.0);
    blend(a, b, t)
}

/// Closest pair between two segments `(a0,a1)` and `(b0,b1)`.
fn segment_segment_closest<P>(a0: &P, a1: &P, b0: &P, b1: &P) -> (P, P)
where
    P: Point<Scalar = f64> + PointMut + Default,
{
    // Crossing segments share a point — the closest pair is that point
    // on both. Compute it directly from the line-line intersection.
    if let Some(pt) = segment_intersection(a0, a1, b0, b1) {
        return (copy_point(&pt), pt);
    }

    // Otherwise the minimum is one of the four endpoint projections.
    let c1 = (copy_point(a0), foot_on_segment(a0, b0, b1));
    let c2 = (copy_point(a1), foot_on_segment(a1, b0, b1));
    let c3 = (foot_on_segment(b0, a0, a1), copy_point(b0));
    let c4 = (foot_on_segment(b1, a0, a1), copy_point(b1));

    let mut best = c1;
    let mut best_d = squared_distance(&best.0, &best.1);
    for cand in [c2, c3, c4] {
        let d = squared_distance(&cand.0, &cand.1);
        if d < best_d {
            best_d = d;
            best = cand;
        }
    }
    best
}

/// Proper-crossing intersection point of two 2D segments, or `None`
/// when they do not cross (parallel, collinear, or disjoint).
fn segment_intersection<P>(a0: &P, a1: &P, b0: &P, b1: &P) -> Option<P>
where
    P: Point<Scalar = f64> + PointMut + Default,
{
    let (x1, y1) = (a0.get::<0>(), a0.get::<1>());
    let (x2, y2) = (a1.get::<0>(), a1.get::<1>());
    let (x3, y3) = (b0.get::<0>(), b0.get::<1>());
    let (x4, y4) = (b1.get::<0>(), b1.get::<1>());

    let denom = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3);
    if denom == 0.0 {
        return None;
    }
    let t = ((x3 - x1) * (y4 - y3) - (y3 - y1) * (x4 - x3)) / denom;
    let u = ((x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1)) / denom;
    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
        let mut out = P::default();
        out.set::<0>(x1 + t * (x2 - x1));
        out.set::<1>(y1 + t * (y2 - y1));
        Some(out)
    } else {
        None
    }
}

/// Compute `(dot(p − a, b − a), dot(b − a, b − a))` over 2D.
#[inline]
fn dots<P: Point<Scalar = f64>>(p: &P, a: &P, b: &P) -> (f64, f64) {
    let apx = p.get::<0>() - a.get::<0>();
    let apy = p.get::<1>() - a.get::<1>();
    let abx = b.get::<0>() - a.get::<0>();
    let aby = b.get::<1>() - a.get::<1>();
    (apx * abx + apy * aby, abx * abx + aby * aby)
}

/// Squared 2D distance between two points.
#[inline]
fn squared_distance<P: Point<Scalar = f64>>(a: &P, b: &P) -> f64 {
    let dx = a.get::<0>() - b.get::<0>();
    let dy = a.get::<1>() - b.get::<1>();
    dx * dx + dy * dy
}

/// Linear per-dimension blend `out[D] = a[D] + t·(b[D] − a[D])`.
#[inline]
fn blend<P>(a: &P, b: &P, t: f64) -> P
where
    P: Point<Scalar = f64> + PointMut + Default,
{
    let mut out = P::default();
    geometry_trait::fold_dims((), a, |(), _p, d| {
        let av = get_dim(a, d);
        let bv = get_dim(b, d);
        set_dim(&mut out, d, av + t * (bv - av));
    });
    out
}

/// Copy a point coordinate-by-coordinate (avoids a `Copy` bound where
/// only `PointMut + Default` is available).
#[inline]
fn copy_point<P>(a: &P) -> P
where
    P: Point<Scalar = f64> + PointMut + Default,
{
    let mut out = P::default();
    geometry_trait::fold_dims((), a, |(), _p, d| {
        set_dim(&mut out, d, get_dim(a, d));
    });
    out
}

#[inline]
fn get_dim<P: Point<Scalar = f64>>(p: &P, d: usize) -> f64 {
    match d {
        0 => p.get::<0>(),
        1 => p.get::<1>(),
        2 => p.get::<2>(),
        3 => p.get::<3>(),
        _ => unreachable!(),
    }
}

#[inline]
fn set_dim<P: PointMut<Scalar = f64>>(p: &mut P, d: usize, v: f64) {
    match d {
        0 => p.set::<0>(v),
        1 => p.set::<1>(v),
        2 => p.set::<2>(v),
        3 => p.set::<3>(v),
        _ => unreachable!(),
    }
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
}
