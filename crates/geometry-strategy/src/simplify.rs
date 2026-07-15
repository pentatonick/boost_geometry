//! Line simplification — Douglas–Peucker.
//!
//! Mirrors `boost::geometry::strategy::simplify::douglas_peucker<P, DistStr>`
//! from `boost/geometry/strategies/agnostic/simplify_douglas_peucker.hpp`.
//! Recursive split: for the current point range, find the vertex
//! furthest from the chord between the endpoints; if that distance is
//! below `max_distance`, drop all interior vertices; otherwise split at
//! the furthest vertex and recurse on the two halves.
//!
//! The strategy is *agnostic* in Boost — it accepts any point-to-segment
//! distance kernel, so the same recursion works in Cartesian, spherical,
//! and geographic coordinate systems. This port ships the Cartesian
//! flavour first, keyed on
//! [`crate::PointToSegment`]`<`[`crate::Pythagoras`]`>`.

use alloc::vec;
use alloc::vec::Vec;

use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::SameAs;
use geometry_trait::{Linestring, Point, PointMut};

use crate::cartesian::{PointToSegment, Pythagoras};
use crate::distance::DistanceStrategy;

/// A strategy for simplifying a geometry.
///
/// Mirrors the per-coordinate-system simplify-strategy concept from
/// `boost/geometry/strategies/simplify.hpp`. `simplify` returns a *new*
/// geometry of the same kind.
pub trait SimplifyStrategy<G> {
    /// The simplified geometry type.
    type Output;

    /// Return a simplified copy of `g`, dropping vertices whose
    /// deviation from the retained chord is below `max_distance`.
    fn simplify(&self, g: &G, max_distance: f64) -> Self::Output;
}

/// Douglas–Peucker line simplification.
///
/// Parameterised on a point-to-segment distance strategy. The default
/// [`crate::PointToSegment`]`<`[`crate::Pythagoras`]`>`
/// matches Boost's `strategies/agnostic/simplify_douglas_peucker.hpp`
/// default.
#[derive(Debug, Default, Clone, Copy)]
pub struct DouglasPeucker<D = PointToSegment<Pythagoras>>(pub D);

/// Visvalingam–Whyatt area-ranked line simplification.
///
/// Repeatedly removes the interior vertex with the smallest adjacent-triangle
/// area while that area is at most the tolerance supplied to
/// [`SimplifyStrategy::simplify`]. Endpoints are always retained.
///
/// Implements the method from Visvalingam and Whyatt, “Line Generalisation by
/// Repeated Elimination of Points” (1993). Boost.Geometry has no equivalent
/// strategy; this is an opt-in peer of [`DouglasPeucker`].
#[derive(Debug, Default, Clone, Copy)]
pub struct VisvalingamWhyatt;

/// Topology-preserving Visvalingam–Whyatt line simplification.
///
/// Uses the same area ranking as [`VisvalingamWhyatt`], and applies the Davies
/// refinement when removing a vertex would introduce a self-intersection: the
/// preceding retained vertex is removed next so the transient crossing is
/// eliminated. The implementation uses an allocation-only quadratic scan,
/// keeping the strategy available in `no_std` builds.
#[derive(Debug, Default, Clone, Copy)]
pub struct VisvalingamWhyattPreserve;

impl<P, L, D> SimplifyStrategy<L> for DouglasPeucker<D>
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    L: Linestring<Point = P>,
    D: DistanceStrategy<P, geometry_model::Segment<P>, Out = f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = geometry_model::Linestring<P>;

    fn simplify(&self, ls: &L, max_distance: f64) -> Self::Output {
        let pts: Vec<P> = ls.points().copied().collect();
        // A negative tolerance is a legal input that Boost copies through
        // unchanged (`algorithms/simplify.hpp:294,359`, `|| max_distance
        // < 0`). Guarding it here is also load-bearing for termination:
        // with `eps < 0` and a fully collinear/coincident run, every
        // interior deviation is `0.0`, so `dp_recurse`'s `max_d > eps`
        // guard is satisfied while `max_i` never advances past `lo` —
        // the second recursive call would then be identical to its
        // parent and recurse forever (stack overflow). Returning early
        // keeps every vertex, matching Boost.
        if pts.len() < 3 || max_distance < 0.0 {
            return geometry_model::Linestring::from_vec(pts);
        }

        let mut keep = vec![false; pts.len()];
        keep[0] = true;
        keep[pts.len() - 1] = true;
        dp_recurse(&pts, 0, pts.len() - 1, max_distance, &self.0, &mut keep);

        let out: Vec<P> = pts
            .iter()
            .zip(keep.iter())
            .filter_map(|(p, &k)| if k { Some(*p) } else { None })
            .collect();
        geometry_model::Linestring::from_vec(out)
    }
}

impl<P, L> SimplifyStrategy<L> for VisvalingamWhyatt
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    L: Linestring<Point = P>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = geometry_model::Linestring<P>;

    fn simplify(&self, ls: &L, max_distance: f64) -> Self::Output {
        visvalingam_whyatt(ls, max_distance, false)
    }
}

impl<P, L> SimplifyStrategy<L> for VisvalingamWhyattPreserve
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    L: Linestring<Point = P>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = geometry_model::Linestring<P>;

    fn simplify(&self, ls: &L, max_distance: f64) -> Self::Output {
        visvalingam_whyatt(ls, max_distance, true)
    }
}

fn visvalingam_whyatt<P, L>(
    ls: &L,
    minimum_area: f64,
    preserve_topology: bool,
) -> geometry_model::Linestring<P>
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    L: Linestring<Point = P>,
{
    let points: Vec<P> = ls.points().copied().collect();
    if points.len() < 3 || minimum_area <= 0.0 || minimum_area.is_nan() {
        return geometry_model::Linestring::from_vec(points);
    }

    let mut retained: Vec<usize> = (0..points.len()).collect();
    let mut forced_predecessor = None;

    while retained.len() > 2 {
        let selected = forced_predecessor
            .take()
            .and_then(|index| retained.iter().position(|candidate| *candidate == index))
            .filter(|slot| *slot > 0 && *slot + 1 < retained.len())
            .map(|slot| (slot, 0.0))
            .or_else(|| smallest_triangle(&points, &retained));
        // `retained.len() > 2` guarantees at least one interior triangle, so
        // the fallback scan always produces a candidate even when a forced
        // predecessor is no longer eligible.
        let (slot, area) = selected.expect("an interior triangle is available");
        if area > minimum_area {
            break;
        }

        let creates_crossing =
            preserve_topology && removal_creates_crossing(&points, &retained, slot);
        let predecessor = retained[slot - 1];
        retained.remove(slot);
        if creates_crossing {
            forced_predecessor = Some(predecessor);
        }
    }

    geometry_model::Linestring::from_vec(retained.into_iter().map(|index| points[index]).collect())
}

fn smallest_triangle<P>(points: &[P], retained: &[usize]) -> Option<(usize, f64)>
where
    P: Point<Scalar = f64>,
{
    let mut selected = None;
    for slot in 1..retained.len().saturating_sub(1) {
        let area = triangle_area(
            &points[retained[slot - 1]],
            &points[retained[slot]],
            &points[retained[slot + 1]],
        );
        if selected.is_none_or(|(_, smallest)| area < smallest) {
            selected = Some((slot, area));
        }
    }
    selected
}

#[inline]
fn triangle_area<P>(first: &P, middle: &P, last: &P) -> f64
where
    P: Point<Scalar = f64>,
{
    let twice_area = (middle.get::<0>() - first.get::<0>()) * (last.get::<1>() - first.get::<1>())
        - (middle.get::<1>() - first.get::<1>()) * (last.get::<0>() - first.get::<0>());
    twice_area.abs() * 0.5
}

fn removal_creates_crossing<P>(points: &[P], retained: &[usize], slot: usize) -> bool
where
    P: Point<Scalar = f64>,
{
    let left = retained[slot - 1];
    let current = retained[slot];
    let right = retained[slot + 1];

    retained.windows(2).any(|edge| {
        let start = edge[0];
        let end = edge[1];
        if start == left
            || end == left
            || start == current
            || end == current
            || start == right
            || end == right
        {
            return false;
        }
        segments_intersect(&points[left], &points[right], &points[start], &points[end])
    })
}

fn segments_intersect<P>(first: &P, second: &P, third: &P, fourth: &P) -> bool
where
    P: Point<Scalar = f64>,
{
    let o1 = orientation(first, second, third);
    let o2 = orientation(first, second, fourth);
    let o3 = orientation(third, fourth, first);
    let o4 = orientation(third, fourth, second);

    if o1 != o2 && o3 != o4 && o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0 {
        return true;
    }
    (o1 == 0 && point_on_segment(third, first, second))
        || (o2 == 0 && point_on_segment(fourth, first, second))
        || (o3 == 0 && point_on_segment(first, third, fourth))
        || (o4 == 0 && point_on_segment(second, third, fourth))
}

#[inline]
fn orientation<P>(first: &P, second: &P, third: &P) -> i8
where
    P: Point<Scalar = f64>,
{
    let cross = (second.get::<0>() - first.get::<0>()) * (third.get::<1>() - first.get::<1>())
        - (second.get::<1>() - first.get::<1>()) * (third.get::<0>() - first.get::<0>());
    if cross > 0.0 {
        1
    } else if cross < 0.0 {
        -1
    } else {
        0
    }
}

#[inline]
fn point_on_segment<P>(point: &P, first: &P, second: &P) -> bool
where
    P: Point<Scalar = f64>,
{
    let x = point.get::<0>();
    let y = point.get::<1>();
    first.get::<0>().min(second.get::<0>()) <= x
        && x <= first.get::<0>().max(second.get::<0>())
        && first.get::<1>().min(second.get::<1>()) <= y
        && y <= first.get::<1>().max(second.get::<1>())
}

/// Recursive Douglas–Peucker split over `pts[lo..=hi]`.
///
/// Marks the vertex furthest from the chord `pts[lo]`-`pts[hi]` as kept
/// when its distance exceeds `eps`, then recurses on the two halves.
/// Mirrors the `douglas_peucker::apply` recursion in
/// `boost/geometry/strategies/agnostic/simplify_douglas_peucker.hpp`.
fn dp_recurse<P, D>(pts: &[P], lo: usize, hi: usize, eps: f64, dist: &D, keep: &mut [bool])
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    D: DistanceStrategy<P, geometry_model::Segment<P>, Out = f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    if hi <= lo + 1 {
        return;
    }
    let seg = geometry_model::Segment::new(pts[lo], pts[hi]);

    // Find the interior vertex furthest from the chord.
    let mut max_d = 0.0_f64;
    let mut max_i = lo;
    for (i, p) in pts.iter().enumerate().take(hi).skip(lo + 1) {
        let d = dist.distance(p, &seg);
        if d > max_d {
            max_d = d;
            max_i = i;
        }
    }

    // If the furthest deviation exceeds the tolerance, keep that vertex
    // and recurse on the two halves; otherwise all interior vertices
    // drop.
    if max_d > eps {
        keep[max_i] = true;
        dp_recurse(pts, lo, max_i, eps, dist, keep);
        dp_recurse(pts, max_i, hi, eps, dist, keep);
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Simplified coordinates are exact literals."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/simplify.cpp` — a straight
    //! polyline collapses to its endpoints, and a spike above tolerance
    //! survives while a sub-tolerance wiggle drops.

    extern crate alloc;

    use super::{
        DouglasPeucker, SimplifyStrategy, orientation, point_on_segment, segments_intersect,
    };
    use crate::cartesian::{PointToSegment, Pythagoras};
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};
    use geometry_trait::Point as _;

    type Pt = Point2D<f64, Cartesian>;

    fn default_dp() -> DouglasPeucker<PointToSegment<Pythagoras>> {
        DouglasPeucker::default()
    }

    fn coords(ls: &Linestring<Pt>) -> Vec<(f64, f64)> {
        ls.0.iter().map(|p| (p.get::<0>(), p.get::<1>())).collect()
    }

    #[test]
    fn straight_three_point_line_collapses_to_endpoints() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (2., 0.)];
        let s = default_dp().simplify(&ls, 0.5);
        assert_eq!(coords(&s), vec![(0., 0.), (2., 0.)]);
    }

    #[test]
    fn collinear_polyline_collapses_to_endpoints() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 1.), (2., 2.), (3., 3.)];
        let s = default_dp().simplify(&ls, 0.001);
        assert_eq!(coords(&s), vec![(0., 0.), (3., 3.)]);
    }

    #[test]
    fn tiny_deviation_drops_but_large_deviation_kept() {
        // A near-straight run with one vertex bumped far off the chord.
        // Small tolerance drops the tiny wiggle but keeps the spike.
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.01), (2., 0.), (3., 5.), (4., 0.)];
        let s = default_dp().simplify(&ls, 0.5);
        let c = coords(&s);
        assert_eq!(c.first().unwrap(), &(0., 0.));
        assert_eq!(c.last().unwrap(), &(4., 0.));
        // The (3, 5) spike survives; the (1, 0.01) wiggle does not.
        assert!(c.contains(&(3., 5.)));
        assert!(!c.contains(&(1., 0.01)));
    }

    #[test]
    fn short_linestring_is_returned_unchanged() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 1.)];
        let s = default_dp().simplify(&ls, 10.0);
        assert_eq!(coords(&s), vec![(0., 0.), (1., 1.)]);
    }

    #[test]
    fn negative_tolerance_copies_through_without_overflow() {
        // Regression: a negative tolerance over a fully collinear run
        // used to recurse forever (max_d == 0.0 > eps, but the split
        // index never advanced) and abort the process with a stack
        // overflow. Boost copies the range through unchanged for
        // `max_distance < 0` (simplify.hpp:294,359); so must the port.
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (2., 0.)];
        let s = default_dp().simplify(&ls, -0.5);
        assert_eq!(coords(&s), vec![(0., 0.), (1., 0.), (2., 0.)]);
        // Coincident interior points, also negative tolerance.
        let dup: Linestring<Pt> = linestring![(0., 0.), (0., 0.), (0., 0.), (5., 0.)];
        let s2 = default_dp().simplify(&dup, -1.0);
        assert_eq!(s2.0.len(), 4, "negative tolerance keeps every vertex");
    }

    #[test]
    fn private_collinear_intersection_guards_cover_every_endpoint_order() {
        let a = Pt::new(0.0, 0.0);
        let b = Pt::new(2.0, 0.0);
        assert!(segments_intersect(
            &a,
            &b,
            &Pt::new(1.0, 0.0),
            &Pt::new(1.0, 1.0)
        ));
        assert!(segments_intersect(
            &a,
            &b,
            &Pt::new(1.0, 1.0),
            &Pt::new(1.0, 0.0)
        ));
        assert!(segments_intersect(
            &Pt::new(1.0, 0.0),
            &Pt::new(1.0, 1.0),
            &a,
            &b,
        ));
        assert!(segments_intersect(
            &Pt::new(1.0, 1.0),
            &Pt::new(1.0, 0.0),
            &a,
            &b,
        ));
        assert_eq!(orientation(&a, &b, &Pt::new(1.0, 1.0)), 1);
        assert_eq!(orientation(&a, &b, &Pt::new(1.0, -1.0)), -1);
        assert_eq!(orientation(&a, &b, &Pt::new(1.0, 0.0)), 0);
        assert!(point_on_segment(&Pt::new(1.0, 0.0), &a, &b));
    }
}
