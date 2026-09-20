//! Convex hull strategies.
//!
//! Mirrors `boost::geometry::strategy::convex_hull::*` from
//! `boost/geometry/strategies/agnostic/hull_graham_andrew.hpp`. The Rust
//! port ships Andrew's monotone chain ([`MonotoneChain`]) as the
//! default — same `O(n log n)` complexity as Graham-Andrew, cleaner to
//! translate.
//!
//! The hull is emitted as a *clockwise, closed* [`Ring`] (first point
//! repeated at the end), matching Boost's default output ring
//! orientation from `boost/geometry/algorithms/convex_hull.hpp`.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{Linestring, MultiPoint, Polygon, Ring};
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait};

/// A strategy for computing the convex hull of a geometry.
///
/// Mirrors the per-coordinate-system convex-hull-strategy concept from
/// `boost/geometry/strategies/convex_hull.hpp`.
pub trait ConvexHullStrategy<G> {
    /// The point type of the output ring.
    type OutputPoint: Point;
    /// The output ring type — [`Ring`]`<OutputPoint, true, true>`
    /// (clockwise, closed) per Boost's default in
    /// `boost/geometry/algorithms/convex_hull.hpp`.
    type Output;

    /// Compute the convex hull of `g`.
    fn convex_hull(&self, g: &G) -> Self::Output;
}

/// Andrew's monotone chain. Cartesian-only — the orientation predicate
/// `((b − a) × (c − b))` is a 2-D Cartesian cross product.
///
/// This implementation drops collinear boundary points (`<= 0` in the
/// cross-product check), so the hull carries only its true corners.
/// Boost's `hull_graham_andrew.hpp` uses a strict `< 0` and keeps
/// collinear points on the boundary; the port trades that for a minimal
/// vertex set.
#[derive(Debug, Default, Clone, Copy)]
pub struct MonotoneChain;

/// Gather every point of `g` into `out`. Recursive over polygon rings
/// and multi-geometries; reused by every [`MonotoneChain`] impl below.
#[doc(hidden)]
pub trait CollectPoints {
    /// The point type gathered.
    type Point: Point + Copy;
    /// Push every point of `self` onto `out`.
    fn collect_points(&self, out: &mut Vec<Self::Point>);
}

impl<P: Point + Copy> CollectPoints for MultiPoint<P> {
    type Point = P;
    fn collect_points(&self, out: &mut Vec<P>) {
        out.extend(self.0.iter().copied());
    }
}

impl<P: Point + Copy> CollectPoints for Linestring<P> {
    type Point = P;
    fn collect_points(&self, out: &mut Vec<P>) {
        out.extend(self.0.iter().copied());
    }
}

impl<P: Point + Copy, const CW: bool, const CL: bool> CollectPoints for Ring<P, CW, CL> {
    type Point = P;
    fn collect_points(&self, out: &mut Vec<P>) {
        out.extend(self.0.iter().copied());
    }
}

impl<P: Point + Copy, const CW: bool, const CL: bool> CollectPoints for Polygon<P, CW, CL> {
    type Point = P;
    fn collect_points(&self, out: &mut Vec<P>) {
        // Boost feeds only the outer-ring points into the hull kernel;
        // a hull that included interior-ring points is a different
        // algorithm (`hull_graham_andrew.hpp`).
        out.extend(self.exterior().points().copied());
    }
}

/// 2-D cross product of `(b − a)` and `(c − b)`. Positive = CCW turn,
/// negative = CW turn, zero = collinear.
fn cross_2d<P: Point>(a: &P, b: &P, c: &P) -> P::Scalar {
    let ux = b.get::<0>() - a.get::<0>();
    let uy = b.get::<1>() - a.get::<1>();
    let vx = c.get::<0>() - b.get::<0>();
    let vy = c.get::<1>() - b.get::<1>();
    ux * vy - uy * vx
}

/// Andrew's monotone chain over `pts`. Returns the hull vertices in
/// counter-clockwise order without the closing duplicate.
fn monotone_chain<P>(mut pts: Vec<P>) -> Vec<P>
where
    P: Point + Copy,
{
    // 1. Sort by (x, then y) ascending.
    pts.sort_by(|a, b| {
        a.get::<0>()
            .partial_cmp(&b.get::<0>())
            .unwrap_or(core::cmp::Ordering::Equal)
            .then_with(|| {
                a.get::<1>()
                    .partial_cmp(&b.get::<1>())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
    });

    // 2/3. Lower hull, walking left to right.
    let mut h: Vec<P> = Vec::with_capacity(pts.len() * 2);
    for &p in &pts {
        while h.len() >= 2 && cross_2d(&h[h.len() - 2], &h[h.len() - 1], &p) <= P::Scalar::ZERO {
            h.pop();
        }
        h.push(p);
    }

    // 4. Upper hull, walking right to left.
    let lower_len = h.len() + 1;
    for &p in pts.iter().rev().skip(1) {
        while h.len() >= lower_len
            && cross_2d(&h[h.len() - 2], &h[h.len() - 1], &p) <= P::Scalar::ZERO
        {
            h.pop();
        }
        h.push(p);
    }

    // The last point equals the first — drop the duplicate.
    h.pop();
    h
}

impl<G, P> ConvexHullStrategy<G> for MonotoneChain
where
    G: CollectPoints<Point = P>,
    P: Point + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type OutputPoint = P;
    type Output = Ring<P, true, true>;

    fn convex_hull(&self, g: &G) -> Self::Output {
        let mut pts = Vec::new();
        g.collect_points(&mut pts);
        if pts.len() < 3 {
            // Degenerate: a 0-, 1-, or 2-point hull is the input itself,
            // closed onto its first vertex like every other hull so the
            // closed-declared output ring keeps its invariant.
            let mut ring = pts;
            if let Some(&first) = ring.first() {
                ring.push(first);
            }
            return Ring::from_vec(ring);
        }

        // `monotone_chain` returns a CCW hull; reverse for the
        // clockwise output ring, then repeat the first point to close.
        let mut hull = monotone_chain(pts);
        hull.reverse();
        let first = hull[0];
        hull.push(first);
        Ring::from_vec(hull)
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Hull corner coordinates are exact literals."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/convex_hull.cpp` — the hull of a
    //! square-plus-interior point keeps only the four corners.

    use super::{ConvexHullStrategy, MonotoneChain};
    use geometry_cs::Cartesian;
    use geometry_model::{MultiPoint, Point2D};
    use geometry_trait::{Point as _, Ring as _};

    type Pt = Point2D<f64, Cartesian>;

    fn multi_point(points: &[(f64, f64)]) -> MultiPoint<Pt> {
        let mut mp = MultiPoint::new();
        for &(x, y) in points {
            mp.push(Pt::new(x, y));
        }
        mp
    }

    #[test]
    fn square_plus_interior_point_keeps_four_corners() {
        let mp = multi_point(&[(0., 0.), (4., 0.), (4., 4.), (0., 4.), (2., 2.)]);
        let hull = MonotoneChain.convex_hull(&mp);
        // 4 distinct corners + closing duplicate = 5 stored.
        assert_eq!(hull.points().count(), 5);
        assert_eq!(
            hull.0.first().unwrap().get::<0>(),
            hull.0.last().unwrap().get::<0>()
        );
        assert_eq!(
            hull.0.first().unwrap().get::<1>(),
            hull.0.last().unwrap().get::<1>()
        );
        // The interior (2, 2) point is not on the hull boundary.
        let has_interior = hull
            .0
            .iter()
            .any(|p| p.get::<0>() == 2.0 && p.get::<1>() == 2.0);
        assert!(!has_interior);
    }

    #[test]
    fn hull_of_two_points_is_the_input_closed() {
        let mp = multi_point(&[(0., 0.), (1., 1.)]);
        let hull = MonotoneChain.convex_hull(&mp);
        let pts: alloc::vec::Vec<(f64, f64)> = hull
            .0
            .iter()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        assert_eq!(pts, alloc::vec![(0., 0.), (1., 1.), (0., 0.)]);
    }

    /// The hull is emitted clockwise and closed, carrying only the
    /// corners: positive signed area under Boost's clockwise-positive
    /// convention and a right turn at every vertex.
    #[test]
    fn hull_is_wound_clockwise_and_carries_only_the_corners() {
        use crate::area::{AreaStrategy, ShoelaceArea};
        let mp = multi_point(&[(0., 0.), (4., 0.), (4., 4.), (0., 4.), (2., 2.), (1., 3.)]);
        let hull = MonotoneChain.convex_hull(&mp);
        let signed = ShoelaceArea.area(&hull);
        assert!(
            signed > 0.0,
            "declared-CW ring must have positive signed area, got {signed}"
        );
        assert!((signed - 16.0).abs() < 1e-12);
        let pts: alloc::vec::Vec<(f64, f64)> = hull
            .0
            .iter()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        assert_eq!(pts.len(), 5);
        assert_eq!(pts[0], pts[4]);
        for w in pts.windows(3) {
            let cross =
                (w[1].0 - w[0].0) * (w[2].1 - w[1].1) - (w[1].1 - w[0].1) * (w[2].0 - w[1].0);
            assert!(cross < 0.0, "left turn in a clockwise hull: {pts:?}");
        }
    }
}
