//! `LineInterpolateStrategy<L>` — point at fractional arc-length `t`.
//!
//! Mirrors `boost::geometry::strategy::line_interpolate::cartesian`
//! from `boost/geometry/strategies/line_interpolate/cartesian.hpp`.

use alloc::vec::Vec;

use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::SameAs;
use geometry_trait::{Linestring, Point, PointMut, ordinate, set_ordinate};

use crate::cartesian::Pythagoras;
use crate::distance::DistanceStrategy;

/// A strategy for interpolating a point at a fractional arc-length
/// along a linestring.
///
/// Mirrors the per-coordinate-system line-interpolate-strategy concept
/// from `boost/geometry/strategies/line_interpolate.hpp`.
pub trait LineInterpolateStrategy<L: Linestring> {
    /// Walk `ls` and return the point at fractional arc-length `t`
    /// (in `[0, 1]`).
    ///
    /// Mirrors `boost::geometry::strategy::line_interpolate::cartesian::
    /// apply`. `t = 0` returns the first point, `t = 1` the last; `t`
    /// outside `[0, 1]` clamps to the endpoints.
    fn interpolate(&self, ls: &L, t: f64) -> L::Point;
}

/// Cartesian Pythagorean arc-length interpolation.
///
/// Mirrors `boost::geometry::strategy::line_interpolate::cartesian`
/// from `boost/geometry/strategies/line_interpolate/cartesian.hpp`.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianLineInterpolate;

impl<L, P> LineInterpolateStrategy<L> for CartesianLineInterpolate
where
    L: Linestring<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    Pythagoras: DistanceStrategy<P, P, Out = f64>,
{
    fn interpolate(&self, ls: &L, t: f64) -> P {
        let pts: Vec<&P> = ls.points().collect();
        if pts.is_empty() {
            return P::default();
        }
        if pts.len() == 1 || t <= 0.0 {
            return *pts[0];
        }
        if t >= 1.0 {
            return *pts[pts.len() - 1];
        }

        // Total arc length.
        let mut total = 0.0_f64;
        for w in pts.windows(2) {
            total += Pythagoras.distance(w[0], w[1]);
        }

        let target = t * total;
        if target <= 0.0 {
            return *pts[0];
        }

        // C++: `interpolate_point_linear::apply`
        // (`algorithms/line_interpolate.hpp`) walks the distances
        // accumulated so far and takes the fraction over the segment's
        // share of that sum, not over its own length.
        let mut previous_distance = 0.0_f64;
        for w in pts.windows(2) {
            let current_distance = previous_distance + Pythagoras.distance(w[0], w[1]);
            if current_distance >= target {
                let fraction =
                    (target - previous_distance) / (current_distance - previous_distance);
                return blend(w[0], w[1], fraction);
            }
            previous_distance = current_distance;
        }
        *pts[pts.len() - 1]
    }
}

/// The point `fraction` of the way from `p0` to `p1`.
///
/// C++: `strategy::line_interpolate::cartesian::apply`
/// (`strategies/cartesian/line_interpolate.hpp`), which evaluates the
/// convex combination as `p1·fraction + p0·(1 − fraction)`, so a fraction
/// of `1` lands on `p1` exactly.
#[inline]
fn blend<P>(p0: &P, p1: &P, fraction: f64) -> P
where
    P: Point<Scalar = f64> + PointMut + Default,
{
    let one_minus_fraction = 1.0 - fraction;
    let mut out = P::default();
    for dimension in 0..P::DIM {
        set_ordinate(
            &mut out,
            dimension,
            ordinate(p1, dimension) * fraction + ordinate(p0, dimension) * one_minus_fraction,
        );
    }
    out
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Interpolated coordinates are exact literals."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/line_interpolate.cpp:30-75`.

    use super::{CartesianLineInterpolate, LineInterpolateStrategy};
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};
    use geometry_trait::Point as _;

    type Pt = Point2D<f64, Cartesian>;

    fn close(got: Pt, x: f64, y: f64) -> bool {
        (got.get::<0>() - x).abs() < 1e-9 && (got.get::<1>() - y).abs() < 1e-9
    }

    #[test]
    fn t_zero_returns_first_point() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        let p = CartesianLineInterpolate.interpolate(&ls, 0.0);
        assert!(close(p, 0., 0.));
    }

    #[test]
    fn t_one_returns_last_point() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        let p = CartesianLineInterpolate.interpolate(&ls, 1.0);
        assert!(close(p, 10., 0.));
    }

    #[test]
    fn t_half_returns_midpoint() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        let p = CartesianLineInterpolate.interpolate(&ls, 0.5);
        assert!(close(p, 5., 0.));
    }

    /// The point is Boost's to the last bit: `p1·f + p0·(1 − f)` gives
    /// `0.39999999999999997` here (`aed7bc3`), where `p0 + f·(p1 − p0)`
    /// rounds to `0.4`.
    #[test]
    fn interpolated_point_is_boosts_convex_combination() {
        let ls: Linestring<Pt> = linestring![(0.1, 0.), (0.7, 0.)];
        let p = CartesianLineInterpolate.interpolate(&ls, 0.5);
        assert_eq!(
            (p.get::<0>(), p.get::<1>()),
            (0.399_999_999_999_999_97, 0.0)
        );
    }

    #[test]
    fn t_inside_second_segment() {
        // total length 2 + 3 = 5; t=0.6 lands at arc 3.0 → 1.0 into the
        // second segment → (2, 1).
        let ls: Linestring<Pt> = linestring![(0., 0.), (2., 0.), (2., 3.)];
        let p = CartesianLineInterpolate.interpolate(&ls, 0.6);
        assert!(close(p, 2., 1.));
    }
}
