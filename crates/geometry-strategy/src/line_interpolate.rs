//! `LineInterpolateStrategy<L>` — point at fractional arc-length `t`.
//!
//! Mirrors `boost::geometry::strategy::line_interpolate::cartesian`
//! from `boost/geometry/strategies/line_interpolate/cartesian.hpp`.

use alloc::vec::Vec;

use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::SameAs;
use geometry_trait::{Linestring, Point, PointMut};

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

        // Walk segments accumulating length until we pass `target`.
        let mut acc = 0.0_f64;
        for w in pts.windows(2) {
            let d = Pythagoras.distance(w[0], w[1]);
            let next = acc + d;
            if next >= target {
                let frac = if d > 0.0 { (target - acc) / d } else { 0.0 };
                return blend(w[0], w[1], frac);
            }
            acc = next;
        }
        *pts[pts.len() - 1]
    }
}

/// Linear per-dimension blend: `out[D] = a[D] + t·(b[D] − a[D])` for
/// each dimension `D ∈ 0..P::DIM`.
///
/// Mirrors the per-coordinate interpolation inside
/// `line_interpolate/cartesian.hpp::apply`.
#[inline]
fn blend<P>(a: &P, b: &P, t: f64) -> P
where
    P: Point<Scalar = f64> + PointMut + Default,
{
    let mut out = P::default();
    geometry_trait::fold_dims((), a, |(), _p, d| {
        let av = match d {
            0 => a.get::<0>(),
            1 => a.get::<1>(),
            2 => a.get::<2>(),
            3 => a.get::<3>(),
            _ => unreachable!(),
        };
        let bv = match d {
            0 => b.get::<0>(),
            1 => b.get::<1>(),
            2 => b.get::<2>(),
            3 => b.get::<3>(),
            _ => unreachable!(),
        };
        let v = av + t * (bv - av);
        match d {
            0 => out.set::<0>(v),
            1 => out.set::<1>(v),
            2 => out.set::<2>(v),
            3 => out.set::<3>(v),
            _ => unreachable!(),
        }
    });
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

    #[test]
    fn t_inside_second_segment() {
        // total length 2 + 3 = 5; t=0.6 lands at arc 3.0 → 1.0 into the
        // second segment → (2, 1).
        let ls: Linestring<Pt> = linestring![(0., 0.), (2., 0.), (2., 3.)];
        let p = CartesianLineInterpolate.interpolate(&ls, 0.6);
        assert!(close(p, 2., 1.));
    }

}
