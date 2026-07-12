//! Densification strategies.
//!
//! Mirrors `boost::geometry::strategy::densify::*` from
//! `boost/geometry/strategies/densify/cartesian.hpp`. The Cartesian
//! impl walks each segment and inserts evenly-spaced intermediate
//! points whenever the segment length exceeds `max_distance`.
//!
//! Spherical / geographic densify (interpolate along great-circle
//! arcs / geodesics) lands later via the matching CS variants of
//! [`crate::Pythagoras`] and the corresponding `transform` strategies
//! — out of scope here.

use alloc::vec::Vec;

use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::Linestring;
use geometry_tag::SameAs;
use geometry_trait::{Linestring as LinestringTrait, Point, PointMut};

use crate::cartesian::Pythagoras;
use crate::distance::DistanceStrategy;

/// A strategy for densifying a geometry — inserting intermediate
/// vertices so no segment exceeds `max_distance`.
///
/// Mirrors the per-coordinate-system densify-strategy concept from
/// `boost/geometry/strategies/densify.hpp`. `densify` returns a *new*
/// geometry of the same kind.
pub trait DensifyStrategy<G> {
    /// The densified geometry type.
    type Output;

    /// Return a densified copy of `g` where every output segment is
    /// no longer than `max_distance`.
    fn densify(&self, g: &G, max_distance: f64) -> Self::Output;
}

/// Cartesian densify — straight-line interpolation between
/// consecutive points.
///
/// Mirrors `boost::geometry::strategy::densify::cartesian` from
/// `boost/geometry/strategies/densify/cartesian.hpp`.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianDensify;

impl<L, P> DensifyStrategy<L> for CartesianDensify
where
    L: LinestringTrait<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    Pythagoras: DistanceStrategy<P, P, Out = f64>,
{
    type Output = Linestring<P>;

    fn densify(&self, ls: &L, max_distance: f64) -> Self::Output {
        let pts: Vec<P> = ls.points().copied().collect();
        // A non-positive (or NaN) threshold cannot subdivide anything:
        // Boost's algorithm layer rejects `max_distance <= 0` with
        // `invalid_input_exception` before the strategy ever runs
        // (`algorithms/densify.hpp`), and the port's algorithm-layer
        // `densify` panics likewise. Guarding here as well keeps a
        // direct strategy call from computing `d_total / 0.0 == inf`,
        // whose saturating cast yields `n == usize::MAX` — a debug
        // overflow panic at `n + 1` and an unbounded push loop in
        // release. Copy-through mirrors the negative-tolerance stance
        // of `DouglasPeucker::simplify` (`simplify.rs:70`).
        #[allow(
            clippy::neg_cmp_op_on_partial_ord,
            reason = "NaN must take the guard branch"
        )]
        if !(max_distance > 0.0) {
            return Linestring::from_vec(pts);
        }
        let mut out: Vec<P> = Vec::with_capacity(pts.len() * 2);
        if pts.is_empty() {
            return Linestring::from_vec(out);
        }

        for w in pts.windows(2) {
            out.push(w[0]);
            let d_total = Pythagoras.distance(&w[0], &w[1]);
            // Number of *intermediate* points inserted on this edge.
            // Mirrors `densify/cartesian.hpp::apply` exactly:
            // `n = int(len / threshold)` (a truncation = floor for the
            // positive `len`), then the edge is divided into `n + 1`
            // equal sub-segments with the intermediate points placed at
            // `i / (n + 1)` for `i in 1..=n`. Using `ceil` here would
            // diverge from Boost on exact-integer ratios (e.g.
            // `len == 2·max` would emit one fewer point and leave a
            // sub-segment exactly equal to `max` instead of shorter).
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss,
                reason = "d_total / max_distance >= 0 keeps the floor non-negative and small; \
                          the fraction cast loses no meaningful precision for realistic counts."
            )]
            {
                let n = (d_total / max_distance) as usize;
                if n > 0 {
                    let den = (n + 1) as f64;
                    for i in 1..=n {
                        let t = i as f64 / den;
                        out.push(interpolate(&w[0], &w[1], t));
                    }
                }
            }
        }
        out.push(*pts.last().unwrap());
        Linestring::from_vec(out)
    }
}

/// Linear per-dimension interpolation: `out[D] = a[D] + t·(b[D] − a[D])`
/// for each dimension `D ∈ 0..P::DIM`.
///
/// Mirrors the point-blend inside `densify/cartesian.hpp::apply` that
/// walks each coordinate of the two endpoints.
#[inline]
fn interpolate<P>(a: &P, b: &P, t: f64) -> P
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
    reason = "Densified coordinates are exact literals."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/densify.cpp:42-65`: a segment
    //! is cut into equal sub-segments none of which exceeds
    //! `max_distance`, and total length is preserved.

    use super::{CartesianDensify, DensifyStrategy};
    use crate::cartesian::Pythagoras;
    use crate::distance::DistanceStrategy;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};
    use geometry_trait::{Linestring as _, Point as _};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn segment_of_length_10_max_2_5_yields_6_points() {
        // Boost: `n = int(10 / 2.5) = 4` intermediate points, dividing
        // the edge into `n + 1 = 5` sub-segments of length 2 each
        // (strictly below `max = 2.5`). Points at i/5 for i in 1..=4.
        // A `ceil`-based count would wrongly yield only 5 points with
        // sub-segments of exactly 2.5.
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        let out = CartesianDensify.densify(&ls, 2.5);
        let xs: alloc::vec::Vec<f64> = out.points().map(Pt::get::<0>).collect();
        assert_eq!(xs, alloc::vec![0.0, 2.0, 4.0, 6.0, 8.0, 10.0]);
    }

    #[test]
    fn exact_integer_ratio_matches_boost_denominator() {
        // Regression: `len == 3·max` (an exact-integer ratio) is the
        // case where `ceil` and Boost's `floor`+1 diverge. Boost:
        // `n = int(6 / 2) = 3` → 4 sub-segments, points at 1.5/3/4.5.
        let ls: Linestring<Pt> = linestring![(0., 0.), (6., 0.)];
        let out = CartesianDensify.densify(&ls, 2.0);
        let xs: alloc::vec::Vec<f64> = out.points().map(Pt::get::<0>).collect();
        assert_eq!(xs, alloc::vec![0.0, 1.5, 3.0, 4.5, 6.0]);
    }

    #[test]
    fn no_output_segment_exceeds_max_distance() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.), (10., 7.)];
        let out = CartesianDensify.densify(&ls, 1.0);
        let pts: alloc::vec::Vec<&Pt> = out.points().collect();
        for w in pts.windows(2) {
            assert!(Pythagoras.distance(w[0], w[1]) <= 1.0 + 1e-9);
        }
    }

    #[test]
    fn non_positive_max_distance_copies_through_without_hanging() {
        // Regression: `max_distance == 0.0` used to drive
        // `d_total / 0.0 == inf`, whose saturating cast made
        // `n == usize::MAX` — a debug overflow panic at `n + 1` and an
        // unbounded release-mode push loop. The strategy now copies the
        // input through unchanged for zero / negative / NaN thresholds.
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        for bad in [0.0, -1.0, f64::NAN] {
            let out = CartesianDensify.densify(&ls, bad);
            let xs: alloc::vec::Vec<f64> = out.points().map(Pt::get::<0>).collect();
            assert_eq!(xs, alloc::vec![0.0, 10.0], "max_distance = {bad}");
        }
    }
}
