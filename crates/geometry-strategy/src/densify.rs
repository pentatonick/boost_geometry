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

#[cfg(not(feature = "std"))]
use geometry_coords::math::Float;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::Linestring;
use geometry_tag::SameAs;
use geometry_trait::{Linestring as LinestringTrait, Point, PointMut, ordinate, set_ordinate};

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
            densify_segment(&w[0], &w[1], max_distance, &mut out);
        }
        out.push(*pts.last().unwrap());
        Linestring::from_vec(out)
    }
}

/// Push the points that cut `p0 → p1` into equal parts no longer than
/// `length_threshold`, between its two ends.
///
/// C++: `strategy::densify::cartesian::apply`
/// (`strategies/cartesian/densify.hpp`): `n = int(len / threshold)` points
/// — a truncation, the floor of the positive ratio — at
/// `p0 + (p1 − p0)·i / (n + 1)` for `i in 1..=n`, each evaluated in that
/// order so the points are Boost's to the last bit. Truncating rather than
/// rounding up means `len == 2·max` gets one point more than `ceil` would,
/// leaving sub-segments shorter than `max` instead of equal to it.
fn densify_segment<P>(p0: &P, p1: &P, length_threshold: f64, out: &mut Vec<P>)
where
    P: Point<Scalar = f64> + PointMut + Default,
{
    let direction = |dimension| ordinate(p1, dimension) - ordinate(p0, dimension);
    // C++: `dot_product(dir01, dir01)`, which adds the squares from the
    // last dimension down.
    let dot = (0..P::DIM)
        .rev()
        .map(|dimension| direction(dimension) * direction(dimension))
        .reduce(|later, square| square + later)
        .unwrap_or(0.0);
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "C++ truncates the non-negative ratio to an integer the same way"
    )]
    let n = (dot.sqrt() / length_threshold) as usize;
    if n == 0 {
        return;
    }
    // A count no `Vec` can hold — a length that overflowed to infinity
    // saturates the cast — panics here rather than wrapping `n + 1`.
    out.reserve(n);
    #[allow(
        clippy::cast_precision_loss,
        reason = "C++ converts the counts to the calculation type the same way"
    )]
    let den = (n + 1) as f64;
    for i in 1..=n {
        #[allow(
            clippy::cast_precision_loss,
            reason = "C++ converts the counts to the calculation type the same way"
        )]
        let num = i as f64;
        let mut point = P::default();
        for dimension in 0..P::DIM {
            set_ordinate(
                &mut point,
                dimension,
                ordinate(p0, dimension) + direction(dimension) * num / den,
            );
        }
        out.push(point);
    }
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

    /// The points are Boost's to the last bit: `p0 + d·i / (n + 1)`
    /// (`aed7bc3`), where `p0 + (i / (n + 1))·d` rounds `10 / 7` to
    /// `1.4285714285714284`.
    #[test]
    fn densified_points_are_boosts_to_the_last_bit() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        let out = CartesianDensify.densify(&ls, 1.5);
        let xs: alloc::vec::Vec<f64> = out.points().map(Pt::get::<0>).collect();
        assert_eq!(
            xs,
            alloc::vec![
                0.0,
                1.428_571_428_571_428_6,
                2.857_142_857_142_857,
                4.285_714_285_714_286,
                5.714_285_714_285_714,
                7.142_857_142_857_143,
                8.571_428_571_428_571,
                10.0
            ]
        );
    }

    /// A segment whose length overflows to infinity asks for more points
    /// than any `Vec` holds; that panics rather than looping forever.
    #[test]
    #[should_panic(expected = "capacity overflow")]
    fn an_infinitely_long_segment_panics_instead_of_looping() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (f64::MAX, f64::MAX)];
        let _ = CartesianDensify.densify(&ls, 1.0);
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
