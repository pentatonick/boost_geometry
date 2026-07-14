//! `line_interpolate(ls, t) -> Point` — point at fractional
//! arc-length `t ∈ [0, 1]` along `ls`.
//!
//! Mirrors `boost::geometry::line_interpolate(ls, length, out)` from
//! `boost/geometry/algorithms/line_interpolate.hpp`. Boost takes an
//! *absolute* `length`; the Rust port takes a *fraction* — easier to
//! reason about and matching the common GIS API
//! (`PostGIS::ST_LineInterpolatePoint`, Shapely's
//! `interpolate(normalized=True)`). Values of `t` outside `[0, 1]`
//! clamp to the endpoints, matching Boost's silent-clamp behaviour.

use geometry_strategy::{CartesianLineInterpolate, LineInterpolateStrategy};
use geometry_trait::Linestring;

/// Return the point at fractional arc-length `t ∈ [0, 1]` along `ls`,
/// measured by accumulated Pythagorean segment length.
///
/// Mirrors `boost::geometry::line_interpolate` from
/// `boost/geometry/algorithms/line_interpolate.hpp`. `t = 0` returns
/// the first point, `t = 1` the last; both short-circuit without
/// walking the whole linestring.
#[inline]
#[must_use]
pub fn line_interpolate<L>(ls: &L, t: f64) -> L::Point
where
    L: Linestring,
    CartesianLineInterpolate: LineInterpolateStrategy<L>,
{
    CartesianLineInterpolate.interpolate(ls, t)
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Interpolated coordinates are exact literals."
)]
mod tests {
    //! Reference from
    //! `boost/geometry/test/algorithms/line_interpolate.cpp:30-75`.

    use super::line_interpolate;
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
        assert!(close(line_interpolate(&ls, 0.0), 0., 0.));
    }

    #[test]
    fn t_one_returns_last_point() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        assert!(close(line_interpolate(&ls, 1.0), 10., 0.));
    }

    #[test]
    fn t_half_returns_midpoint() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        assert!(close(line_interpolate(&ls, 0.5), 5., 0.));
    }

    #[test]
    fn t_at_segment_boundary_returns_vertex() {
        // total length 2 + 3 = 5; t=0.4 lands at arc-length 2.0 →
        // exactly the joining vertex (2, 0).
        let ls: Linestring<Pt> = linestring![(0., 0.), (2., 0.), (2., 3.)];
        assert!(close(line_interpolate(&ls, 0.4), 2., 0.));
    }

    #[test]
    fn t_inside_second_segment() {
        // t=0.6 lands at arc 3.0 → 1.0 into the second segment → (2, 1).
        let ls: Linestring<Pt> = linestring![(0., 0.), (2., 0.), (2., 3.)];
        assert!(close(line_interpolate(&ls, 0.6), 2., 1.));
    }

    /// An empty linestring returns the default point (the degenerate
    /// guard); a single-point linestring returns that point.
    #[test]
    fn degenerate_inputs() {
        let empty: Linestring<Pt> = linestring![];
        assert!(close(line_interpolate(&empty, 0.5), 0., 0.));
        let single: Linestring<Pt> = linestring![(3., 4.)];
        assert!(close(line_interpolate(&single, 0.5), 3., 4.));
    }

    /// A 3D linestring blends the third ordinate too (the `2 =>` arms
    /// of the strategy's per-dimension blend).
    #[test]
    #[allow(clippy::float_cmp, reason = "midpoint ordinates are exact literals")]
    fn three_d_midpoint_blends_z() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let ls: Linestring<P3> =
            Linestring::from_vec(alloc::vec![P3::new(0., 0., 0.), P3::new(10., 0., 4.)]);
        let p = line_interpolate(&ls, 0.5);
        assert_eq!(p.get::<0>(), 5.0);
        assert_eq!(p.get::<2>(), 2.0);
    }

    /// A 4D linestring blends the fourth ordinate (the `3 =>` arms).
    #[test]
    #[allow(clippy::float_cmp, reason = "midpoint ordinates are exact literals")]
    fn four_d_midpoint_blends_all_ordinates() {
        use geometry_model::Point;
        use geometry_trait::PointMut as _;
        type P4 = Point<f64, 4, Cartesian>;
        let mut a = P4::default();
        a.set::<3>(8.0);
        let mut b = P4::default();
        b.set::<0>(10.0);
        let ls: Linestring<P4> = Linestring::from_vec(alloc::vec![a, b]);
        let p = line_interpolate(&ls, 0.5);
        assert_eq!(p.get::<0>(), 5.0);
        assert_eq!(p.get::<3>(), 4.0);
    }
}
