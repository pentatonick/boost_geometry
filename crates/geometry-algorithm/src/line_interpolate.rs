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
}
