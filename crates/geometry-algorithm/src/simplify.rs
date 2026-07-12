//! `simplify(g, max_distance)` — return a Douglas–Peucker-simplified
//! copy of `g`.
//!
//! Mirrors `boost::geometry::simplify(g, max_distance)` from
//! `boost/geometry/algorithms/simplify.hpp`. The Cartesian default is
//! [`geometry_strategy::DouglasPeucker`]`<`[`geometry_strategy::PointToSegment`]`<`[`geometry_strategy::Pythagoras`]`>>`,
//! matching Boost's default from
//! `boost/geometry/strategies/agnostic/simplify_douglas_peucker.hpp`.
//!
//! v1 ships linestring simplify only. Ring / polygon simplify (which
//! must preserve ring closure) lands with the areal-simplify task.

use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_strategy::{DouglasPeucker, PointToSegment, Pythagoras, SimplifyStrategy};
use geometry_tag::SameAs;
use geometry_trait::{Linestring, Point, PointMut};

/// Return a Douglas–Peucker-simplified copy of the linestring `g`.
///
/// Vertices whose perpendicular distance to the chord between their
/// retained neighbours is below `max_distance` are dropped; the first
/// and last vertices are always kept.
///
/// Mirrors `boost::geometry::simplify(linestring, max_distance)` from
/// `boost/geometry/algorithms/simplify.hpp`.
#[must_use]
pub fn simplify<G, P>(g: &G, max_distance: f64) -> geometry_model::Linestring<P>
where
    G: Linestring<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let strategy = DouglasPeucker::<PointToSegment<Pythagoras>>::default();
    strategy.simplify(g, max_distance)
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Simplified coordinates are exact literals."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/simplify.cpp` — a straight
    //! polyline collapses to its endpoints and a wiggle below tolerance
    //! is dropped while a spike above tolerance is kept.

    use super::simplify;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};
    use geometry_trait::Point as _;

    type Pt = Point2D<f64, Cartesian>;

    fn coords(ls: &Linestring<Pt>) -> alloc::vec::Vec<(f64, f64)> {
        ls.0.iter().map(|p| (p.get::<0>(), p.get::<1>())).collect()
    }

    #[test]
    fn straight_three_point_line_collapses() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (2., 0.)];
        let s = simplify(&ls, 0.5);
        assert_eq!(coords(&s), alloc::vec![(0., 0.), (2., 0.)]);
    }

    #[test]
    fn zigzag_with_tiny_deviation_collapses() {
        // Every interior vertex sits within 0.05 of the (0,0)-(4,0)
        // chord, so a 0.5 tolerance drops them all.
        let ls: Linestring<Pt> =
            linestring![(0., 0.), (1., 0.05), (2., -0.05), (3., 0.05), (4., 0.)];
        let s = simplify(&ls, 0.5);
        assert_eq!(coords(&s), alloc::vec![(0., 0.), (4., 0.)]);
    }

    #[test]
    fn spike_above_tolerance_survives() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.01), (2., 5.), (3., 0.01), (4., 0.)];
        let s = simplify(&ls, 0.5);
        let c = coords(&s);
        assert_eq!(c.first().unwrap(), &(0., 0.));
        assert_eq!(c.last().unwrap(), &(4., 0.));
        assert!(c.contains(&(2., 5.)));
    }
}
