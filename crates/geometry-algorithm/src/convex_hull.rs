//! `convex_hull(&g) -> Ring<G::Point>` — minimal enclosing ring.
//!
//! Mirrors `boost::geometry::convex_hull(g, hull)` from
//! `boost/geometry/algorithms/convex_hull.hpp`. Boost's overload takes
//! the hull through an out-parameter; the Rust port returns it by value.
//! The kernel is Andrew's monotone chain
//! ([`geometry_strategy::MonotoneChain`]), the port's
//! stand-in for Boost's default Graham-Andrew strategy from
//! `boost/geometry/strategies/agnostic/hull_graham_andrew.hpp`.

use geometry_strategy::{ConvexHullStrategy, MonotoneChain};
use geometry_trait::Point;

/// Compute the convex hull of `g` as a clockwise, closed [`Ring`].
///
/// The output ring repeats its first vertex at the end (closed) and is
/// wound clockwise, matching Boost's default output ring from
/// `boost/geometry/algorithms/convex_hull.hpp`.
///
/// [`Ring`]: geometry_model::Ring
#[must_use]
pub fn convex_hull<G, P>(g: &G) -> geometry_model::Ring<P, true, true>
where
    MonotoneChain: ConvexHullStrategy<G, Output = geometry_model::Ring<P, true, true>>,
    P: Point,
{
    MonotoneChain.convex_hull(g)
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Hull corner coordinates are exact literals."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/convex_hull.cpp` — the hull of a
    //! square-plus-interior point keeps only the four corners; the hull
    //! of a concave polygon drops its reflex vertex.

    use super::convex_hull;
    use geometry_cs::Cartesian;
    use geometry_model::{MultiPoint, Point2D, Polygon, polygon};
    use geometry_trait::{Point as _, Ring as _};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn square_plus_interior_point_hull_has_four_corners() {
        let mp = MultiPoint(alloc::vec![
            Pt::new(0., 0.),
            Pt::new(4., 0.),
            Pt::new(4., 4.),
            Pt::new(0., 4.),
            Pt::new(2., 2.),
        ]);
        let hull = convex_hull(&mp);
        // 4 distinct corners + closing duplicate = 5 stored.
        assert_eq!(hull.points().count(), 5);
        // Closed: first == last.
        assert_eq!(
            hull.0.first().unwrap().get::<0>(),
            hull.0.last().unwrap().get::<0>()
        );
        assert_eq!(
            hull.0.first().unwrap().get::<1>(),
            hull.0.last().unwrap().get::<1>()
        );
    }

    #[test]
    fn concave_polygon_hull_drops_reflex_vertex() {
        // The (2, 1) vertex is a reflex dent in the left edge; the hull
        // is the enclosing quad.
        let pg: Polygon<Pt> =
            polygon![[(0., 0.), (4., 0.), (2., 1.), (4., 4.), (0., 4.), (0., 0.)]];
        let hull = convex_hull(&pg);
        // 4 corners + closing duplicate.
        assert_eq!(hull.points().count(), 5);
        let has_reflex = hull
            .0
            .iter()
            .any(|p| p.get::<0>() == 2.0 && p.get::<1>() == 1.0);
        assert!(!has_reflex);
    }
}
