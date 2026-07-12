//! `is_empty(&g)` — true iff `num_points(g) == 0`.
//!
//! Mirrors `boost::geometry::is_empty` from
//! `boost/geometry/algorithms/is_empty.hpp`. Boost's contract is
//! exactly `num_points(g) == 0`
//! (`boost/geometry/test/algorithms/is_empty.cpp:74`:
//! `BOOST_CHECK_EQUAL(detected, bg::num_points(geometry) == 0)`), so
//! this leans on [`crate::num_points::num_points`] rather
//! than re-walking the kind hierarchy. Singletons (`Point` / `Box` /
//! `Segment`) always return `false` because they carry an implicit
//! point set.

use crate::num_points::{NumPointsStrategy, NumPointsStrategyForKind, num_points};
use geometry_trait::Geometry;

/// True iff `g` has no points.
///
/// Mirrors `boost::geometry::is_empty(g)` from
/// `boost/geometry/algorithms/is_empty.hpp`.
#[inline]
#[must_use]
pub fn is_empty<G>(g: &G) -> bool
where
    G: Geometry,
    G::Kind: NumPointsStrategyForKind,
    <G::Kind as NumPointsStrategyForKind>::S: NumPointsStrategy<G>,
{
    num_points(g) == 0
}

#[cfg(test)]
mod tests {
    //! Reference values from
    //! `geometry/test/algorithms/is_empty.cpp`.

    use super::is_empty;
    use geometry_cs::Cartesian;
    use geometry_model::{Box, Linestring, MultiPoint, Point2D, Polygon, Segment, linestring};

    type Pt = Point2D<f64, Cartesian>;

    /// `is_empty.cpp:110-111` — points are never empty.
    #[test]
    fn point_is_never_empty() {
        assert!(!is_empty(&Pt::new(0.0, 0.0)));
        assert!(!is_empty(&Pt::new(1.0, 1.0)));
    }

    /// `is_empty.cpp:116` — a zero-length segment still has two points.
    #[test]
    fn segment_zero_length_is_not_empty() {
        let s = Segment::new(Pt::new(0.0, 0.0), Pt::new(0.0, 0.0));
        assert!(!is_empty(&s));
    }

    /// `is_empty.cpp:122` — a box is never empty.
    #[test]
    fn box_is_never_empty() {
        let b = Box::from_corners(Pt::new(0.0, 0.0), Pt::new(1.0, 1.0));
        assert!(!is_empty(&b));
    }

    /// `is_empty.cpp:134` — `LINESTRING()` (no points) is empty.
    #[test]
    fn empty_linestring_is_empty() {
        let ls: Linestring<Pt> = Linestring::new();
        assert!(is_empty(&ls));
    }

    /// `is_empty.cpp:135` — a non-empty linestring is not empty.
    #[test]
    fn nonempty_linestring_is_not_empty() {
        let ls: Linestring<Pt> = linestring![(0.0, 0.0), (1.0, 1.0)];
        assert!(!is_empty(&ls));
    }

    /// `is_empty.cpp:145` — `MULTIPOINT()` is empty.
    #[test]
    fn empty_multi_point_is_empty() {
        let mp: MultiPoint<Pt> = MultiPoint(Vec::new());
        assert!(is_empty(&mp));
    }

    /// `is_empty.cpp:226` — a polygon with an empty outer ring is empty.
    #[test]
    fn polygon_with_zero_point_outer_is_empty() {
        let pg: Polygon<Pt> = Polygon::default();
        assert!(is_empty(&pg));
    }
}
