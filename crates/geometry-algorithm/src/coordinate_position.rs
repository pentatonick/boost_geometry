//! Tri-state point-in-geometry classification.
//!
//! Boost's Cartesian winding strategy computes a three-way result before
//! `within` and `covered_by` reduce it to booleans. This module exposes that
//! distinction through the same public tag-dispatched strategy path.

use geometry_strategy::{WithinStrategy, WithinStrategyForKind};
use geometry_trait::{Geometry, Point};

use crate::{covered_by, within};

/// The position of a point relative to a geometry.
///
/// Mirrors the `-1` / `0` / `+1` result of
/// `strategy/cartesian/point_in_poly_winding.hpp:69-74` with descriptive
/// variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CoordinatePosition {
    /// The point lies in the strict interior.
    Inside,
    /// The point lies on the geometry boundary.
    OnBoundary,
    /// The point lies outside the geometry, including inside a polygon hole.
    Outside,
}

/// Classify a point as inside, on the boundary, or outside a geometry.
///
/// This preserves the tri-state result that [`within`] and [`covered_by`]
/// expose as separate boolean predicates.
#[inline]
#[must_use]
pub fn coordinate_position<P, G>(point: &P, geometry: &G) -> CoordinatePosition
where
    P: Point,
    G: Geometry,
    G::Kind: WithinStrategyForKind,
    <G::Kind as WithinStrategyForKind>::S: WithinStrategy<P, G>,
{
    if within(point, geometry) {
        CoordinatePosition::Inside
    } else if covered_by(point, geometry) {
        CoordinatePosition::OnBoundary
    } else {
        CoordinatePosition::Outside
    }
}

#[cfg(test)]
mod tests {
    use super::{CoordinatePosition, coordinate_position};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};

    type P = Point2D<f64, Cartesian>;

    fn square_with_hole() -> Polygon<P> {
        polygon![
            [(0.0, 0.0), (0.0, 5.0), (5.0, 5.0), (5.0, 0.0), (0.0, 0.0)],
            [(2.0, 2.0), (3.0, 2.0), (3.0, 3.0), (2.0, 3.0), (2.0, 2.0)]
        ]
    }

    #[test]
    fn distinguishes_interior_boundary_and_exterior() {
        let polygon = square_with_hole();
        assert_eq!(
            coordinate_position(&P::new(1.0, 1.0), &polygon),
            CoordinatePosition::Inside
        );
        assert_eq!(
            coordinate_position(&P::new(0.0, 1.0), &polygon),
            CoordinatePosition::OnBoundary
        );
        assert_eq!(
            coordinate_position(&P::new(2.5, 2.5), &polygon),
            CoordinatePosition::Outside
        );
    }
}
