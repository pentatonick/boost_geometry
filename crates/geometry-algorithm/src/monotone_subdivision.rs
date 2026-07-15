//! Y-monotone subdivision of a Cartesian polygon.
//!
//! Boost.Geometry has no monotone-partition entry. Every triangle is
//! y-monotone, so the native ear-cut triangulation supplies a valid (though
//! finer than maximal) monotone subdivision without introducing a second
//! polygon-cutting kernel.

use alloc::vec::Vec;

use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::Polygon as ModelPolygon;
use geometry_tag::SameAs;
use geometry_trait::{Point, Polygon};

use crate::triangulate_earcut::triangulate_earcut;

/// Subdivide a Cartesian polygon into y-monotone owned polygons.
///
/// The current native contract returns triangular pieces; triangles are valid
/// y-monotone polygons and preserve the source area exactly up to floating-point
/// arithmetic.
#[inline]
#[must_use]
pub fn monotone_subdivision<Pg, P>(polygon: &Pg) -> Vec<ModelPolygon<P>>
where
    Pg: Polygon<Point = P>,
    P: Point<Scalar = f64> + Copy,
    P::Cs: CoordinateSystem,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    triangulate_earcut(polygon)
}

#[cfg(test)]
mod tests {
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, Ring};

    use super::monotone_subdivision;

    #[test]
    fn reflex_polygon_subdivides_to_triangles() {
        type P = Point2D<f64, Cartesian>;
        let polygon: Polygon<P> = Polygon::new(Ring::from_vec(alloc::vec![
            P::new(0.0, 0.0),
            P::new(0.0, 2.0),
            P::new(1.0, 1.0),
            P::new(2.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 0.0),
        ]));
        let pieces = monotone_subdivision(&polygon);
        assert_eq!(pieces.len(), 3);
        assert!(pieces.iter().all(|piece| piece.outer.0.len() == 4));
    }
}
