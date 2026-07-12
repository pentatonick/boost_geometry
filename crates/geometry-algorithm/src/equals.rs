//! `equals(&a, &b)` — see
//! `boost/geometry/algorithms/equals.hpp`.
//!
//! Cartesian-only in v1. The per-pair strategy is selected by the
//! tag-keyed [`geometry_strategy::EqualsPairStrategy`] picker on
//! `(A::Kind, B::Kind)`, so any concept-adapted foreign type resolves
//! through the same path as the equivalent `geometry-model` value.

use geometry_strategy::{EqualsPairStrategy, EqualsStrategy};
use geometry_trait::Geometry;

/// `true` iff `a` and `b` describe the same point set.
///
/// Mirrors `boost::geometry::equals(a, b)` from
/// `boost/geometry/algorithms/equals.hpp`. Polygon equality is
/// up-to-rotation and traversal direction; vertex-order normalisation
/// happens inside the strategy kernel.
#[inline]
#[must_use]
pub fn equals<A, B>(a: &A, b: &B) -> bool
where
    A: Geometry,
    B: Geometry,
    A::Kind: EqualsPairStrategy<B::Kind>,
    <A::Kind as EqualsPairStrategy<B::Kind>>::S: EqualsStrategy<A, B>,
{
    <<A::Kind as EqualsPairStrategy<B::Kind>>::S as Default>::default().equals(a, b)
}

#[cfg(test)]
mod tests {
    use super::equals;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};

    type P = Point2D<f64, Cartesian>;

    fn pt(x: f64, y: f64) -> P {
        Point2D::new(x, y)
    }

    #[test]
    fn equals_same_point() {
        assert!(equals(&pt(1.0, 2.0), &pt(1.0, 2.0)));
        assert!(!equals(&pt(1.0, 2.0), &pt(1.0, 2.1)));
    }

    #[test]
    fn equals_polygon_rotated_and_reversed() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(4.0, 4.0), (0.0, 4.0), (0.0, 0.0), (4.0, 0.0), (4.0, 4.0)]];
        assert!(equals(&a, &b));
    }
}
