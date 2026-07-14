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

    /// Point equality compares all three ordinates in 3D — the `2 =>`
    /// arm. Points equal in x,y but differing in z are not equal.
    #[test]
    fn equals_point_in_three_dimensions() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let a = P3::new(1.0, 2.0, 3.0);
        assert!(equals(&a, &P3::new(1.0, 2.0, 3.0)));
        assert!(!equals(&a, &P3::new(1.0, 2.0, 9.0)));
    }

    /// Two polygons with matching holes (given in a different order) are
    /// equal — the interior-ring permutation match with a `break`.
    #[test]
    fn equals_polygon_with_holes_matched_by_permutation() {
        let a: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)],
            [(5.0, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 5.0)]
        ];
        // Same polygon, holes listed in the opposite order.
        let b: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(5.0, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 5.0)],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)]
        ];
        assert!(equals(&a, &b));
    }

    /// Polygons whose exteriors match but whose hole *counts* differ are
    /// not equal (the count-mismatch short-circuit).
    #[test]
    fn polygon_not_equals_on_different_hole_count() {
        let a: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)]
        ];
        let b: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        assert!(!equals(&a, &b));
    }

    /// Polygons with the same hole count but a hole that has no match are
    /// not equal (the `if !found { return false }` arm).
    #[test]
    fn polygon_not_equals_when_a_hole_has_no_match() {
        let a: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)]
        ];
        let b: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(7.0, 7.0), (8.0, 7.0), (8.0, 8.0), (7.0, 7.0)]
        ];
        assert!(!equals(&a, &b));
    }

    /// Polygons whose exterior rings differ in vertex count are not equal
    /// (the `av.len() != bv.len()` short-circuit in `rings_equal`).
    #[test]
    fn polygon_not_equals_on_different_vertex_count() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 0.0)]]; // triangle
        let b: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]]; // square
        assert!(!equals(&a, &b));
    }
}
