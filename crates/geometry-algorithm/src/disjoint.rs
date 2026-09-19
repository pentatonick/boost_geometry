//! `disjoint(&a, &b)` — see
//! `boost/geometry/algorithms/disjoint.hpp`.
//!
//! Defined as the negation of [`crate::intersects()`] for every pair
//! the intersects kernel handles. Mirrors Boost's interface header
//! that resolves one of the two through the other
//! (`algorithms/detail/intersects/interface.hpp:64-78`).

use geometry_coords::CoordinateScalar;
use geometry_strategy::{CartesianIntersects, IntersectsStrategy};
use geometry_trait::{Box as BoxTrait, Geometry, Point as PointTrait};

/// `true` iff `a` and `b` share **no** point.
///
/// Mirrors `boost::geometry::disjoint(a, b)` from
/// `boost/geometry/algorithms/disjoint.hpp`.
#[inline]
#[must_use]
pub fn disjoint<A, B>(a: &A, b: &B) -> bool
where
    CartesianIntersects: IntersectsStrategy<A, B>,
{
    !CartesianIntersects.intersects(a, b)
}

/// `disjoint` for two axis-aligned boxes — a direct separating-axis
/// test, skipping the general intersects machinery.
///
/// Two boxes are disjoint iff they are separated on **either** axis:
/// one's maximum on an axis is below the other's minimum. Mirrors the
/// box/box specialisation in
/// `boost/geometry/strategies/cartesian/disjoint_box_box.hpp`, which
/// short-circuits per axis rather than routing through `intersects`.
///
/// This is a CC5 fast path: `disjoint(Box, Box)` is one of the hottest
/// pair-combinations (envelope pruning, rtree node tests), and the
/// per-axis comparison is far cheaper than the generic areal intersects.
///
/// # Examples
///
/// ```
/// use geometry_algorithm::disjoint_box_box;
/// use geometry_cs::Cartesian;
/// use geometry_model::{Box, Point2D};
///
/// type P = Point2D<f64, Cartesian>;
/// let a = Box::from_corners(P::new(0.0, 0.0), P::new(2.0, 2.0));
/// let far = Box::from_corners(P::new(5.0, 5.0), P::new(6.0, 6.0));
/// let overlapping = Box::from_corners(P::new(1.0, 1.0), P::new(3.0, 3.0));
/// assert!(disjoint_box_box(&a, &far));
/// assert!(!disjoint_box_box(&a, &overlapping));
/// ```
#[inline]
#[must_use]
pub fn disjoint_box_box<A, B, T>(a: &A, b: &B) -> bool
where
    A: BoxTrait,
    B: BoxTrait,
    <A as Geometry>::Point: PointTrait<Scalar = T>,
    <B as Geometry>::Point: PointTrait<Scalar = T>,
    T: CoordinateScalar,
{
    (0..<A as Geometry>::Point::DIM).any(|d| separated_on_axis(a, b, d))
}

/// Are the two boxes separated along axis `d`? `get_indexed::<I, D>`:
/// `I = 0` is the min corner, `I = 1` the max corner; one arm per
/// dimension up to `MAX_DIM`.
fn separated_on_axis<A, B, T>(a: &A, b: &B, d: usize) -> bool
where
    A: BoxTrait,
    B: BoxTrait,
    <A as Geometry>::Point: PointTrait<Scalar = T>,
    <B as Geometry>::Point: PointTrait<Scalar = T>,
    T: CoordinateScalar,
{
    let (a_min, a_max, b_min, b_max) = match d {
        0 => (
            a.get_indexed::<0, 0>(),
            a.get_indexed::<1, 0>(),
            b.get_indexed::<0, 0>(),
            b.get_indexed::<1, 0>(),
        ),
        1 => (
            a.get_indexed::<0, 1>(),
            a.get_indexed::<1, 1>(),
            b.get_indexed::<0, 1>(),
            b.get_indexed::<1, 1>(),
        ),
        2 => (
            a.get_indexed::<0, 2>(),
            a.get_indexed::<1, 2>(),
            b.get_indexed::<0, 2>(),
            b.get_indexed::<1, 2>(),
        ),
        3 => (
            a.get_indexed::<0, 3>(),
            a.get_indexed::<1, 3>(),
            b.get_indexed::<0, 3>(),
            b.get_indexed::<1, 3>(),
        ),
        _ => panic!("disjoint_box_box: DIM exceeds MAX_DIM (4)"),
    };
    a_max < b_min || b_max < a_min
}

#[cfg(test)]
mod tests {
    use super::disjoint;
    use crate::intersects::intersects;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};

    type P = Point2D<f64, Cartesian>;
    type LS = Linestring<P>;

    #[test]
    fn disjoint_matches_negated_intersects() {
        let a: LS = linestring![(0.0, 0.0), (2.0, 0.0)];
        let b: LS = linestring![(10.0, 10.0), (11.0, 11.0)];
        assert!(disjoint(&a, &b));
        assert!(!intersects(&a, &b));
    }

    #[test]
    fn disjoint_false_when_intersecting() {
        let a: LS = linestring![(0.0, 0.0), (2.0, 0.0)];
        let b: LS = linestring![(1.0, -1.0), (1.0, 1.0)];
        assert!(!disjoint(&a, &b));
        assert!(intersects(&a, &b));
    }

    /// Polygon–polygon disjointness: separated squares are disjoint,
    /// overlapping ones are not.
    #[test]
    fn polygon_pair_disjointness() {
        use geometry_model::{Polygon, polygon};
        let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
        let apart: Polygon<P> = polygon![[
            (10.0, 10.0),
            (14.0, 10.0),
            (14.0, 14.0),
            (10.0, 14.0),
            (10.0, 10.0)
        ]];
        let overlapping: Polygon<P> =
            polygon![[(2.0, 2.0), (6.0, 2.0), (6.0, 6.0), (2.0, 6.0), (2.0, 2.0)]];
        assert!(disjoint(&a, &apart));
        assert!(!disjoint(&a, &overlapping));
    }

    #[test]
    fn box_box_fast_path() {
        use crate::disjoint::disjoint_box_box;
        use geometry_model::Box;

        let a = Box::from_corners(P::new(0.0, 0.0), P::new(2.0, 2.0));
        // Separated on x.
        let right = Box::from_corners(P::new(5.0, 0.0), P::new(6.0, 2.0));
        assert!(disjoint_box_box(&a, &right));
        // Separated on y.
        let above = Box::from_corners(P::new(0.0, 5.0), P::new(2.0, 6.0));
        assert!(disjoint_box_box(&a, &above));
        // Overlapping.
        let over = Box::from_corners(P::new(1.0, 1.0), P::new(3.0, 3.0));
        assert!(!disjoint_box_box(&a, &over));
        // Touching edges count as *not* disjoint (closed boxes).
        let touch = Box::from_corners(P::new(2.0, 0.0), P::new(4.0, 2.0));
        assert!(!disjoint_box_box(&a, &touch));
    }

    /// `disjoint_box_box.hpp` loops over every dimension: boxes that
    /// coincide in `x`/`y` but are separated along `z` are disjoint.
    #[test]
    fn box_box_three_d_separated_along_z_is_disjoint() {
        use crate::disjoint::disjoint_box_box;
        use geometry_model::{Box, Point3D};
        type P3 = Point3D<f64, Cartesian>;
        let a = Box::from_corners(P3::new(0.0, 0.0, 0.0), P3::new(1.0, 1.0, 1.0));
        let b = Box::from_corners(P3::new(0.0, 0.0, 5.0), P3::new(1.0, 1.0, 6.0));
        let c = Box::from_corners(P3::new(0.5, 0.5, 0.5), P3::new(2.0, 2.0, 2.0));
        assert!(disjoint_box_box(&a, &b));
        assert!(disjoint_box_box(&b, &a));
        assert!(!disjoint_box_box(&a, &c));
    }
}
