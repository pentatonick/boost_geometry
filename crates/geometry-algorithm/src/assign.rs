//! Write coordinates into a mutable geometry.
//!
//! Mirrors `boost::geometry::assign_values` from
//! `boost/geometry/algorithms/assign.hpp`. Boost exposes variadic
//! overloads (`assign_values(p, x, y)`, `assign_values(p, x, y, z)`,
//! …); the Rust port takes a slice so one signature covers every
//! arity. The arity check that Boost gets for free from SFINAE (a
//! two-argument call on a 3-D point fails to compile) becomes a
//! runtime `assert!` on the slice length here.

use geometry_trait::PointMut;

/// Write `values[D]` to `p.set::<D>(values[D])` for every dimension
/// `D` in `0..P::DIM`.
///
/// Mirrors `boost::geometry::assign_values(p, v...)` from
/// `boost/geometry/algorithms/assign.hpp`.
///
/// # Panics
///
/// Panics if `values.len() < P::DIM`. The bound is checked at
/// runtime — Boost relies on the variadic arity matching `DIM` at
/// compile time via SFINAE, a guarantee the slice signature trades
/// away for arity-agnostic ergonomics.
pub fn assign_values<P: PointMut>(p: &mut P, values: &[P::Scalar]) {
    assert!(
        values.len() >= P::DIM,
        "assign_values: need at least {} values, got {}",
        P::DIM,
        values.len(),
    );
    // `PointMut::set` needs `&mut self`, so the dimension recursion
    // cannot ride on `fold_dims` (which hands the closure a shared
    // `&P`). Dispatch on the monomorphised `P::DIM` instead, falling
    // through so that a 3-D point also writes dims 0 and 1. The arms
    // track `geometry_trait::point::MAX_DIM`.
    let dim = P::DIM;
    assert!(dim <= 4, "assign_values: DIM exceeds MAX_DIM (4)");
    if dim > 0 {
        p.set::<0>(values[0]);
    }
    if dim > 1 {
        p.set::<1>(values[1]);
    }
    if dim > 2 {
        p.set::<2>(values[2]);
    }
    if dim > 3 {
        p.set::<3>(values[3]);
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Assigned coordinates are read back as exact literals."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/assign.cpp` — a slice written
    //! into a point comes back coordinate-for-coordinate.

    use super::assign_values;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Point3D};
    use geometry_trait::Point as _;

    #[test]
    fn point2_round_trip() {
        let mut p: Point2D<f64, Cartesian> = Point2D::default();
        assign_values(&mut p, &[3.0, 4.0]);
        assert_eq!(p.get::<0>(), 3.0);
        assert_eq!(p.get::<1>(), 4.0);
    }

    #[test]
    fn point3_round_trip() {
        let mut p: Point3D<f64, Cartesian> = Point3D::default();
        assign_values(&mut p, &[1.0, 2.0, 3.0]);
        assert_eq!(p.get::<0>(), 1.0);
        assert_eq!(p.get::<1>(), 2.0);
        assert_eq!(p.get::<2>(), 3.0);
    }

    /// A 4-D point writes all four ordinates (the `dim > 3` arm).
    #[test]
    fn point4_round_trip() {
        let mut p: geometry_model::Point<f64, 4, Cartesian> = geometry_model::Point::default();
        assign_values(&mut p, &[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(p.get::<0>(), 1.0);
        assert_eq!(p.get::<1>(), 2.0);
        assert_eq!(p.get::<2>(), 3.0);
        assert_eq!(p.get::<3>(), 4.0);
    }

    #[test]
    #[should_panic(expected = "assign_values: need at least 2")]
    fn too_few_values_panics() {
        let mut p: Point2D<f64, Cartesian> = Point2D::default();
        assign_values(&mut p, &[1.0]);
    }
}
