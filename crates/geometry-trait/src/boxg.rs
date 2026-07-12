//! The [`Box`] concept and the [`box_min`] / [`box_max`] materialiser
//! helpers. The module is named `boxg` because `box` is a Rust
//! keyword; the trait itself keeps the C++ name.
//!
//! Mirrors `doc/concept/box.qbk` and the model in
//! `boost/geometry/geometries/box.hpp`. Boxes are axis-aligned and
//! described by a minimum and a maximum corner point; the two-axis
//! `(corner, dimension)` access is provided by
//! [`crate::IndexedAccess`].
//!
//! Design notes:
//! * The corner indices match Boost's
//!   `boost::geometry::min_corner` / `max_corner` constants
//!   (`boost/geometry/core/access.hpp:36-39`) and are re-exposed
//!   through [`crate::corner::MIN`] / [`crate::corner::MAX`].
//! * The `min_corner()` / `max_corner()` accessors on
//!   `model::box` (`boost/geometry/geometries/box.hpp:138-171`) are
//!   replaced by the free helpers [`box_min`] / [`box_max`], which
//!   reconstruct a [`crate::Point`] value via `Default::default()`
//!   followed by per-dimension `set::<D>` writes — the same shared
//!   materialiser used by `segment_start` / `segment_end`.

use crate::geometry::Geometry;
use crate::indexed_access::{IndexedAccess, corner};
use crate::point::PointMut;
use crate::segment::materialise;
use geometry_tag::BoxTag;

/// An axis-aligned bounding box.
///
/// Models `doc/concept/box.qbk` — equivalent in C++ to the trait
/// specialisations on `model::box` in
/// `boost/geometry/geometries/box.hpp:187-247`. Implementers provide
/// [`IndexedAccess`] for `I` ∈ {[`corner::MIN`], [`corner::MAX`]}.
/// The two corner points can but need not be materialised as `Point`
/// values; [`box_min`] / [`box_max`] reconstruct them from the
/// indexed accessors on demand.
///
/// # Examples
///
/// ```
/// use geometry_trait::Box;
/// // A generic helper that accepts any `Box` impl.
/// fn _accepts_box<B: Box>(_b: &B) {}
/// ```
pub trait Box: Geometry<Kind = BoxTag> + IndexedAccess {}

/// Materialise the minimum corner of a box as a [`crate::Point`]
/// value.
///
/// Mirrors `b.min_corner()` on `model::box`
/// (`boost/geometry/geometries/box.hpp:138-144`). The point is built
/// via [`Default::default`] and one `set::<D>` call per dimension,
/// with each value sourced from
/// `b.get_indexed::<{ corner::MIN }, D>()`.
///
/// # Examples
///
/// ```
/// // See `geometry_model::Box::min_corner` and the integration tests
/// // under `crates/geometry-model/` for a fully-built example.
/// use geometry_trait::{box_min, Box, PointMut};
/// fn _ex<B: Box>(b: &B) where B::Point: Default + PointMut { let _ = box_min(b); }
/// ```
pub fn box_min<B: Box>(b: &B) -> B::Point
where
    B::Point: Default + PointMut,
{
    materialise::<B, { corner::MIN }>(b)
}

/// Materialise the maximum corner of a box as a [`crate::Point`]
/// value.
///
/// Mirrors `b.max_corner()` on `model::box`
/// (`boost/geometry/geometries/box.hpp:149-155`). The point is built
/// via [`Default::default`] and one `set::<D>` call per dimension,
/// with each value sourced from
/// `b.get_indexed::<{ corner::MAX }, D>()`.
///
/// # Examples
///
/// ```
/// use geometry_trait::{box_max, Box, PointMut};
/// fn _ex<B: Box>(b: &B) where B::Point: Default + PointMut { let _ = box_max(b); }
/// ```
pub fn box_max<B: Box>(b: &B) -> B::Point
where
    B::Point: Default + PointMut,
{
    materialise::<B, { corner::MAX }>(b)
}

#[cfg(test)]
mod tests {
    //! Synthetic Box round-trip — mirrors
    //! `boost/geometry/test/core/access.cpp:55-86` for the box case.
    //! The `MyBox` here is *not* `geometry-model::box`; that lands
    //! in T17.

    use super::*;
    use crate::point::Point;
    use geometry_cs::Cartesian;
    use geometry_tag::PointTag;
    // `PointMut` is already in scope via the module-level `use`.

    #[derive(Default)]
    struct Xy {
        x: f64,
        y: f64,
    }

    impl Geometry for Xy {
        type Kind = PointTag;
        type Point = Self;
    }

    impl Point for Xy {
        type Scalar = f64;
        type Cs = Cartesian;
        const DIM: usize = 2;

        fn get<const D: usize>(&self) -> f64 {
            if D == 0 { self.x } else { self.y }
        }
    }

    impl PointMut for Xy {
        fn set<const D: usize>(&mut self, v: f64) {
            if D == 0 {
                self.x = v;
            } else {
                self.y = v;
            }
        }
    }

    struct MyBox {
        c: [[f64; 2]; 2],
    }

    impl Geometry for MyBox {
        type Kind = BoxTag;
        type Point = Xy;
    }

    impl IndexedAccess for MyBox {
        fn get_indexed<const I: usize, const D: usize>(&self) -> f64 {
            self.c[I][D]
        }
        fn set_indexed<const I: usize, const D: usize>(&mut self, v: f64) {
            self.c[I][D] = v;
        }
    }

    impl Box for MyBox {}

    #[test]
    fn box_indexed_round_trip() {
        let mut b = MyBox { c: [[0.0; 2]; 2] };
        b.set_indexed::<{ corner::MIN }, 0>(1.0);
        b.set_indexed::<{ corner::MIN }, 1>(2.0);
        b.set_indexed::<{ corner::MAX }, 0>(3.0);
        b.set_indexed::<{ corner::MAX }, 1>(4.0);
        assert_eq!(
            b.get_indexed::<{ corner::MIN }, 0>().to_bits(),
            1.0_f64.to_bits()
        );
        assert_eq!(
            b.get_indexed::<{ corner::MAX }, 1>().to_bits(),
            4.0_f64.to_bits()
        );
    }

    #[test]
    fn box_corners_materialise() {
        let mut b = MyBox { c: [[0.0; 2]; 2] };
        b.set_indexed::<{ corner::MIN }, 0>(1.0);
        b.set_indexed::<{ corner::MIN }, 1>(2.0);
        b.set_indexed::<{ corner::MAX }, 0>(3.0);
        b.set_indexed::<{ corner::MAX }, 1>(4.0);

        let lo = box_min(&b);
        let hi = box_max(&b);
        assert_eq!(lo.get::<0>().to_bits(), 1.0_f64.to_bits());
        assert_eq!(lo.get::<1>().to_bits(), 2.0_f64.to_bits());
        assert_eq!(hi.get::<0>().to_bits(), 3.0_f64.to_bits());
        assert_eq!(hi.get::<1>().to_bits(), 4.0_f64.to_bits());
    }
}
