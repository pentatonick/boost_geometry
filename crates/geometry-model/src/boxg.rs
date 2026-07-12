//! Default `model::Box<P>` — two corner points (min / max) stored
//! inline. The module is named `boxg` because `box` is a Rust
//! keyword; the type itself keeps the C++ name.
//!
//! Mirrors `boost::geometry::model::box<Point>` declared in
//! `boost/geometry/geometries/box.hpp:64-172`, together with the
//! trait specialisations the same header provides under
//! `namespace traits` (`tag`, `point_type`, `indexed_access`) at
//! `boost/geometry/geometries/box.hpp:187-247`. The Rust port
//! collapses those specialisations into the three trait impls below
//! ([`Geometry`], [`IndexedAccess`], [`BoxTrait`]), keyed off the
//! generic point parameter `P`.
//!
//! Boost stores the two corners as `m_min_corner` / `m_max_corner`
//! members (`box.hpp:165-170`); the Rust port stores them in a
//! `[P; 2]` array so the `(I, D)` indexed access can write
//! `corners[I].set::<D>(v)` without a `match` on `I`. The shared
//! corner indices come from
//! [`geometry_trait::corner::MIN`] / [`geometry_trait::corner::MAX`]
//! (mirroring `boost::geometry::min_corner` / `max_corner` at
//! `boost/geometry/core/access.hpp:36-39`).

use geometry_tag::BoxTag;
use geometry_trait::{
    Box as BoxTrait, Geometry, IndexedAccess, Point as PointTrait, PointMut, corner,
};

/// Axis-aligned bounding box.
///
/// Mirrors `boost::geometry::model::box<P>` from
/// `boost/geometry/geometries/box.hpp:64-172`. The two corners are
/// addressed by [`corner::MIN`] / [`corner::MAX`], matching the
/// `traits::indexed_access<box, min_corner|max_corner, D>`
/// specialisations at `boost/geometry/geometries/box.hpp:213-247`.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Box<P: PointTrait> {
    corners: [P; 2],
}

impl<P: PointTrait> Box<P> {
    /// Construct a box from its minimum and maximum corners.
    ///
    /// Mirrors the `box(Point const& min_corner, Point const& max_corner)`
    /// constructor at `boost/geometry/geometries/box.hpp:103-110`.
    #[must_use]
    pub const fn from_corners(min: P, max: P) -> Self {
        Self {
            corners: [min, max],
        }
    }

    /// Borrow the minimum corner.
    ///
    /// Mirrors `b.min_corner()` on `model::box`
    /// (`boost/geometry/geometries/box.hpp:138-144`); returns a
    /// reference rather than copying.
    #[must_use]
    pub const fn min(&self) -> &P {
        &self.corners[corner::MIN]
    }

    /// Borrow the maximum corner.
    ///
    /// Mirrors `b.max_corner()` on `model::box`
    /// (`boost/geometry/geometries/box.hpp:149-155`).
    #[must_use]
    pub const fn max(&self) -> &P {
        &self.corners[corner::MAX]
    }
}

/// Tag this concrete type as a Box. Mirrors the
/// `traits::tag<model::box<...>>` specialisation at
/// `boost/geometry/geometries/box.hpp:189-192`.
impl<P: PointTrait> Geometry for Box<P> {
    type Kind = BoxTag;
    type Point = P;
}

/// Two-axis access on the box. Collapses the
/// `traits::indexed_access<box, min_corner|max_corner, D>`
/// specialisations at `boost/geometry/geometries/box.hpp:213-247`
/// into one Rust impl: the `I` axis selects the corner, `D` then
/// dispatches to `Point::get` / `Point::set` on that corner.
impl<P: PointMut> IndexedAccess for Box<P> {
    #[inline]
    fn get_indexed<const I: usize, const D: usize>(&self) -> P::Scalar {
        self.corners[I].get::<D>()
    }

    #[inline]
    fn set_indexed<const I: usize, const D: usize>(&mut self, value: P::Scalar) {
        self.corners[I].set::<D>(value);
    }
}

/// Wire the Box concept up. The trait body is empty — the
/// concept is satisfied by the two super-bounds [`Geometry`] and
/// [`IndexedAccess`] already proven above.
impl<P: PointMut> BoxTrait for Box<P> {}

#[cfg(test)]
mod tests {
    //! Round-trip + tag-resolution + `check_box` witness for
    //! [`Box`]. Mirrors
    //! `boost/geometry/test/core/access.cpp:55-86` (the
    //! `test_indexed_get_set` helper applied to a box) and the
    //! box-tag arm of `boost/geometry/test/core/tag.cpp`.

    use super::Box;
    use crate::point::Point2D;
    use geometry_cs::Cartesian;
    use geometry_tag::BoxTag;
    use geometry_trait::{Geometry, IndexedAccess, check_box, corner};

    #[test]
    fn box_round_trip_all_four_slots() {
        let mut b = Box::from_corners(
            Point2D::<f64, Cartesian>::default(),
            Point2D::<f64, Cartesian>::default(),
        );
        b.set_indexed::<{ corner::MIN }, 0>(1.0);
        b.set_indexed::<{ corner::MIN }, 1>(2.0);
        b.set_indexed::<{ corner::MAX }, 0>(3.0);
        b.set_indexed::<{ corner::MAX }, 1>(4.0);

        // Bit-equal comparison: same rationale as the segment test —
        // values written and read back without arithmetic must
        // round-trip exactly.
        assert_eq!(
            b.get_indexed::<{ corner::MIN }, 0>().to_bits(),
            1.0_f64.to_bits()
        );
        assert_eq!(
            b.get_indexed::<{ corner::MIN }, 1>().to_bits(),
            2.0_f64.to_bits()
        );
        assert_eq!(
            b.get_indexed::<{ corner::MAX }, 0>().to_bits(),
            3.0_f64.to_bits()
        );
        assert_eq!(
            b.get_indexed::<{ corner::MAX }, 1>().to_bits(),
            4.0_f64.to_bits()
        );
    }

    #[test]
    fn box_min_and_max_accessors() {
        use geometry_trait::Point as PointTrait;

        let lo = Point2D::<f64, Cartesian>::new(1.0, 2.0);
        let hi = Point2D::<f64, Cartesian>::new(3.0, 4.0);
        let b = Box::from_corners(lo, hi);
        // `Point<T, D, Cs>` derives `PartialEq`, but only when its
        // phantom `Cs` parameter does — and `Cartesian` does not.
        // Compare per-coordinate via the `Point` trait instead.
        assert_eq!(b.min().get::<0>().to_bits(), 1.0_f64.to_bits());
        assert_eq!(b.min().get::<1>().to_bits(), 2.0_f64.to_bits());
        assert_eq!(b.max().get::<0>().to_bits(), 3.0_f64.to_bits());
        assert_eq!(b.max().get::<1>().to_bits(), 4.0_f64.to_bits());
    }

    #[test]
    fn box_kind_is_box_tag() {
        fn k<T: Geometry<Kind = BoxTag>>() {}
        k::<Box<Point2D<f64, Cartesian>>>();
    }

    #[test]
    fn box_satisfies_concept() {
        check_box::<Box<Point2D<f64, Cartesian>>>();
    }
}
