//! Two-axis indexed access for geometries that are addressed by an
//! `(index, dim)` pair rather than `dim` alone (Box, Segment).
//!
//! Mirrors `boost::geometry::traits::indexed_access<G, Index, Dimension>`
//! from `boost/geometry/core/access.hpp:79-84`, together with the
//! `min_corner` / `max_corner` constants declared in the same header
//! at `boost/geometry/core/access.hpp:36-39`.
//!
//! Design notes:
//! * Both axes — the corner index `I` and the dimension `D` — are
//!   const generics, matching Boost's `template <std::size_t Index,
//!   std::size_t Dimension>` on `traits::indexed_access`. Out-of-range
//!   `I` or `D` is rejected at the call site rather than at runtime
//!   (see `tests/ui/indexed_out_of_range.rs`).
//! * The Boost C++ side specialises `traits::indexed_access` separately
//!   per `(Geometry, Index, Dimension)` triple. On the Rust side a
//!   single trait with two const generics replaces all of those
//!   specialisations.

use crate::geometry::Geometry;
use crate::point::Point;

/// Two-axis (`Index`, `Dimension`) access for geometries built out of
/// a pair of corner points — Box and Segment.
///
/// Mirrors `boost::geometry::traits::indexed_access<G, Index, Dimension>`
/// from `boost/geometry/core/access.hpp:79-84`. The associated
/// scalar type is projected through the geometry's `Point`, matching
/// `boost::geometry::coordinate_type<G>::type` for indexed geometries.
///
/// For a Box, `I` is one of [`corner::MIN`] / [`corner::MAX`]; for a
/// Segment, `I` is the endpoint index `0` / `1`. `D` is the
/// coordinate dimension, with the same `0 <= D < Point::DIM`
/// invariant as [`Point::get`].
///
/// # Examples
///
/// ```
/// use geometry_trait::{corner, IndexedAccess};
///
/// // Generic helpers can require `IndexedAccess` and use the corner
/// // constants — same Box and Segment surface.
/// fn x_min<G>(g: &G) -> <G::Point as geometry_trait::Point>::Scalar
/// where G: IndexedAccess { g.get_indexed::<{ corner::MIN }, 0>() }
/// ```
pub trait IndexedAccess: Geometry {
    /// Read coordinate `D` of corner `I`.
    ///
    /// Mirrors `boost::geometry::traits::indexed_access<G, I, D>::get(g)`
    /// (`boost/geometry/core/access.hpp:79-84`). Both `I` and `D` are
    /// const generics: passing an out-of-range value is rejected at
    /// the call site rather than at runtime.
    fn get_indexed<const I: usize, const D: usize>(&self) -> <Self::Point as Point>::Scalar;

    /// Write coordinate `D` of corner `I`.
    ///
    /// Mirrors `boost::geometry::traits::indexed_access<G, I, D>::set(g, v)`
    /// (`boost/geometry/core/access.hpp:79-84`).
    fn set_indexed<const I: usize, const D: usize>(
        &mut self,
        value: <Self::Point as Point>::Scalar,
    );
}

/// Indices used when addressing the two corners of a Box.
///
/// # Examples
///
/// ```
/// use geometry_trait::corner;
/// assert_eq!(corner::MIN, 0);
/// assert_eq!(corner::MAX, 1);
/// ```
///
/// Mirrors `boost::geometry::min_corner` and `boost::geometry::max_corner`
/// declared at `boost/geometry/core/access.hpp:36-39`. For Segment,
/// the same `0` / `1` integers select the first / second endpoint;
/// segment endpoints are not "min" or "max", so call sites that read
/// segments should write the literal `0` / `1` rather than these
/// constants — matching the Boost convention where `min_corner` /
/// `max_corner` are box-specific names even though the underlying
/// integers are shared.
pub mod corner {
    /// Index of the minimum corner of a Box.
    ///
    /// Mirrors `boost::geometry::min_corner`
    /// (`boost/geometry/core/access.hpp:36`).
    pub const MIN: usize = 0;

    /// Index of the maximum corner of a Box.
    ///
    /// Mirrors `boost::geometry::max_corner`
    /// (`boost/geometry/core/access.hpp:39`).
    pub const MAX: usize = 1;
}

#[cfg(test)]
mod tests {
    //! Smoke test mirroring `boost/geometry/test/core/access.cpp:55-86`
    //! (`test_indexed_get_set` / `test_indexed_get`): set all four
    //! `(I, D)` slots of a 2D Box-like value, read them back, assert
    //! the values round-trip.

    use super::{IndexedAccess, corner};
    use crate::geometry::Geometry;
    use crate::point::Point;
    use geometry_cs::Cartesian;
    use geometry_tag::{BoxTag, PointTag};

    struct P2(f64, f64);

    impl Geometry for P2 {
        type Kind = PointTag;
        type Point = Self;
    }

    impl Point for P2 {
        type Scalar = f64;
        type Cs = Cartesian;
        const DIM: usize = 2;

        fn get<const D: usize>(&self) -> f64 {
            const {
                assert!(D < <P2 as Point>::DIM, "Point::get: dimension out of range");
            }
            if D == 0 { self.0 } else { self.1 }
        }
    }

    struct B {
        corners: [[f64; 2]; 2],
    }

    impl Geometry for B {
        type Kind = BoxTag;
        type Point = P2;
    }

    impl IndexedAccess for B {
        fn get_indexed<const I: usize, const D: usize>(&self) -> f64 {
            const {
                assert!(I < 2, "IndexedAccess::get_indexed: index out of range");
                assert!(D < 2, "IndexedAccess::get_indexed: dimension out of range");
            }
            self.corners[I][D]
        }

        fn set_indexed<const I: usize, const D: usize>(&mut self, v: f64) {
            const {
                assert!(I < 2, "IndexedAccess::set_indexed: index out of range");
                assert!(D < 2, "IndexedAccess::set_indexed: dimension out of range");
            }
            self.corners[I][D] = v;
        }
    }

    #[test]
    fn indexed_get_set_roundtrip() {
        let point = P2(5.0, 6.0);
        assert_eq!(point.get::<0>().to_bits(), 5.0_f64.to_bits());
        assert_eq!(point.get::<1>().to_bits(), 6.0_f64.to_bits());

        let mut b = B {
            corners: [[0.0; 2]; 2],
        };
        b.set_indexed::<{ corner::MIN }, 0>(1.0);
        b.set_indexed::<{ corner::MIN }, 1>(2.0);
        b.set_indexed::<{ corner::MAX }, 0>(3.0);
        b.set_indexed::<{ corner::MAX }, 1>(4.0);

        // Bit-equal comparison: each slot was written with a literal
        // f64 constant and read back without arithmetic, so the round
        // trip must be exact. Using `to_bits()` sidesteps the
        // `clippy::float_cmp` lint without weakening the assertion.
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
    fn corner_constants_match_boost() {
        // `boost/geometry/core/access.hpp:36-39` pins MIN = 0, MAX = 1.
        assert_eq!(corner::MIN, 0);
        assert_eq!(corner::MAX, 1);
    }
}
