//! The default `model::Point<T, D, Cs>`.
//!
//! Mirrors `boost::geometry::model::point<CoordinateType, DimensionCount,
//! CoordinateSystem>` declared in `boost/geometry/geometries/point.hpp`,
//! together with the matching trait specialisations (`tag`,
//! `coordinate_type`, `coordinate_system`, `dimension`, `access`) the
//! same header provides under `namespace traits`. The Rust port collapses
//! those five specialisations to one `Point` impl, plus the
//! [`Geometry`] super-bound for the tag.
//!
//! The two convenience aliases [`Point2D`] and [`Point3D`] play the role
//! of `boost/geometry/geometries/point_xy.hpp` and
//! `boost/geometry/geometries/point_xyz.hpp` — they fix the dimension
//! parameter but reuse the same const-generic body.

use core::marker::PhantomData;

use geometry_coords::CoordinateScalar;
use geometry_cs::{Cartesian, CoordinateSystem};
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point as PointTrait, PointMut};

/// Default Point — a `[T; D]` array plus a phantom CS tag.
///
/// Mirrors `boost::geometry::model::point<CoordinateType, DimensionCount,
/// CoordinateSystem>` from `boost/geometry/geometries/point.hpp`. The
/// `#[repr(transparent)]` annotation guarantees the in-memory layout
/// matches the underlying `[T; D]`, so callers can safely treat the
/// point as an array of coordinates when interfacing with FFI or SIMD
/// code — the same invariant Boost relies on for its
/// `m_values[DimensionCount]` member.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Point<T: CoordinateScalar, const D: usize, Cs: CoordinateSystem = Cartesian> {
    coords: [T; D],
    _cs: PhantomData<Cs>,
}

// Serde support (behind the `serde` feature). `serde` cannot derive
// over the const-generic `[T; D]` field, so `Point` gets a hand-written
// impl that (de)serialises the coordinates as a length-`D` sequence and
// reconstructs the phantom CS tag. The other model types are plain
// derives (see their own modules).
#[cfg(feature = "serde")]
impl<T, const D: usize, Cs> serde::Serialize for Point<T, D, Cs>
where
    T: CoordinateScalar + serde::Serialize,
    Cs: CoordinateSystem,
{
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeTuple;
        let mut tup = serializer.serialize_tuple(D)?;
        for coord in &self.coords {
            tup.serialize_element(coord)?;
        }
        tup.end()
    }
}

#[cfg(feature = "serde")]
impl<'de, T, const D: usize, Cs> serde::Deserialize<'de> for Point<T, D, Cs>
where
    T: CoordinateScalar + serde::Deserialize<'de>,
    Cs: CoordinateSystem,
{
    fn deserialize<De: serde::Deserializer<'de>>(deserializer: De) -> Result<Self, De::Error> {
        struct CoordsVisitor<T, const D: usize>(PhantomData<T>);

        impl<'de, T, const D: usize> serde::de::Visitor<'de> for CoordsVisitor<T, D>
        where
            T: CoordinateScalar + serde::Deserialize<'de>,
        {
            type Value = [T; D];

            fn expecting(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
                write!(f, "a sequence of {D} coordinates")
            }

            fn visit_seq<A: serde::de::SeqAccess<'de>>(
                self,
                mut seq: A,
            ) -> Result<[T; D], A::Error> {
                // Build the array element by element; `[T; D]` has no
                // `Default`, so collect into a temporary and convert.
                let mut coords: [T; D] = [T::ZERO; D];
                for (i, slot) in coords.iter_mut().enumerate() {
                    *slot = seq
                        .next_element::<T>()?
                        .ok_or_else(|| serde::de::Error::invalid_length(i, &self))?;
                }
                Ok(coords)
            }
        }

        let coords = deserializer.deserialize_tuple(D, CoordsVisitor::<T, D>(PhantomData))?;
        Ok(Point {
            coords,
            _cs: PhantomData,
        })
    }
}

/// 2D point alias.
///
/// Mirrors `boost::geometry::model::d2::point_xy<CoordinateType,
/// CoordinateSystem>` from `boost/geometry/geometries/point_xy.hpp`.
pub type Point2D<T, Cs = Cartesian> = Point<T, 2, Cs>;

/// 3D point alias.
///
/// Mirrors `boost::geometry::model::d3::point_xyz<CoordinateType,
/// CoordinateSystem>` from `boost/geometry/geometries/point_xyz.hpp`.
pub type Point3D<T, Cs = Cartesian> = Point<T, 3, Cs>;

// ---- Constructors -----------------------------------------------------
//
// Boost exposes three overloads of `point(...)` gated on the arity
// matching `DimensionCount` (`point.hpp:132-185`). Rust has no SFINAE,
// so we split into three inherent impls keyed on a specific const `D`.
// Calling `Point2D::new(v0, v1, v2)` then fails to type-check because
// there is no `new(_, _, _)` on `Point<T, 2, Cs>` — the mirror of the
// C++ `enable_if_t<is_coordinates_number_leq<C, 3, DimensionCount>>`
// guard.

impl<T: CoordinateScalar, Cs: CoordinateSystem> Point<T, 1, Cs> {
    /// Construct a 1-D point.
    ///
    /// Mirrors the single-argument `point(v0)` constructor at
    /// `boost/geometry/geometries/point.hpp:133-149`.
    #[must_use]
    pub const fn new(v0: T) -> Self {
        Self {
            coords: [v0],
            _cs: PhantomData,
        }
    }
}

impl<T: CoordinateScalar, Cs: CoordinateSystem> Point<T, 2, Cs> {
    /// Construct a 2-D point.
    ///
    /// Mirrors the two-argument `point(v0, v1)` constructor at
    /// `boost/geometry/geometries/point.hpp:151-167`.
    #[must_use]
    pub const fn new(v0: T, v1: T) -> Self {
        Self {
            coords: [v0, v1],
            _cs: PhantomData,
        }
    }
}

impl<T: CoordinateScalar, Cs: CoordinateSystem> Point<T, 3, Cs> {
    /// Construct a 3-D point.
    ///
    /// Mirrors the three-argument `point(v0, v1, v2)` constructor at
    /// `boost/geometry/geometries/point.hpp:169-185`.
    #[must_use]
    pub const fn new(v0: T, v1: T, v2: T) -> Self {
        Self {
            coords: [v0, v1, v2],
            _cs: PhantomData,
        }
    }
}

// ---- Default ----------------------------------------------------------
//
// `#[derive(Default)]` would require `[T; D]: Default`, whose
// const-generic blanket impl is still unstable. We provide an explicit
// impl using `CoordinateScalar::ZERO` — the Rust analogue of Boost's
// zero-init `m_values{}` at `point.hpp:113-119`.

impl<T: CoordinateScalar, const D: usize, Cs: CoordinateSystem> Default for Point<T, D, Cs> {
    #[inline]
    fn default() -> Self {
        Self {
            coords: [T::ZERO; D],
            _cs: PhantomData,
        }
    }
}

// ---- Trait impls ------------------------------------------------------

/// Tag this concrete type as a Point. Mirrors the
/// `traits::tag<model::point<...>>` specialisation at
/// `boost/geometry/geometries/point.hpp:241-244`.
impl<T: CoordinateScalar, const D: usize, Cs: CoordinateSystem> Geometry for Point<T, D, Cs> {
    type Kind = PointTag;
    type Point = Self;
}

/// Wire the Point concept up. Collapses the four trait specialisations
/// at `boost/geometry/geometries/point.hpp:246-299`
/// (`coordinate_type`, `coordinate_system`, `dimension`, `access`)
/// into one Rust impl.
impl<T: CoordinateScalar, const D: usize, Cs: CoordinateSystem> PointTrait for Point<T, D, Cs> {
    type Scalar = T;
    type Cs = Cs;
    const DIM: usize = D;

    #[inline]
    fn get<const DIDX: usize>(&self) -> T {
        self.coords[DIDX]
    }
}

impl<T: CoordinateScalar, const D: usize, Cs: CoordinateSystem> PointMut for Point<T, D, Cs> {
    #[inline]
    fn set<const DIDX: usize>(&mut self, value: T) {
        self.coords[DIDX] = value;
    }
}
