//! [`Point`] adapters for `nalgebra::Vector2` / `nalgebra::Vector3`.
//!
//! `nalgebra` vectors are `(x, y[, z])` in memory just like its
//! points, so they adapt to the [`Point`] concept with the same
//! shape as [`crate::NaPoint2`] / [`crate::NaPoint3`]. As with the
//! points, the orphan rule forbids implementing the foreign
//! [`geometry_trait::Point`] for the foreign `nalgebra::Vector2`
//! directly, so each is wrapped in a local `#[repr(transparent)]`
//! newtype. Coordinate access goes through `nalgebra`'s
//! `Index<usize>` impl (`self.0[D]`).

use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut};
use nalgebra::{Scalar, Vector2, Vector3};

/// Shape-only adapter for `nalgebra::Vector2<T>`.
///
/// `#[repr(transparent)]` so `&NaVector2<T>` is layout-compatible
/// with `&nalgebra::Vector2<T>`. Pins `Cs = Cartesian`, `DIM = 2`.
///
/// # Examples
///
/// ```
/// use geometry_adapt_nalgebra::NaVector2;
/// use geometry_algorithm::distance;
/// use nalgebra::Vector2;
///
/// let a = NaVector2::new(Vector2::new(0.0_f64, 0.0));
/// let b = NaVector2::new(Vector2::new(3.0_f64, 4.0));
/// assert_eq!(distance(&a, &b), 5.0);
/// ```
#[repr(transparent)]
pub struct NaVector2<T: Scalar>(pub Vector2<T>);

impl<T: Scalar> NaVector2<T> {
    /// Wrap a `nalgebra::Vector2` so the geometry concepts apply to it.
    #[inline]
    #[must_use]
    pub const fn new(inner: Vector2<T>) -> Self {
        Self(inner)
    }

    /// Unwrap, returning the `nalgebra::Vector2`.
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> Vector2<T> {
        self.0
    }
}

impl<T: CoordinateScalar + Scalar> Geometry for NaVector2<T> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar + Scalar> Point for NaVector2<T> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = 2;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        self.0[D]
    }
}

impl<T: CoordinateScalar + Scalar> PointMut for NaVector2<T> {
    #[inline]
    fn set<const D: usize>(&mut self, value: T) {
        self.0[D] = value;
    }
}

/// Shape-only adapter for `nalgebra::Vector3<T>`.
///
/// `#[repr(transparent)]` so `&NaVector3<T>` is layout-compatible
/// with `&nalgebra::Vector3<T>`. Pins `Cs = Cartesian`, `DIM = 3`.
///
/// # Examples
///
/// ```
/// use geometry_adapt_nalgebra::NaVector3;
/// use geometry_algorithm::distance;
/// use nalgebra::Vector3;
///
/// let origin = NaVector3::new(Vector3::new(0.0_f64, 0.0, 0.0));
/// let z_axis = NaVector3::new(Vector3::new(0.0_f64, 0.0, 1.0));
/// assert_eq!(distance(&origin, &z_axis), 1.0);
/// ```
#[repr(transparent)]
pub struct NaVector3<T: Scalar>(pub Vector3<T>);

impl<T: Scalar> NaVector3<T> {
    /// Wrap a `nalgebra::Vector3` so the geometry concepts apply to it.
    #[inline]
    #[must_use]
    pub const fn new(inner: Vector3<T>) -> Self {
        Self(inner)
    }

    /// Unwrap, returning the `nalgebra::Vector3`.
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> Vector3<T> {
        self.0
    }
}

impl<T: CoordinateScalar + Scalar> Geometry for NaVector3<T> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar + Scalar> Point for NaVector3<T> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = 3;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        self.0[D]
    }
}

impl<T: CoordinateScalar + Scalar> PointMut for NaVector3<T> {
    #[inline]
    fn set<const D: usize>(&mut self, value: T) {
        self.0[D] = value;
    }
}
