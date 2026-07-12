//! [`Point`] adapters for `nalgebra::Point2` / `nalgebra::Point3`.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*;
//! Boost has no `nalgebra` analogue). The orphan rule forbids
//! `impl geometry_trait::Point for nalgebra::Point2<T>` directly —
//! both trait and type are foreign — so the foreign point is wrapped
//! in a local `#[repr(transparent)]` newtype and the concepts hang
//! off the wrapper, exactly as `geometry_adapt::Adapt` does for raw
//! arrays and tuples.
//!
//! Both wrappers pin `Cs = Cartesian`; `nalgebra` points are
//! read-write, so each implements [`PointMut`] too. Coordinate access
//! goes through `nalgebra`'s `Index<usize>` impl (`self.0[D]`).

use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut};
use nalgebra::{Point2, Point3, Scalar};

/// Shape-only adapter for `nalgebra::Point2<T>`.
///
/// `#[repr(transparent)]` so `&NaPoint2<T>` is layout-compatible with
/// `&nalgebra::Point2<T>` — the wrapper is purely a place to hang the
/// [`Point`] / [`PointMut`] impls that coherence forbids on the
/// foreign type. Pins `Cs = Cartesian`, `DIM = 2`.
///
/// # Examples
///
/// ```
/// use geometry_adapt_nalgebra::NaPoint2;
/// use geometry_algorithm::distance;
/// use nalgebra::Point2;
///
/// let a = NaPoint2::new(Point2::new(0.0_f64, 0.0));
/// let b = NaPoint2::new(Point2::new(3.0_f64, 4.0));
/// assert_eq!(distance(&a, &b), 5.0);
/// ```
#[repr(transparent)]
pub struct NaPoint2<T: Scalar>(pub Point2<T>);

impl<T: Scalar> NaPoint2<T> {
    /// Wrap a `nalgebra::Point2` so the geometry concepts apply to it.
    #[inline]
    #[must_use]
    pub const fn new(inner: Point2<T>) -> Self {
        Self(inner)
    }

    /// Unwrap, returning the `nalgebra::Point2`.
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> Point2<T> {
        self.0
    }
}

impl<T: CoordinateScalar + Scalar> Geometry for NaPoint2<T> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar + Scalar> Point for NaPoint2<T> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = 2;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        self.0[D]
    }
}

impl<T: CoordinateScalar + Scalar> PointMut for NaPoint2<T> {
    #[inline]
    fn set<const D: usize>(&mut self, value: T) {
        self.0[D] = value;
    }
}

/// Shape-only adapter for `nalgebra::Point3<T>`.
///
/// `#[repr(transparent)]` so `&NaPoint3<T>` is layout-compatible with
/// `&nalgebra::Point3<T>`. Pins `Cs = Cartesian`, `DIM = 3`.
///
/// # Examples
///
/// ```
/// use geometry_adapt_nalgebra::NaPoint3;
/// use geometry_algorithm::distance;
/// use nalgebra::Point3;
///
/// let origin = NaPoint3::new(Point3::new(0.0_f64, 0.0, 0.0));
/// let x_axis = NaPoint3::new(Point3::new(1.0_f64, 0.0, 0.0));
/// assert_eq!(distance(&origin, &x_axis), 1.0);
/// ```
#[repr(transparent)]
pub struct NaPoint3<T: Scalar>(pub Point3<T>);

impl<T: Scalar> NaPoint3<T> {
    /// Wrap a `nalgebra::Point3` so the geometry concepts apply to it.
    #[inline]
    #[must_use]
    pub const fn new(inner: Point3<T>) -> Self {
        Self(inner)
    }

    /// Unwrap, returning the `nalgebra::Point3`.
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> Point3<T> {
        self.0
    }
}

impl<T: CoordinateScalar + Scalar> Geometry for NaPoint3<T> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar + Scalar> Point for NaPoint3<T> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = 3;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        self.0[D]
    }
}

impl<T: CoordinateScalar + Scalar> PointMut for NaPoint3<T> {
    #[inline]
    fn set<const D: usize>(&mut self, value: T) {
        self.0[D] = value;
    }
}
