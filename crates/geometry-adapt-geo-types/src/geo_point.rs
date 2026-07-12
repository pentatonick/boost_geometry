//! [`Point`] adapter for `geo_types::Point<T>` — the newtype `geo-types`
//! wraps around a single [`Coord`](geo_types::Coord).
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). As with [`GeoCoord`](crate::GeoCoord),
//! the orphan rule forbids implementing the foreign
//! [`geometry_trait::Point`] for the foreign `geo_types::Point<T>`
//! directly, so the value is wrapped in a local
//! `#[repr(transparent)]` newtype and the concepts hang off the
//! wrapper.
//!
//! The wrapper pins `Cs = Cartesian`, `DIM = 2`. `geo_types::Point` is
//! read-write, so [`PointMut`] is implemented too.

use core::ops::{Deref, DerefMut};

use geo_types::{CoordNum, Point as GtPoint};
use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut};

/// Shape-only adapter for `geo_types::Point<T>`.
///
/// `#[repr(transparent)]` so `GeoPoint<T>` shares the in-memory layout
/// of the underlying `geo_types::Point<T>`. Pins `Cs = Cartesian`,
/// `DIM = 2`; `get::<0>` is `x`, `get::<1>` is `y`.
///
/// # Examples
///
/// ```
/// use geo_types::Point;
/// use geometry_adapt_geo_types::GeoPoint;
/// use geometry_algorithm::distance;
///
/// let a = GeoPoint::new(Point::new(0.0_f64, 0.0));
/// let b = GeoPoint::new(Point::new(3.0_f64, 4.0));
/// assert_eq!(distance(&a, &b), 5.0);
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[repr(transparent)]
pub struct GeoPoint<T: CoordNum>(pub GtPoint<T>);

impl<T: CoordNum> GeoPoint<T> {
    /// Wrap a `geo_types::Point` so the geometry concepts apply to it.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::Point;
    /// use geometry_adapt_geo_types::GeoPoint;
    ///
    /// let p = GeoPoint::new(Point::new(1.0_f64, 2.0));
    /// assert_eq!(p.into_inner().x(), 1.0);
    /// ```
    #[inline]
    #[must_use]
    pub const fn new(inner: GtPoint<T>) -> Self {
        Self(inner)
    }

    /// Unwrap, returning the `geo_types::Point`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::Point;
    /// use geometry_adapt_geo_types::GeoPoint;
    ///
    /// let p = GeoPoint::new(Point::new(1.0_f64, 2.0));
    /// assert_eq!(p.into_inner().y(), 2.0);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> GtPoint<T> {
        self.0
    }
}

impl<T: CoordNum> Deref for GeoPoint<T> {
    type Target = GtPoint<T>;

    #[inline]
    fn deref(&self) -> &GtPoint<T> {
        &self.0
    }
}

impl<T: CoordNum> DerefMut for GeoPoint<T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut GtPoint<T> {
        &mut self.0
    }
}

impl<T: CoordNum> From<GtPoint<T>> for GeoPoint<T> {
    #[inline]
    fn from(inner: GtPoint<T>) -> Self {
        Self(inner)
    }
}

impl<T: CoordNum> From<GeoPoint<T>> for GtPoint<T> {
    #[inline]
    fn from(wrapper: GeoPoint<T>) -> Self {
        wrapper.0
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoPoint<T> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar + CoordNum> Point for GeoPoint<T> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = 2;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        if D == 0 { self.0.0.x } else { self.0.0.y }
    }
}

impl<T: CoordinateScalar + CoordNum> PointMut for GeoPoint<T> {
    #[inline]
    fn set<const D: usize>(&mut self, value: T) {
        if D == 0 {
            self.0.0.x = value;
        } else {
            self.0.0.y = value;
        }
    }
}
