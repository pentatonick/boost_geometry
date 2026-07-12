//! [`Point`] adapter for `geo_types::Coord<T>` — the `(x, y)` ordinate
//! pair that every `geo-types` container is built from.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). The orphan rule forbids
//! `impl geometry_trait::Point for geo_types::Coord<T>` directly —
//! both trait and type are foreign to this crate — so the foreign
//! coordinate is wrapped in a local `#[repr(transparent)]` newtype and
//! the concepts hang off the wrapper, exactly as
//! [`geometry_adapt::Adapt`](https://docs.rs/geometry-adapt) does for
//! raw arrays and tuples.
//!
//! The wrapper pins `Cs = Cartesian`, `DIM = 2`. `geo_types::Coord` is
//! read-write, so [`PointMut`] is implemented too.

use core::ops::{Deref, DerefMut};

use geo_types::{Coord, CoordNum};
use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut};

/// Shape-only adapter for `geo_types::Coord<T>`.
///
/// `#[repr(transparent)]` so `GeoCoord<T>` shares the in-memory layout
/// of the underlying `geo_types::Coord<T>` — the wrapper is purely a
/// place to hang the [`Point`] / [`PointMut`] impls that coherence
/// forbids on the foreign type. Pins `Cs = Cartesian`, `DIM = 2`;
/// `get::<0>` is `x`, `get::<1>` is `y`.
///
/// # Examples
///
/// ```
/// use geo_types::coord;
/// use geometry_adapt_geo_types::GeoCoord;
/// use geometry_algorithm::distance;
///
/// let a = GeoCoord::new(coord! { x: 0.0_f64, y: 0.0 });
/// let b = GeoCoord::new(coord! { x: 3.0_f64, y: 4.0 });
/// assert_eq!(distance(&a, &b), 5.0);
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[repr(transparent)]
pub struct GeoCoord<T: CoordNum>(pub Coord<T>);

impl<T: CoordNum> GeoCoord<T> {
    /// Wrap a `geo_types::Coord` so the geometry concepts apply to it.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::coord;
    /// use geometry_adapt_geo_types::GeoCoord;
    ///
    /// let c = GeoCoord::new(coord! { x: 1.0_f64, y: 2.0 });
    /// assert_eq!(c.into_inner().x, 1.0);
    /// ```
    #[inline]
    #[must_use]
    pub const fn new(inner: Coord<T>) -> Self {
        Self(inner)
    }

    /// Unwrap, returning the `geo_types::Coord`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::coord;
    /// use geometry_adapt_geo_types::GeoCoord;
    ///
    /// let c = GeoCoord::new(coord! { x: 1.0_f64, y: 2.0 });
    /// assert_eq!(c.into_inner().y, 2.0);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> Coord<T> {
        self.0
    }
}

impl<T: CoordNum> Deref for GeoCoord<T> {
    type Target = Coord<T>;

    #[inline]
    fn deref(&self) -> &Coord<T> {
        &self.0
    }
}

impl<T: CoordNum> DerefMut for GeoCoord<T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Coord<T> {
        &mut self.0
    }
}

impl<T: CoordNum> From<Coord<T>> for GeoCoord<T> {
    #[inline]
    fn from(inner: Coord<T>) -> Self {
        Self(inner)
    }
}

impl<T: CoordNum> From<GeoCoord<T>> for Coord<T> {
    #[inline]
    fn from(wrapper: GeoCoord<T>) -> Self {
        wrapper.0
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoCoord<T> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar + CoordNum> Point for GeoCoord<T> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = 2;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        if D == 0 { self.0.x } else { self.0.y }
    }
}

impl<T: CoordinateScalar + CoordNum> PointMut for GeoCoord<T> {
    #[inline]
    fn set<const D: usize>(&mut self, value: T) {
        if D == 0 {
            self.0.x = value;
        } else {
            self.0.y = value;
        }
    }
}
