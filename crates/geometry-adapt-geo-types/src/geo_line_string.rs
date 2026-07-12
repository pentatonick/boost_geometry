//! [`Linestring`] adapter for `geo_types::LineString<T>`.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). The [`Linestring`] concept's
//! `points()` method yields `&Self::Point`, so the elements must
//! physically exist as [`GeoCoord`] wrappers. Because this crate is
//! `#![forbid(unsafe_code)]` it cannot re-cast `&geo_types::Coord` to
//! `&GeoCoord`; instead the wrapper stores its own `Vec<GeoCoord<T>>`,
//! converting element-by-element on construction (both coordinate
//! types are `Copy`, so this is a cheap ordinate copy). Round-tripping
//! back to `geo_types::LineString<T>` via [`into_inner`](GeoLineString::into_inner)
//! is lossless.

use alloc::vec::Vec;

use geo_types::{CoordNum, LineString};
use geometry_coords::CoordinateScalar;
use geometry_tag::LinestringTag;
use geometry_trait::{Geometry, Linestring};

use crate::geo_coord::GeoCoord;

/// Shape-only adapter for `geo_types::LineString<T>`.
///
/// Stores the vertices as [`GeoCoord<T>`] so the [`Linestring`]
/// concept's `points()` iterator can hand back `&GeoCoord<T>`
/// references. Construct with [`new`](Self::new) from a
/// `geo_types::LineString`; recover the original with
/// [`into_inner`](Self::into_inner).
///
/// # Examples
///
/// ```
/// use geo_types::LineString;
/// use geometry_adapt_geo_types::GeoLineString;
/// use geometry_algorithm::length;
///
/// let ls = GeoLineString::new(LineString::from(vec![(0.0_f64, 0.0), (3.0, 4.0)]));
/// assert_eq!(length(&ls), 5.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct GeoLineString<T: CoordNum>(Vec<GeoCoord<T>>);

impl<T: CoordNum> GeoLineString<T> {
    /// Wrap a `geo_types::LineString`, copying its coordinates into
    /// wrapped [`GeoCoord`] vertices.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::LineString;
    /// use geometry_adapt_geo_types::GeoLineString;
    ///
    /// let ls = GeoLineString::new(LineString::from(vec![(0.0_f64, 0.0), (1.0, 1.0)]));
    /// assert_eq!(ls.into_inner().0.len(), 2);
    /// ```
    #[inline]
    #[must_use]
    pub fn new(inner: LineString<T>) -> Self {
        Self(inner.0.into_iter().map(GeoCoord::new).collect())
    }

    /// Recover the wrapped `geo_types::LineString`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::LineString;
    /// use geometry_adapt_geo_types::GeoLineString;
    ///
    /// let original = LineString::from(vec![(0.0_f64, 0.0), (1.0, 1.0)]);
    /// let ls = GeoLineString::new(original.clone());
    /// assert_eq!(ls.into_inner(), original);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> LineString<T> {
        LineString(self.0.into_iter().map(GeoCoord::into_inner).collect())
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoLineString<T> {
    type Kind = LinestringTag;
    type Point = GeoCoord<T>;
}

impl<T: CoordinateScalar + CoordNum> Linestring for GeoLineString<T> {
    fn points(&self) -> impl ExactSizeIterator<Item = &GeoCoord<T>> + Clone {
        self.0.iter()
    }
}
