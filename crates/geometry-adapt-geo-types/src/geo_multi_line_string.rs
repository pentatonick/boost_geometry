//! [`MultiLinestring`] adapter for `geo_types::MultiLineString<T>`.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). Stores the member line strings as
//! [`GeoLineString`] so the [`MultiLinestring`] concept's
//! `linestrings()` iterator can hand back `&GeoLineString<T>`.

use alloc::vec::Vec;

use geo_types::{CoordNum, MultiLineString};
use geometry_coords::CoordinateScalar;
use geometry_tag::MultiLinestringTag;
use geometry_trait::{Geometry, MultiLinestring as MultiLinestringTrait};

use crate::geo_coord::GeoCoord;
use crate::geo_line_string::GeoLineString;

/// Shape-only adapter for `geo_types::MultiLineString<T>`.
///
/// # Examples
///
/// ```
/// use geo_types::{LineString, MultiLineString};
/// use geometry_adapt_geo_types::GeoMultiLineString;
/// use geometry_trait::MultiLinestring as _;
///
/// let mls = GeoMultiLineString::new(MultiLineString::new(vec![
///     LineString::from(vec![(0.0_f64, 0.0), (1.0, 1.0)]),
///     LineString::from(vec![(2.0, 2.0), (3.0, 3.0), (4.0, 4.0)]),
/// ]));
/// assert_eq!(mls.linestrings().count(), 2);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct GeoMultiLineString<T: CoordNum>(Vec<GeoLineString<T>>);

impl<T: CoordNum> GeoMultiLineString<T> {
    /// Wrap a `geo_types::MultiLineString`, copying its member line
    /// strings into wrapped [`GeoLineString`] elements.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::{LineString, MultiLineString};
    /// use geometry_adapt_geo_types::GeoMultiLineString;
    ///
    /// let mls = GeoMultiLineString::new(MultiLineString::new(vec![
    ///     LineString::from(vec![(0.0_f64, 0.0), (1.0, 1.0)]),
    /// ]));
    /// assert_eq!(mls.into_inner().0.len(), 1);
    /// ```
    #[inline]
    #[must_use]
    pub fn new(inner: MultiLineString<T>) -> Self {
        Self(inner.0.into_iter().map(GeoLineString::new).collect())
    }

    /// Recover the wrapped `geo_types::MultiLineString`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::{LineString, MultiLineString};
    /// use geometry_adapt_geo_types::GeoMultiLineString;
    ///
    /// let original = MultiLineString::new(vec![
    ///     LineString::from(vec![(0.0_f64, 0.0), (1.0, 1.0)]),
    /// ]);
    /// let mls = GeoMultiLineString::new(original.clone());
    /// assert_eq!(mls.into_inner(), original);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> MultiLineString<T> {
        MultiLineString::new(self.0.into_iter().map(GeoLineString::into_inner).collect())
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoMultiLineString<T> {
    type Kind = MultiLinestringTag;
    type Point = GeoCoord<T>;
}

impl<T: CoordinateScalar + CoordNum> MultiLinestringTrait for GeoMultiLineString<T> {
    type ItemLinestring = GeoLineString<T>;

    fn linestrings(&self) -> impl ExactSizeIterator<Item = &GeoLineString<T>> {
        self.0.iter()
    }
}
