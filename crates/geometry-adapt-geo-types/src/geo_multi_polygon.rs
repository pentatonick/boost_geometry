//! [`MultiPolygon`] adapter for `geo_types::MultiPolygon<T>`.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). Stores the member polygons as
//! [`GeoPolygon`] so the [`MultiPolygon`] concept's `polygons()`
//! iterator can hand back `&GeoPolygon<T>`.

use alloc::vec::Vec;

use geo_types::{CoordNum, MultiPolygon};
use geometry_coords::CoordinateScalar;
use geometry_tag::MultiPolygonTag;
use geometry_trait::{Geometry, MultiPolygon as MultiPolygonTrait};

use crate::geo_coord::GeoCoord;
use crate::geo_polygon::GeoPolygon;

/// Shape-only adapter for `geo_types::MultiPolygon<T>`.
///
/// # Examples
///
/// ```
/// use geo_types::{LineString, MultiPolygon, Polygon};
/// use geometry_adapt_geo_types::GeoMultiPolygon;
/// use geometry_algorithm::multi_polygon_area;
///
/// // Two unit squares wound clockwise (matching the ring default) each
/// // contribute +1 to the signed area.
/// let square = |o: f64| Polygon::new(
///     LineString::from(vec![
///         (o, 0.0), (o, 1.0), (o + 1.0, 1.0), (o + 1.0, 0.0), (o, 0.0),
///     ]),
///     vec![],
/// );
/// let mpg = GeoMultiPolygon::new(MultiPolygon::new(vec![square(0.0), square(2.0)]));
/// assert_eq!(multi_polygon_area(&mpg), 2.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct GeoMultiPolygon<T: CoordNum>(Vec<GeoPolygon<T>>);

impl<T: CoordNum> GeoMultiPolygon<T> {
    /// Wrap a `geo_types::MultiPolygon`, copying its member polygons
    /// into wrapped [`GeoPolygon`] elements.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::{LineString, MultiPolygon, Polygon};
    /// use geometry_adapt_geo_types::GeoMultiPolygon;
    ///
    /// let poly = Polygon::new(
    ///     LineString::from(vec![(0.0_f64, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]),
    ///     vec![],
    /// );
    /// let mpg = GeoMultiPolygon::new(MultiPolygon::new(vec![poly]));
    /// assert_eq!(mpg.into_inner().0.len(), 1);
    /// ```
    #[inline]
    #[must_use]
    pub fn new(inner: MultiPolygon<T>) -> Self {
        Self(inner.0.into_iter().map(GeoPolygon::new).collect())
    }

    /// Recover the wrapped `geo_types::MultiPolygon`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::{LineString, MultiPolygon, Polygon};
    /// use geometry_adapt_geo_types::GeoMultiPolygon;
    ///
    /// let original = MultiPolygon::new(vec![Polygon::new(
    ///     LineString::from(vec![(0.0_f64, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]),
    ///     vec![],
    /// )]);
    /// let mpg = GeoMultiPolygon::new(original.clone());
    /// assert_eq!(mpg.into_inner(), original);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> MultiPolygon<T> {
        MultiPolygon::new(self.0.into_iter().map(GeoPolygon::into_inner).collect())
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoMultiPolygon<T> {
    type Kind = MultiPolygonTag;
    type Point = GeoCoord<T>;
}

impl<T: CoordinateScalar + CoordNum> MultiPolygonTrait for GeoMultiPolygon<T> {
    type ItemPolygon = GeoPolygon<T>;

    fn polygons(&self) -> impl ExactSizeIterator<Item = &GeoPolygon<T>> {
        self.0.iter()
    }
}
