//! [`MultiPoint`] adapter for `geo_types::MultiPoint<T>`.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). A `geo_types::MultiPoint` stores a
//! `Vec<geo_types::Point>`; this wrapper stores the elements as
//! [`GeoPoint`] so the [`MultiPoint`] concept's `points()` iterator can
//! hand back `&GeoPoint<T>` under `#![forbid(unsafe_code)]`.

use alloc::vec::Vec;

use geo_types::{CoordNum, MultiPoint, Point};
use geometry_coords::CoordinateScalar;
use geometry_tag::MultiPointTag;
use geometry_trait::{Geometry, MultiPoint as MultiPointTrait};

use crate::geo_point::GeoPoint;

/// Shape-only adapter for `geo_types::MultiPoint<T>`.
///
/// # Examples
///
/// ```
/// use geo_types::MultiPoint;
/// use geometry_adapt_geo_types::GeoMultiPoint;
/// use geometry_trait::MultiPoint as _;
///
/// let mp = GeoMultiPoint::new(MultiPoint::from(vec![(0.0_f64, 0.0), (1.0, 1.0)]));
/// assert_eq!(mp.points().count(), 2);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct GeoMultiPoint<T: CoordNum>(Vec<GeoPoint<T>>);

impl<T: CoordNum> GeoMultiPoint<T> {
    /// Wrap a `geo_types::MultiPoint`, copying its points into wrapped
    /// [`GeoPoint`] elements.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::MultiPoint;
    /// use geometry_adapt_geo_types::GeoMultiPoint;
    ///
    /// let mp = GeoMultiPoint::new(MultiPoint::from(vec![(0.0_f64, 0.0)]));
    /// assert_eq!(mp.into_inner().0.len(), 1);
    /// ```
    #[inline]
    #[must_use]
    pub fn new(inner: MultiPoint<T>) -> Self {
        Self(inner.0.into_iter().map(GeoPoint::new).collect())
    }

    /// Recover the wrapped `geo_types::MultiPoint`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::MultiPoint;
    /// use geometry_adapt_geo_types::GeoMultiPoint;
    ///
    /// let original = MultiPoint::from(vec![(0.0_f64, 0.0), (1.0, 1.0)]);
    /// let mp = GeoMultiPoint::new(original.clone());
    /// assert_eq!(mp.into_inner(), original);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> MultiPoint<T> {
        MultiPoint(
            self.0
                .into_iter()
                .map(GeoPoint::into_inner)
                .collect::<Vec<Point<T>>>(),
        )
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoMultiPoint<T> {
    type Kind = MultiPointTag;
    type Point = GeoPoint<T>;
}

impl<T: CoordinateScalar + CoordNum> MultiPointTrait for GeoMultiPoint<T> {
    type ItemPoint = GeoPoint<T>;

    fn points(&self) -> impl ExactSizeIterator<Item = &GeoPoint<T>> {
        self.0.iter()
    }
}
