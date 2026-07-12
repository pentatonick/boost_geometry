//! [`Ring`] adapter for `geo_types::LineString<T>`.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). `geo-types` has no separate `Ring`
//! type — a ring is spelled as a `LineString` that the caller keeps
//! closed and correctly oriented (the OGC `LinearRing` convention, see
//! `geo_types::LineString`'s own docs). This wrapper therefore adapts a
//! `geo_types::LineString<T>` to the [`Ring`] concept, and inherits the
//! concept's Boost-matching defaults `closure() = Closed`,
//! `point_order() = Clockwise`. **The caller asserts closure and
//! orientation** — this wrapper does not validate or re-wind the ring.
//!
//! As with [`GeoLineString`](crate::GeoLineString), the vertices are
//! stored as [`GeoCoord`] wrappers so `points()` can yield
//! `&GeoCoord<T>` under `#![forbid(unsafe_code)]`.

use alloc::vec::Vec;

use geo_types::{CoordNum, LineString};
use geometry_coords::CoordinateScalar;
use geometry_tag::RingTag;
use geometry_trait::{Geometry, Ring};

use crate::geo_coord::GeoCoord;

/// Shape-only adapter presenting a `geo_types::LineString<T>` as a
/// [`Ring`].
///
/// `geo-types` does not distinguish rings from line strings; the
/// caller is responsible for supplying a closed, correctly-oriented
/// ring. Inherits the concept defaults `closure() = Closed`,
/// `point_order() = Clockwise`.
///
/// **Winding caveat.** This default is `Clockwise` (Boost's convention),
/// but `geo-types`' own convention winds an exterior ring
/// *counter-clockwise*. The wrapper does not re-wind, so a `geo-types`-
/// native ring reaches signed-area functions (`ring_area`, `area`) with
/// the opposite sign — a CCW exterior yields a *negative* area here.
/// Orientation-independent queries (`within`, `centroid`, `envelope`,
/// `perimeter`) are unaffected. Re-wind with `geometry_algorithm::correct`
/// (or geo-types' own `orient`) first if you need Boost's sign.
///
/// # Examples
///
/// ```
/// use geo_types::LineString;
/// use geometry_adapt_geo_types::GeoRing;
/// use geometry_algorithm::ring_area;
///
/// // Vertices matching the ring's declared clockwise order give a
/// // positive signed area of 1 for a unit square.
/// let ring = GeoRing::new(LineString::from(vec![
///     (0.0_f64, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0),
/// ]));
/// assert_eq!(ring_area(&ring), 1.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct GeoRing<T: CoordNum>(Vec<GeoCoord<T>>);

impl<T: CoordNum> GeoRing<T> {
    /// Wrap a `geo_types::LineString` as a ring, copying its
    /// coordinates into wrapped [`GeoCoord`] vertices.
    ///
    /// The caller asserts the line string is a valid ring (closed and
    /// correctly oriented); this constructor performs no validation.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::LineString;
    /// use geometry_adapt_geo_types::GeoRing;
    ///
    /// let ring = GeoRing::new(LineString::from(vec![
    ///     (0.0_f64, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0),
    /// ]));
    /// assert_eq!(ring.into_inner().0.len(), 4);
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
    /// use geometry_adapt_geo_types::GeoRing;
    ///
    /// let original = LineString::from(vec![
    ///     (0.0_f64, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0),
    /// ]);
    /// let ring = GeoRing::new(original.clone());
    /// assert_eq!(ring.into_inner(), original);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> LineString<T> {
        LineString(self.0.into_iter().map(GeoCoord::into_inner).collect())
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoRing<T> {
    type Kind = RingTag;
    type Point = GeoCoord<T>;
}

impl<T: CoordinateScalar + CoordNum> Ring for GeoRing<T> {
    fn points(&self) -> impl ExactSizeIterator<Item = &GeoCoord<T>> + Clone {
        self.0.iter()
    }
    // Inherit concept defaults: closure() = Closed, point_order() = Clockwise.
}
