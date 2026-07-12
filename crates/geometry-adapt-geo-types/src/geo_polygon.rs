//! [`Polygon`] adapter for `geo_types::Polygon<T>`.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). A `geo_types::Polygon` is an exterior
//! `LineString` plus a `Vec` of interior `LineString`s; this wrapper
//! adapts both to [`GeoRing`], so the [`Polygon`] concept can hand back
//! `&GeoRing<T>` for the exterior and an `ExactSizeIterator` of the
//! interiors.

use alloc::vec::Vec;

use geo_types::{CoordNum, Polygon};
use geometry_coords::CoordinateScalar;
use geometry_tag::PolygonTag;
use geometry_trait::{Geometry, Polygon as PolygonTrait};

use crate::geo_coord::GeoCoord;
use crate::geo_ring::GeoRing;

/// Shape-only adapter for `geo_types::Polygon<T>`.
///
/// Stores the exterior and interior rings as [`GeoRing<T>`] so the
/// [`Polygon`](geometry_trait::Polygon) concept's accessors can return
/// references into wrapped storage. Construct with [`new`](Self::new)
/// from a `geo_types::Polygon`; recover it with
/// [`into_inner`](Self::into_inner).
///
/// # Examples
///
/// ```
/// use geo_types::{LineString, Polygon};
/// use geometry_adapt_geo_types::GeoPolygon;
/// use geometry_algorithm::area;
///
/// let exterior = LineString::from(vec![
///     (0.0_f64, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0),
/// ]);
/// let poly = GeoPolygon::new(Polygon::new(exterior, vec![]));
/// assert_eq!(area(&poly), 16.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct GeoPolygon<T: CoordNum> {
    exterior: GeoRing<T>,
    interiors: Vec<GeoRing<T>>,
}

impl<T: CoordNum> GeoPolygon<T> {
    /// Wrap a `geo_types::Polygon`, copying its exterior and interior
    /// rings into wrapped [`GeoRing`] storage.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::{LineString, Polygon};
    /// use geometry_adapt_geo_types::GeoPolygon;
    ///
    /// let exterior = LineString::from(vec![
    ///     (0.0_f64, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0),
    /// ]);
    /// let poly = GeoPolygon::new(Polygon::new(exterior, vec![]));
    /// assert_eq!(poly.into_inner().interiors().len(), 0);
    /// ```
    #[inline]
    #[must_use]
    pub fn new(inner: Polygon<T>) -> Self {
        let (exterior, interiors) = inner.into_inner();
        Self {
            exterior: GeoRing::new(exterior),
            interiors: interiors.into_iter().map(GeoRing::new).collect(),
        }
    }

    /// Recover the wrapped `geo_types::Polygon`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::{LineString, Polygon};
    /// use geometry_adapt_geo_types::GeoPolygon;
    ///
    /// let exterior = LineString::from(vec![
    ///     (0.0_f64, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0),
    /// ]);
    /// let original = Polygon::new(exterior, vec![]);
    /// let poly = GeoPolygon::new(original.clone());
    /// assert_eq!(poly.into_inner(), original);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> Polygon<T> {
        Polygon::new(
            self.exterior.into_inner(),
            self.interiors
                .into_iter()
                .map(GeoRing::into_inner)
                .collect(),
        )
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoPolygon<T> {
    type Kind = PolygonTag;
    type Point = GeoCoord<T>;
}

impl<T: CoordinateScalar + CoordNum> PolygonTrait for GeoPolygon<T> {
    type Ring = GeoRing<T>;

    fn exterior(&self) -> &GeoRing<T> {
        &self.exterior
    }

    fn interiors(&self) -> impl ExactSizeIterator<Item = &GeoRing<T>> {
        self.interiors.iter()
    }
}
