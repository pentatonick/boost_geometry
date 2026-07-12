//! [`GeometryCollection`] adapter for `geo_types::GeometryCollection<T>`.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). A `geo_types::GeometryCollection` is a
//! `Vec` of heterogeneous `geo_types::Geometry` values; the kernel's
//! [`GeometryCollection`] concept requires a single element type whose
//! items are yielded by reference. This wrapper therefore stores the
//! members as [`DynGeometry<T, Cartesian>`](geometry_model::DynGeometry)
//! — the kernel's runtime-tagged geometry — converting each element on
//! construction via the [`From`] impls in
//! [`crate::dyn_conversion`]. `items()` then hands back
//! `&DynGeometry<T, Cartesian>`.
//!
//! Because construction routes through those conversions, the `Line` /
//! `Rect` / `Triangle` kind-normalisation documented on
//! [`crate::dyn_conversion`] applies here too.

use alloc::vec::Vec;

use geo_types::{CoordNum, GeometryCollection};
use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_model::DynGeometry;
use geometry_tag::GeometryCollectionTag;
use geometry_trait::{Geometry, GeometryCollection as GeometryCollectionTrait};

use crate::dyn_conversion::{from_dyn_geometry, to_dyn_geometry};

/// Shape-only adapter for `geo_types::GeometryCollection<T>`.
///
/// Stores the members as [`DynGeometry<T, Cartesian>`] so the
/// [`GeometryCollection`](geometry_trait::GeometryCollection) concept's
/// `items()` iterator can hand back `&DynGeometry<T, Cartesian>`.
///
/// # Examples
///
/// ```
/// use geo_types::{Geometry, GeometryCollection, Point};
/// use geometry_adapt_geo_types::GeoCollection;
/// use geometry_trait::GeometryCollection as _;
///
/// let gc = GeoCollection::new(GeometryCollection(vec![
///     Geometry::Point(Point::new(0.0_f64, 0.0)),
///     Geometry::Point(Point::new(1.0, 1.0)),
/// ]));
/// assert_eq!(gc.items().count(), 2);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct GeoCollection<T: CoordinateScalar + CoordNum>(Vec<DynGeometry<T, Cartesian>>);

impl<T: CoordinateScalar + CoordNum> GeoCollection<T> {
    /// Wrap a `geo_types::GeometryCollection`, converting each member
    /// into a kernel [`DynGeometry`].
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::{Geometry, GeometryCollection, Point};
    /// use geometry_adapt_geo_types::GeoCollection;
    ///
    /// let gc = GeoCollection::new(GeometryCollection(vec![
    ///     Geometry::Point(Point::new(0.0_f64, 0.0)),
    /// ]));
    /// assert_eq!(gc.into_inner().0.len(), 1);
    /// ```
    #[inline]
    #[must_use]
    pub fn new(inner: GeometryCollection<T>) -> Self {
        Self(inner.0.into_iter().map(to_dyn_geometry).collect())
    }

    /// Recover a `geo_types::GeometryCollection`, converting each
    /// member back from its kernel [`DynGeometry`] form.
    ///
    /// Subject to the `Line` / `Rect` / `Triangle` kind-normalisation
    /// documented on [`crate::dyn_conversion`].
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::{Geometry, GeometryCollection, Point};
    /// use geometry_adapt_geo_types::GeoCollection;
    ///
    /// let original = GeometryCollection(vec![Geometry::Point(Point::new(1.0_f64, 2.0))]);
    /// let gc = GeoCollection::new(original.clone());
    /// assert_eq!(gc.into_inner(), original);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> GeometryCollection<T> {
        GeometryCollection(self.0.into_iter().map(from_dyn_geometry).collect())
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoCollection<T> {
    type Kind = GeometryCollectionTag;
    type Point = geometry_model::Point2D<T, Cartesian>;
}

impl<T: CoordinateScalar + CoordNum> GeometryCollectionTrait for GeoCollection<T> {
    type Item = DynGeometry<T, Cartesian>;

    fn items(&self) -> impl ExactSizeIterator<Item = &DynGeometry<T, Cartesian>> {
        self.0.iter()
    }
}
