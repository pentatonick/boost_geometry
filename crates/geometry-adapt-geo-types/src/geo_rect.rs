//! [`Box`](geometry_trait::Box) adapter for `geo_types::Rect<T>` — an
//! axis-aligned bounding box.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). A `geo_types::Rect` is described by a
//! minimum and maximum corner; this wrapper adapts it to the
//! [`Box`](geometry_trait::Box) concept via [`IndexedAccess`]
//! (`corner::MIN` = min corner, `corner::MAX` = max corner).
//!
//! The two corners are stored inline as `[GeoCoord<T>; 2]` rather than
//! as a live `geo_types::Rect`, so per-dimension `set_indexed` writes
//! never transiently violate `Rect`'s min-≤-max invariant; the
//! invariant is re-established when [`into_inner`](GeoRect::into_inner)
//! reconstructs the `Rect`.

use geo_types::{CoordNum, Rect, coord};
use geometry_coords::CoordinateScalar;
use geometry_tag::BoxTag;
use geometry_trait::{Box as BoxTrait, Geometry, IndexedAccess};

use crate::geo_coord::GeoCoord;

/// Shape-only adapter for `geo_types::Rect<T>`.
///
/// Corner `corner::MIN` is the rectangle's minimum, `corner::MAX`
/// its maximum; `get_indexed::<I, 0>` reads `x` and `<I, 1>` reads `y`.
/// The point type is [`GeoCoord<T>`], matching the container wrappers.
///
/// # Examples
///
/// ```
/// use geo_types::Rect;
/// use geometry_adapt_geo_types::GeoRect;
/// use geometry_trait::{box_min, box_max, Point};
///
/// let rect = GeoRect::new(Rect::new((0.0_f64, 0.0), (3.0, 4.0)));
/// assert_eq!(box_min(&rect).get::<0>(), 0.0);
/// assert_eq!(box_max(&rect).get::<1>(), 4.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoRect<T: CoordNum> {
    corners: [GeoCoord<T>; 2],
}

impl<T: CoordNum> GeoRect<T> {
    /// Wrap a `geo_types::Rect` so the [`Box`](geometry_trait::Box)
    /// concept applies.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::Rect;
    /// use geometry_adapt_geo_types::GeoRect;
    ///
    /// let rect = GeoRect::new(Rect::new((0.0_f64, 0.0), (2.0, 2.0)));
    /// assert_eq!(rect.into_inner().min().x, 0.0);
    /// ```
    #[inline]
    #[must_use]
    pub fn new(inner: Rect<T>) -> Self {
        Self {
            corners: [GeoCoord::new(inner.min()), GeoCoord::new(inner.max())],
        }
    }

    /// Recover the wrapped `geo_types::Rect`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::Rect;
    /// use geometry_adapt_geo_types::GeoRect;
    ///
    /// let original = Rect::new((0.0_f64, 0.0), (3.0, 4.0));
    /// let rect = GeoRect::new(original);
    /// assert_eq!(rect.into_inner(), original);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> Rect<T> {
        Rect::new(self.corners[0].into_inner(), self.corners[1].into_inner())
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoRect<T> {
    type Kind = BoxTag;
    type Point = GeoCoord<T>;
}

impl<T: CoordinateScalar + CoordNum> IndexedAccess for GeoRect<T> {
    #[inline]
    fn get_indexed<const I: usize, const D: usize>(&self) -> T {
        let c = self.corners[I].0;
        if D == 0 { c.x } else { c.y }
    }

    #[inline]
    fn set_indexed<const I: usize, const D: usize>(&mut self, value: T) {
        let c = &mut self.corners[I].0;
        if D == 0 {
            *c = coord! { x: value, y: c.y };
        } else {
            *c = coord! { x: c.x, y: value };
        }
    }
}

impl<T: CoordinateScalar + CoordNum> BoxTrait for GeoRect<T> {}
