//! [`Segment`] adapter for `geo_types::Line<T>` — a two-point segment.
//!
//! Mirrors `boost/geometry/geometries/adapted/` (the *pattern*; Boost
//! has no `geo-types` analogue). A `geo_types::Line` carries a `start`
//! and `end` [`Coord`](geo_types::Coord); this wrapper adapts it to the
//! [`Segment`] concept, which is reached through [`IndexedAccess`]
//! (endpoint index `0` = start, `1` = end).

use geo_types::{CoordNum, Line};
use geometry_coords::CoordinateScalar;
use geometry_tag::SegmentTag;
use geometry_trait::{Geometry, IndexedAccess, Segment};

use crate::geo_coord::GeoCoord;

/// Shape-only adapter for `geo_types::Line<T>`.
///
/// Endpoint `0` is `start`, endpoint `1` is `end`; `get_indexed::<I,
/// 0>` reads `x` and `<I, 1>` reads `y`. The point type is
/// [`GeoCoord<T>`], matching the container wrappers.
///
/// # Examples
///
/// ```
/// use geo_types::Line;
/// use geometry_adapt_geo_types::GeoLine;
/// use geometry_trait::{segment_start, segment_end, Point};
///
/// let line = GeoLine::new(Line::new((0.0_f64, 0.0), (3.0, 4.0)));
/// let a = segment_start(&line);
/// let b = segment_end(&line);
/// assert_eq!(a.get::<0>(), 0.0);
/// assert_eq!(b.get::<1>(), 4.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoLine<T: CoordNum>(Line<T>);

impl<T: CoordNum> GeoLine<T> {
    /// Wrap a `geo_types::Line` so the [`Segment`] concept applies.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::Line;
    /// use geometry_adapt_geo_types::GeoLine;
    ///
    /// let line = GeoLine::new(Line::new((0.0_f64, 0.0), (1.0, 1.0)));
    /// assert_eq!(line.into_inner().start.x, 0.0);
    /// ```
    #[inline]
    #[must_use]
    pub const fn new(inner: Line<T>) -> Self {
        Self(inner)
    }

    /// Recover the wrapped `geo_types::Line`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geo_types::Line;
    /// use geometry_adapt_geo_types::GeoLine;
    ///
    /// let original = Line::new((0.0_f64, 0.0), (3.0, 4.0));
    /// let line = GeoLine::new(original);
    /// assert_eq!(line.into_inner(), original);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> Line<T> {
        self.0
    }
}

impl<T: CoordinateScalar + CoordNum> Geometry for GeoLine<T> {
    type Kind = SegmentTag;
    type Point = GeoCoord<T>;
}

impl<T: CoordinateScalar + CoordNum> IndexedAccess for GeoLine<T> {
    #[inline]
    fn get_indexed<const I: usize, const D: usize>(&self) -> T {
        let coord = if I == 0 { self.0.start } else { self.0.end };
        if D == 0 { coord.x } else { coord.y }
    }

    #[inline]
    fn set_indexed<const I: usize, const D: usize>(&mut self, value: T) {
        let coord = if I == 0 {
            &mut self.0.start
        } else {
            &mut self.0.end
        };
        if D == 0 {
            coord.x = value;
        } else {
            coord.y = value;
        }
    }
}

impl<T: CoordinateScalar + CoordNum> Segment for GeoLine<T> {}
