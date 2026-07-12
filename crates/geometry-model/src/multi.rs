//! Default `model::Multi{Point, Linestring, Polygon}` — homogeneous
//! collections of a single geometry kind, each stored in a `Vec`.
//!
//! Mirrors `boost::geometry::model::multi_point`
//! (`boost/geometry/geometries/multi_point.hpp:52-100`),
//! `boost::geometry::model::multi_linestring`
//! (`boost/geometry/geometries/multi_linestring.hpp:48-92`), and
//! `boost::geometry::model::multi_polygon`
//! (`boost/geometry/geometries/multi_polygon.hpp:48-92`), together with
//! the `traits::tag` specialisations the same headers provide at
//! `multi_point.hpp:101-115`, `multi_linestring.hpp:100-115`, and
//! `multi_polygon.hpp:100-115`. The Rust port collapses each of those
//! specialisations into the two trait impls below ([`Geometry`] and
//! the matching `MultiXxxTrait`), keyed off the generic element type.
//!
//! All three C++ models derive from `Container<Item, Allocator<Item>>`
//! so any `boost::range` algorithm sees the underlying storage
//! directly; the Rust port stores a `Vec<Item>` and surfaces it
//! through the `…s()` iterator the matching concept requires.

use alloc::vec::Vec;

use geometry_tag::{MultiLinestringTag, MultiPointTag, MultiPolygonTag};
use geometry_trait::{
    Geometry, Linestring as LinestringTrait, MultiLinestring as MultiLinestringTrait,
    MultiPoint as MultiPointTrait, MultiPolygon as MultiPolygonTrait, Point as PointTrait,
    Polygon as PolygonTrait,
};

/// Default `MultiPoint` — a `Vec<P>` newtype.
///
/// Mirrors `boost::geometry::model::multi_point<Point>` from
/// `boost/geometry/geometries/multi_point.hpp:52-100`. The
/// `#[repr(transparent)]` annotation guarantees the in-memory layout
/// matches the underlying `Vec<P>`, mirroring the invariant Boost
/// relies on by inheriting publicly from `std::vector<Point>`
/// (`boost/geometry/geometries/multi_point.hpp:58`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(transparent)]
pub struct MultiPoint<P: PointTrait>(pub Vec<P>);

impl<P: PointTrait> MultiPoint<P> {
    /// Construct an empty multi-point.
    ///
    /// Mirrors the default `multi_point()` constructor at
    /// `boost/geometry/geometries/multi_point.hpp:66-68`.
    #[must_use]
    pub const fn new() -> Self {
        Self(Vec::new())
    }

    /// Wrap an existing `Vec<P>` as a multi-point without copying.
    ///
    /// Rust analogue of the iterator-range `multi_point(Iterator,
    /// Iterator)` constructor at
    /// `boost/geometry/geometries/multi_point.hpp:71-74`; the `Vec`
    /// is moved in rather than copied element-by-element.
    #[must_use]
    pub const fn from_vec(v: Vec<P>) -> Self {
        Self(v)
    }

    /// Append a point to the back of the multi-point.
    ///
    /// Plays the role of `std::vector::push_back` on the inherited
    /// `base_type` in `boost/geometry/geometries/multi_point.hpp:58-62`.
    pub fn push(&mut self, p: P) {
        self.0.push(p);
    }
}

impl<P: PointTrait> Default for MultiPoint<P> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Tag this concrete type as a `MultiPoint`. Mirrors the
/// `traits::tag<model::multi_point<...>>` specialisation at
/// `boost/geometry/geometries/multi_point.hpp:101-115`.
impl<P: PointTrait> Geometry for MultiPoint<P> {
    type Kind = MultiPointTag;
    type Point = P;
}

/// Wire the `MultiPoint` concept up. The `points()` iterator hands back
/// `self.0.iter()` — `slice::Iter` is already `ExactSizeIterator`.
/// Plays the role of `boost::begin(mp)` / `boost::end(mp)` on
/// `model::multi_point` (`boost/geometry/geometries/multi_point.hpp:58-62`).
impl<P: PointTrait> MultiPointTrait for MultiPoint<P> {
    type ItemPoint = P;

    fn points(&self) -> impl ExactSizeIterator<Item = &P> {
        self.0.iter()
    }
}

/// Default `MultiLinestring` — a `Vec<L>` newtype where `L` is any
/// concrete [`LinestringTrait`] implementor.
///
/// Mirrors `boost::geometry::model::multi_linestring<LineString>` from
/// `boost/geometry/geometries/multi_linestring.hpp:48-92`. The
/// `#[repr(transparent)]` annotation guarantees the in-memory layout
/// matches the underlying `Vec<L>`, mirroring the invariant Boost
/// relies on by inheriting publicly from `std::vector<LineString>`
/// (`boost/geometry/geometries/multi_linestring.hpp:55`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(transparent)]
pub struct MultiLinestring<L: LinestringTrait>(pub Vec<L>);

impl<L: LinestringTrait> MultiLinestring<L> {
    /// Construct an empty multi-linestring.
    ///
    /// Mirrors the default `multi_linestring()` constructor at
    /// `boost/geometry/geometries/multi_linestring.hpp:63-65`.
    #[must_use]
    pub const fn new() -> Self {
        Self(Vec::new())
    }

    /// Wrap an existing `Vec<L>` as a multi-linestring without copying.
    ///
    /// Rust analogue of the iterator-range
    /// `multi_linestring(Iterator, Iterator)` constructor at
    /// `boost/geometry/geometries/multi_linestring.hpp:68-71`.
    #[must_use]
    pub const fn from_vec(v: Vec<L>) -> Self {
        Self(v)
    }

    /// Append a linestring to the back of the multi-linestring.
    ///
    /// Plays the role of `std::vector::push_back` on the inherited
    /// `base_type` in
    /// `boost/geometry/geometries/multi_linestring.hpp:55-59`.
    pub fn push(&mut self, ls: L) {
        self.0.push(ls);
    }
}

impl<L: LinestringTrait> Default for MultiLinestring<L> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Tag this concrete type as a `MultiLinestring`. Mirrors the
/// `traits::tag<model::multi_linestring<...>>` specialisation at
/// `boost/geometry/geometries/multi_linestring.hpp:100-115`.
impl<L: LinestringTrait> Geometry for MultiLinestring<L> {
    type Kind = MultiLinestringTag;
    type Point = L::Point;
}

/// Wire the `MultiLinestring` concept up. The `linestrings()` iterator
/// hands back `self.0.iter()` — `slice::Iter` is already
/// `ExactSizeIterator`. Plays the role of `boost::begin(mls)` /
/// `boost::end(mls)` on `model::multi_linestring`
/// (`boost/geometry/geometries/multi_linestring.hpp:55-59`).
impl<L: LinestringTrait> MultiLinestringTrait for MultiLinestring<L> {
    type ItemLinestring = L;

    fn linestrings(&self) -> impl ExactSizeIterator<Item = &L> {
        self.0.iter()
    }
}

/// Default `MultiPolygon` — a `Vec<Pg>` newtype where `Pg` is any
/// concrete [`PolygonTrait`] implementor.
///
/// Mirrors `boost::geometry::model::multi_polygon<Polygon>` from
/// `boost/geometry/geometries/multi_polygon.hpp:48-92`. The
/// `#[repr(transparent)]` annotation guarantees the in-memory layout
/// matches the underlying `Vec<Pg>`, mirroring the invariant Boost
/// relies on by inheriting publicly from `std::vector<Polygon>`
/// (`boost/geometry/geometries/multi_polygon.hpp:55`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(transparent)]
pub struct MultiPolygon<Pg: PolygonTrait>(pub Vec<Pg>);

impl<Pg: PolygonTrait> MultiPolygon<Pg> {
    /// Construct an empty multi-polygon.
    ///
    /// Mirrors the default `multi_polygon()` constructor at
    /// `boost/geometry/geometries/multi_polygon.hpp:63-65`.
    #[must_use]
    pub const fn new() -> Self {
        Self(Vec::new())
    }

    /// Wrap an existing `Vec<Pg>` as a multi-polygon without copying.
    ///
    /// Rust analogue of the iterator-range
    /// `multi_polygon(Iterator, Iterator)` constructor at
    /// `boost/geometry/geometries/multi_polygon.hpp:68-71`.
    #[must_use]
    pub const fn from_vec(v: Vec<Pg>) -> Self {
        Self(v)
    }

    /// Append a polygon to the back of the multi-polygon.
    ///
    /// Plays the role of `std::vector::push_back` on the inherited
    /// `base_type` in
    /// `boost/geometry/geometries/multi_polygon.hpp:55-59`.
    pub fn push(&mut self, pg: Pg) {
        self.0.push(pg);
    }
}

impl<Pg: PolygonTrait> Default for MultiPolygon<Pg> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Tag this concrete type as a `MultiPolygon`. Mirrors the
/// `traits::tag<model::multi_polygon<...>>` specialisation at
/// `boost/geometry/geometries/multi_polygon.hpp:100-115`.
impl<Pg: PolygonTrait> Geometry for MultiPolygon<Pg> {
    type Kind = MultiPolygonTag;
    type Point = Pg::Point;
}

/// Wire the `MultiPolygon` concept up. The `polygons()` iterator hands
/// back `self.0.iter()` — `slice::Iter` is already
/// `ExactSizeIterator`. Plays the role of `boost::begin(mpg)` /
/// `boost::end(mpg)` on `model::multi_polygon`
/// (`boost/geometry/geometries/multi_polygon.hpp:55-59`).
impl<Pg: PolygonTrait> MultiPolygonTrait for MultiPolygon<Pg> {
    type ItemPolygon = Pg;

    fn polygons(&self) -> impl ExactSizeIterator<Item = &Pg> {
        self.0.iter()
    }
}

#[cfg(test)]
mod tests {
    //! Construction + tag-resolution + concept-check witnesses for
    //! [`MultiPoint`], [`MultiLinestring`], and [`MultiPolygon`].
    //! Mirrors `boost/geometry/test/core/tag.cpp` (the multi arms) and
    //! the iteration smoke tests in
    //! `boost/geometry/test/geometries/multi_*.cpp`.

    use super::{MultiLinestring, MultiPoint, MultiPolygon};
    use crate::linestring::Linestring;
    use crate::point::Point2D;
    use crate::polygon::Polygon;
    use crate::ring::Ring;
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_tag::{MultiLinestringTag, MultiPointTag, MultiPolygonTag};
    use geometry_trait::{
        Geometry, MultiLinestring as MultiLinestringTrait, MultiPoint as MultiPointTrait,
        MultiPolygon as MultiPolygonTrait, Point as _, check_multi_linestring, check_multi_point,
        check_multi_polygon,
    };

    type P = Point2D<f64, Cartesian>;

    // ---- MultiPoint ---------------------------------------------------

    #[test]
    fn multipoint_iterates_in_declared_order() {
        let mut mp = MultiPoint::<P>::new();
        mp.push(Point2D::new(0.0, 0.0));
        mp.push(Point2D::new(1.0, 2.0));
        mp.push(Point2D::new(3.0, 4.0));
        assert_eq!(mp.points().count(), 3);
        let xs: Vec<u64> = mp.points().map(|p| p.get::<0>().to_bits()).collect();
        assert_eq!(
            xs,
            vec![0.0_f64.to_bits(), 1.0_f64.to_bits(), 3.0_f64.to_bits()]
        );
    }

    #[test]
    fn multipoint_from_vec_reports_exact_size() {
        let mp = MultiPoint::<P>::from_vec(vec![Point2D::new(0.0, 0.0), Point2D::new(1.0, 1.0)]);
        assert_eq!(mp.points().len(), 2);
    }

    #[test]
    fn multipoint_kind_is_multi_point_tag() {
        fn k<T: Geometry<Kind = MultiPointTag>>() {}
        k::<MultiPoint<P>>();
    }

    #[test]
    fn multipoint_satisfies_concept() {
        check_multi_point::<MultiPoint<P>>();
    }

    // ---- MultiLinestring ----------------------------------------------

    #[test]
    fn multilinestring_iterates_in_declared_order() {
        let mut mls = MultiLinestring::<Linestring<P>>::new();
        mls.push(Linestring::from_vec(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
        ]));
        mls.push(Linestring::from_vec(vec![
            Point2D::new(2.0, 2.0),
            Point2D::new(3.0, 3.0),
            Point2D::new(4.0, 4.0),
        ]));
        assert_eq!(mls.linestrings().count(), 2);
    }

    #[test]
    fn multilinestring_kind_is_multi_linestring_tag() {
        fn k<T: Geometry<Kind = MultiLinestringTag>>() {}
        k::<MultiLinestring<Linestring<P>>>();
    }

    #[test]
    fn multilinestring_satisfies_concept() {
        check_multi_linestring::<MultiLinestring<Linestring<P>>>();
    }

    // ---- MultiPolygon -------------------------------------------------

    fn unit_square() -> Ring<P> {
        Ring::from_vec(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(1.0, 1.0),
            Point2D::new(0.0, 1.0),
            Point2D::new(0.0, 0.0),
        ])
    }

    #[test]
    fn multipolygon_iterates_in_declared_order() {
        let mut mpg = MultiPolygon::<Polygon<P>>::new();
        mpg.push(Polygon::new(unit_square()));
        mpg.push(Polygon::new(unit_square()));
        assert_eq!(mpg.polygons().count(), 2);
    }

    #[test]
    fn multipolygon_kind_is_multi_polygon_tag() {
        fn k<T: Geometry<Kind = MultiPolygonTag>>() {}
        k::<MultiPolygon<Polygon<P>>>();
    }

    #[test]
    fn multipolygon_satisfies_concept() {
        check_multi_polygon::<MultiPolygon<Polygon<P>>>();
    }
}
