//! Default `model::Polygon<P, CW, CL>` — an exterior [`Ring`] plus zero
//! or more interior rings (holes).
//!
//! Mirrors `boost::geometry::model::polygon<Point, ClockWise, Closed,
//! PointList, RingList, PointAlloc, RingAlloc>` declared in
//! `boost/geometry/geometries/polygon.hpp:66-100`, together with the
//! trait specialisations the same header provides under
//! `namespace traits` (`tag`, `ring_const_type`, `ring_mutable_type`,
//! `exterior_ring`, `interior_const_type`, `interior_mutable_type`,
//! `interior_rings`, `point_order`, `closure`) at
//! `boost/geometry/geometries/polygon.hpp:150-340`. The Rust port
//! collapses those specialisations into the two trait impls below
//! ([`Geometry`], [`PolygonTrait`]), keyed off the generic point
//! parameter `P` and the two const-generic booleans inherited from
//! [`Ring`].
//!
//! Boost encodes closure and traversal direction at the type level
//! through the same two `bool` template parameters that flow down to
//! the exterior and interior `model::ring`s
//! (`boost/geometry/geometries/polygon.hpp:84`); the Rust port mirrors
//! that by sharing the same two const generics between the polygon and
//! every ring it contains, so a polygon with custom orientation is a
//! distinct type, exactly like
//! `model::polygon<P, false, false>` is in C++. Boost defaults
//! `ClockWise = true, Closed = true`
//! (`boost/geometry/geometries/polygon.hpp:69-70`); the Rust defaults
//! match.

use alloc::vec::Vec;

use geometry_tag::PolygonTag;
use geometry_trait::{Geometry, Point as PointTrait, Polygon as PolygonTrait};

use crate::Ring;

/// Default Polygon — an outer [`Ring`] and a `Vec` of inner rings,
/// all sharing the same point type and the same closure / traversal
/// const generics.
///
/// Mirrors `boost::geometry::model::polygon<Point, ClockWise, Closed>`
/// from `boost/geometry/geometries/polygon.hpp:66-100`. The defaults
/// (`CLOCKWISE = true`, `CLOSED = true`) match Boost's defaults
/// (`boost/geometry/geometries/polygon.hpp:69-70`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Polygon<P: PointTrait, const CLOCKWISE: bool = true, const CLOSED: bool = true> {
    /// The exterior ring (outer boundary). Mirrors `m_outer` in
    /// `boost/geometry/geometries/polygon.hpp:139`.
    pub outer: Ring<P, CLOCKWISE, CLOSED>,
    /// The interior rings (holes), in declared order. Mirrors
    /// `m_inners` in `boost/geometry/geometries/polygon.hpp:140`.
    pub inners: Vec<Ring<P, CLOCKWISE, CLOSED>>,
}

impl<P: PointTrait, const CW: bool, const CL: bool> Polygon<P, CW, CL> {
    /// Construct a polygon from an outer ring with no holes.
    ///
    /// Rust analogue of constructing a `boost::geometry::model::polygon`
    /// then assigning to `.outer()` and leaving `.inners()` empty
    /// (`boost/geometry/geometries/polygon.hpp:87-91, 96-101`).
    #[must_use]
    pub const fn new(outer: Ring<P, CW, CL>) -> Self {
        Self {
            outer,
            inners: Vec::new(),
        }
    }

    /// Construct a polygon from an outer ring plus a `Vec` of inner
    /// rings.
    ///
    /// Rust analogue of constructing a `boost::geometry::model::polygon`
    /// and then assigning to both `.outer()` and `.inners()`
    /// (`boost/geometry/geometries/polygon.hpp:87-91`).
    #[must_use]
    pub const fn with_inners(outer: Ring<P, CW, CL>, inners: Vec<Ring<P, CW, CL>>) -> Self {
        Self { outer, inners }
    }
}

impl<P: PointTrait, const CW: bool, const CL: bool> Default for Polygon<P, CW, CL> {
    #[inline]
    fn default() -> Self {
        Self {
            outer: Ring::new(),
            inners: Vec::new(),
        }
    }
}

/// Tag this concrete type as a Polygon. Mirrors the
/// `traits::tag<model::polygon<...>>` specialisation at
/// `boost/geometry/geometries/polygon.hpp:158-172`.
impl<P: PointTrait, const CW: bool, const CL: bool> Geometry for Polygon<P, CW, CL> {
    type Kind = PolygonTag;
    type Point = P;
}

/// Wire the Polygon concept up. The `exterior()` accessor hands back
/// `&self.outer` and `interiors()` hands back `self.inners.iter()` —
/// `slice::Iter` is already `ExactSizeIterator`. Mirrors the
/// `traits::exterior_ring` and `traits::interior_rings` specialisations
/// at `boost/geometry/geometries/polygon.hpp:200-260` that dispatch on
/// the same generic parameters.
impl<P: PointTrait, const CW: bool, const CL: bool> PolygonTrait for Polygon<P, CW, CL> {
    type Ring = Ring<P, CW, CL>;

    fn exterior(&self) -> &Self::Ring {
        &self.outer
    }

    fn interiors(&self) -> impl ExactSizeIterator<Item = &Self::Ring> {
        self.inners.iter()
    }
}

#[cfg(test)]
mod tests {
    //! Construction + tag-resolution + `check_polygon` witness for
    //! [`Polygon`]. Mirrors `boost/geometry/test/core/tag.cpp` (the
    //! polygon-tag arm) and the exterior/interior accessor tests in
    //! `boost/geometry/test/geometries/polygon.cpp`.

    use super::Polygon;
    use crate::point::Point2D;
    use crate::ring::Ring;
    use alloc::vec;
    use geometry_cs::Cartesian;
    use geometry_tag::PolygonTag;
    use geometry_trait::{Geometry, Polygon as PolygonTrait, check_polygon};

    type P = Point2D<f64, Cartesian>;

    fn unit_square_outer() -> Ring<P> {
        Ring::from_vec(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(10.0, 0.0),
            Point2D::new(10.0, 10.0),
            Point2D::new(0.0, 10.0),
            Point2D::new(0.0, 0.0),
        ])
    }

    fn hole(x: f64, y: f64) -> Ring<P> {
        Ring::from_vec(vec![
            Point2D::new(x, y),
            Point2D::new(x + 1.0, y),
            Point2D::new(x + 1.0, y + 1.0),
            Point2D::new(x, y + 1.0),
            Point2D::new(x, y),
        ])
    }

    #[test]
    fn polygon_new_has_no_holes() {
        let poly: Polygon<P> = Polygon::new(unit_square_outer());
        assert_eq!(poly.exterior().0.len(), 5);
        assert_eq!(poly.interiors().count(), 0);
    }

    #[test]
    fn polygon_with_two_holes() {
        let mut poly: Polygon<P> = Polygon::new(unit_square_outer());
        poly.inners.push(hole(1.0, 1.0));
        poly.inners.push(hole(5.0, 5.0));
        assert_eq!(poly.interiors().count(), 2);
    }

    #[test]
    fn polygon_with_inners_constructor() {
        let poly: Polygon<P> =
            Polygon::with_inners(unit_square_outer(), vec![hole(1.0, 1.0), hole(5.0, 5.0)]);
        assert_eq!(poly.interiors().count(), 2);
    }

    #[test]
    fn polygon_kind_is_polygon_tag() {
        fn k<T: Geometry<Kind = PolygonTag>>() {}
        k::<Polygon<P>>();
        k::<Polygon<P, false, false>>();
    }

    #[test]
    fn polygon_satisfies_concept() {
        check_polygon::<Polygon<P>>();
        check_polygon::<Polygon<P, false, false>>();
    }
}
