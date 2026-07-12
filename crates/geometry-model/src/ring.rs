//! Default `model::Ring<P, CW, CL>` — an ordered sequence of points
//! forming a (closed or open, clockwise or counter-clockwise)
//! boundary, stored in a `Vec<P>`.
//!
//! Mirrors `boost::geometry::model::ring<Point, ClockWise, Closed,
//! Container, Allocator>` declared in
//! `boost/geometry/geometries/ring.hpp:56-99`, together with the
//! trait specialisations the same header provides under
//! `namespace traits` (`tag`, `point_order`, `closure`) at
//! `boost/geometry/geometries/ring.hpp:104-168`. The Rust port
//! collapses those four specialisations into the two trait impls
//! below ([`Geometry`], [`RingTrait`]), keyed off the generic point
//! parameter `P` and the two const-generic booleans.
//!
//! Boost encodes closure and traversal direction at the type level
//! through the two `bool` template parameters; the Rust port mirrors
//! that with two const generics so a ring with custom orientation is
//! a distinct type, exactly like `model::ring<P, false, false>` is in
//! C++. Boost defaults `ClockWise = true, Closed = true`
//! (`boost/geometry/geometries/ring.hpp:59`); the Rust defaults
//! match.

use alloc::vec::Vec;

use geometry_tag::RingTag;
use geometry_trait::{Closure, Geometry, Point as PointTrait, PointOrder, Ring as RingTrait};

/// Default Ring — a `Vec<P>` newtype carrying closure and traversal
/// direction as const generics.
///
/// Mirrors `boost::geometry::model::ring<Point, ClockWise, Closed>`
/// from `boost/geometry/geometries/ring.hpp:56-99`. The defaults
/// (`CLOCKWISE = true`, `CLOSED = true`) match Boost's defaults
/// (`boost/geometry/geometries/ring.hpp:59`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ring<P: PointTrait, const CLOCKWISE: bool = true, const CLOSED: bool = true>(pub Vec<P>);

impl<P: PointTrait, const CW: bool, const CL: bool> Ring<P, CW, CL> {
    /// Construct an empty ring.
    ///
    /// Mirrors the default `ring()` constructor at
    /// `boost/geometry/geometries/ring.hpp:71-73`.
    #[must_use]
    pub const fn new() -> Self {
        Self(Vec::new())
    }

    /// Wrap an existing `Vec<P>` as a ring without copying.
    ///
    /// Rust analogue of the iterator-range `ring(Iterator, Iterator)`
    /// constructor at `boost/geometry/geometries/ring.hpp:76-79`; the
    /// `Vec` is moved in rather than copied element-by-element.
    #[must_use]
    pub const fn from_vec(v: Vec<P>) -> Self {
        Self(v)
    }

    /// Append a point to the back of the ring.
    ///
    /// Plays the role of `std::vector::push_back` on the inherited
    /// `base_type` in `boost/geometry/geometries/ring.hpp:63-67`.
    pub fn push(&mut self, p: P) {
        self.0.push(p);
    }
}

impl<P: PointTrait, const CW: bool, const CL: bool> Default for Ring<P, CW, CL> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Tag this concrete type as a Ring. Mirrors the
/// `traits::tag<model::ring<...>>` specialisation at
/// `boost/geometry/geometries/ring.hpp:108-118`.
impl<P: PointTrait, const CW: bool, const CL: bool> Geometry for Ring<P, CW, CL> {
    type Kind = RingTag;
    type Point = P;
}

/// Wire the Ring concept up. The `points()` iterator hands back
/// `self.0.iter()` — `slice::Iter` is already
/// `ExactSizeIterator + Clone`. The `closure()` and `point_order()`
/// methods read the two const-generic booleans, mirroring the four
/// `traits::point_order` / `traits::closure` specialisations at
/// `boost/geometry/geometries/ring.hpp:121-168` that dispatch on the
/// same two template parameters.
impl<P: PointTrait, const CW: bool, const CL: bool> RingTrait for Ring<P, CW, CL> {
    fn points(&self) -> impl ExactSizeIterator<Item = &P> + Clone {
        self.0.iter()
    }

    fn closure(&self) -> Closure {
        if CL { Closure::Closed } else { Closure::Open }
    }

    fn point_order(&self) -> PointOrder {
        if CW {
            PointOrder::Clockwise
        } else {
            PointOrder::CounterClockwise
        }
    }
}

#[cfg(test)]
mod tests {
    //! Iteration + tag-resolution + closure/order + `check_ring`
    //! witness for [`Ring`]. Mirrors
    //! `boost/geometry/test/core/tag.cpp` (the ring-tag arm) and
    //! `boost/geometry/test/core/ring.cpp` (the closure / `point_order`
    //! cases on `model::ring<P, CW, CL>`).

    use super::Ring;
    use crate::point::Point2D;
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_tag::RingTag;
    use geometry_trait::{
        Closure, Geometry, Point as _, PointOrder, Ring as RingTrait, check_ring,
    };

    #[test]
    fn ring_iterates_in_declared_order() {
        let mut r = Ring::<Point2D<f64, Cartesian>>::new();
        r.push(Point2D::new(0.0, 0.0));
        r.push(Point2D::new(1.0, 0.0));
        r.push(Point2D::new(0.0, 1.0));
        r.push(Point2D::new(0.0, 0.0));
        assert_eq!(r.points().count(), 4);
        let xs: Vec<u64> = r.points().map(|p| p.get::<0>().to_bits()).collect();
        assert_eq!(
            xs,
            vec![
                0.0_f64.to_bits(),
                1.0_f64.to_bits(),
                0.0_f64.to_bits(),
                0.0_f64.to_bits(),
            ]
        );
    }

    #[test]
    fn ring_defaults_are_closed_clockwise() {
        let r = Ring::<Point2D<f64, Cartesian>>::new();
        assert_eq!(r.closure(), Closure::Closed);
        assert_eq!(r.point_order(), PointOrder::Clockwise);
    }

    #[test]
    fn ring_with_custom_const_generics() {
        let r = Ring::<Point2D<f64, Cartesian>, false, false>::new();
        assert_eq!(r.closure(), Closure::Open);
        assert_eq!(r.point_order(), PointOrder::CounterClockwise);
    }

    #[test]
    fn ring_kind_is_ring_tag() {
        fn k<T: Geometry<Kind = RingTag>>() {}
        k::<Ring<Point2D<f64, Cartesian>>>();
        k::<Ring<Point2D<f64, Cartesian>, false, false>>();
    }

    #[test]
    fn ring_satisfies_concept() {
        check_ring::<Ring<Point2D<f64, Cartesian>>>();
        check_ring::<Ring<Point2D<f64, Cartesian>, false, false>>();
    }
}
