//! The [`Ring`] concept: an ordered sequence of points forming a
//! closed boundary.
//!
//! Mirrors `doc/concept/ring.qbk` and the model in
//! `boost/geometry/geometries/ring.hpp`. The two ring-only metafunctions
//! `boost::geometry::traits::closure<G>` and
//! `boost::geometry::traits::point_order<G>` (from
//! `boost/geometry/core/closure.hpp` and
//! `boost/geometry/core/point_order.hpp`) fold into trait methods on
//! [`Ring`] here, with the same defaults Boost ships
//! (`closed` / `clockwise`).

use crate::closure::Closure;
use crate::geometry::Geometry;
use crate::point_order::PointOrder;
use geometry_tag::RingTag;

/// A ring — an ordered sequence of points whose first and last
/// either coincide ([`Closure::Closed`]) or are implicitly connected
/// ([`Closure::Open`]).
///
/// Mirrors the Ring concept (`doc/concept/ring.qbk`); the canonical
/// model is `boost::geometry::model::ring` in
/// `boost/geometry/geometries/ring.hpp`, which is parameterised on
/// `bool ClockWise = true, bool Closed = true` — the Boost defaults
/// match the defaults on [`Ring::point_order`] and [`Ring::closure`].
///
/// As with [`crate::Linestring`], points are exposed as an
/// `ExactSizeIterator + Clone` returned via RPITIT so each impl can
/// reuse whatever iterator its container provides.
///
/// # Examples
///
/// ```
/// use geometry_trait::{Closure, PointOrder, Ring};
/// fn ring_defaults<R: Ring>(r: &R) -> (Closure, PointOrder) {
///     (r.closure(), r.point_order())
/// }
/// ```
pub trait Ring: Geometry<Kind = RingTag> {
    /// The points of this ring, in declared order.
    ///
    /// Plays the role of `boost::begin(r)` / `boost::end(r)` from
    /// `boost/geometry/geometries/ring.hpp` when read together.
    fn points(&self) -> impl ExactSizeIterator<Item = &Self::Point> + Clone;

    /// Whether this ring's last point repeats its first.
    ///
    /// Mirrors `boost::geometry::traits::closure<R>::value`
    /// (`boost/geometry/core/closure.hpp`). Defaults to
    /// [`Closure::Closed`], matching Boost's default specialisation
    /// `traits::closure<G>::value = closed`.
    fn closure(&self) -> Closure {
        Closure::Closed
    }

    /// The traversal direction of this ring's boundary.
    ///
    /// Mirrors `boost::geometry::traits::point_order<R>::value`
    /// (`boost/geometry/core/point_order.hpp`). Defaults to
    /// [`PointOrder::Clockwise`], matching Boost's default
    /// specialisation `traits::point_order<G>::value = clockwise`.
    fn point_order(&self) -> PointOrder {
        PointOrder::Clockwise
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::point::{Point, PointMut};
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_tag::PointTag;

    struct Xy(f64, f64);

    impl Geometry for Xy {
        type Kind = PointTag;
        type Point = Self;
    }

    impl Point for Xy {
        type Scalar = f64;
        type Cs = Cartesian;
        const DIM: usize = 2;

        fn get<const D: usize>(&self) -> f64 {
            if D == 0 { self.0 } else { self.1 }
        }
    }

    impl PointMut for Xy {
        fn set<const D: usize>(&mut self, v: f64) {
            if D == 0 {
                self.0 = v;
            } else {
                self.1 = v;
            }
        }
    }

    struct VRing(Vec<Xy>);

    impl Geometry for VRing {
        type Kind = RingTag;
        type Point = Xy;
    }

    impl Ring for VRing {
        fn points(&self) -> impl ExactSizeIterator<Item = &Xy> + Clone {
            self.0.iter()
        }
        // Inherit defaults: closure() = Closed, point_order() = Clockwise.
    }

    #[test]
    fn ring_defaults_are_closed_clockwise() {
        let r = VRing(vec![
            Xy(0.0, 0.0),
            Xy(1.0, 0.0),
            Xy(1.0, 1.0),
            Xy(0.0, 1.0),
            Xy(0.0, 0.0),
        ]);
        assert_eq!(r.closure(), Closure::Closed);
        assert_eq!(r.point_order(), PointOrder::Clockwise);
        assert_eq!(r.points().count(), 5);
    }

    #[test]
    fn ring_iterates_in_declared_order() {
        let r = VRing(vec![Xy(0.0, 0.0), Xy(1.0, 0.0), Xy(0.0, 1.0), Xy(0.0, 0.0)]);
        let xs: Vec<f64> = r.points().map(Xy::get::<0>).collect();
        assert_eq!(xs, vec![0.0, 1.0, 0.0, 0.0]);
    }

    // A ring impl that overrides both defaults — confirms the trait
    // methods are not `final`-by-default.
    struct OpenCcw(Vec<Xy>);

    impl Geometry for OpenCcw {
        type Kind = RingTag;
        type Point = Xy;
    }

    impl Ring for OpenCcw {
        fn points(&self) -> impl ExactSizeIterator<Item = &Xy> + Clone {
            self.0.iter()
        }
        fn closure(&self) -> Closure {
            Closure::Open
        }
        fn point_order(&self) -> PointOrder {
            PointOrder::CounterClockwise
        }
    }

    #[test]
    fn ring_defaults_can_be_overridden() {
        let r = OpenCcw(vec![Xy(0.0, 0.0), Xy(1.0, 0.0), Xy(0.0, 1.0)]);
        assert_eq!(r.closure(), Closure::Open);
        assert_eq!(r.point_order(), PointOrder::CounterClockwise);
        assert_eq!(r.points().count(), 3);
    }
}
