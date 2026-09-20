//! Borrowing segment model whose indexed access forwards to two points.
//!
//! Mirrors `boost::geometry::model::pointing_segment<Point>` from
//! `geometries/pointing_segment.hpp:39-72` and its indexed-access
//! specializations at lines 91-137.

use geometry_tag::SegmentTag;
use geometry_trait::{Geometry, IndexedAccess, PointMut, Segment};

/// A segment borrowing two mutable endpoints instead of storing copies.
///
/// Mirrors `model::pointing_segment` from
/// `geometries/pointing_segment.hpp:39-72`. Boost stores nullable raw
/// pointers because its segment iterator requires default construction. Rust
/// represents only the valid, non-null state and uses the borrow checker to
/// prevent the endpoints from outliving their source.
#[derive(Debug)]
pub struct PointingSegment<'a, P: PointMut> {
    start: &'a mut P,
    end: &'a mut P,
}

impl<'a, P: PointMut> PointingSegment<'a, P> {
    /// Borrow two endpoints as a segment.
    ///
    /// Mirrors `pointing_segment(p1, p2)` from
    /// `geometries/pointing_segment.hpp:67-71` while eliminating the C++
    /// model's nullable default state.
    #[inline]
    #[must_use]
    pub const fn new(start: &'a mut P, end: &'a mut P) -> Self {
        Self { start, end }
    }

    /// Borrow the first endpoint.
    #[inline]
    #[must_use]
    pub const fn start(&self) -> &P {
        self.start
    }

    /// Borrow the second endpoint.
    #[inline]
    #[must_use]
    pub const fn end(&self) -> &P {
        self.end
    }
}

impl<P: PointMut> Geometry for PointingSegment<'_, P> {
    type Kind = SegmentTag;
    type Point = P;
}

impl<P: PointMut> IndexedAccess for PointingSegment<'_, P> {
    #[inline]
    fn get_indexed<const I: usize, const D: usize>(&self) -> P::Scalar {
        match I {
            0 => self.start.get::<D>(),
            1 => self.end.get::<D>(),
            _ => panic!("PointingSegment::get_indexed: endpoint index {I} is out of range"),
        }
    }

    #[inline]
    fn set_indexed<const I: usize, const D: usize>(&mut self, value: P::Scalar) {
        match I {
            0 => self.start.set::<D>(value),
            1 => self.end.set::<D>(value),
            _ => panic!("PointingSegment::set_indexed: endpoint index {I} is out of range"),
        }
    }
}

impl<P: PointMut> Segment for PointingSegment<'_, P> {}

#[cfg(test)]
mod tests {
    //! An endpoint index past the second must fail loudly: the two-way
    //! branch form silently aliased every out-of-range `I` onto `end`.

    use geometry_cs::Cartesian;
    use geometry_trait::IndexedAccess as _;

    use super::PointingSegment;
    use crate::Point2D;

    type P = Point2D<f64, Cartesian>;

    #[test]
    #[should_panic(expected = "endpoint index 2 is out of range")]
    fn reading_a_third_endpoint_panics_instead_of_aliasing_the_end() {
        let mut start = P::new(1.0, 2.0);
        let mut end = P::new(3.0, 4.0);
        let segment = PointingSegment::new(&mut start, &mut end);
        let _ = segment.get_indexed::<2, 0>();
    }

    #[test]
    #[should_panic(expected = "endpoint index 2 is out of range")]
    fn writing_a_third_endpoint_panics_instead_of_aliasing_the_end() {
        let mut start = P::new(1.0, 2.0);
        let mut end = P::new(3.0, 4.0);
        let mut segment = PointingSegment::new(&mut start, &mut end);
        segment.set_indexed::<2, 0>(9.0);
    }
}
