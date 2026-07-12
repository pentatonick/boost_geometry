//! Default `model::Segment<P>` — two endpoint points stored inline.
//!
//! Mirrors `boost::geometry::model::segment<Point>` declared in
//! `boost/geometry/geometries/segment.hpp:55-71`, together with the
//! trait specialisations the same header provides under
//! `namespace traits` (`tag`, `point_type`, `indexed_access`) at
//! `boost/geometry/geometries/segment.hpp:119-183`. The Rust port
//! collapses those specialisations into the four trait impls below
//! ([`Geometry`], [`IndexedAccess`], [`SegmentTrait`]), keyed off the
//! generic point parameter `P`.
//!
//! Boost derives `model::segment` from `std::pair<Point, Point>`; the
//! Rust port stores the same two points in a `[P; 2]` array so the
//! `(I, D)` indexed access can write
//! `endpoints[I].set::<D>(v)` without a `match` on `I`.

use geometry_tag::SegmentTag;
use geometry_trait::{
    Geometry, IndexedAccess, Point as PointTrait, PointMut, Segment as SegmentTrait,
};

/// Two-point segment.
///
/// Mirrors `boost::geometry::model::segment<P>` from
/// `boost/geometry/geometries/segment.hpp:55-71`. The two endpoints
/// are addressed by index `0` (start) and `1` (end), matching the
/// `traits::indexed_access<segment, 0|1, D>` specialisations at
/// `boost/geometry/geometries/segment.hpp:136-169`.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Segment<P: PointTrait> {
    endpoints: [P; 2],
}

impl<P: PointTrait> Segment<P> {
    /// Construct a segment from its two endpoints.
    ///
    /// Mirrors the `segment(Point const& p1, Point const& p2)`
    /// constructor at `boost/geometry/geometries/segment.hpp:67-70`.
    #[must_use]
    pub const fn new(start: P, end: P) -> Self {
        Self {
            endpoints: [start, end],
        }
    }

    /// Borrow the start endpoint (`I = 0`).
    ///
    /// Mirrors reading `s.first` on `model::segment` — the Boost
    /// `traits::indexed_access<segment, 0, D>::get(...)` accessor at
    /// `boost/geometry/geometries/segment.hpp:142-150` returns a
    /// coordinate of that endpoint; this method returns the whole
    /// point.
    #[must_use]
    pub const fn start(&self) -> &P {
        &self.endpoints[0]
    }

    /// Borrow the end endpoint (`I = 1`).
    ///
    /// Mirrors reading `s.second` on `model::segment` — the Boost
    /// `traits::indexed_access<segment, 1, D>::get(...)` accessor at
    /// `boost/geometry/geometries/segment.hpp:160-168`.
    #[must_use]
    pub const fn end(&self) -> &P {
        &self.endpoints[1]
    }
}

/// Tag this concrete type as a Segment. Mirrors the
/// `traits::tag<model::segment<...>>` specialisation at
/// `boost/geometry/geometries/segment.hpp:121-124`.
impl<P: PointTrait> Geometry for Segment<P> {
    type Kind = SegmentTag;
    type Point = P;
}

/// Two-axis access on the segment. Collapses the
/// `traits::indexed_access<segment, 0|1, D>` specialisations at
/// `boost/geometry/geometries/segment.hpp:136-169` into one Rust
/// impl: the `I` axis selects the endpoint, `D` then dispatches to
/// `Point::get` / `Point::set` on that endpoint.
impl<P: PointMut> IndexedAccess for Segment<P> {
    #[inline]
    fn get_indexed<const I: usize, const D: usize>(&self) -> P::Scalar {
        self.endpoints[I].get::<D>()
    }

    #[inline]
    fn set_indexed<const I: usize, const D: usize>(&mut self, value: P::Scalar) {
        self.endpoints[I].set::<D>(value);
    }
}

/// Wire the Segment concept up. The trait body is empty — the
/// concept is satisfied by the two super-bounds [`Geometry`] and
/// [`IndexedAccess`] already proven above.
impl<P: PointMut> SegmentTrait for Segment<P> {}

#[cfg(test)]
mod tests {
    //! Round-trip + tag-resolution + `check_segment` witness for
    //! [`Segment`]. Mirrors
    //! `boost/geometry/test/core/access.cpp:55-86` (the
    //! `test_indexed_get_set` helper applied to a segment) and the
    //! segment-tag arm of `boost/geometry/test/core/tag.cpp`.

    use super::Segment;
    use crate::point::Point2D;
    use geometry_cs::Cartesian;
    use geometry_tag::SegmentTag;
    use geometry_trait::{Geometry, IndexedAccess, check_segment};

    #[test]
    fn segment_round_trip_all_four_slots() {
        let mut s = Segment::new(
            Point2D::<f64, Cartesian>::default(),
            Point2D::<f64, Cartesian>::default(),
        );
        s.set_indexed::<0, 0>(1.0);
        s.set_indexed::<0, 1>(2.0);
        s.set_indexed::<1, 0>(3.0);
        s.set_indexed::<1, 1>(4.0);

        // Bit-equal comparison: each slot was written with a literal
        // f64 constant and read back without arithmetic, so the round
        // trip must be exact. `to_bits()` sidesteps `clippy::float_cmp`.
        assert_eq!(s.get_indexed::<0, 0>().to_bits(), 1.0_f64.to_bits());
        assert_eq!(s.get_indexed::<0, 1>().to_bits(), 2.0_f64.to_bits());
        assert_eq!(s.get_indexed::<1, 0>().to_bits(), 3.0_f64.to_bits());
        assert_eq!(s.get_indexed::<1, 1>().to_bits(), 4.0_f64.to_bits());
    }

    #[test]
    fn segment_start_and_end_accessors() {
        use geometry_trait::Point as PointTrait;

        let a = Point2D::<f64, Cartesian>::new(1.0, 2.0);
        let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
        let s = Segment::new(a, b);
        // `Point<T, D, Cs>` derives `PartialEq`, but only when its
        // phantom `Cs` parameter does — and `Cartesian` does not.
        // Compare per-coordinate via the `Point` trait instead.
        assert_eq!(s.start().get::<0>().to_bits(), 1.0_f64.to_bits());
        assert_eq!(s.start().get::<1>().to_bits(), 2.0_f64.to_bits());
        assert_eq!(s.end().get::<0>().to_bits(), 3.0_f64.to_bits());
        assert_eq!(s.end().get::<1>().to_bits(), 4.0_f64.to_bits());
    }

    #[test]
    fn segment_kind_is_segment_tag() {
        fn k<T: Geometry<Kind = SegmentTag>>() {}
        k::<Segment<Point2D<f64, Cartesian>>>();
    }

    #[test]
    fn segment_satisfies_concept() {
        check_segment::<Segment<Point2D<f64, Cartesian>>>();
    }
}
