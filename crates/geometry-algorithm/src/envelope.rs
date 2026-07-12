//! `envelope(&g)` — the axis-aligned bounding box of `g`.
//!
//! Mirrors `boost::geometry::envelope` from
//! `boost/geometry/algorithms/envelope.hpp` and the implementation
//! ladder rooted at
//! `boost/geometry/algorithms/detail/envelope/interface.hpp`. The
//! Boost free function takes the bounding box by out-parameter
//! (`envelope(g, mbr)`); the Rust analogue returns the box by value
//! because we have no const-correctness story for an out-parameter
//! that would make ownership any clearer than a return.
//!
//! Cartesian-only in v1; spherical / geographic envelope strategies
//! arrive alongside the Haversine / Andoyer / Vincenty distance
//! strategies in later tasks.

use geometry_strategy::{EnvelopeStrategy, EnvelopeStrategyForKind};
use geometry_trait::Geometry;

/// Axis-aligned bounding box of `g`.
///
/// Mirrors `boost::geometry::envelope(g, mbr)` from
/// `boost/geometry/algorithms/envelope.hpp` — the C++ side mutates
/// `mbr` in place; the Rust side returns a fresh
/// `geometry_model::Box<G::Point>`.
///
/// Supported geometry kinds: `Point`, `Linestring`, `Ring`, `Polygon`,
/// `Segment`, `Box`, `MultiPoint`, `MultiLinestring`, `MultiPolygon` —
/// each selected by the tag-keyed
/// [`geometry_strategy::EnvelopeStrategyForKind`] picker, so any
/// concept-adapted foreign type resolves through the same per-kind impl
/// in [`geometry_strategy::envelope`] as the equivalent model value.
#[inline]
#[must_use]
pub fn envelope<G>(
    g: &G,
) -> <<G::Kind as EnvelopeStrategyForKind>::S as EnvelopeStrategy<G>>::Output
where
    G: Geometry,
    G::Kind: EnvelopeStrategyForKind,
    <G::Kind as EnvelopeStrategyForKind>::S: EnvelopeStrategy<G>,
{
    <<G::Kind as EnvelopeStrategyForKind>::S as Default>::default().envelope(g)
}

#[cfg(test)]
mod tests {
    //! Reference values come from
    //! `geometry/test/algorithms/envelope_expand/envelope.cpp:38-54`
    //! (the `test_2d` arm). Each test cites the line it mirrors.

    use super::envelope;
    use geometry_cs::Cartesian;
    use geometry_model::{Box, Linestring, Point2D, Polygon, Segment, linestring, polygon};
    use geometry_trait::IndexedAccess as _;

    type P = Point2D<f64, Cartesian>;

    fn assert_2d(b: &Box<P>, xmin: f64, xmax: f64, ymin: f64, ymax: f64) {
        assert_eq!(b.get_indexed::<0, 0>().to_bits(), xmin.to_bits());
        assert_eq!(b.get_indexed::<0, 1>().to_bits(), ymin.to_bits());
        assert_eq!(b.get_indexed::<1, 0>().to_bits(), xmax.to_bits());
        assert_eq!(b.get_indexed::<1, 1>().to_bits(), ymax.to_bits());
    }

    /// `envelope.cpp:38` — `POINT(1 1)` → `(1,1) (1,1)`.
    #[test]
    fn point_envelope_collapses() {
        let p = Point2D::<f64, Cartesian>::new(1.0, 1.0);
        assert_2d(&envelope(&p), 1.0, 1.0, 1.0, 1.0);
    }

    /// `envelope.cpp:39` — `LINESTRING(1 1,2 2)` → `(1,1) (2,2)`.
    #[test]
    fn linestring_two_points() {
        let ls: Linestring<P> = linestring![(1.0, 1.0), (2.0, 2.0)];
        assert_2d(&envelope(&ls), 1.0, 2.0, 1.0, 2.0);
    }

    /// `envelope.cpp:40` — square polygon `(1,1)-(3,3)`.
    #[test]
    fn polygon_axis_aligned_square() {
        let p: Polygon<P> = polygon![[(1.0, 1.0), (1.0, 3.0), (3.0, 3.0), (3.0, 1.0), (1.0, 1.0),]];
        assert_2d(&envelope(&p), 1.0, 3.0, 1.0, 3.0);
    }

    /// `envelope.cpp:43` — `BOX(1 1,3 3)` — envelope is the box.
    #[test]
    fn box_envelope_is_self() {
        let b = Box::from_corners(
            Point2D::<f64, Cartesian>::new(1.0, 1.0),
            Point2D::<f64, Cartesian>::new(3.0, 3.0),
        );
        assert_2d(&envelope(&b), 1.0, 3.0, 1.0, 3.0);
    }

    /// `envelope.cpp:48` — non-convex closed CW ring; envelope
    /// tightens to the extremes `(0,1)-(7,9)`.
    #[test]
    fn ring_non_convex() {
        let p: Polygon<P> = polygon![[(4.0, 1.0), (0.0, 7.0), (7.0, 9.0), (4.0, 1.0)]];
        assert_2d(&envelope(&p), 0.0, 7.0, 1.0, 9.0);
    }

    /// `envelope.cpp:54` — `SEGMENT(1 1,3 3)`.
    #[test]
    fn segment_envelope() {
        let s = Segment::new(
            Point2D::<f64, Cartesian>::new(1.0, 1.0),
            Point2D::<f64, Cartesian>::new(3.0, 3.0),
        );
        assert_2d(&envelope(&s), 1.0, 3.0, 1.0, 3.0);
    }
}
