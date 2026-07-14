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

    /// `envelope.cpp:48-51` — a standalone ring envelopes like the
    /// polygon built from it (orientation/closure insensitive).
    #[test]
    fn ring_envelope_direct() {
        let r: geometry_model::Ring<P> = geometry_model::Ring::from_vec(vec![
            Point2D::new(4.0, 1.0),
            Point2D::new(0.0, 7.0),
            Point2D::new(7.0, 9.0),
            Point2D::new(4.0, 1.0),
        ]);
        assert_2d(&envelope(&r), 0.0, 7.0, 1.0, 9.0);
    }

    /// Multi-point with all-negative coordinates proves the box is
    /// seeded from the first real point, not from zero; an empty
    /// multi-point returns the degenerate origin box (the documented
    /// divergence from Boost's inverted empty envelope).
    #[test]
    fn multi_point_envelope() {
        let mp = geometry_model::MultiPoint(vec![
            Point2D::<f64, Cartesian>::new(-3.0, -1.0),
            Point2D::<f64, Cartesian>::new(-1.0, -4.0),
        ]);
        assert_2d(&envelope(&mp), -3.0, -1.0, -4.0, -1.0);
        let empty = geometry_model::MultiPoint::<P>(vec![]);
        assert_2d(&envelope(&empty), 0.0, 0.0, 0.0, 0.0);
    }

    /// Multi-linestring: bounds span every member.
    #[test]
    fn multi_linestring_envelope_spans_members() {
        let mls = geometry_model::MultiLinestring::<Linestring<P>>(vec![
            linestring![(1.0, 1.0), (2.0, 2.0)],
            linestring![(-5.0, 4.0), (0.0, 0.0)],
        ]);
        assert_2d(&envelope(&mls), -5.0, 2.0, 0.0, 4.0);
    }

    /// Multi-polygon: bounds span every member's exterior.
    #[test]
    fn multi_polygon_envelope_spans_members() {
        let mpg = geometry_model::MultiPolygon::<Polygon<P>>(vec![
            polygon![[(1.0, 1.0), (1.0, 3.0), (3.0, 3.0), (3.0, 1.0), (1.0, 1.0)]],
            polygon![[(10.0, -2.0), (10.0, 0.0), (12.0, 0.0), (10.0, -2.0)]],
        ]);
        assert_2d(&envelope(&mpg), 1.0, 12.0, -2.0, 3.0);
    }

    /// A 3D segment envelope carries the third ordinate (the `2 =>`
    /// arms of the strategy's per-dimension walk).
    #[test]
    fn three_d_segment_envelope() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let s = Segment::new(P3::new(1.0, 2.0, 9.0), P3::new(3.0, 0.0, -1.0));
        let b = envelope(&s);
        assert_eq!(b.get_indexed::<0, 2>().to_bits(), (-1.0_f64).to_bits());
        assert_eq!(b.get_indexed::<1, 2>().to_bits(), 9.0_f64.to_bits());
    }

    /// A 4D point envelope carries the fourth ordinate (the `3 =>` arms).
    #[test]
    fn four_d_point_envelope() {
        use geometry_model::Point;
        use geometry_trait::PointMut as _;
        type P4 = Point<f64, 4, Cartesian>;
        let mut p = P4::default();
        p.set::<3>(7.0);
        let b = envelope(&p);
        assert_eq!(b.get_indexed::<0, 3>().to_bits(), 7.0_f64.to_bits());
        assert_eq!(b.get_indexed::<1, 3>().to_bits(), 7.0_f64.to_bits());
    }
}
