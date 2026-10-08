//! `is_simple(&g) -> bool` — OGC Simple Feature predicate.
//!
//! Mirrors `boost::geometry::is_simple(g)` from
//! `boost/geometry/algorithms/is_simple.hpp`. The v1 linestring
//! implementation is brute-force `O(n²)`; a sweepline-based variant
//! lands once `phase_03`'s overlay infrastructure is in place.
//!
//! A linestring is *simple* iff it is not empty, no two non-adjacent
//! segments intersect, adjacent segments touch only at their shared vertex
//! (no zero-length edge, no collinear doubling-back), and it has no
//! repeated non-consecutive vertex. A closed linestring / ring is allowed
//! to share exactly its first and last vertex — that is the ring closure,
//! not a self-intersection — judged as Boost judges it: its first and last
//! segment must meet only at the start of the first.
//!
//! Boost's self-turn walk records no turn for two collinear segments
//! running opposite ways that each leave through the other's start
//! (`collinear_opposite` in `algorithms/detail/overlay/get_turn_info.hpp`),
//! and so misses that overlap when no spike or neighbouring segment gives
//! it away — which happens only where Boost's tolerance reads segments that
//! are not quite collinear as collinear. This port reports the overlap.
//!
//! An areal geometry (polygon) is *simple* iff every ring is non-empty
//! and free of consecutive duplicate vertices — nothing more. This
//! mirrors Boost exactly (`algorithms/detail/is_simple/areal.hpp`):
//! "a Polygon is always a simple geometric object provided that it is
//! valid". Self-intersections, ring touches, and ring crossings are
//! validity concerns — see `geometry_overlay::validity::is_valid_polygon`.

use alloc::vec::Vec;
use core::cmp::Ordering;

use geometry_coords::CoordinateScalar;
use geometry_model::{Linestring, Polygon, Ring, Segment};
use geometry_strategy::{CartesianIntersects, IntersectsStrategy, SegmentMeeting, segment_meeting};
use geometry_trait::{Linestring as LinestringTrait, Point, Polygon as PolygonTrait};

/// `true` iff `g` satisfies the OGC "is simple" predicate.
///
/// Mirrors `boost::geometry::is_simple` from
/// `boost/geometry/algorithms/is_simple.hpp`. Dispatch happens through
/// the [`IsSimple`] trait so linestrings and polygons share one entry
/// point.
#[inline]
#[must_use]
pub fn is_simple<G: IsSimple>(g: &G) -> bool {
    g.is_simple()
}

/// Kind-keyed backing trait for [`is_simple`]. Hidden from the public
/// surface — callers reach it through the free function only.
#[doc(hidden)]
pub trait IsSimple {
    /// `true` iff `self` is simple.
    fn is_simple(&self) -> bool;
}

impl<P> IsSimple for Linestring<P>
where
    P: Point,
    P::Scalar: CoordinateScalar,
    CartesianIntersects: IntersectsStrategy<Segment<P>, Segment<P>>,
    P: geometry_trait::PointMut + Default + Copy,
{
    fn is_simple(&self) -> bool {
        let pts: Vec<P> = self.points().copied().collect();
        linestring_points_simple(&pts)
    }
}

impl<P, const CW: bool, const CL: bool> IsSimple for Polygon<P, CW, CL>
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    fn is_simple(&self) -> bool {
        // Boost's areal is_simple is deliberately shallow: every ring
        // must be non-empty and lack consecutive duplicate points —
        // nothing else. "A Polygon is always a simple geometric object
        // provided that it is valid"
        // (`algorithms/detail/is_simple/areal.hpp`); ring touches,
        // crossings, and self-intersections are `is_valid`'s business
        // (see `geometry_overlay::validity`), not simplicity's.
        ring_lacks_duplicates(self.exterior()) && self.interiors().all(|r| ring_lacks_duplicates(r))
    }
}

/// Boost's `is_simple_ring`: non-empty and no consecutive duplicate
/// vertices, walked over the closeable view — for an open ring the
/// implicit (last, first) closing pair is also checked; for a closed
/// ring the stored closing repetition is the ring closure, not a
/// duplicate. Mirrors `! detail::is_valid::has_duplicates<Ring>` in
/// `algorithms/detail/is_simple/areal.hpp`.
fn ring_lacks_duplicates<P, const CW: bool, const CL: bool>(r: &Ring<P, CW, CL>) -> bool
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let pts: &[P] = &r.0;
    if pts.is_empty() {
        return false;
    }
    for w in pts.windows(2) {
        if points_equal(&w[0], &w[1]) {
            return false;
        }
    }
    if matches!(
        geometry_trait::Ring::closure(r),
        geometry_trait::Closure::Open
    ) && pts.len() >= 2
        && points_equal(&pts[pts.len() - 1], &pts[0])
    {
        // Open ring whose stored last already equals the first: under
        // the closeable view that IS a consecutive duplicate.
        return false;
    }
    true
}

/// The shared linestring-simplicity walk over a point slice.
fn linestring_points_simple<P>(pts: &[P]) -> bool
where
    P: Point + geometry_trait::PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
    CartesianIntersects: IntersectsStrategy<Segment<P>, Segment<P>>,
{
    // Boost: `! boost::empty(linestring) && …`
    // (`algorithms/detail/is_simple/linear.hpp:227-238`).
    if pts.is_empty() {
        return false;
    }
    if pts.len() < 2 {
        return true;
    }

    // Is this a closed loop (first vertex coincides with the last)? Then
    // the first and last segments legitimately share that vertex.
    let closed = points_equal(&pts[0], &pts[pts.len() - 1]);
    // `has_spikes` also looks at the closing vertex of a closed linestring
    // (`apply_at_closure`), between the last segment and the first.
    if closed && pts.len() > 2 && is_spike(&pts[pts.len() - 2], &pts[0], &pts[1]) {
        return false;
    }

    let segs: Vec<Segment<P>> = pts.windows(2).map(|w| Segment::new(w[0], w[1])).collect();

    for i in 0..segs.len() {
        // Zero-length segment (a repeated consecutive vertex) is never
        // simple.
        if points_equal(&pts[i], &pts[i + 1]) {
            return false;
        }
        for j in (i + 1)..segs.len() {
            let adjacent = j == i + 1;
            if adjacent {
                // Adjacent segments share their join vertex — permitted.
                // Reject only a spike, the outgoing edge doubling back
                // over the incoming one.
                if is_spike(&pts[i], &pts[i + 1], &pts[j + 1]) {
                    return false;
                }
            } else if closed && i == 0 && j == segs.len() - 1 {
                // First and last segment of a closed loop meet at the
                // shared closing vertex — that is the ring closure, not a
                // self-intersection — but they may meet elsewhere too.
                if !meets_only_at_closure(&pts[0], &pts[1], &pts[j], &pts[j + 1]) {
                    return false;
                }
            } else if CartesianIntersects.intersects(&segs[i], &segs[j]) {
                return false;
            }
        }
    }
    true
}

/// Whether the first segment `p1 → p2` and the last `q1 → q2` of a closed
/// linestring meet only where the linestring closes.
///
/// Boost accepts their meeting only as one `method_none` turn at the start
/// of the first segment (`is_acceptable_turn`,
/// `algorithms/detail/is_simple/linear.hpp:85-108`): an `'a'` or `'f'`
/// meeting (`algorithms/detail/overlay/get_turn_info.hpp:1462,1590-1600`)
/// whose fraction along the first segment is zero by `math::equals`.
fn meets_only_at_closure<P: Point>(p1: &P, p2: &P, q1: &P, q2: &P) -> bool
where
    P::Scalar: CoordinateScalar,
{
    let p1 = (p1.get::<0>(), p1.get::<1>());
    let p2 = (p2.get::<0>(), p2.get::<1>());
    let q1 = (q1.get::<0>(), q1.get::<1>());
    let q2 = (q2.get::<0>(), q2.get::<1>());
    match segment_meeting(p1, p2, q1, q2) {
        SegmentMeeting::Disjoint => true,
        // A collinear touch in one direction, the last segment running on
        // into the first, is an `'a'` meeting at `p1`
        // (`policies/relate/direction.hpp:279-297`). Any other collinear
        // meeting overlaps the first segment.
        SegmentMeeting::Collinear { positions, .. } => positions == [3, 4, 0, 1],
        SegmentMeeting::AtP1 => {
            // `p1` and `q2` are on each other's line, so the meeting is `'f'`
            // if `q1` is on the first segment's line, `'t'` (a touch, not
            // `method_none`) if `p2` is on the last's, and `'a'` otherwise
            // (`policies/relate/direction.hpp`, `segments_crosses`).
            let touch = P::Scalar::side_by_triangle(p1, p2, q1) != Ordering::Equal
                && P::Scalar::side_by_triangle(q1, q2, p2) == Ordering::Equal;
            // The fraction is Cramer's rule's, along the first segment
            // (`strategies/cartesian/intersection.hpp:264-275,433-437`).
            let along = |a: P::Scalar, b: P::Scalar| (b - a).to_measure();
            let numerator =
                along(q1.0, q2.0) * along(q1.1, p1.1) - along(q1.1, q2.1) * along(q1.0, p1.0);
            !touch
                && numerator.tolerant_eq(
                    <<P::Scalar as CoordinateScalar>::Measure as CoordinateScalar>::ZERO,
                )
        }
        _ => false,
    }
}

/// Coordinate-wise 2D equality of two points, by `math::equals` as
/// Boost's `has_duplicates` compares them.
#[inline]
fn points_equal<P: Point>(a: &P, b: &P) -> bool {
    a.get::<0>().tolerant_eq(b.get::<0>()) && a.get::<1>().tolerant_eq(b.get::<1>())
}

/// `true` iff `current` is a spike between `previous` and `next`: the
/// three collinear and `next` not beyond `previous`, seen from `current`.
///
/// Boost's `has_spikes` asks `is_spike_or_equal(next, current, previous)`
/// (`algorithms/detail/is_valid/has_spikes.hpp:128`), which is
/// `point_is_spike_or_equal(previous, next, current)`
/// (`algorithms/detail/point_is_spike_or_equal.hpp:46-65,97-104`): the side
/// of `previous` from `next` to `current`, then its `direction_code`. The
/// order matters to the floating-point side test.
#[inline]
fn is_spike<P: Point>(previous: &P, current: &P, next: &P) -> bool
where
    P::Scalar: CoordinateScalar,
{
    let previous = (previous.get::<0>(), previous.get::<1>());
    let current = (current.get::<0>(), current.get::<1>());
    let next = (next.get::<0>(), next.get::<1>());
    P::Scalar::side_by_triangle(next, current, previous) == core::cmp::Ordering::Equal
        && P::Scalar::direction_code(next, current, previous) != core::cmp::Ordering::Greater
}

#[cfg(test)]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/is_simple.cpp:46-120`.

    use super::is_simple;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, Polygon, linestring, polygon};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn two_point_is_simple() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 2.)];
        assert!(is_simple(&ls));
    }

    #[test]
    fn three_point_is_simple() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 2.), (2., 3.)];
        assert!(is_simple(&ls));
    }

    #[test]
    fn consecutive_duplicate_not_simple() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (0., 0.), (1., 0.)];
        assert!(!is_simple(&ls));
    }

    #[test]
    fn figure_eight_not_simple() {
        let ls: Linestring<Pt> =
            linestring![(0., 0.), (1., 0.), (2., 0.), (1., 1.), (1., 0.), (1., -1.)];
        assert!(!is_simple(&ls));
    }

    #[test]
    fn bowtie_linestring_not_simple() {
        // (0,0)-(2,2)-(2,0)-(0,2): the first and third segments cross.
        let ls: Linestring<Pt> = linestring![(0., 0.), (2., 2.), (2., 0.), (0., 2.)];
        assert!(!is_simple(&ls));
    }

    #[test]
    fn closed_simple_quadrilateral() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (1., 1.), (0., 0.)];
        assert!(is_simple(&ls));
    }

    /// Boost: `! boost::empty(linestring) && …`
    /// (`algorithms/detail/is_simple/linear.hpp:227-238`).
    #[test]
    fn empty_linestring_is_not_simple() {
        let ls: Linestring<Pt> = Linestring::new();
        assert!(!is_simple(&ls));
    }

    /// A closed linestring may start in the middle of a straight edge:
    /// its last segment runs on into its first, the one turn Boost accepts.
    /// Boost (`aed7bc3`): simple.
    #[test]
    fn closed_linestring_may_start_mid_edge() {
        let ls: Linestring<Pt> =
            linestring![(0., 0.), (2., 0.), (2., 2.), (-2., 2.), (-2., 0.), (0., 0.)];
        assert!(is_simple(&ls));
    }

    /// A sliver: the last vertex lies, up to rounding, on the first
    /// segment, so the last segment runs back over the first. Boost reads
    /// them as collinear and overlapping. Boost (`aed7bc3`): not simple.
    #[test]
    fn closing_segment_running_back_over_the_first_is_not_simple() {
        let ls: Linestring<Pt> = linestring![
            (935.072_535_154_818_4, -831.668_093_369_019_5),
            (96.604_363_826_790_83, -268.950_777_185_098_73),
            (348.144_815_225_199_2, -437.765_972_040_275_1),
            (935.072_535_154_818_4, -831.668_093_369_019_5),
        ];
        assert!(!is_simple(&ls));
    }

    /// A last point within rounding of the first closes the linestring, and
    /// the closing turn must lie at the start of the first segment by
    /// `math::equals` on its Cramer's-rule fraction. Off along the last
    /// segment it does; off diagonally it does not. Boost (`aed7bc3`):
    /// simple, then not simple.
    #[test]
    fn rounding_off_closure_is_judged_by_the_closing_turn() {
        let along: Linestring<Pt> = linestring![
            (10., 10.),
            (10., 11.),
            (11., 11.),
            (11., 10.),
            (10.000_000_000_000_002, 10.)
        ];
        assert!(is_simple(&along));
        let diagonal: Linestring<Pt> = linestring![
            (10., 10.),
            (10., 11.),
            (11., 11.),
            (11., 10.),
            (10.000_000_000_000_002, 10.000_000_000_000_002)
        ];
        assert!(!is_simple(&diagonal));
    }

    /// A closed linestring whose last segment doubles back over its first
    /// has a spike at the closing vertex (`has_spikes`,
    /// `apply_at_closure`): not simple.
    #[test]
    fn spike_at_the_closing_vertex_is_not_simple() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (2., 0.), (2., 2.), (1., 0.), (0., 0.)];
        assert!(!is_simple(&ls));
    }

    #[test]
    fn closed_square_is_simple() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)];
        assert!(is_simple(&ls));
    }

    #[test]
    fn unit_square_polygon_is_simple() {
        let pg: Polygon<Pt> = polygon![[(0., 0.), (4., 0.), (4., 3.), (0., 3.), (0., 0.)]];
        assert!(is_simple(&pg));
    }

    #[test]
    fn polygon_with_disjoint_hole_is_simple() {
        let pg: Polygon<Pt> = polygon![
            [(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)],
            [(2., 2.), (4., 2.), (4., 4.), (2., 4.), (2., 2.)],
        ];
        assert!(is_simple(&pg));
    }

    #[test]
    fn bowtie_polygon_is_simple_but_invalid() {
        // Boost parity: the bow-tie has no duplicate vertices, so it is
        // SIMPLE — and invalid (geometry_overlay::validity::is_valid_ring
        // reports SelfIntersection; not asserted here because algorithm
        // must not depend on overlay — that would be a dependency cycle).
        let pg: Polygon<Pt> = polygon![[(0., 0.), (2., 2.), (0., 2.), (2., 0.), (0., 0.)]];
        assert!(is_simple(&pg));
    }

    #[test]
    fn hole_touching_outer_is_simple() {
        // Boost parity: areal is_simple checks only per-ring duplicate
        // points ("a Polygon is always a simple geometric object provided
        // that it is valid", detail/is_simple/areal.hpp). A hole touching
        // the exterior is a VALIDITY question, not a simplicity one.
        let pg: Polygon<Pt> = polygon![
            [(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)],
            [(0., 0.), (5., 5.), (10., 0.), (5., 0.), (0., 0.)],
        ];
        assert!(is_simple(&pg));
    }

    #[test]
    fn polygon_with_consecutive_duplicate_is_not_simple() {
        // Boost's has_duplicates: a consecutive repeated vertex makes
        // the ring (and so the polygon) non-simple.
        let pg: Polygon<Pt> =
            polygon![[(0., 0.), (4., 0.), (4., 0.), (4., 4.), (0., 4.), (0., 0.)]];
        assert!(!is_simple(&pg));
    }

    #[test]
    fn polygon_with_duplicate_in_hole_is_not_simple() {
        let pg: Polygon<Pt> = polygon![
            [(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)],
            [(2., 2.), (4., 2.), (4., 2.), (4., 4.), (2., 4.), (2., 2.)],
        ];
        assert!(!is_simple(&pg));
    }

    #[test]
    fn polygon_with_empty_exterior_is_not_simple() {
        // Boost: "not empty and lacking duplicate points" — empty
        // fails the first clause.
        use geometry_model::Ring;
        let pg: Polygon<Pt> = Polygon::new(Ring::new());
        assert!(!is_simple(&pg));
    }
}
