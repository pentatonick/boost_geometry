//! OVL2.T2 / OVL2.T3 — collect the turns between two rings, and
//! between two polygons.
//!
//! Mirrors `boost/geometry/algorithms/detail/overlay/get_turns.hpp`,
//! which drives the segment-intersection kernel over every segment pair
//! of the two inputs and records a turn per intersection. The port
//! keeps the same shape: a doubly-nested walk over the ring segments,
//! calling [`segment_intersection`](fn@crate::predicate::segment_intersection::segment_intersection)
//! and pushing a [`Turn`] for each hit.
//!
//! This task does **not** classify the turns beyond the raw
//! segment-intersection outcome — that is OVL2.T4
//! ([`classify`](super::classify)). Here
//! every turn is emitted with [`Method::None`] and unset operations,
//! carrying only the point and the two [`SegmentId`]s.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_model::Segment;
use geometry_trait::{Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait};

use super::info::{Method, Operation, RingKind, SegmentId, Turn};
use crate::predicate::segment_intersection::{SegmentIntersection, segment_intersection};
use crate::turn::classify::set_from_outcome;

/// The point bound shared by every `get_turns` entry point: the point
/// must be constructible (for the intersection point Boost returns in
/// the input type) and cheaply copyable so segments can be built from
/// vertex pairs. Blanket-implemented for every qualifying point type;
/// callers never name it directly.
pub trait TurnPoint: PointMut + Default + Copy {}
impl<P: PointMut + Default + Copy> TurnPoint for P {}

/// Collect the turns between two rings.
///
/// Iterates every segment of `r1` against every segment of `r2`, calls
/// the segment-intersection kernel, and pushes a classified [`Turn`]
/// per intersection. `source1` / `source2` label which input each ring
/// is (`0` / `1`), and `ring1` / `ring2` record which ring of that
/// input it is — so the resulting [`SegmentId`]s are globally unique.
///
/// Mirrors the ring-pair inner loop of Boost's `get_turns`
/// (`get_turns.hpp`). Collinear-overlap intersections contribute their
/// two delimiting points as two turns, matching Boost recording a turn
/// at each end of a collinear stretch.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{Point2D, Ring};
/// use geometry_overlay::turn::{get_turns_ring_ring, RingKind};
///
/// type P = Point2D<f64, Cartesian>;
/// // Two squares overlapping at a corner — their boundaries cross
/// // at two points.
/// let a: Ring<P> = Ring::from_vec(vec![
///     P::new(0.0, 0.0), P::new(2.0, 0.0), P::new(2.0, 2.0), P::new(0.0, 2.0), P::new(0.0, 0.0),
/// ]);
/// let b: Ring<P> = Ring::from_vec(vec![
///     P::new(1.0, 1.0), P::new(3.0, 1.0), P::new(3.0, 3.0), P::new(1.0, 3.0), P::new(1.0, 1.0),
/// ]);
/// let turns = get_turns_ring_ring(&a, 0, RingKind::Exterior, &b, 1, RingKind::Exterior);
/// assert_eq!(turns.len(), 2);
/// ```
#[must_use]
pub fn get_turns_ring_ring<R1, R2, P>(
    r1: &R1,
    source1: usize,
    ring1: RingKind,
    r2: &R2,
    source2: usize,
    ring2: RingKind,
) -> Vec<Turn<P>>
where
    R1: RingTrait<Point = P>,
    R2: RingTrait<Point = P>,
    P: TurnPoint,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let segs1 = ring_segments(r1);
    let segs2 = ring_segments(r2);
    let mut turns = Vec::new();
    collect_pair(&segs1, source1, ring1, &segs2, source2, ring2, &mut turns);
    turns
}

/// Collect the turns between two polygons.
///
/// Generalises [`get_turns_ring_ring`] over every `(exterior, interior)`
/// ring pair of the two polygons: exterior×exterior, exterior×hole,
/// hole×exterior, and hole×hole. Mirrors the ring-walk in Boost's
/// `get_turns` for areal inputs (`get_turns.hpp`).
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::turn::get_turns_polygon_polygon;
///
/// type P = Point2D<f64, Cartesian>;
/// let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
/// let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
/// let turns = get_turns_polygon_polygon(&a, &b);
/// assert_eq!(turns.len(), 2);
/// ```
#[must_use]
pub fn get_turns_polygon_polygon<G1, G2, P>(g1: &G1, g2: &G2) -> Vec<Turn<P>>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: TurnPoint,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let mut turns = Vec::new();

    // Every ring of g1 (source 0) against every ring of g2 (source 1).
    let rings1 = rings_of(g1);
    let rings2 = rings_of(g2);
    for &(ring1, r1) in &rings1 {
        let segs1 = ring_segments(r1);
        for &(ring2, r2) in &rings2 {
            let segs2 = ring_segments(r2);
            collect_pair(&segs1, 0, ring1, &segs2, 1, ring2, &mut turns);
        }
    }
    turns
}

/// Intersect every segment of the first ring against every segment of
/// the second, classify each hit at emission, and append the resulting
/// turns to `out`.
///
/// A collinear overlap contributes a turn at each end of the shared
/// stretch, matching Boost recording a turn at both ends of a collinear
/// run.
#[allow(clippy::too_many_arguments)]
fn collect_pair<P>(
    segs1: &[(P, P)],
    source1: usize,
    ring1: RingKind,
    segs2: &[(P, P)],
    source2: usize,
    ring2: RingKind,
    out: &mut Vec<Turn<P>>,
) where
    P: TurnPoint,
    P::Scalar: CoordinateScalar + Into<f64> + PartialEq,
{
    for (i1, &(a1, b1)) in segs1.iter().enumerate() {
        let s1 = Segment::new(a1, b1);
        for (i2, &(a2, b2)) in segs2.iter().enumerate() {
            let s2 = Segment::new(a2, b2);
            let id1 = SegmentId {
                source_index: source1,
                ring: ring1,
                segment_index: i1,
            };
            let id2 = SegmentId {
                source_index: source2,
                ring: ring2,
                segment_index: i2,
            };
            let outcome = segment_intersection::<Segment<P>, P>(&s1, &s2);
            match outcome {
                SegmentIntersection::Disjoint | SegmentIntersection::OutOfRange => {}
                SegmentIntersection::Single(point) => {
                    out.push(classified(point, id1, id2, &outcome, &a1, &b1, &a2, &b2));
                }
                SegmentIntersection::Collinear { from, to } => {
                    out.push(classified(from, id1, id2, &outcome, &a1, &b1, &a2, &b2));
                    out.push(classified(to, id1, id2, &outcome, &a1, &b1, &a2, &b2));
                }
            }
        }
    }
}

/// Build a turn at `point` and classify it from the intersection
/// outcome and the four segment endpoints.
#[allow(clippy::too_many_arguments)]
fn classified<P>(
    point: P,
    id1: SegmentId,
    id2: SegmentId,
    outcome: &SegmentIntersection<P>,
    a0: &P,
    a1: &P,
    b0: &P,
    b1: &P,
) -> Turn<P>
where
    P: Point,
    P::Scalar: PartialEq,
{
    let mut turn = Turn {
        point,
        method: Method::None,
        operations: [Operation::new(id1), Operation::new(id2)],
        touch_only: false,
    };
    set_from_outcome(&mut turn, outcome, a0, a1, b0, b1);
    turn
}

/// The ordered list of `(start, end)` vertex pairs making up a ring's
/// segments. A closed ring (the default) repeats its first vertex as
/// its last, so consecutive pairs already close the ring; an unclosed
/// ring gets the wrap-around segment appended.
fn ring_segments<R, P>(ring: &R) -> Vec<(P, P)>
where
    R: RingTrait<Point = P>,
    P: Copy + Point,
{
    let pts: Vec<P> = ring.points().copied().collect();
    let mut segs = Vec::new();
    if pts.len() < 2 {
        return segs;
    }
    for w in pts.windows(2) {
        segs.push((w[0], w[1]));
    }
    // Close the ring if the source did not repeat the first vertex.
    let first = pts[0];
    let last = pts[pts.len() - 1];
    if !same_point(&first, &last) {
        segs.push((last, first));
    }
    segs
}

/// Coordinate equality on two points (their `Cs` phantom blocks a
/// derived `PartialEq`, so compare x and y directly).
fn same_point<P: Point>(a: &P, b: &P) -> bool
where
    P::Scalar: PartialEq,
{
    a.get::<0>() == b.get::<0>() && a.get::<1>() == b.get::<1>()
}

/// Every ring of a polygon, tagged with its [`RingKind`]: the exterior
/// first, then each interior ring in order.
fn rings_of<G, P>(g: &G) -> Vec<(RingKind, &G::Ring)>
where
    G: PolygonTrait<Point = P>,
    P: Point,
{
    let mut out: Vec<(RingKind, &G::Ring)> = Vec::new();
    out.push((RingKind::Exterior, g.exterior()));
    for (i, r) in g.interiors().enumerate() {
        out.push((RingKind::Interior(i), r));
    }
    out
}

#[cfg(test)]
mod tests {
    //! OVL2.T2 / T3 done-when: turn counts against hand-checked ring
    //! and polygon pairs. Mirrors the count assertions in
    //! `test/algorithms/overlay/get_turns.cpp`.

    use super::{get_turns_polygon_polygon, get_turns_ring_ring};
    use crate::turn::info::RingKind;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Ring, polygon};

    type P = Point2D<f64, Cartesian>;

    fn square(x: f64, y: f64, s: f64) -> Ring<P> {
        Ring::from_vec(vec![
            P::new(x, y),
            P::new(x + s, y),
            P::new(x + s, y + s),
            P::new(x, y + s),
            P::new(x, y),
        ])
    }

    #[test]
    fn overlapping_squares_two_turns() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        let turns = get_turns_ring_ring(&a, 0, RingKind::Exterior, &b, 1, RingKind::Exterior);
        assert_eq!(turns.len(), 2);
    }

    #[test]
    fn disjoint_squares_no_turns() {
        let a = square(0.0, 0.0, 1.0);
        let b = square(5.0, 5.0, 1.0);
        let turns = get_turns_ring_ring(&a, 0, RingKind::Exterior, &b, 1, RingKind::Exterior);
        assert!(turns.is_empty());
    }

    #[test]
    fn polygon_pair_two_turns() {
        use geometry_model::Polygon;
        let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
        let turns = get_turns_polygon_polygon(&a, &b);
        assert_eq!(turns.len(), 2);
    }

    #[test]
    fn each_turn_names_both_sources() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        let turns = get_turns_ring_ring(&a, 0, RingKind::Exterior, &b, 1, RingKind::Exterior);
        for t in &turns {
            assert_eq!(t.operations[0].seg_id.source_index, 0);
            assert_eq!(t.operations[1].seg_id.source_index, 1);
        }
    }
}
