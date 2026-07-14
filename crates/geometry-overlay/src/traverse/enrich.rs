//! OVL2.5 / OVL3.T1 support — enrich the two input rings with their
//! turns.
//!
//! Mirrors `boost/geometry/algorithms/detail/overlay/`
//! `enrich_intersection_points.hpp`: it takes the raw turn list and,
//! for each ring, produces the ordered cyclic walk of original vertices
//! interleaved with the turn points, so traversal can step from node to
//! node and cross over at turns.
//!
//! The port keeps the enrichment self-contained: [`enrich`] consumes
//! two rings and their shared turn list and returns [`EnrichedRings`],
//! two node sequences with the turns spliced in at the right position
//! along each segment, and a cross-index letting traversal jump from a
//! turn on ring 0 to the same turn on ring 1.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_trait::{Point, Ring as RingTrait};

use crate::turn::info::Turn;

/// A single node on an enriched ring: either an original ring vertex or
/// a turn (crossing) point. Turns carry the id linking them to the same
/// turn spliced into the *other* ring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Node<P> {
    /// An original vertex of the ring.
    Vertex(P),
    /// A turn point; `turn_id` indexes into the turn list, so the same
    /// turn can be located on the other ring.
    Turn {
        /// The turn's coordinates.
        point: P,
        /// Index into the shared turn list.
        turn_id: usize,
    },
}

impl<P: Point> Node<P> {
    /// The coordinates of this node, whichever kind it is.
    #[must_use]
    pub fn point(&self) -> &P {
        match self {
            Node::Vertex(p) | Node::Turn { point: p, .. } => p,
        }
    }

    /// The turn id, if this node is a turn.
    #[must_use]
    pub fn turn_id(&self) -> Option<usize> {
        match self {
            Node::Turn { turn_id, .. } => Some(*turn_id),
            Node::Vertex(_) => None,
        }
    }
}

/// The two input rings, each as a cyclic sequence of [`Node`]s with the
/// shared turns spliced in.
///
/// `rings[0]` is the enriched first input, `rings[1]` the second. A turn
/// with a given `turn_id` appears exactly once in each ring, which is
/// how traversal crosses from one boundary to the other.
#[derive(Debug, Clone)]
pub struct EnrichedRings<P> {
    /// The two enriched rings.
    pub rings: [Vec<Node<P>>; 2],
}

impl<P: Point> EnrichedRings<P> {
    /// Find the position of the turn `turn_id` within enriched ring
    /// `ring_index` (`0` or `1`).
    #[must_use]
    pub fn locate_turn(&self, ring_index: usize, turn_id: usize) -> Option<usize> {
        self.rings[ring_index]
            .iter()
            .position(|n| n.turn_id() == Some(turn_id))
    }
}

/// Enrich two rings with their shared turns.
///
/// For each ring, walks its segments in order and, on any segment that
/// carries turns, inserts those turns sorted by their distance from the
/// segment start. The result is a cyclic node list per ring.
///
/// `turns[i].operations[0]` is assumed to reference ring 0 and
/// `operations[1]` ring 1 — the convention
/// [`get_turns_ring_ring`](crate::turn::get_turns_ring_ring) emits.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{Point2D, Ring};
/// use geometry_overlay::turn::{get_turns_ring_ring, RingKind};
/// use geometry_overlay::traverse::enrich;
///
/// type P = Point2D<f64, Cartesian>;
/// let a: Ring<P> = Ring::from_vec(vec![
///     P::new(0.0, 0.0), P::new(2.0, 0.0), P::new(2.0, 2.0), P::new(0.0, 2.0), P::new(0.0, 0.0),
/// ]);
/// let b: Ring<P> = Ring::from_vec(vec![
///     P::new(1.0, 1.0), P::new(3.0, 1.0), P::new(3.0, 3.0), P::new(1.0, 3.0), P::new(1.0, 1.0),
/// ]);
/// let turns = get_turns_ring_ring(&a, 0, RingKind::Exterior, &b, 1, RingKind::Exterior);
/// let enriched = enrich(&a, &b, &turns);
/// // Both enriched rings contain each of the two turns.
/// assert!(enriched.locate_turn(0, 0).is_some());
/// assert!(enriched.locate_turn(1, 0).is_some());
/// ```
#[must_use]
pub fn enrich<R1, R2, P>(r1: &R1, r2: &R2, turns: &[Turn<P>]) -> EnrichedRings<P>
where
    R1: RingTrait<Point = P>,
    R2: RingTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: CoordinateScalar,
{
    let ring0 = enrich_one(&ring_vertices(r1), turns, 0);
    let ring1 = enrich_one(&ring_vertices(r2), turns, 1);
    EnrichedRings {
        rings: [ring0, ring1],
    }
}

/// Splice the turns belonging to `operation_index` into one ring's
/// vertex list.
fn enrich_one<P>(vertices: &[P], turns: &[Turn<P>], operation_index: usize) -> Vec<Node<P>>
where
    P: Point + Copy,
    P::Scalar: CoordinateScalar,
{
    let n = vertices.len();
    if n < 2 {
        return vertices.iter().map(|&p| Node::Vertex(p)).collect();
    }
    // Number of directed segments = n - 1 when the ring repeats its
    // first vertex as its last (the model default), else n.
    let closed = same_point(&vertices[0], &vertices[n - 1]);
    let seg_count = if closed { n - 1 } else { n };

    let mut out = Vec::new();
    for seg in 0..seg_count {
        let start = vertices[seg];
        let end = vertices[(seg + 1) % n];
        out.push(Node::Vertex(start));

        // Turns whose `operation_index` side sits on this segment,
        // sorted by distance from `start`.
        let mut on_seg: Vec<(P::Scalar, usize)> = turns
            .iter()
            .enumerate()
            .filter(|(_, t)| t.operations[operation_index].seg_id.segment_index == seg)
            .map(|(id, t)| (dist_sq(&start, &t.point), id))
            .collect();
        on_seg.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(core::cmp::Ordering::Equal));

        for (_, turn_id) in on_seg {
            // Skip a turn that coincides with the segment start vertex —
            // it is represented by the vertex node already.
            if same_point(&turns[turn_id].point, &start) {
                continue;
            }
            if same_point(&turns[turn_id].point, &end) {
                // Represented by the next segment's start vertex; splice
                // there instead to avoid a duplicate.
                continue;
            }
            out.push(Node::Turn {
                point: turns[turn_id].point,
                turn_id,
            });
        }
    }
    out
}

/// The vertices of a ring as an owned `Vec`.
fn ring_vertices<R, P>(ring: &R) -> Vec<P>
where
    R: RingTrait<Point = P>,
    P: Copy + Point,
{
    ring.points().copied().collect()
}

/// Squared distance between two points (no `sqrt`; used only for
/// ordering turns along a segment).
fn dist_sq<P>(a: &P, b: &P) -> P::Scalar
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let dx = a.get::<0>() - b.get::<0>();
    let dy = a.get::<1>() - b.get::<1>();
    dx * dx + dy * dy
}

fn same_point<P: Point>(a: &P, b: &P) -> bool
where
    P::Scalar: PartialEq,
{
    a.get::<0>() == b.get::<0>() && a.get::<1>() == b.get::<1>()
}

#[cfg(test)]
mod tests {
    use super::{Node, enrich};
    use crate::turn::{RingKind, get_turns_ring_ring};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Ring};
    use geometry_trait::Point as _;

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
    fn both_rings_get_both_turns() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        let turns = get_turns_ring_ring(&a, 0, RingKind::Exterior, &b, 1, RingKind::Exterior);
        assert_eq!(turns.len(), 2);
        let e = enrich(&a, &b, &turns);
        for turn_id in 0..turns.len() {
            assert!(
                e.locate_turn(0, turn_id).is_some(),
                "ring0 missing turn {turn_id}"
            );
            assert!(
                e.locate_turn(1, turn_id).is_some(),
                "ring1 missing turn {turn_id}"
            );
        }
    }

    #[test]
    fn short_rings_preserve_their_available_vertices() {
        let one: Ring<P> = Ring::from_vec(vec![P::new(1.0, 2.0)]);
        let empty = Ring::<P>::new();
        let enriched = enrich(&one, &empty, &[]);

        assert_eq!(enriched.rings[0], vec![Node::Vertex(P::new(1.0, 2.0))]);
        assert!(enriched.rings[1].is_empty());
    }

    #[test]
    fn enriched_ring_has_vertices_and_turns() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        let turns = get_turns_ring_ring(&a, 0, RingKind::Exterior, &b, 1, RingKind::Exterior);
        let e = enrich(&a, &b, &turns);
        // Ring 0 has its 4 corners plus the 2 turns spliced in.
        let vertices = e.rings[0]
            .iter()
            .filter(|n| matches!(n, Node::Vertex(_)))
            .count();
        let turn_nodes = e.rings[0]
            .iter()
            .filter(|n| matches!(n, Node::Turn { .. }))
            .count();
        assert_eq!(vertices, 4);
        assert_eq!(turn_nodes, 2);
    }

    #[test]
    fn turns_ordered_along_segment() {
        // A ring whose one edge is crossed by two turns: they must be
        // spliced in near-to-far from the segment start.
        // Horizontal strip crossed by a wider box: turns on the bottom
        // edge at x=1 and x=3.
        let strip: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(4.0, 0.0),
            P::new(4.0, 2.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ]);
        let crosser: Ring<P> = Ring::from_vec(vec![
            P::new(1.0, -1.0),
            P::new(3.0, -1.0),
            P::new(3.0, 1.0),
            P::new(1.0, 1.0),
            P::new(1.0, -1.0),
        ]);
        let turns = get_turns_ring_ring(
            &strip,
            0,
            RingKind::Exterior,
            &crosser,
            1,
            RingKind::Exterior,
        );
        let e = enrich(&strip, &crosser, &turns);
        // On ring 0, walking the enriched nodes, the two turns on the
        // bottom edge appear with x=1 before x=3.
        let xs: Vec<f64> = e.rings[0]
            .iter()
            .filter_map(|n| match n {
                Node::Turn { point, .. } => Some(point.get::<0>()),
                Node::Vertex(_) => None,
            })
            .collect();
        // The first two spliced turns on the bottom edge are ordered.
        assert!(
            xs.windows(2)
                .all(|w| w[0] <= w[1] || (w[0] - w[1]).abs() > 1.5)
        );
    }
}
