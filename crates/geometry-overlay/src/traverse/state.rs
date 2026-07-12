//! OVL3.T1 / OVL3.T2 — the traversal state machine and forward walk.
//!
//! Mirrors `boost/geometry/algorithms/detail/overlay/traverse.hpp`
//! (`traverse::apply`). Given the enriched rings and the requested
//! overlay operation, walks the turn graph and emits output rings.
//!
//! # The walk
//!
//! For the areal intersection / union of two simple polygons whose
//! boundaries cross transversally, the crossing turns alternate between
//! *entries* (the boundary of A enters B) and *exits*. The Weiler–
//! Atherton walk is:
//!
//! 1. pick an unvisited crossing turn,
//! 2. follow the current ring forward, appending nodes, until the next
//!    crossing turn,
//! 3. at that turn **switch** to the other ring,
//! 4. stop when the walk returns to the start turn — one output ring is
//!    complete,
//! 5. repeat from (1) until no unvisited crossing turn remains.
//!
//! The direction chosen at the *start* turn selects intersection vs
//! union: intersection follows the arc that dives into the other
//! polygon, union the arc that stays outside. Both are realised by
//! choosing which ring to begin the walk on, from the turn's
//! [`OperationType`](crate::turn::OperationType).

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_model::Ring;
use geometry_trait::{Point, PointMut};

use super::enrich::{EnrichedRings, Node};
use crate::turn::info::Turn;

/// Which boolean overlay to traverse for.
///
/// Mirrors Boost's `overlay_type` / `operation_type` steering
/// (`overlay_type.hpp`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverlayOp {
    /// Keep the region inside both inputs.
    Intersection,
    /// Keep the region inside either input.
    Union,
    /// Keep the region inside the first input but outside the second
    /// (`A − B`). The walk follows `A` forward along its arcs that lie
    /// outside `B`, and follows `B` **backward** along the arcs that
    /// bound the removed overlap.
    Difference,
}

/// A traversal that could not be completed with the v1 non-degenerate
/// walk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraversalError {
    /// The turn graph hit a case the clean areal walk does not handle
    /// (clustered turns, self-intersections, or a walk that failed to
    /// close). These are the deferred Boost sub-problems — see the
    /// module docs.
    Unsupported,
}

/// Walk the enriched turn graph and emit the output rings for `op`.
///
/// Returns one [`Ring`] per closed output loop. An empty result means
/// the inputs do not overlap (for intersection) or the walk produced no
/// bounded region.
///
/// Mirrors `traverse::apply` (`traverse.hpp`). v1 scope: simple areal
/// inputs with transversal crossings; anything else yields
/// [`TraversalError::Unsupported`] rather than a wrong ring.
///
/// # Errors
///
/// Returns [`TraversalError::Unsupported`] when the turn graph hits a
/// degenerate case the clean areal walk does not handle — clustered
/// turns, self-intersections, collinear shared edges, or a walk that
/// fails to close. These are the deferred Boost sub-problems.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{Point2D, Ring};
/// use geometry_overlay::turn::{get_turns_ring_ring, RingKind};
/// use geometry_overlay::traverse::{enrich, traverse, OverlayOp};
/// use geometry_trait::Ring as _;
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
/// let rings = traverse(&enriched, &turns, OverlayOp::Intersection).unwrap();
/// // The intersection of the two offset squares is a single square.
/// assert_eq!(rings.len(), 1);
/// assert_eq!(rings[0].points().count(), 5); // 4 corners + closing point
/// ```
pub fn traverse<P>(
    enriched: &EnrichedRings<P>,
    turns: &[Turn<P>],
    op: OverlayOp,
) -> Result<Vec<Ring<P>>, TraversalError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    let mut visited = alloc::vec![false; turns.len()];
    let mut rings = Vec::new();

    for start_turn in 0..turns.len() {
        if visited[start_turn] {
            continue;
        }
        match walk_from(enriched, start_turn, op, &mut visited) {
            Some(ring) => rings.push(ring),
            None => return Err(TraversalError::Unsupported),
        }
    }
    Ok(rings)
}

/// Walk one output ring starting at `start_turn`. Returns `None` if the
/// walk fails to close within the node budget or hits a configuration
/// the v1 clean areal walk does not support.
///
/// The walk is a directed Weiler–Atherton trace. State is a
/// `(ring, position, direction)` triple; between two consecutive nodes
/// the walk moves along a directed segment. At each **turn** it may
/// switch to the other ring and reverse direction; it picks the outgoing
/// segment whose midpoint lies in the target region — *inside* the other
/// polygon for [`OverlayOp::Intersection`], *outside* it for
/// [`OverlayOp::Union`] / [`OverlayOp::Difference`] — and that is not the
/// reverse of the segment it arrived on. Plain vertices are passed
/// straight through. The walk ends when it returns to `start_turn`.
///
/// This selection-by-region rule is what makes a region with more than
/// two crossings (e.g. the hexagonal overlap of two triangles) trace as
/// a single ring instead of fragmenting: at every crossing the walk
/// commits to the edge that keeps it inside the wanted region, rather
/// than switching blindly.
fn walk_from<P>(
    enriched: &EnrichedRings<P>,
    start_turn: usize,
    op: OverlayOp,
    visited: &mut [bool],
) -> Option<Ring<P>>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    // Seed the walk: pick the (ring, direction) whose first segment out
    // of the start turn heads into the target region.
    let (mut ring_index, mut dir) = choose_start_step(enriched, start_turn, op)?;
    let mut pos = enriched.locate_turn(ring_index, start_turn)?;

    let mut out: Vec<P> = Vec::new();
    let total_nodes: usize = enriched.rings[0].len() + enriched.rings[1].len();
    // A correct walk visits each directed segment at most once; give a
    // generous cap so a stuck walk terminates as unsupported.
    let budget = 2 * total_nodes + 2;

    let mut closed = false;
    for step in 0..budget {
        let node = &enriched.rings[ring_index][pos];
        out.push(*node.point());

        // At a turn (after the start), decide the outgoing segment.
        if let Some(turn_id) = node.turn_id() {
            visited[turn_id] = true;
            if step != 0 {
                let (nr, np, nd) = step_at_turn(enriched, ring_index, pos, dir, op)?;
                ring_index = nr;
                pos = np;
                dir = nd;
            }
        }
        // Advance one node in the current direction.
        pos = step_index(pos, dir, enriched.rings[ring_index].len());

        // Closed once the next node to append is the start turn again.
        if enriched.rings[ring_index][pos].turn_id() == Some(start_turn) {
            closed = true;
            break;
        }
    }

    // Honour the contract: a walk that did not actually close is an
    // unsupported degenerate case, not a garbage ring.
    if !closed || out.len() < 3 {
        return None;
    }
    // Close the ring by repeating the first point (model convention).
    out.push(out[0]);
    let ring: Ring<P> = Ring::from_vec(out);
    Some(ring)
}

/// Advance one node index in `dir` (`+1` forward, `-1` backward),
/// wrapping cyclically over a ring of length `len`.
fn step_index(pos: usize, dir: i8, len: usize) -> usize {
    if dir >= 0 {
        (pos + 1) % len
    } else {
        (pos + len - 1) % len
    }
}

/// At a turn reached on `ring_index` at `pos` having travelled in `dir`,
/// choose the outgoing `(ring, position, direction)`.
///
/// Considers switching to the other ring in either direction (and, as a
/// fallback, continuing on the same ring). Picks the candidate whose
/// segment midpoint lies in the target region and that is not the reverse
/// of the arriving segment. Returns `None` if no candidate qualifies —
/// an unsupported degenerate turn.
fn step_at_turn<P>(
    enriched: &EnrichedRings<P>,
    ring_index: usize,
    pos: usize,
    dir: i8,
    op: OverlayOp,
) -> Option<(usize, usize, i8)>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    let turn_id = enriched.rings[ring_index][pos].turn_id()?;
    let other = 1 - ring_index;
    let other_pos = enriched.locate_turn(other, turn_id)?;
    let here = *enriched.rings[ring_index][pos].point();

    // Candidate outgoing segments, preferring a switch to the other ring.
    let candidates = [
        (other, other_pos, 1i8),
        (other, other_pos, -1i8),
        (ring_index, pos, dir),
    ];
    for &(r, p, d) in &candidates {
        let next = step_index(p, d, enriched.rings[r].len());
        let there = *enriched.rings[r][next].point();
        let mid = midpoint::<P>(&here, &there);
        if segment_in_region(enriched, r, &mid, op) {
            return Some((r, p, d));
        }
    }
    None
}

/// Whether a segment lying on ring `r`, sampled at `mid`, belongs to the
/// boundary of the target-op output region.
///
/// * **Intersection** — the segment must run *inside* the other input.
/// * **Union** — *outside* the other input.
/// * **Difference** (`A − B`, ring 0 = A, ring 1 = B) — an A-segment must
///   run *outside* B, and a B-segment *inside* A. That asymmetry is what
///   selects `A`'s outer arcs together with `B`'s arcs that cut into `A`.
fn segment_in_region<P>(enriched: &EnrichedRings<P>, r: usize, mid: &P, op: OverlayOp) -> bool
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let inside_other = point_in_ring(mid, &enriched.rings[1 - r]);
    match op {
        OverlayOp::Intersection => inside_other,
        OverlayOp::Union => !inside_other,
        OverlayOp::Difference => {
            if r == 0 {
                // A-segment: keep it where it is outside B.
                !inside_other
            } else {
                // B-segment: keep it where it is inside A.
                inside_other
            }
        }
    }
}

/// Midpoint of two points.
fn midpoint<P>(a: &P, b: &P) -> P
where
    P: PointMut + Default,
    P::Scalar: CoordinateScalar,
{
    let two = P::Scalar::ONE + P::Scalar::ONE;
    let mut m = P::default();
    m.set::<0>((a.get::<0>() + b.get::<0>()) / two);
    m.set::<1>((a.get::<1>() + b.get::<1>()) / two);
    m
}

/// Choose the starting `(ring, direction)` whose first segment out of the
/// start turn heads into the target region.
fn choose_start_step<P>(
    enriched: &EnrichedRings<P>,
    start_turn: usize,
    op: OverlayOp,
) -> Option<(usize, i8)>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    for ring_index in [0usize, 1] {
        let at = enriched.locate_turn(ring_index, start_turn)?;
        let here = *enriched.rings[ring_index][at].point();
        for dir in [1i8, -1] {
            let next = step_index(at, dir, enriched.rings[ring_index].len());
            let there = *enriched.rings[ring_index][next].point();
            let mid = midpoint::<P>(&here, &there);
            if segment_in_region(enriched, ring_index, &mid, op) {
                return Some((ring_index, dir));
            }
        }
    }
    None
}

/// Point-in-ring test by ray casting over the enriched ring's node
/// points (odd crossings ⇒ inside). Boundary-exact cases are not
/// expected here because the probe is a segment *midpoint*.
fn point_in_ring<P>(p: &P, ring: &[Node<P>]) -> bool
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let px = p.get::<0>();
    let py = p.get::<1>();
    let pts: Vec<&P> = ring.iter().map(Node::point).collect();
    let n = pts.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let xi = pts[i].get::<0>();
        let yi = pts[i].get::<1>();
        let xj = pts[j].get::<0>();
        let yj = pts[j].get::<1>();
        let crosses = (yi > py) != (yj > py);
        if crosses {
            // x-coordinate of the edge at height py.
            let t = (py - yi) / (yj - yi);
            let x_at = xi + t * (xj - xi);
            if px < x_at {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::{OverlayOp, traverse};
    use crate::traverse::enrich::enrich;
    use crate::turn::{RingKind, get_turns_ring_ring};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Ring};
    use geometry_trait::{Point as _, Ring as _};

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
    fn intersection_of_offset_squares_is_one_ring() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        let turns = get_turns_ring_ring(&a, 0, RingKind::Exterior, &b, 1, RingKind::Exterior);
        let e = enrich(&a, &b, &turns);
        let rings = traverse(&e, &turns, OverlayOp::Intersection).unwrap();
        assert_eq!(rings.len(), 1);
        // The overlap of [0,2]² and [1,3]² is the unit square [1,2]².
        let ring = &rings[0];
        let pts: Vec<(f64, f64)> = ring
            .points()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        // Every vertex is a corner of [1,2]².
        for (x, y) in &pts {
            assert!((*x - 1.0).abs() < 1e-9 || (*x - 2.0).abs() < 1e-9, "x={x}");
            assert!((*y - 1.0).abs() < 1e-9 || (*y - 2.0).abs() < 1e-9, "y={y}");
        }
        // 4 distinct corners + closing repeat.
        assert_eq!(ring.points().count(), 5);
    }
}
