//! Planar arrangement kernel for polygon Boolean operations.
//!
//! Mirrors the combined role of Boost.Geometry's overlay turn collection,
//! colocation handling, enrichment, traversal, and ring selection. Every
//! source boundary is split at crossings and collinear-overlap endpoints;
//! the two sides of each atomic edge are classified against the requested
//! Boolean operation, leaving a directed result-boundary graph to trace.

#![allow(
    clippy::float_cmp,
    reason = "exact equality is used only to recognize identical stored vertices and explicit ring closure"
)]

use alloc::vec::Vec;

use geometry_coords::{
    CoordinateScalar,
    math::{atan2, hypot},
};
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{MultiPolygon, Polygon, Ring, Segment};
use geometry_tag::SameAs;
use geometry_trait::{
    MultiPolygon as MultiPolygonTrait, Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait,
};

use crate::assemble::assemble_traced;
use crate::operation::OverlayError;
use crate::predicate::segment_intersection::{SegmentIntersection, segment_intersection};

/// Boolean truth table applied to the two polygon interiors.
#[derive(Debug, Clone, Copy)]
pub(crate) enum ArealOp {
    Intersection,
    Union,
    Difference,
    SymDifference,
}

impl ArealOp {
    fn apply(self, first: bool, second: bool) -> bool {
        match self {
            Self::Intersection => first && second,
            Self::Union => first || second,
            Self::Difference => first && !second,
            Self::SymDifference => first != second,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Coordinate {
    x: f64,
    y: f64,
}

impl Coordinate {
    fn from_point<P>(point: &P) -> Self
    where
        P: Point,
        P::Scalar: Into<f64>,
    {
        Self {
            x: point.get::<0>().into(),
            y: point.get::<1>().into(),
        }
    }
}

struct Shape {
    rings: Vec<Vec<Coordinate>>,
}

impl Shape {
    fn from_polygon<G, P>(polygon: &G) -> Self
    where
        G: PolygonTrait<Point = P>,
        P: Point,
        P::Scalar: Into<f64>,
    {
        let mut rings = Vec::new();
        rings.push(ring_coordinates(polygon.exterior()));
        rings.extend(polygon.interiors().map(ring_coordinates));
        Self { rings }
    }

    fn from_multi_polygon<G, P>(multi_polygon: &G) -> Self
    where
        G: MultiPolygonTrait<Point = P>,
        P: Point,
        P::Scalar: Into<f64>,
    {
        let mut rings = Vec::new();
        for polygon in multi_polygon.polygons() {
            rings.push(ring_coordinates(polygon.exterior()));
            rings.extend(polygon.interiors().map(ring_coordinates));
        }
        Self { rings }
    }

    fn contains(&self, point: Coordinate) -> bool {
        self.rings
            .iter()
            .fold(false, |inside, ring| inside != ring_contains(ring, point))
    }
}

struct SourceSegment<P> {
    start: P,
    end: P,
    /// `(parameter, point, is_turn)`. The two endpoints are not turns; a point
    /// pushed by the intersection sweep is, including one that lands on an
    /// endpoint.
    splits: Vec<(f64, P, bool)>,
}

impl<P> SourceSegment<P>
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    fn new(start: P, end: P) -> Self {
        Self {
            start,
            end,
            splits: alloc::vec![(0.0, start, false), (1.0, end, false)],
        }
    }

    fn push_split(&mut self, point: P, tolerance: f64) {
        let parameter = segment_parameter(&self.start, &self.end, &point);
        if parameter < -tolerance || parameter > 1.0 + tolerance {
            return;
        }
        if let Some(existing) = self
            .splits
            .iter_mut()
            .find(|(at, _, _)| (at - parameter).abs() <= tolerance)
        {
            // A crossing that lands on a vertex already split here still makes
            // that vertex a turn.
            existing.2 = true;
            return;
        }
        self.splits.push((parameter.clamp(0.0, 1.0), point, true));
    }
}

struct Node<P> {
    point: P,
    coordinate: Coordinate,
    /// Set when any split that resolved to this node was a crossing. Boost
    /// starts each output ring at a turn, so the tracer needs to know which
    /// nodes are turns; see `push_ring`.
    is_turn: bool,
    /// Where this node sits along **each** operand's boundary — and, where it
    /// lands exactly on a vertex, counted as the *end* of the segment arriving
    /// there rather than the start of the one leaving.
    ///
    /// That normalisation is Boost's: `get_turns` attaches an intersection at
    /// a segment endpoint to the segment it terminates, so a turn on an
    /// operand's *first* vertex is the last position on that ring, not the
    /// first.
    ///
    /// Both entries matter. Boost walks the first operand's sections in the
    /// outer loop and the second operand's in the inner, so its turns come out
    /// ordered by the pair, and two turns on the same stretch of the first
    /// operand are separated by where they sit on the second. `usize::MAX`
    /// means the operand's boundary does not pass through this node, which is
    /// true of every vertex that is not a turn.
    arrival: [usize; 2],
    /// How far along that segment. It orders two turns only once both segment
    /// indices have tied — the fraction must not outrank the second operand,
    /// or two turns sharing one edge come out in the wrong order.
    offset: [f64; 2],
    /// Whether this node is a *vertex* of each operand, as against a point
    /// that only lies on its boundary because the other operand touches there.
    is_vertex: [bool; 2],
}

#[derive(Debug, Clone, Copy)]
struct Edge {
    start: usize,
    end: usize,
    /// Which operands' boundaries run along this edge. A stretch the two share
    /// is carried by both.
    ///
    /// Boost walks one operand at a time between turns and copies *that*
    /// operand's vertices, so a point sitting inside a segment of the operand
    /// being walked never reaches the output, whoever else has a vertex there.
    /// Reproducing that needs to know who carries each edge — see
    /// `drop_points_interior_to_a_walked_segment`.
    carried_by: [bool; 2],
}

impl Edge {
    fn joins(&self, other: &Self) -> bool {
        self.start == other.start && self.end == other.end
    }
}

/// Execute a polygon Boolean operation through a split-edge arrangement.
pub(crate) fn overlay<G1, G2, P>(
    first: &G1,
    second: &G2,
    operation: ArealOp,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    overlay_arrangement(
        &Shape::from_polygon(first),
        &Shape::from_polygon(second),
        polygon_segments(first),
        polygon_segments(second),
        operation,
    )
}

/// The same operation over multi-polygons.
///
/// Boost dispatches every areal Boolean through one overlay regardless of how
/// many polygons each operand holds, so this is the same kernel over the union
/// of every operand's rings rather than a second algorithm. A single polygon
/// is the one-member case.
pub(crate) fn overlay_multi<G1, G2, P>(
    first: &G1,
    second: &G2,
    operation: ArealOp,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: MultiPolygonTrait<Point = P>,
    G2: MultiPolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    overlay_arrangement(
        &Shape::from_multi_polygon(first),
        &Shape::from_multi_polygon(second),
        multi_polygon_segments(first),
        multi_polygon_segments(second),
        operation,
    )
}

fn overlay_arrangement<P>(
    first_shape: &Shape,
    second_shape: &Shape,
    mut first_segments: Vec<SourceSegment<P>>,
    mut second_segments: Vec<SourceSegment<P>>,
    operation: ArealOp,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let scale = coordinate_scale(first_shape, second_shape);
    let snap_tolerance = scale * 1e-10;
    let parameter_tolerance = 1e-10;

    for first_segment in &mut first_segments {
        for second_segment in &mut second_segments {
            let first_model = Segment::new(first_segment.start, first_segment.end);
            let second_model = Segment::new(second_segment.start, second_segment.end);
            match segment_intersection(&first_model, &second_model) {
                SegmentIntersection::Disjoint => {}
                SegmentIntersection::Single(point) => {
                    first_segment.push_split(point, parameter_tolerance);
                    second_segment.push_split(point, parameter_tolerance);
                }
                SegmentIntersection::Collinear { from, to } => {
                    first_segment.push_split(from, parameter_tolerance);
                    first_segment.push_split(to, parameter_tolerance);
                    second_segment.push_split(from, parameter_tolerance);
                    second_segment.push_split(to, parameter_tolerance);
                }
                SegmentIntersection::OutOfRange => return Err(OverlayError::Unsupported),
            }
        }
    }

    let mut nodes = Vec::new();
    let mut candidates = Vec::new();
    append_atomic_edges(
        &mut first_segments,
        &mut nodes,
        &mut candidates,
        0,
        snap_tolerance,
    );
    append_atomic_edges(
        &mut second_segments,
        &mut nodes,
        &mut candidates,
        1,
        snap_tolerance,
    );

    let sample_distance = (scale * 1e-8).max(snap_tolerance * 32.0);
    let mut boundary: Vec<Edge> = Vec::new();
    for candidate in candidates {
        let start = nodes[candidate.start].coordinate;
        let end = nodes[candidate.end].coordinate;
        let delta = (end.x - start.x, end.y - start.y);
        let length = hypot(delta.0, delta.1);
        debug_assert!(length > snap_tolerance);
        let midpoint = Coordinate {
            x: (start.x + end.x) * 0.5,
            y: (start.y + end.y) * 0.5,
        };
        let offset = sample_distance.min(length * 1e-4);
        let normal = (-delta.1 / length * offset, delta.0 / length * offset);
        let left = Coordinate {
            x: midpoint.x + normal.0,
            y: midpoint.y + normal.1,
        };
        let right = Coordinate {
            x: midpoint.x - normal.0,
            y: midpoint.y - normal.1,
        };
        let left_result = operation.apply(first_shape.contains(left), second_shape.contains(left));
        let right_result =
            operation.apply(first_shape.contains(right), second_shape.contains(right));
        if left_result == right_result {
            continue;
        }
        let edge = if left_result {
            candidate
        } else {
            Edge {
                start: candidate.end,
                end: candidate.start,
                carried_by: candidate.carried_by,
            }
        };
        // The same stretch reaches here once per operand that carries it, so
        // merge rather than drop the second: who carries an edge is what says
        // whether a point on it is interior to a walked segment.
        match boundary.iter_mut().find(|held| held.joins(&edge)) {
            Some(held) => {
                held.carried_by[0] |= edge.carried_by[0];
                held.carried_by[1] |= edge.carried_by[1];
            }
            None => boundary.push(edge),
        }
    }

    let rings = trace_rings(&nodes, &boundary, snap_tolerance)?;
    Ok(assemble_traced(rings))
}

fn ring_coordinates<R>(ring: &R) -> Vec<Coordinate>
where
    R: RingTrait,
    <R::Point as Point>::Scalar: Into<f64>,
{
    let mut coordinates: Vec<_> = ring.points().map(Coordinate::from_point).collect();
    if coordinates.len() >= 2 {
        let first = coordinates[0];
        let last = coordinates[coordinates.len() - 1];
        if first.x == last.x && first.y == last.y {
            coordinates.pop();
        }
    }
    coordinates
}

fn multi_polygon_segments<G, P>(multi_polygon: &G) -> Vec<SourceSegment<P>>
where
    G: MultiPolygonTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut segments = Vec::new();
    for polygon in multi_polygon.polygons() {
        append_ring_segments(polygon.exterior(), &mut segments);
        for ring in polygon.interiors() {
            append_ring_segments(ring, &mut segments);
        }
    }
    segments
}

fn polygon_segments<G, P>(polygon: &G) -> Vec<SourceSegment<P>>
where
    G: PolygonTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut segments = Vec::new();
    append_ring_segments(polygon.exterior(), &mut segments);
    for ring in polygon.interiors() {
        append_ring_segments(ring, &mut segments);
    }
    segments
}

fn append_ring_segments<R, P>(ring: &R, output: &mut Vec<SourceSegment<P>>)
where
    R: RingTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let points: Vec<P> = ring.points().copied().collect();
    if points.len() < 2 {
        return;
    }
    for pair in points.windows(2) {
        if points_differ(&pair[0], &pair[1]) {
            output.push(SourceSegment::new(pair[0], pair[1]));
        }
    }
    if points_differ(points.last().expect("nonempty"), &points[0]) {
        output.push(SourceSegment::new(
            *points.last().expect("nonempty"),
            points[0],
        ));
    }
}

fn append_atomic_edges<P>(
    segments: &mut [SourceSegment<P>],
    nodes: &mut Vec<Node<P>>,
    output: &mut Vec<Edge>,
    operand: usize,
    tolerance: f64,
) where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    for (index, segment) in segments.iter_mut().enumerate() {
        segment
            .splits
            .sort_by(|left, right| left.0.total_cmp(&right.0));
        for pair in segment.splits.windows(2) {
            debug_assert!((pair[1].0 - pair[0].0).abs() > 1e-12);
            let start = canonical_node(nodes, pair[0].1, pair[0].2, tolerance);
            let end = canonical_node(nodes, pair[1].1, pair[1].2, tolerance);
            // Only the far end of a split counts as an arrival, which is what
            // pushes a ring's first vertex to the end of its own ring: it is
            // reached as the last segment's endpoint, not the first's start.
            if nodes[end].arrival[operand] == usize::MAX {
                nodes[end].arrival[operand] = index;
                nodes[end].offset[operand] = pair[1].0;
            }
            if pair[0].0 == 0.0 || pair[0].0 == 1.0 {
                nodes[start].is_vertex[operand] = true;
            }
            if pair[1].0 == 0.0 || pair[1].0 == 1.0 {
                nodes[end].is_vertex[operand] = true;
            }
            if start != end {
                let mut carried_by = [false; 2];
                carried_by[operand] = true;
                output.push(Edge {
                    start,
                    end,
                    carried_by,
                });
            }
        }
    }
}

fn canonical_node<P>(nodes: &mut Vec<Node<P>>, point: P, is_turn: bool, tolerance: f64) -> usize
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let coordinate = Coordinate::from_point(&point);
    if let Some(index) = nodes.iter().position(|node| {
        hypot(
            node.coordinate.x - coordinate.x,
            node.coordinate.y - coordinate.y,
        ) <= tolerance
    }) {
        nodes[index].is_turn |= is_turn;
        return index;
    }
    nodes.push(Node {
        point,
        coordinate,
        is_turn,
        arrival: [usize::MAX; 2],
        offset: [0.0; 2],
        is_vertex: [false; 2],
    });
    nodes.len() - 1
}

type TracedRing<P> = (Ring<P>, bool);

fn trace_rings<P>(
    nodes: &[Node<P>],
    edges: &[Edge],
    tolerance: f64,
) -> Result<Vec<TracedRing<P>>, OverlayError>
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut used = alloc::vec![false; edges.len()];
    // Each ring is kept with the node it starts at, so the whole set can be
    // put back into source order below, and with whether the traversal
    // assembled it from turns rather than copying it whole from one operand.
    let mut rings: Vec<(usize, Ring<P>, bool)> = Vec::new();
    for seed in 0..edges.len() {
        if used[seed] {
            continue;
        }
        let first = edges[seed].start;
        let mut edge_index = seed;
        let mut node_indices = alloc::vec![first];
        // `carried[i]` is who carries the edge from `node_indices[i]` to the
        // node after it, so it always has one entry fewer.
        let mut carried: alloc::vec::Vec<[bool; 2]> = alloc::vec::Vec::new();
        for _ in 0..=edges.len() {
            debug_assert!(!used[edge_index]);
            used[edge_index] = true;
            let edge = edges[edge_index];
            node_indices.push(edge.end);
            carried.push(edge.carried_by);

            // A node the walk has already stood on closes a ring right here,
            // not only when the walk returns to the seed. Where two lobes of
            // the result meet at a single point, the traversal passes through
            // that point twice; carrying on to the seed splices the lobes into
            // one self-touching ring, which is not a valid polygon and is not
            // what `boost::geometry::intersection` returns. Cut the loop out,
            // keep the path up to that node, and carry on walking.
            if let Some(start) = node_indices[..node_indices.len() - 1]
                .iter()
                .position(|&index| index == edge.end)
            {
                let loop_nodes = node_indices.split_off(start);
                let loop_carried = carried.split_off(start);
                node_indices.push(edge.end);
                push_ring(&mut rings, nodes, &loop_nodes, &loop_carried, tolerance);
            }

            if edge.end == first {
                break;
            }
            edge_index = next_edge(nodes, edges, &used, edge).ok_or(OverlayError::Unsupported)?;
        }
        debug_assert_eq!(node_indices.last().copied(), Some(first));
    }

    // Which ring comes out first is observable — it decides the order of the
    // polygons in the result — and Boost's is not the order the seeds happened
    // to fall in. It traces from turns in source order, so ordering the rings
    // by the node each one starts at reproduces it.
    rings.sort_by_key(|&(start, _, _)| start);
    Ok(rings
        .into_iter()
        .map(|(_, ring, traced)| (ring, traced))
        .collect())
}

/// Remove a point the first operand runs straight past.
///
/// This arrangement splits a segment wherever anything touches it, so a vertex
/// of one operand landing part-way along a segment of the other becomes a
/// point on the traced ring. Boost's traversal does not work that way: between
/// two turns it copies the vertices of the single operand it is walking, and a
/// point that operand has no vertex at is simply passed over.
///
/// Along a stretch both operands carry, the operand walked is the first, so
/// that is the one whose vertices survive. Two guards keep the rule honest:
/// the point must be collinear with its neighbours, because anywhere the
/// outline actually turns the point is part of the shape whoever owns it; and
/// a point the first operand does have a vertex at stays, because Boost would
/// copy it.
///
/// Not every such point is caught. Where the two operands run the same way
/// along a shared edge, Boost emits one turn for the overlap and puts it at
/// the far end, so the near end goes too — 12 of 112 shared-edge pairs still
/// differ on that, and closing them needs `get_turns`' own collinear turn
/// handling rather than a rule over the arrangement.
fn drop_points_interior_to_a_walked_segment<P>(
    nodes: &[Node<P>],
    cycle: &mut Vec<usize>,
    carried: &mut Vec<[bool; 2]>,
) where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut index = 0;
    while index < cycle.len() && cycle.len() > 3 {
        let length = cycle.len();
        let incoming = carried[(index + length - 1) % length];
        let outgoing = carried[index];
        let node = &nodes[cycle[index]];
        // Boost walks one operand at a time between turns and copies *that*
        // operand's vertices. Along a stretch both operands carry, it walks
        // the first — so a point the first operand runs straight past, having
        // no vertex there, never reaches the output however many vertices the
        // second has at it.
        let walked_over = incoming[0] && outgoing[0] && !node.is_vertex[0];
        if !walked_over {
            index += 1;
            continue;
        }
        let previous = nodes[cycle[(index + length - 1) % length]].coordinate;
        let here = node.coordinate;
        let next = nodes[cycle[(index + 1) % length]].coordinate;
        let side = (here.x - previous.x) * (next.y - previous.y)
            - (here.y - previous.y) * (next.x - previous.x);
        if side != 0.0 {
            index += 1;
            continue;
        }
        // The two edges become one, carried by whoever carried both halves.
        let merged = [incoming[0] && outgoing[0], incoming[1] && outgoing[1]];
        cycle.remove(index);
        carried.remove(index);
        let previous_edge = (index + cycle.len() - 1) % cycle.len();
        carried[previous_edge] = merged;
        index = 0;
    }
}

/// Append one traced cycle, dropping it when it encloses no area.
///
/// The cycle starts wherever the traversal happened to seed, which carries no
/// meaning — but which vertex a ring starts at is observable downstream, and
/// Boost's answer is not arbitrary: it begins each output ring at a *turn*, the
/// first one in source order. A ring with no turn at all was copied whole from
/// one operand and keeps that operand's own starting vertex. Reproduced here,
/// because a consumer that simplifies the ring afterwards will pin its first
/// vertex and the choice reaches the output.
fn push_ring<P>(
    rings: &mut Vec<(usize, Ring<P>, bool)>,
    nodes: &[Node<P>],
    node_indices: &[usize],
    carried: &[[bool; 2]],
    tolerance: f64,
) where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let area = node_indices.windows(2).fold(0.0, |sum, pair| {
        let a = nodes[pair[0]].coordinate;
        let b = nodes[pair[1]].coordinate;
        sum + a.x * b.y - b.x * a.y
    }) * 0.5;
    if area.abs() <= tolerance * tolerance {
        return;
    }

    // The cycle is closed, so its last index repeats its first.
    let mut cycle = node_indices[..node_indices.len() - 1].to_vec();
    let mut carried = carried.to_vec();
    drop_points_interior_to_a_walked_segment(nodes, &mut cycle, &mut carried);
    let cycle = &cycle[..];
    // Boost begins each output ring at the first *turn* along the first
    // operand's boundary — `Node::arrival`, which is that order with Boost's
    // endpoint normalisation applied.
    // Boost walks the first operand's sections in the outer loop and the
    // second's in the inner, so its turns are ordered by the pair.
    let first_turn_by_arrival = cycle
        .iter()
        .copied()
        .enumerate()
        .filter(|&(_, index)| nodes[index].is_turn)
        .min_by(|&(_, left), &(_, right)| {
            let (a, b) = (&nodes[left], &nodes[right]);
            a.arrival[0]
                .cmp(&b.arrival[0])
                .then(a.arrival[1].cmp(&b.arrival[1]))
                .then(a.offset[0].total_cmp(&b.offset[0]))
        })
        .map(|(position, _)| position);
    // A ring with no turn was copied whole from one operand, and keeps that
    // operand's own starting vertex rather than whichever end of it the
    // traversal happened to seed from — a hole is walked against its stored
    // direction, so those differ. That vertex is the one created first, which
    // is node order, not arrival order.
    let first_node = || {
        cycle
            .iter()
            .copied()
            .enumerate()
            .min_by_key(|&(_, index)| index)
            .map(|(position, _)| position)
    };
    let first_turn = first_turn_by_arrival.or_else(first_node).unwrap_or(0);

    let mut points: Vec<P> = cycle[first_turn..]
        .iter()
        .chain(&cycle[..first_turn])
        .map(|&index| nodes[index].point)
        .collect();
    if let Some(&first) = points.first() {
        points.push(first);
    }
    // The ring is cleaned once it is in its final winding, not here: which
    // vertex `clean_closing_dups_and_spikes` leaves at the front depends on
    // the direction the ring runs in, and that is decided in `assemble`.
    rings.push((
        cycle[first_turn],
        Ring::from_vec(points),
        first_turn_by_arrival.is_some(),
    ));
}

fn next_edge<P>(nodes: &[Node<P>], edges: &[Edge], used: &[bool], incoming: Edge) -> Option<usize>
where
    P: Point,
{
    let previous = nodes[incoming.start].coordinate;
    let vertex = nodes[incoming.end].coordinate;
    let incoming_direction = (vertex.x - previous.x, vertex.y - previous.y);
    edges
        .iter()
        .enumerate()
        .filter(|(index, edge)| !used[*index] && edge.start == incoming.end)
        .min_by(|(_, left), (_, right)| {
            let left_turn = turn_angle(incoming_direction, vertex, nodes[left.end].coordinate);
            let right_turn = turn_angle(incoming_direction, vertex, nodes[right.end].coordinate);
            left_turn.total_cmp(&right_turn)
        })
        .map(|(index, _)| index)
}

fn turn_angle(incoming: (f64, f64), vertex: Coordinate, next: Coordinate) -> f64 {
    let outgoing = (next.x - vertex.x, next.y - vertex.y);
    let cross = incoming.0 * outgoing.1 - incoming.1 * outgoing.0;
    let dot = incoming.0 * outgoing.0 + incoming.1 * outgoing.1;
    let angle = atan2(cross, dot);
    if angle < 0.0 {
        angle + core::f64::consts::TAU
    } else {
        angle
    }
}

fn segment_parameter<P>(start: &P, end: &P, point: &P) -> f64
where
    P: Point,
    P::Scalar: Into<f64>,
{
    let start = Coordinate::from_point(start);
    let end = Coordinate::from_point(end);
    let point = Coordinate::from_point(point);
    let delta = (end.x - start.x, end.y - start.y);
    if delta.0.abs() >= delta.1.abs() {
        debug_assert_ne!(delta.0, 0.0);
        (point.x - start.x) / delta.0
    } else {
        debug_assert_ne!(delta.1, 0.0);
        (point.y - start.y) / delta.1
    }
}

fn points_differ<P>(first: &P, second: &P) -> bool
where
    P: Point,
    P::Scalar: Into<f64>,
{
    let first = Coordinate::from_point(first);
    let second = Coordinate::from_point(second);
    first.x != second.x || first.y != second.y
}

fn ring_contains(ring: &[Coordinate], point: Coordinate) -> bool {
    let mut inside = false;
    for index in 0..ring.len() {
        let start = ring[index];
        let end = ring[(index + 1) % ring.len()];
        if (start.y > point.y) != (end.y > point.y)
            && point.x < (end.x - start.x) * (point.y - start.y) / (end.y - start.y) + start.x
        {
            inside = !inside;
        }
    }
    inside
}

fn coordinate_scale(first: &Shape, second: &Shape) -> f64 {
    first
        .rings
        .iter()
        .chain(&second.rings)
        .flatten()
        .fold(1.0_f64, |scale, coordinate| {
            scale.max(coordinate.x.abs()).max(coordinate.y.abs())
        })
}

#[cfg(test)]
mod tests {
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    use super::{Coordinate, Edge, Node, trace_rings};

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn trace_rings_discards_a_closed_zero_area_cycle() {
        let nodes = [
            Node {
                point: P::new(0.0, 0.0),
                coordinate: Coordinate { x: 0.0, y: 0.0 },
                is_turn: false,
                arrival: [0, 0],
                offset: [0.0; 2],
                is_vertex: [true; 2],
            },
            Node {
                point: P::new(1.0, 0.0),
                coordinate: Coordinate { x: 1.0, y: 0.0 },
                is_turn: false,
                arrival: [1, 1],
                offset: [0.0; 2],
                is_vertex: [true; 2],
            },
            Node {
                point: P::new(2.0, 0.0),
                coordinate: Coordinate { x: 2.0, y: 0.0 },
                is_turn: false,
                arrival: [2, 2],
                offset: [0.0; 2],
                is_vertex: [true; 2],
            },
        ];
        let edges = [
            Edge {
                start: 0,
                end: 1,
                carried_by: [true; 2],
            },
            Edge {
                start: 1,
                end: 2,
                carried_by: [true; 2],
            },
            Edge {
                start: 2,
                end: 0,
                carried_by: [true; 2],
            },
        ];

        assert!(trace_rings(&nodes, &edges, 1e-10).unwrap().is_empty());
    }
}
