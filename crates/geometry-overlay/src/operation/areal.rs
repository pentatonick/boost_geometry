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
use core::cmp::Ordering;

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
use crate::operation::section_partition::{Bounds, VisitRank};
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

    /// Whether the second operand's rings are walked backwards.
    ///
    /// C++: `difference` dispatches the overlay with `Reverse2 = true`, so
    /// `sectionalize` reads that operand through a reversed view and every
    /// section and segment index it hands to `get_turns` counts from the other
    /// end of the ring. That is what orders the turns, so it decides which one
    /// a result ring starts at. Nothing else here depends on the direction:
    /// the arrangement reorients each edge by which side the result lies on.
    fn walks_second_operand_backwards(self) -> bool {
        matches!(self, Self::Difference)
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
    /// Which monotone run of its ring this segment belongs to. C++:
    /// `sectionalize`, whose sections are what `get_turns` iterates over.
    section: usize,
}

impl<P> SourceSegment<P>
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    fn new(start: P, end: P, section: usize) -> Self {
        Self {
            start,
            end,
            splits: alloc::vec![(0.0, start, false), (1.0, end, false)],
            section,
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
    /// The section of each operand that reaches this node, which outranks the
    /// segment: `get_turns` partitions both operands into sections first and
    /// walks the pairs, so two turns in the same pair of sections keep their
    /// segment order while turns in different pairs do not.
    section: [usize; 2],
    /// Where that pair of sections falls in the order `partition` visits them
    /// — the whole of a turn's position in `m_turns`, above its segments.
    /// `usize::MAX` until the arrangement knows both operands.
    pair_rank: usize,
    /// How far along that segment. It orders two turns only once both segment
    /// indices have tied — the fraction must not outrank the second operand,
    /// or two turns sharing one edge come out in the wrong order.
    offset: [f64; 2],
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
    /// Which section of each operand runs along it, `usize::MAX` for an
    /// operand that does not. Sections never span a ring, so the lowest one on
    /// a cycle names the ring the cycle came out of — which is the whole of
    /// `ring_identifier` for a ring nothing crossed.
    section: [usize; 2],
}

impl Edge {
    fn joins(&self, other: &Self) -> bool {
        self.start == other.start && self.end == other.end
    }
}

/// Where a turn sits in `get_turns`' collection order.
///
/// C++: `partition` decides which pair of sections is looked at when, and
/// inside a pair `get_turns_in_sections` walks the first section's segments
/// outer and the second's inner — so a turn's place in `m_turns` is the pair's
/// rank and then that nesting, and `traverse` starts its rings in `m_turns`
/// order.
#[derive(Clone, Copy)]
struct TurnOrder {
    pair_rank: usize,
    arrivals: [usize; 2],
    offset: f64,
}

impl TurnOrder {
    fn of<P>(node: &Node<P>) -> Self {
        Self {
            pair_rank: node.pair_rank,
            arrivals: node.arrival,
            offset: node.offset[0],
        }
    }

    fn compare(&self, other: &Self) -> Ordering {
        self.pair_rank
            .cmp(&other.pair_rank)
            .then_with(|| self.arrivals.cmp(&other.arrivals))
            .then_with(|| self.offset.total_cmp(&other.offset))
    }
}

/// Where a finished ring falls in the output.
///
/// C++: `add_rings` walks the selected rings in `ring_identifier` order. A
/// ring copied whole from an operand carries that operand's own identifier —
/// source, then position within it — so every one of those precedes the
/// traversed rings and they keep the operand's own order; a traversed ring is
/// identified by when `traverse` started it, which is where `get_turns` put
/// the turn it started from.
struct RingStart {
    traversed: bool,
    source: usize,
    ring: usize,
    turn: TurnOrder,
    second_operand: bool,
    node: usize,
}

impl RingStart {
    fn compare(&self, other: &Self) -> Ordering {
        self.traversed.cmp(&other.traversed).then_with(|| {
            if self.traversed {
                self.turn
                    .compare(&other.turn)
                    .then_with(|| self.second_operand.cmp(&other.second_operand))
                    .then_with(|| self.node.cmp(&other.node))
            } else {
                // Untouched rings are ordered by their identifier alone, which
                // has nothing to do with where a turn fell.
                self.source
                    .cmp(&other.source)
                    .then_with(|| self.ring.cmp(&other.ring))
            }
        })
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
        polygon_segments(first, false),
        polygon_segments(second, operation.walks_second_operand_backwards()),
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
        multi_polygon_segments(first, false),
        multi_polygon_segments(second, operation.walks_second_operand_backwards()),
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

    // C++: `partition` is handed the two section lists, and the order it
    // visits their pairs in is the order the turns end up in.
    let ranks = VisitRank::of(
        &section_bounds(&first_segments),
        &section_bounds(&second_segments),
    );
    for node in &mut nodes {
        node.pair_rank = ranks.rank(node.section[0], node.section[1]);
    }

    let sample_distance = (scale * 1e-8).max(snap_tolerance * 32.0);
    let mut boundary: Vec<Edge> = Vec::new();
    for candidate in candidates {
        let start = nodes[candidate.start].coordinate;
        let end = nodes[candidate.end].coordinate;
        let delta = (end.x - start.x, end.y - start.y);
        let length = hypot(delta.0, delta.1);
        debug_assert!(length > snap_tolerance);
        let midpoint = Coordinate {
            x: f64::midpoint(start.x, end.x),
            y: f64::midpoint(start.y, end.y),
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
        // Oriented so the filled side is on the right, which walks an outer
        // ring clockwise and a hole counter-clockwise — the directions Boost's
        // traversal produces, and the ones `append_no_collinear` and
        // `clean_closing_dups_and_spikes` are written against. Both look at
        // the point *before* the one they judge, so a ring traced the other
        // way round drops the vertex at the far end of a straight run instead
        // of the near one.
        let edge = if right_result {
            candidate
        } else {
            Edge {
                start: candidate.end,
                end: candidate.start,
                carried_by: candidate.carried_by,
                section: candidate.section,
            }
        };
        // The same stretch reaches here once per operand that carries it, so
        // merge rather than drop the second: who carries an edge is what says
        // whether a point on it is interior to a walked segment.
        match boundary.iter_mut().find(|held| held.joins(&edge)) {
            Some(held) => {
                held.carried_by[0] |= edge.carried_by[0];
                held.carried_by[1] |= edge.carried_by[1];
                held.section[0] = held.section[0].min(edge.section[0]);
                held.section[1] = held.section[1].min(edge.section[1]);
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

fn multi_polygon_segments<G, P>(multi_polygon: &G, backwards: bool) -> Vec<SourceSegment<P>>
where
    G: MultiPolygonTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut segments = Vec::new();
    let mut sections = Sectionizer::new(0);
    for polygon in multi_polygon.polygons() {
        append_ring_segments(polygon.exterior(), &mut segments, &mut sections, backwards);
        for ring in polygon.interiors() {
            append_ring_segments(ring, &mut segments, &mut sections, backwards);
        }
    }
    segments
}

fn polygon_segments<G, P>(polygon: &G, backwards: bool) -> Vec<SourceSegment<P>>
where
    G: PolygonTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut segments = Vec::new();
    let mut sections = Sectionizer::new(0);
    append_ring_segments(polygon.exterior(), &mut segments, &mut sections, backwards);
    for ring in polygon.interiors() {
        append_ring_segments(ring, &mut segments, &mut sections, backwards);
    }
    segments
}

/// The box of every section, in section order.
///
/// C++: each `section` carries the `bounding_box` `sectionalize` expanded over
/// its segments, and that box is the whole of what `partition` reasons about.
fn section_bounds<P>(segments: &[SourceSegment<P>]) -> Vec<Bounds>
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut bounds: Vec<Bounds> = Vec::new();
    for segment in segments {
        let box_ = Bounds::around(
            [
                segment.start.get::<0>().into(),
                segment.start.get::<1>().into(),
            ],
            [segment.end.get::<0>().into(), segment.end.get::<1>().into()],
        );
        match bounds.get_mut(segment.section) {
            Some(held) => held.expand(&box_),
            // Sections are numbered from zero and in order, so a segment
            // either extends the section being built or opens the next one.
            None => bounds.push(box_),
        }
    }
    bounds
}

/// C++: `sectionalize`'s cap, "defaults to 10, this seems to give the fastest
/// results".
const MAX_SEGMENTS_PER_SECTION: usize = 10;

/// A section is a run of consecutive segments heading the same way in both
/// dimensions. C++: `sectionalize`, which starts a new one whenever the pair
/// of signs changes, or the run grows past `max_count`. Sections do not span
/// rings.
struct Sectionizer {
    next: usize,
    directions: Option<(i8, i8)>,
    count: usize,
}

impl Sectionizer {
    fn new(next: usize) -> Self {
        Self {
            next,
            directions: None,
            count: 0,
        }
    }

    fn start_ring(&mut self) {
        if self.count > 0 {
            self.next += 1;
        }
        self.directions = None;
        self.count = 0;
    }

    fn section_for<P>(&mut self, start: &P, end: &P) -> usize
    where
        P: Point,
        P::Scalar: Into<f64>,
    {
        let sign = |a: f64, b: f64| -> i8 {
            if b > a {
                1
            } else if b < a {
                -1
            } else {
                0
            }
        };
        let directions = (
            sign(start.get::<0>().into(), end.get::<0>().into()),
            sign(start.get::<1>().into(), end.get::<1>().into()),
        );
        if self.count > 0
            && (Some(directions) != self.directions || self.count > MAX_SEGMENTS_PER_SECTION)
        {
            self.next += 1;
            self.count = 0;
        }
        if self.count == 0 {
            self.directions = Some(directions);
        }
        self.count += 1;
        self.next
    }
}

fn append_ring_segments<R, P>(
    ring: &R,
    output: &mut Vec<SourceSegment<P>>,
    sections: &mut Sectionizer,
    backwards: bool,
) where
    R: RingTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut points: Vec<P> = ring.points().copied().collect();
    if backwards {
        // C++: `reversible_view`, which reverses the closed ring — so a ring
        // that stores its closing point still starts and ends on it, and one
        // that does not still closes back to its own first vertex.
        points.reverse();
    }
    if points.len() < 2 {
        return;
    }
    sections.start_ring();
    for pair in points.windows(2) {
        if points_differ(&pair[0], &pair[1]) {
            let section = sections.section_for(&pair[0], &pair[1]);
            output.push(SourceSegment::new(pair[0], pair[1], section));
        }
    }
    let last = *points.last().expect("nonempty");
    if points_differ(&last, &points[0]) {
        let section = sections.section_for(&last, &points[0]);
        output.push(SourceSegment::new(last, points[0], section));
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
        let section = segment.section;
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
                nodes[end].section[operand] = section;
            }
            if start != end {
                let mut carried_by = [false; 2];
                carried_by[operand] = true;
                let mut sections = [usize::MAX; 2];
                sections[operand] = section;
                output.push(Edge {
                    start,
                    end,
                    carried_by,
                    section: sections,
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
        section: [usize::MAX; 2],
        pair_rank: usize::MAX,
        offset: [0.0; 2],
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
    // Each ring is kept with where it starts, so the whole set can be put back
    // into source order below.
    let mut rings: Vec<(RingStart, Ring<P>)> = Vec::new();
    for seed in 0..edges.len() {
        if used[seed] {
            continue;
        }
        let first = edges[seed].start;
        let mut edge_index = seed;
        let mut node_indices = alloc::vec![first];
        // `along[i]` is the edge from `node_indices[i]` to the node after it,
        // so it always has one entry fewer.
        let mut along: alloc::vec::Vec<Edge> = alloc::vec::Vec::new();
        for _ in 0..=edges.len() {
            debug_assert!(!used[edge_index]);
            used[edge_index] = true;
            let edge = edges[edge_index];
            node_indices.push(edge.end);
            along.push(edge);

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
                let loop_along = along.split_off(start);
                node_indices.push(edge.end);
                push_ring(&mut rings, nodes, &loop_nodes, &loop_along, tolerance);
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
    // to fall in.
    rings.sort_by(|(left, _), (right, _)| left.compare(right));
    Ok(rings
        .into_iter()
        .map(|(start, ring)| (ring, start.traversed))
        .collect())
}

/// Append a turn point, dropping whatever it now runs straight through.
///
/// C++: `append_no_collinear`. Once the point is on, any point before it that
/// the new one continues the line of is redundant and comes off — repeatedly,
/// because removing one can leave the next in the same position.
///
/// Boost applies this to turn points only. The ring vertices copied between
/// two turns go on through `append_no_dups_or_spikes`, which takes out
/// duplicates and spikes but leaves a vertex that merely continues straight,
/// so an operand's own collinear vertex survives while a turn's does not.
fn append_no_collinear<P>(points: &mut Vec<P>, point: P)
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let at = |p: &P| (p.get::<0>().into(), p.get::<1>().into());
    let (x, y) = at(&point);
    if points.len() == 1 {
        let (fx, fy) = at(&points[0]);
        if fx == x && fy == y {
            return;
        }
    }
    points.push(point);
    while points.len() >= 3 {
        let (ax, ay) = at(&points[points.len() - 3]);
        let (bx, by) = at(&points[points.len() - 2]);
        if (bx - ax) * (y - ay) - (by - ay) * (x - ax) != 0.0 {
            return;
        }
        let last = points.pop().expect("just pushed");
        points.pop();
        points.push(last);
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
    rings: &mut Vec<(RingStart, Ring<P>)>,
    nodes: &[Node<P>],
    node_indices: &[usize],
    along: &[Edge],
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
    let cycle = &node_indices[..node_indices.len() - 1];
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
            TurnOrder::of(&nodes[left]).compare(&TurnOrder::of(&nodes[right]))
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

    // C++: the traversal appends a *turn* point with `append_no_collinear` and
    // the ring vertices between turns with `copy_segments`, which does not
    // check for collinearity. So a turn that carries the outline straight on
    // replaces the point before it, and a vertex of the operand being walked
    // never does.
    //
    // This is what keeps the other operand's corner out of the result where
    // the two run along the same edge: the traversal reaches that corner as a
    // turn, appends it, and then the next turn — the far end of the shared
    // stretch — is collinear with it and takes its place.
    let mut points: Vec<P> = Vec::with_capacity(cycle.len() + 1);
    for &index in cycle[first_turn..].iter().chain(&cycle[..first_turn]) {
        let node = &nodes[index];
        if !node.is_turn {
            points.push(node.point);
            continue;
        }
        append_no_collinear(&mut points, node.point);
    }
    // The traversal closes a ring by arriving back at the turn it started
    // from, and that arrival is an append like any other — which is exactly
    // where the point before it goes, when the start carries the outline
    // straight on through it.
    //
    // Only a *traced* ring closes that way. One with no turn on it was never
    // traversed at all: `add_rings` copies it out of its operand through
    // `convert_ring`, which appends nothing and drops nothing, so its last
    // vertex stays even where it continues the line straight into the first.
    if let Some(&first) = points.first() {
        if first_turn_by_arrival.is_some() {
            append_no_collinear(&mut points, first);
        } else {
            points.push(first);
        }
    }
    // The ring is cleaned once it is in its final winding, not here: which
    // vertex `clean_closing_dups_and_spikes` leaves at the front depends on
    // the direction the ring runs in, and that is decided in `assemble`.
    // Two rings can begin at the same node — where the result touches itself
    // at a point, both lobes start there. Boost separates them by operand:
    // `iterate` tries operation 0 before operation 1 at a turn, so the lobe
    // traced along the first operand is emitted first.
    let leaves_along_first_operand = along.get(first_turn).is_some_and(|edge| edge.carried_by[0]);
    // C++: a ring no turn lands on is emitted by `add_rings` under its own
    // `ring_identifier` — source first, then where it sits in that operand.
    // Its vertices say nothing about which: this arrangement gives two rings
    // that meet at a point the same node, so the lowest node on a cycle can
    // belong to a different ring altogether. The lowest section does not,
    // because a section never spans a ring.
    let source = usize::from(!along.iter().all(|edge| edge.carried_by[0]));
    let ring = along
        .iter()
        .map(|edge| edge.section[source])
        .min()
        .unwrap_or(usize::MAX);
    rings.push((
        RingStart {
            traversed: first_turn_by_arrival.is_some(),
            source,
            ring,
            turn: TurnOrder::of(&nodes[cycle[first_turn]]),
            second_operand: !leaves_along_first_operand,
            node: cycle[first_turn],
        },
        Ring::from_vec(points),
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
                section: [0, 0],
                pair_rank: 0,
                offset: [0.0; 2],
            },
            Node {
                point: P::new(1.0, 0.0),
                coordinate: Coordinate { x: 1.0, y: 0.0 },
                is_turn: false,
                arrival: [1, 1],
                section: [1, 1],
                pair_rank: 1,
                offset: [0.0; 2],
            },
            Node {
                point: P::new(2.0, 0.0),
                coordinate: Coordinate { x: 2.0, y: 0.0 },
                is_turn: false,
                arrival: [2, 2],
                section: [2, 2],
                pair_rank: 2,
                offset: [0.0; 2],
            },
        ];
        let edges = [
            Edge {
                start: 0,
                end: 1,
                carried_by: [true; 2],
                section: [0; 2],
            },
            Edge {
                start: 1,
                end: 2,
                carried_by: [true; 2],
                section: [0; 2],
            },
            Edge {
                start: 2,
                end: 0,
                carried_by: [true; 2],
                section: [0; 2],
            },
        ];

        assert!(trace_rings(&nodes, &edges, 1e-10).unwrap().is_empty());
    }
}
