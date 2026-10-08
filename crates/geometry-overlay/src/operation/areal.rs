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

use alloc::collections::BTreeMap;
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
}

impl ArealOp {
    fn apply(self, first: bool, second: bool) -> bool {
        match self {
            Self::Intersection => first && second,
            Self::Union => first || second,
            Self::Difference => first && !second,
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

#[derive(Clone)]
struct SourceSegment<P> {
    start: P,
    end: P,
    /// `(parameter, point, is_turn)`. The two endpoints are not turns; a point
    /// pushed by the intersection sweep is, including one that lands on an
    /// endpoint.
    splits: Vec<(f64, P, bool)>,
    /// `(parameter, point, within_member)` inside the segment where another
    /// ring of its own operand meets it: a ring of the same polygon — a hole
    /// touching the exterior, two holes touching — or another member of a
    /// multi-polygon. Not turns; see `split_where_rings_touch`.
    contacts: Vec<(f64, P, bool)>,
    /// Which monotone run of its ring this segment belongs to. C++:
    /// `sectionalize`, whose sections are what `get_turns` iterates over.
    section: usize,
    /// Which polygon of its operand the segment's ring belongs to.
    member: usize,
}

impl<P> SourceSegment<P>
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    fn new(start: P, end: P, section: usize, member: usize) -> Self {
        Self {
            start,
            end,
            splits: alloc::vec![(0.0, start, false), (1.0, end, false)],
            contacts: Vec::new(),
            section,
            member,
        }
    }

    /// How much of the segment's parameter `distance` along it spans.
    fn parameter_span(&self, distance: f64) -> f64 {
        let start = Coordinate::from_point(&self.start);
        let end = Coordinate::from_point(&self.end);
        distance / hypot(end.x - start.x, end.y - start.y)
    }

    /// Record where another ring of this segment's own operand meets it.
    /// Its endpoints, and any point a turn already split, need no contact.
    ///
    /// Points that close together are not merged here but where they become
    /// nodes (`canonical_node`), by their distance apart, which is the same
    /// on every segment: merged here, by how far apart they lie along one
    /// segment, a split could be merged on one segment and kept apart on
    /// another that crosses it, and leave the two meeting at different
    /// nodes.
    fn push_contact(&mut self, point: P, snap: f64, within_member: bool) {
        let tolerance = self.parameter_span(snap);
        let parameter = segment_parameter(&self.start, &self.end, &point);
        if parameter <= tolerance || parameter >= 1.0 - tolerance {
            return;
        }
        let taken = |at: f64| at == parameter;
        if self.splits.iter().any(|(at, _, _)| taken(*at))
            || self.contacts.iter().any(|(at, _, _)| taken(*at))
        {
            return;
        }
        self.contacts.push((parameter, point, within_member));
    }

    /// Record a turn on the segment. As with a contact, a point close to one
    /// already recorded is merged with it only once both are nodes.
    fn push_split(&mut self, point: P, snap: f64) {
        let tolerance = self.parameter_span(snap);
        let parameter = segment_parameter(&self.start, &self.end, &point);
        if parameter < -tolerance || parameter > 1.0 + tolerance {
            return;
        }
        let parameter = parameter.clamp(0.0, 1.0);
        if let Some(existing) = self.splits.iter_mut().find(|(at, _, _)| *at == parameter) {
            // A crossing that lands on a vertex already split here still makes
            // that vertex a turn.
            existing.2 = true;
            return;
        }
        self.splits.push((parameter, point, true));
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
    /// Per operand, set where its own rings touch here and cut one of its
    /// segments: `Some(true)` for two rings of one polygon, `Some(false)` for
    /// two members of a multi-polygon. See `KeptTouches`.
    touch: [Option<bool>; 2],
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
    /// Reproducing that needs to know who carries each edge.
    carried_by: [bool; 2],
    /// Which section of each operand runs along it, `usize::MAX` for an
    /// operand that does not. Sections never span a ring, so the lowest one on
    /// a cycle names the ring the cycle came out of — which is the whole of
    /// `ring_identifier` for a ring nothing crossed.
    section: [usize; 2],
    /// Which source segment of each operand the edge was cut from,
    /// `usize::MAX` for an operand that does not carry it. Two edges cut from
    /// one segment meet at a point inside it, which is no vertex.
    segment: [usize; 2],
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
    /// Traversed from one of an operand's own touch points — Boost's self
    /// turns, which it collects after every turn between the operands.
    from_self_turn: bool,
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
                self.from_self_turn
                    .cmp(&other.from_self_turn)
                    .then_with(|| self.turn.compare(&other.turn))
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
    first_segments: Vec<SourceSegment<P>>,
    second_segments: Vec<SourceSegment<P>>,
    operation: ArealOp,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let scale = coordinate_scale(first_shape, second_shape);
    // Points a snap distance apart are one node. Where the operands come
    // closer than that without meeting — near copies of one another, a
    // vertex a hair off an edge — the samples that tell an edge's sides
    // apart can misjudge one, and the boundary does not close. The
    // arrangement is then made again at a coarser snap, which folds such
    // slivers into the edges they hug rather than refusing the operation
    // over them.
    match arrangement_at(
        first_shape,
        second_shape,
        first_segments.clone(),
        second_segments.clone(),
        operation,
        scale,
        scale * 1e-10,
    ) {
        Err(OverlayError::Unsupported) => arrangement_at(
            first_shape,
            second_shape,
            first_segments,
            second_segments,
            operation,
            scale,
            scale * 1e-7,
        ),
        result => result,
    }
}

fn arrangement_at<P>(
    first_shape: &Shape,
    second_shape: &Shape,
    mut first_segments: Vec<SourceSegment<P>>,
    mut second_segments: Vec<SourceSegment<P>>,
    operation: ArealOp,
    scale: f64,
    snap_tolerance: f64,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    split_where_operands_meet(&mut first_segments, &mut second_segments, snap_tolerance)?;
    split_where_rings_touch(&mut first_segments, snap_tolerance)?;
    split_where_rings_touch(&mut second_segments, snap_tolerance)?;

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
    let carriers = stretch_carriers(&candidates);
    let sources = [&first_segments, &second_segments];
    let mut boundary: Vec<Edge> = Vec::new();
    for &candidate in &candidates {
        let start = nodes[candidate.start].coordinate;
        let end = nodes[candidate.end].coordinate;
        let delta = (end.x - start.x, end.y - start.y);
        let length = hypot(delta.0, delta.1);
        debug_assert!(length > snap_tolerance);
        let midpoint = Coordinate {
            x: f64::midpoint(start.x, end.x),
            y: f64::midpoint(start.y, end.y),
        };
        // An operand that does not run along the edge lies on one side of it
        // only, and is asked at the edge itself. One that does is asked either
        // side of the segment of its own the edge was cut from — the edge's
        // ends may be another operand's vertices a hair off that segment —
        // and closer than any other of its edges comes: a sample that strayed
        // past one would read a sliver thinner than the sample distance — two
        // edges a rounding error from coinciding, a vertex a hair off an edge
        // — as filled on both sides, and drop edges whose nodes stay apart,
        // leaving the boundary with loose ends. One that runs along it there
        // and back is asked either side of the edge itself, past the spike
        // its two runs bound: they were made one stretch by snapping their
        // nodes together, so they lie no further apart than that.
        let carried = carriers[&stretch_key(&candidate)];
        let reach = sample_distance.min(length * 1e-4);
        let clearance = clearance(&candidate, carried, &candidates, &nodes, midpoint, reach);
        let offset = reach.min(clearance / 2.0);
        let sides = |shape: &Shape, operand: usize| {
            let (left, right) = match carried[operand] {
                Carry::Clear => {
                    let inside = shape.contains(midpoint);
                    return (inside, inside);
                }
                Carry::Along(index) => beside(&sources[operand][index], midpoint, delta, offset),
                Carry::Folded => across(
                    midpoint,
                    delta,
                    (snap_tolerance * 2.0).max(offset).min(clearance / 2.0),
                ),
            };
            (shape.contains(left), shape.contains(right))
        };
        let (first_left, first_right) = sides(first_shape, 0);
        let (second_left, second_right) = sides(second_shape, 1);
        let left_result = operation.apply(first_left, second_left);
        let right_result = operation.apply(first_right, second_right);
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
                segment: candidate.segment,
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
                held.segment[0] = held.segment[0].min(edge.segment[0]);
                held.segment[1] = held.segment[1].min(edge.segment[1]);
            }
            None => boundary.push(edge),
        }
    }
    settle_short_edges(&mut boundary, &candidates, &nodes, sample_distance);

    let kept = KeptTouches::of(operation, nodes.iter().any(|node| node.is_turn));
    let rings = trace_rings(&nodes, &boundary, kept, snap_tolerance)?;
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
    for (member, polygon) in multi_polygon.polygons().enumerate() {
        append_ring_segments(
            polygon.exterior(),
            &mut segments,
            &mut sections,
            backwards,
            member,
        );
        for ring in polygon.interiors() {
            append_ring_segments(ring, &mut segments, &mut sections, backwards, member);
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
    append_ring_segments(
        polygon.exterior(),
        &mut segments,
        &mut sections,
        backwards,
        0,
    );
    for ring in polygon.interiors() {
        append_ring_segments(ring, &mut segments, &mut sections, backwards, 0);
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
    member: usize,
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
            output.push(SourceSegment::new(pair[0], pair[1], section, member));
        }
    }
    let last = *points.last().expect("nonempty");
    if points_differ(&last, &points[0]) {
        let section = sections.section_for(&last, &points[0]);
        output.push(SourceSegment::new(last, points[0], section, member));
    }
}

/// Split both operands' segments wherever the two meet — the turns.
///
/// A vertex of one operand within `snap` of a segment of the other meets it
/// too, whichever side of it the exact predicate puts the vertex: the nodes
/// are merged that close, and an edge is classified by sampling beside it,
/// so a segment left whole past a vertex that near would be judged across a
/// sliver its samples cannot see, and leave the boundary with a loose end.
fn split_where_operands_meet<P>(
    first: &mut [SourceSegment<P>],
    second: &mut [SourceSegment<P>],
    snap: f64,
) -> Result<(), OverlayError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    for first_segment in first {
        for second_segment in &mut *second {
            let first_model = Segment::new(first_segment.start, first_segment.end);
            let second_model = Segment::new(second_segment.start, second_segment.end);
            match segment_intersection(&first_model, &second_model) {
                SegmentIntersection::Disjoint => {}
                SegmentIntersection::Single(point) => {
                    first_segment.push_split(point, snap);
                    second_segment.push_split(point, snap);
                }
                SegmentIntersection::Collinear { from, to } => {
                    first_segment.push_split(from, snap);
                    first_segment.push_split(to, snap);
                    second_segment.push_split(from, snap);
                    second_segment.push_split(to, snap);
                }
                SegmentIntersection::OutOfRange => return Err(OverlayError::Unsupported),
            }
            for vertex in [second_segment.start, second_segment.end] {
                if passes_within(first_segment, &vertex, snap) {
                    first_segment.push_split(vertex, snap);
                    second_segment.push_split(vertex, snap);
                }
            }
            for vertex in [first_segment.start, first_segment.end] {
                if passes_within(second_segment, &vertex, snap) {
                    second_segment.push_split(vertex, snap);
                    first_segment.push_split(vertex, snap);
                }
            }
        }
    }
    Ok(())
}

/// Whether `segment` runs within `snap` of `point` away from its ends.
fn passes_within<P>(segment: &SourceSegment<P>, point: &P, snap: f64) -> bool
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let tolerance = segment.parameter_span(snap);
    let start = Coordinate::from_point(&segment.start);
    let end = Coordinate::from_point(&segment.end);
    let point = Coordinate::from_point(point);
    let along = (end.x - start.x, end.y - start.y);
    let at = ((point.x - start.x) * along.0 + (point.y - start.y) * along.1)
        / (along.0 * along.0 + along.1 * along.1);
    at > tolerance && at < 1.0 - tolerance && distance_to_span(point, (start, end)) <= snap
}

/// Split an operand's segments where its own rings meet: a hole touching
/// the exterior, two holes touching, or two members of a multi-polygon.
///
/// Those points are not turns — the operand meets itself there, not the
/// other operand — but a segment left whole across one is classified by side
/// samples taken at its middle, and when the touching ring's vertex sits
/// there the samples land on that ring's boundary and the edge drops out of
/// the result, leaving the tracer an outline it cannot close. Boost's
/// traversal never classifies a stretch by sampling, so it needs no such
/// cut.
fn split_where_rings_touch<P>(
    segments: &mut [SourceSegment<P>],
    snap: f64,
) -> Result<(), OverlayError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let bounds = |segment: &SourceSegment<P>| {
        let (start, end) = (
            Coordinate::from_point(&segment.start),
            Coordinate::from_point(&segment.end),
        );
        (
            start.x.min(end.x),
            start.y.min(end.y),
            start.x.max(end.x),
            start.y.max(end.y),
        )
    };
    for later in 1..segments.len() {
        let (head, tail) = segments.split_at_mut(later);
        let later = &mut tail[0];
        let (min_x, min_y, max_x, max_y) = bounds(later);
        for earlier in head.iter_mut() {
            let (low_x, low_y, high_x, high_y) = bounds(earlier);
            if low_x > max_x || high_x < min_x || low_y > max_y || high_y < min_y {
                continue;
            }
            let within_member = earlier.member == later.member;
            let earlier_model = Segment::new(earlier.start, earlier.end);
            let later_model = Segment::new(later.start, later.end);
            match segment_intersection(&earlier_model, &later_model) {
                SegmentIntersection::Disjoint => {}
                SegmentIntersection::Single(point) => {
                    earlier.push_contact(point, snap, within_member);
                    later.push_contact(point, snap, within_member);
                }
                SegmentIntersection::Collinear { from, to } => {
                    for point in [from, to] {
                        earlier.push_contact(point, snap, within_member);
                        later.push_contact(point, snap, within_member);
                    }
                }
                SegmentIntersection::OutOfRange => return Err(OverlayError::Unsupported),
            }
        }
    }
    Ok(())
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
    // Each stretch between consecutive splits, with the parameters it spans.
    let mut stretches = Vec::new();
    for (index, segment) in segments.iter_mut().enumerate() {
        let section = segment.section;
        segment
            .splits
            .sort_by(|left, right| left.0.total_cmp(&right.0));
        for pair in segment.splits.windows(2) {
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
                stretches.push((index, start, end, pair[0].0, pair[1].0));
            }
        }
    }

    // Contacts are cut in only now, so every node they add comes after the
    // vertices and turns: an untouched ring keeps reading its first vertex
    // off node order (`push_ring`).
    for (index, start, end, from, to) in stretches {
        let segment = &segments[index];
        let mut cuts: Vec<(f64, P, bool)> = segment
            .contacts
            .iter()
            .copied()
            .filter(|(at, _, _)| *at > from && *at < to)
            .collect();
        cuts.sort_by(|left, right| left.0.total_cmp(&right.0));
        let mut ends: Vec<usize> = cuts
            .into_iter()
            .map(|(_, point, within_member)| {
                let node = canonical_node(nodes, point, false, tolerance);
                nodes[node].touch[operand].get_or_insert(within_member);
                node
            })
            .collect();
        ends.push(end);
        let mut previous = start;
        for next in ends {
            if next == previous {
                continue;
            }
            let mut carried_by = [false; 2];
            carried_by[operand] = true;
            let mut sections = [usize::MAX; 2];
            sections[operand] = segment.section;
            let mut source = [usize::MAX; 2];
            source[operand] = index;
            output.push(Edge {
                start: previous,
                end: next,
                carried_by,
                section: sections,
                segment: source,
            });
            previous = next;
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
        touch: [None; 2],
    });
    nodes.len() - 1
}

type TracedRing<P> = (Ring<P>, bool);

/// Which of the points where an operand's own rings touch Boost keeps as
/// turns, and so passes — appends — where a traced ring runs through them.
///
/// C++: `overlay` adds an operand's self turns only once the two operands
/// have turns between them, and `enrich_discard_turns` then drops every turn
/// whose operations both oppose the operation being built — `union` for an
/// intersection or a difference, `intersection` for a union. Two rings of
/// one polygon touch with both operations `intersection`, two members of a
/// multi-polygon with both `union`, and an operand walked backwards — the
/// second one of a difference — has the two swapped.
#[derive(Clone, Copy)]
struct KeptTouches {
    /// `[operand][within_member]`.
    kept: [[bool; 2]; 2],
}

impl KeptTouches {
    fn of(operation: ArealOp, operands_meet: bool) -> Self {
        let mut kept = [[false; 2]; 2];
        for (operand, kinds) in kept.iter_mut().enumerate() {
            let backwards = operand == 1 && operation.walks_second_operand_backwards();
            for (within_member, keeps) in kinds.iter_mut().enumerate() {
                let intersection = (within_member == 1) != backwards;
                let opposed = intersection == matches!(operation, ArealOp::Union);
                *keeps = operands_meet && !opposed;
            }
        }
        Self { kept }
    }

    fn keeps(self, touch: [Option<bool>; 2]) -> bool {
        touch.iter().zip(self.kept).any(|(touch, kept)| {
            touch.is_some_and(|within_member| kept[usize::from(within_member)])
        })
    }
}

/// Walk the result boundary into closed rings, one face at a time.
///
/// Every edge carries the result on its right. From each node the walk
/// continues along the first unused edge counter-clockwise from the one it
/// arrived on (`next_edge`), which keeps it against the face it is tracing,
/// so two lobes of the result that touch — at one point or at several — come
/// out as separate rings and the region between them is never walked. A walk
/// that returns to a node before its seed has gone round a hole that touches
/// the ring it is on; that loop is cut out as a ring of its own.
///
/// C++: `traverse`, whose `select_turn` picks the outgoing operation by
/// `sort_by_side` where several meet, and whose rings `add_rings` then
/// assembles by containment.
fn trace_rings<P>(
    nodes: &[Node<P>],
    edges: &[Edge],
    kept: KeptTouches,
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
            // not only when the walk returns to the seed. A face whose hole
            // touches its outer ring at a point — or whose two holes touch —
            // has one connected boundary, and the walk passes through that
            // point twice; carrying on to the seed would splice the outer
            // ring and the hole into one self-touching ring, which is not a
            // valid polygon and is not what `boost::geometry::intersection`
            // returns. Cut the loop out, keep the path up to that node, and
            // carry on walking.
            if let Some(start) = node_indices[..node_indices.len() - 1]
                .iter()
                .position(|&index| index == edge.end)
            {
                let loop_nodes = node_indices.split_off(start);
                let loop_along = along.split_off(start);
                node_indices.push(edge.end);
                push_ring(&mut rings, nodes, &loop_nodes, &loop_along, kept, tolerance);
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
    kept: KeptTouches,
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
    // A point where another ring of the walked operand touched one of its
    // segments cuts that segment in the arrangement but is no vertex of it:
    // the walk arrives and leaves along the same source segment.
    let count = cycle.len();
    let keeps = |position: usize| {
        let (arriving, leaving) = (along[(position + count - 1) % count], along[position]);
        nodes[cycle[position]].is_turn
            || !(0..2).any(|operand| {
                arriving.segment[operand] != usize::MAX
                    && arriving.segment[operand] == leaving.segment[operand]
            })
    };
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
            .filter(|&(position, _)| keeps(position))
            .min_by_key(|&(_, index)| index)
            .map(|(position, _)| position)
    };
    // A ring with no turn between the operands on it is still traversed when
    // a kept self turn is (`KeptTouches`): Boost starts it there, and only
    // after every ring started at a turn between the operands.
    let first_self_turn = || {
        first_turn_by_arrival.is_none().then(|| {
            cycle
                .iter()
                .copied()
                .enumerate()
                .filter(|&(_, index)| kept.keeps(nodes[index].touch))
                .min_by_key(|&(_, index)| index)
                .map(|(position, _)| position)
        })?
    };
    let first_traversed = first_turn_by_arrival.or_else(first_self_turn);
    let traced = first_traversed.is_some();
    let first_turn = first_traversed.or_else(first_node).unwrap_or(0);

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
    for position in (first_turn..count).chain(0..first_turn) {
        let node = &nodes[cycle[position]];
        // A self turn Boost keeps is passed like any other turn by a traced
        // ring, even where it is no vertex of the walked segment.
        let self_turn = traced && kept.keeps(node.touch);
        if !keeps(position) && !self_turn {
            continue;
        }
        if !node.is_turn && !self_turn {
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
        if traced {
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
            traversed: traced,
            from_self_turn: first_turn_by_arrival.is_none(),
            source,
            ring,
            turn: TurnOrder::of(&nodes[cycle[first_turn]]),
            second_operand: !leaves_along_first_operand,
            node: cycle[first_turn],
        },
        Ring::from_vec(points),
    ));
}

/// The edge the walk leaves a node along: the first unused one
/// counter-clockwise from the edge it arrived on.
///
/// Every edge carries the result on its right, so the wedge immediately
/// counter-clockwise of the arriving edge is filled and the next edge round
/// bounds that same wedge. Leaving along it keeps the walk on the one face of
/// the arrangement it started on. C++: `sort_by_side` ranks the operations at
/// a turn by their angle round it, and `traversal::select_turn` continues
/// along the one adjacent to the incoming segment on the side being
/// traversed.
///
/// Any other exit — the smallest turn overall, say — carries the walk across
/// to a lobe that merely touches this one. Where two result lobes meet at two
/// or more points that splices them into one outline and leaves the region
/// between them as a hole against its own outer ring, which is not a valid
/// polygon and not what Boost returns.
fn next_edge<P>(nodes: &[Node<P>], edges: &[Edge], used: &[bool], incoming: Edge) -> Option<usize>
where
    P: Point,
{
    let previous = nodes[incoming.start].coordinate;
    let vertex = nodes[incoming.end].coordinate;
    let back = (previous.x - vertex.x, previous.y - vertex.y);
    edges
        .iter()
        .enumerate()
        .filter(|(index, edge)| !used[*index] && edge.start == incoming.end)
        .min_by(|(_, left), (_, right)| {
            let left_turn = turn_angle(back, vertex, nodes[left.end].coordinate);
            let right_turn = turn_angle(back, vertex, nodes[right.end].coordinate);
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

/// Settle the boundary where a stretch too short for its sides to be told
/// apart was judged wrongly.
///
/// A boundary leaves every node as often as it reaches it. A stretch not
/// much longer than the snap distance, that two operands were snapped onto
/// where their segments cross, has no sides the samples can tell apart, and
/// a misjudged one leaves its two nodes out of balance, one short of a
/// departure and the other of an arrival. Adding the stretch the way that
/// settles both, or taking it away where it was kept the other way round,
/// closes the boundary again, and a stretch that short moves no area that
/// shows. Only stretches shorter than `reach` are touched; anything longer
/// is left for the tracer to refuse.
fn settle_short_edges<P>(
    boundary: &mut Vec<Edge>,
    candidates: &[Edge],
    nodes: &[Node<P>],
    reach: f64,
) {
    let mut balance = alloc::vec![0_isize; nodes.len()];
    for edge in boundary.iter() {
        balance[edge.start] += 1;
        balance[edge.end] -= 1;
    }
    if balance.iter().all(|&departures| departures == 0) {
        return;
    }
    let length = |edge: &Edge| {
        let (start, end) = (nodes[edge.start].coordinate, nodes[edge.end].coordinate);
        hypot(end.x - start.x, end.y - start.y)
    };
    let mut short: Vec<&Edge> = candidates
        .iter()
        .filter(|candidate| length(candidate) <= reach)
        .collect();
    short.sort_by(|left, right| length(left).total_cmp(&length(right)));
    for candidate in short {
        for (from, to) in [
            (candidate.start, candidate.end),
            (candidate.end, candidate.start),
        ] {
            if balance[from] >= 0 || balance[to] <= 0 {
                continue;
            }
            match boundary
                .iter()
                .position(|held| held.start == to && held.end == from)
            {
                Some(index) => {
                    boundary.remove(index);
                }
                None => boundary.push(Edge {
                    start: from,
                    end: to,
                    ..*candidate
                }),
            }
            balance[from] += 1;
            balance[to] -= 1;
        }
    }
}

/// Two points `offset` either side of `segment`'s line, abreast of `point`:
/// the first to the left of `direction`, the second to its right.
fn beside<P>(
    segment: &SourceSegment<P>,
    point: Coordinate,
    direction: (f64, f64),
    offset: f64,
) -> (Coordinate, Coordinate)
where
    P: Point,
    P::Scalar: Into<f64>,
{
    let start = Coordinate::from_point(&segment.start);
    let end = Coordinate::from_point(&segment.end);
    let along = (end.x - start.x, end.y - start.y);
    let length = hypot(along.0, along.1);
    let at = ((point.x - start.x) * along.0 + (point.y - start.y) * along.1) / (length * length);
    let foot = Coordinate {
        x: start.x + at * along.0,
        y: start.y + at * along.1,
    };
    let turn = if along.0 * direction.0 + along.1 * direction.1 < 0.0 {
        -offset
    } else {
        offset
    };
    let normal = (-along.1 / length * turn, along.0 / length * turn);
    (
        Coordinate {
            x: foot.x + normal.0,
            y: foot.y + normal.1,
        },
        Coordinate {
            x: foot.x - normal.0,
            y: foot.y - normal.1,
        },
    )
}

/// How each operand runs along each stretch of the arrangement: along the
/// first source segment of its that does, or there and back where its runs
/// one way and the other cancel.
fn stretch_carriers(candidates: &[Edge]) -> BTreeMap<(usize, usize), [Carry; 2]> {
    // The first source segment along each stretch, and how many more times
    // the operand runs it one way round than the other.
    let mut runs = BTreeMap::new();
    for candidate in candidates {
        let held = runs
            .entry(stretch_key(candidate))
            .or_insert([(None, 0_isize); 2]);
        let way = if candidate.start < candidate.end {
            1
        } else {
            -1
        };
        for ((source, net), (&carried, &segment)) in held
            .iter_mut()
            .zip(candidate.carried_by.iter().zip(&candidate.segment))
        {
            if carried {
                source.get_or_insert(segment);
                *net += way;
            }
        }
    }
    runs.into_iter()
        .map(|(stretch, held)| {
            (
                stretch,
                held.map(|(source, net)| match source {
                    None => Carry::Clear,
                    Some(_) if net == 0 => Carry::Folded,
                    Some(index) => Carry::Along(index),
                }),
            )
        })
        .collect()
}

/// How close to `midpoint` the nearest other stretch comes that an operand
/// running along `candidate` also runs along, looking no further than
/// `reach`; infinite when none comes that close.
fn clearance<P>(
    candidate: &Edge,
    carried: [Carry; 2],
    candidates: &[Edge],
    nodes: &[Node<P>],
    midpoint: Coordinate,
    reach: f64,
) -> f64 {
    candidates
        .iter()
        .filter(|other| {
            stretch_key(other) != stretch_key(candidate)
                && carried
                    .iter()
                    .zip(other.carried_by)
                    .any(|(carry, by)| by && !matches!(carry, Carry::Clear))
        })
        .map(|other| (nodes[other.start].coordinate, nodes[other.end].coordinate))
        .filter(|&(from, to)| {
            from.x.min(to.x) <= midpoint.x + reach
                && from.x.max(to.x) >= midpoint.x - reach
                && from.y.min(to.y) <= midpoint.y + reach
                && from.y.max(to.y) >= midpoint.y - reach
        })
        .map(|span| distance_to_span(midpoint, span))
        .fold(f64::INFINITY, f64::min)
}

/// Two points `offset` either side of `point`, square to `direction`: the
/// first to its left, the second to its right.
fn across(point: Coordinate, direction: (f64, f64), offset: f64) -> (Coordinate, Coordinate) {
    let length = hypot(direction.0, direction.1);
    let normal = (
        -direction.1 / length * offset,
        direction.0 / length * offset,
    );
    (
        Coordinate {
            x: point.x + normal.0,
            y: point.y + normal.1,
        },
        Coordinate {
            x: point.x - normal.0,
            y: point.y - normal.1,
        },
    )
}

/// How one operand runs along a stretch of the arrangement.
#[derive(Clone, Copy)]
enum Carry {
    /// Not at all: the operand lies on one side of the stretch only.
    Clear,
    /// Along the source segment with this index.
    Along(usize),
    /// There and back, as the two sides of a spike too thin to part: the
    /// operand is the same on both sides of it.
    Folded,
}

/// The two nodes a stretch runs between, in either direction.
fn stretch_key(edge: &Edge) -> (usize, usize) {
    (edge.start.min(edge.end), edge.start.max(edge.end))
}

/// How far `point` is from the segment `span`.
fn distance_to_span(point: Coordinate, (start, end): (Coordinate, Coordinate)) -> f64 {
    let delta = (end.x - start.x, end.y - start.y);
    let length_squared = delta.0 * delta.0 + delta.1 * delta.1;
    let along = if length_squared > 0.0 {
        (((point.x - start.x) * delta.0 + (point.y - start.y) * delta.1) / length_squared)
            .clamp(0.0, 1.0)
    } else {
        0.0
    };
    hypot(
        point.x - (start.x + along * delta.0),
        point.y - (start.y + along * delta.1),
    )
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

    use super::{Coordinate, Edge, KeptTouches, Node, trace_rings};

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
                touch: [None; 2],
            },
            Node {
                point: P::new(1.0, 0.0),
                coordinate: Coordinate { x: 1.0, y: 0.0 },
                is_turn: false,
                arrival: [1, 1],
                section: [1, 1],
                pair_rank: 1,
                offset: [0.0; 2],
                touch: [None; 2],
            },
            Node {
                point: P::new(2.0, 0.0),
                coordinate: Coordinate { x: 2.0, y: 0.0 },
                is_turn: false,
                arrival: [2, 2],
                section: [2, 2],
                pair_rank: 2,
                offset: [0.0; 2],
                touch: [None; 2],
            },
        ];
        let edges = [
            Edge {
                start: 0,
                end: 1,
                carried_by: [true; 2],
                section: [0; 2],
                segment: [0; 2],
            },
            Edge {
                start: 1,
                end: 2,
                carried_by: [true; 2],
                section: [0; 2],
                segment: [1; 2],
            },
            Edge {
                start: 2,
                end: 0,
                carried_by: [true; 2],
                section: [0; 2],
                segment: [2; 2],
            },
        ];

        let kept = KeptTouches {
            kept: [[false; 2]; 2],
        };
        assert_eq!(trace_rings(&nodes, &edges, kept, 1e-10).unwrap().len(), 0);
    }
}
