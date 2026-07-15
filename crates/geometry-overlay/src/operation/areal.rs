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
use geometry_trait::{Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait};

use crate::assemble::assemble_multipolygon;
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

    fn contains(&self, point: Coordinate) -> bool {
        self.rings
            .iter()
            .fold(false, |inside, ring| inside != ring_contains(ring, point))
    }
}

struct SourceSegment<P> {
    start: P,
    end: P,
    splits: Vec<(f64, P)>,
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
            splits: alloc::vec![(0.0, start), (1.0, end)],
        }
    }

    fn push_split(&mut self, point: P, tolerance: f64) {
        let parameter = segment_parameter(&self.start, &self.end, &point);
        if parameter >= -tolerance
            && parameter <= 1.0 + tolerance
            && !self
                .splits
                .iter()
                .any(|(existing, _)| (existing - parameter).abs() <= tolerance)
        {
            self.splits.push((parameter.clamp(0.0, 1.0), point));
        }
    }
}

struct Node<P> {
    point: P,
    coordinate: Coordinate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Edge {
    start: usize,
    end: usize,
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
    let first_shape = Shape::from_polygon(first);
    let second_shape = Shape::from_polygon(second);
    let scale = coordinate_scale(&first_shape, &second_shape);
    let snap_tolerance = scale * 1e-10;
    let parameter_tolerance = 1e-10;

    let mut first_segments = polygon_segments(first);
    let mut second_segments = polygon_segments(second);
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
        snap_tolerance,
    );
    append_atomic_edges(
        &mut second_segments,
        &mut nodes,
        &mut candidates,
        snap_tolerance,
    );

    let sample_distance = (scale * 1e-8).max(snap_tolerance * 32.0);
    let mut boundary = Vec::new();
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
            }
        };
        if !boundary.contains(&edge) {
            boundary.push(edge);
        }
    }

    let rings = trace_rings(&nodes, &boundary, snap_tolerance)?;
    Ok(assemble_multipolygon(rings))
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
    tolerance: f64,
) where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    for segment in segments {
        segment
            .splits
            .sort_by(|left, right| left.0.total_cmp(&right.0));
        for pair in segment.splits.windows(2) {
            debug_assert!((pair[1].0 - pair[0].0).abs() > 1e-12);
            let start = canonical_node(nodes, pair[0].1, tolerance);
            let end = canonical_node(nodes, pair[1].1, tolerance);
            if start != end {
                output.push(Edge { start, end });
            }
        }
    }
}

fn canonical_node<P>(nodes: &mut Vec<Node<P>>, point: P, tolerance: f64) -> usize
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
        return index;
    }
    nodes.push(Node { point, coordinate });
    nodes.len() - 1
}

fn trace_rings<P>(
    nodes: &[Node<P>],
    edges: &[Edge],
    tolerance: f64,
) -> Result<Vec<Ring<P>>, OverlayError>
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut used = alloc::vec![false; edges.len()];
    let mut rings = Vec::new();
    for seed in 0..edges.len() {
        if used[seed] {
            continue;
        }
        let first = edges[seed].start;
        let mut edge_index = seed;
        let mut node_indices = alloc::vec![first];
        for _ in 0..=edges.len() {
            if used[edge_index] {
                return Err(OverlayError::Unsupported);
            }
            used[edge_index] = true;
            let edge = edges[edge_index];
            node_indices.push(edge.end);
            if edge.end == first {
                break;
            }
            edge_index = next_edge(nodes, edges, &used, edge).ok_or(OverlayError::Unsupported)?;
        }
        if node_indices.last().copied() != Some(first) {
            return Err(OverlayError::Unsupported);
        }
        let area = node_indices.windows(2).fold(0.0, |sum, pair| {
            let a = nodes[pair[0]].coordinate;
            let b = nodes[pair[1]].coordinate;
            sum + a.x * b.y - b.x * a.y
        }) * 0.5;
        if area.abs() > tolerance * tolerance {
            rings.push(Ring::from_vec(
                node_indices
                    .into_iter()
                    .map(|index| nodes[index].point)
                    .collect(),
            ));
        }
    }
    Ok(rings)
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
    if delta.0.abs() >= delta.1.abs() && delta.0 != 0.0 {
        (point.x - start.x) / delta.0
    } else if delta.1 != 0.0 {
        (point.y - start.y) / delta.1
    } else {
        0.0
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
