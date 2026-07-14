//! OVL7 — `buffer`: grow a geometry outward by a fixed distance.
//!
//! Mirrors `boost/geometry/algorithms/buffer.hpp` and the buffer
//! strategies under `strategies/buffer/`. A buffer offsets every part of
//! the input outward by `distance`, rounding or mitering the corners,
//! and unions the offset pieces into an output polygon.
//!
//! Cartesian dispatch covers points, linestrings, and simple polygons.
//! Polygon offsets are signed, handle convex and reflex vertices, and move
//! interior rings in the opposite topological direction from the exterior.
//!
//! Join / end / point strategies are modelled as small enums
//! ([`JoinStrategy`], [`PointStrategy`]) mirroring Boost's
//! `join_round` / `join_miter` and `point_circle` / `point_square`
//! strategy types.

// Segment counts convert freely between `usize` and `f64` to lay out
// circle / arc vertices; the values are small angular subdivisions where
// the sub-mantissa precision loss and the non-negative truncation are
// intentional and harmless.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "angular vertex-count arithmetic; values are small and non-negative"
)]
// Zero-length guards and closing-vertex identity compare `f64`s exactly
// on purpose — these are degenerate-case gates, not tolerance checks.
#![allow(clippy::float_cmp, reason = "exact degenerate-case guards")]

use alloc::vec::Vec;

use geometry_coords::{
    CoordinateScalar,
    math::{atan2, ceil, cos, hypot, mul_add, sin},
};
use geometry_cs::{CartesianFamily, CoordinateSystem, FromF64};
use geometry_model::{Linestring, MultiPolygon, Polygon, Ring};
use geometry_strategy::buffer::{
    BufferDistanceStrategy, BufferEndStrategy, BufferJoinStrategy, BufferPointStrategy,
    BufferSettings,
};
use geometry_tag::{
    BoxTag, LinestringTag, MultiLinestringTag, MultiPointTag, MultiPolygonTag, PointTag,
    PolygonTag, RingTag, SameAs, SegmentTag,
};
use geometry_trait::{
    Box as BoxTrait, Geometry, Linestring as LinestringTrait,
    MultiLinestring as MultiLinestringTrait, MultiPoint as MultiPointTrait,
    MultiPolygon as MultiPolygonTrait, Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait,
    Segment as SegmentTrait, box_max, box_min, segment_end, segment_start,
};

use crate::operation::OverlayError;

/// How to fill the wedge at a convex corner of the offset boundary.
///
/// Mirrors `strategy::buffer::join_round` / `join_miter`
/// (`strategies/buffer/buffer_join_round.hpp` and friends).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinStrategy {
    /// Fill the corner with a circular arc of `points_per_circle`
    /// segments. Boost's `join_round`.
    Round {
        /// Segment count of a full circle; the arc uses a proportional
        /// share.
        points_per_circle: usize,
    },
    /// Extend the two offset edges until they meet at a sharp point.
    /// Boost's `join_miter`.
    ///
    /// This compatibility spelling uses Boost's default miter limit of
    /// five times the buffer distance
    /// (`strategies/cartesian/buffer_join_miter.hpp:52-60`). Use
    /// [`BufferSettings`] with [`BufferJoinStrategy::Miter`] to select a
    /// different limit.
    Miter,
}

/// How to approximate a buffered point.
///
/// Mirrors `strategy::buffer::point_circle` / `point_square`
/// (`strategies/buffer/buffer_point_circle.hpp`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointStrategy {
    /// Approximate the buffer disc with a regular polygon of
    /// `points_per_circle` vertices. Boost's `point_circle`.
    Circle {
        /// Vertex count of the approximating polygon.
        points_per_circle: usize,
    },
    /// Approximate the buffer with an axis-aligned square. Boost's
    /// `point_square`.
    Square,
}

/// Per-geometry implementation selected by [`buffer`].
///
/// Rust tag-dispatch adapter for the geometry-specialized call behind
/// `boost::geometry::buffer` in
/// `algorithms/detail/buffer/interface.hpp:246-273`.
#[doc(hidden)]
pub trait BufferStrategy<G: Geometry> {
    fn apply(
        &self,
        geometry: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError>;
}

/// Tag-to-buffer implementation picker.
///
/// Rust counterpart to the geometry dispatch performed by
/// `boost::geometry::buffer` in
/// `algorithms/detail/buffer/interface.hpp:246-273`.
#[doc(hidden)]
pub trait BufferStrategyForKind {
    type S: Default;
}

/// Point buffer implementation selected for [`PointTag`].
///
/// Implements the point arm of the public buffer dispatch from
/// `algorithms/detail/buffer/interface.hpp:246-273`.
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct PointBuffer;

/// Polygon buffer implementation selected for [`PolygonTag`].
///
/// Implements the polygon arm of the public buffer dispatch from
/// `algorithms/detail/buffer/interface.hpp:246-273`.
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct PolygonBuffer;

/// Linestring buffer implementation selected for [`LinestringTag`].
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct LinestringBuffer;

/// Segment buffer implementation selected for [`SegmentTag`].
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct SegmentBuffer;

/// Ring buffer implementation selected for [`RingTag`].
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RingBuffer;

/// Box buffer implementation selected for [`BoxTag`].
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct BoxBuffer;

/// Multi-point buffer implementation selected for [`MultiPointTag`].
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct MultiPointBuffer;

/// Multi-linestring buffer implementation selected for [`MultiLinestringTag`].
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct MultiLinestringBuffer;

/// Multi-polygon buffer implementation selected for [`MultiPolygonTag`].
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct MultiPolygonBuffer;

/// Selects the point arm of `buffer_all` from
/// `algorithms/detail/buffer/interface.hpp:269-273`.
impl BufferStrategyForKind for PointTag {
    type S = PointBuffer;
}

/// Selects the polygon arm of `buffer_all` from
/// `algorithms/detail/buffer/interface.hpp:269-273`.
impl BufferStrategyForKind for PolygonTag {
    type S = PolygonBuffer;
}

impl BufferStrategyForKind for LinestringTag {
    type S = LinestringBuffer;
}

impl BufferStrategyForKind for SegmentTag {
    type S = SegmentBuffer;
}

impl BufferStrategyForKind for RingTag {
    type S = RingBuffer;
}

impl BufferStrategyForKind for BoxTag {
    type S = BoxBuffer;
}

impl BufferStrategyForKind for MultiPointTag {
    type S = MultiPointBuffer;
}

impl BufferStrategyForKind for MultiLinestringTag {
    type S = MultiLinestringBuffer;
}

impl BufferStrategyForKind for MultiPolygonTag {
    type S = MultiPolygonBuffer;
}

/// Buffer a geometry using the public point and join strategies.
///
/// Mirrors `boost::geometry::buffer` from
/// `boost/geometry/algorithms/detail/buffer/interface.hpp:246-273`. Cartesian
/// dispatch supports point, segment, linestring, ring, polygon, box, and all
/// three homogeneous multi-geometry kinds. Point inputs use `point`, linear
/// inputs use all five strategy roles, and areal inputs use signed distance
/// and join policies.
///
/// # Errors
///
/// Returns [`OverlayError::Unsupported`] for non-finite distances, asymmetric
/// areal distances, or degenerate linear inputs.
#[inline]
#[must_use = "buffering can fail and the generated geometry should be used"]
pub fn buffer<G>(
    geometry: &G,
    distance: f64,
    join: JoinStrategy,
    point: PointStrategy,
) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError>
where
    G: Geometry,
    G::Kind: BufferStrategyForKind,
    <G::Kind as BufferStrategyForKind>::S: BufferStrategy<G>,
{
    let settings = BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(distance),
        side: geometry_strategy::buffer::BufferSideStrategy::Straight,
        join: match join {
            JoinStrategy::Round { points_per_circle } => {
                BufferJoinStrategy::Round { points_per_circle }
            }
            JoinStrategy::Miter => BufferJoinStrategy::Miter { limit: 5.0 },
        },
        end: BufferEndStrategy::Round {
            points_per_circle: 36,
        },
        point: match point {
            PointStrategy::Circle { points_per_circle } => {
                BufferPointStrategy::Circle { points_per_circle }
            }
            PointStrategy::Square => BufferPointStrategy::Square,
        },
    };
    buffer_with(geometry, settings)
}

/// Buffer a geometry with Boost's complete distance/side/join/end/point
/// strategy bundle.
///
/// Mirrors the five explicit strategy arguments to `boost::geometry::buffer`
/// from `algorithms/detail/buffer/interface.hpp:246-273`.
///
/// # Errors
///
/// Returns [`OverlayError::Unsupported`] for non-finite/inapplicable distance
/// policies or degenerate linear input.
#[inline]
#[must_use = "buffering can fail and the generated geometry should be used"]
pub fn buffer_with<G>(
    geometry: &G,
    settings: BufferSettings,
) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError>
where
    G: Geometry,
    G::Kind: BufferStrategyForKind,
    <G::Kind as BufferStrategyForKind>::S: BufferStrategy<G>,
{
    <<G::Kind as BufferStrategyForKind>::S as Default>::default().apply(geometry, settings)
}

/// Implements the point arm selected by `buffer_all` at
/// `algorithms/detail/buffer/interface.hpp:269-273`.
impl<G> BufferStrategy<G> for PointBuffer
where
    G: Point + PointMut + Default + Copy,
    G::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <G::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        point_geometry: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G>>, OverlayError> {
        let BufferDistanceStrategy::Symmetric(distance) = settings.distance else {
            return Err(OverlayError::Unsupported);
        };
        if !distance.is_finite() {
            return Err(OverlayError::Unsupported);
        }
        if distance <= 0.0 {
            return Ok(MultiPolygon(alloc::vec![]));
        }
        let point = match settings.point {
            BufferPointStrategy::Circle { points_per_circle } => {
                PointStrategy::Circle { points_per_circle }
            }
            BufferPointStrategy::Square => PointStrategy::Square,
        };
        let ring = buffer_point(point_geometry, distance, point);
        Ok(MultiPolygon(alloc::vec![Polygon::new(ring)]))
    }
}

/// Implements the polygon arm selected by `buffer_all` at
/// `algorithms/detail/buffer/interface.hpp:269-273`.
impl<G> BufferStrategy<G> for PolygonBuffer
where
    G: PolygonTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        polygon: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let BufferDistanceStrategy::Symmetric(distance) = settings.distance else {
            return Err(OverlayError::Unsupported);
        };
        if !distance.is_finite() || distance == 0.0 {
            return Err(OverlayError::Unsupported);
        }
        let Some(outer) = offset_ring(polygon.exterior(), distance, settings.join, true) else {
            return Ok(MultiPolygon(alloc::vec![]));
        };
        let inners = polygon
            .interiors()
            .filter_map(|ring| offset_ring(ring, -distance, settings.join, false))
            .collect::<Vec<_>>();
        let outer_vertices = distinct_vertices(&outer);
        if inners.iter().any(|inner| {
            let inner_vertices = distinct_vertices(inner);
            outer_vertices
                .iter()
                .all(|point| point_in_or_on_ring(*point, &inner_vertices))
        }) {
            return Ok(MultiPolygon::new());
        }

        Ok(MultiPolygon(alloc::vec![Polygon::with_inners(
            outer, inners,
        )]))
    }
}

impl<G> BufferStrategy<G> for LinestringBuffer
where
    G: LinestringTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        line: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let (left, right) = match settings.distance {
            BufferDistanceStrategy::Symmetric(distance) => (distance, distance),
            BufferDistanceStrategy::Asymmetric { left, right } => (left, right),
        };
        if !left.is_finite() || !right.is_finite() || left < 0.0 || right < 0.0 {
            return Err(OverlayError::Unsupported);
        }
        if left == 0.0 && right == 0.0 {
            return Ok(MultiPolygon(alloc::vec![]));
        }
        let polygon = buffer_linestring(line, left, right, settings.join, settings.end)?;
        Ok(MultiPolygon(alloc::vec![polygon]))
    }
}

impl<G> BufferStrategy<G> for SegmentBuffer
where
    G: SegmentTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        segment: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let line: Linestring<G::Point> =
            Linestring::from_vec(alloc::vec![segment_start(segment), segment_end(segment)]);
        LinestringBuffer.apply(&line, settings)
    }
}

impl<G> BufferStrategy<G> for RingBuffer
where
    G: RingTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        ring: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let BufferDistanceStrategy::Symmetric(distance) = settings.distance else {
            return Err(OverlayError::Unsupported);
        };
        if !distance.is_finite() || distance == 0.0 {
            return Err(OverlayError::Unsupported);
        }
        Ok(offset_ring(ring, distance, settings.join, true)
            .map_or_else(MultiPolygon::new, |outer| {
                MultiPolygon::from_vec(alloc::vec![Polygon::new(outer)])
            }))
    }
}

impl<G> BufferStrategy<G> for BoxBuffer
where
    G: BoxTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        bounds: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let minimum = box_min(bounds);
        let maximum = box_max(bounds);
        let min_x = minimum.get::<0>().into();
        let min_y = minimum.get::<1>().into();
        let max_x = maximum.get::<0>().into();
        let max_y = maximum.get::<1>().into();
        let ring: Ring<G::Point> = Ring::from_vec(alloc::vec![
            make_point(min_x, min_y),
            make_point(min_x, max_y),
            make_point(max_x, max_y),
            make_point(max_x, min_y),
            make_point(min_x, min_y),
        ]);
        RingBuffer.apply(&ring, settings)
    }
}

impl<G> BufferStrategy<G> for MultiPointBuffer
where
    G: MultiPointTrait<ItemPoint = <G as Geometry>::Point>,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        points: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let mut output = MultiPolygon::new();
        for point in points.points() {
            output.0.extend(PointBuffer.apply(point, settings)?.0);
        }
        crate::merge::merge_polygons(output.0)
    }
}

impl<G> BufferStrategy<G> for MultiLinestringBuffer
where
    G: MultiLinestringTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        lines: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let mut output = MultiPolygon::new();
        for line in lines.linestrings() {
            output.0.extend(LinestringBuffer.apply(line, settings)?.0);
        }
        crate::merge::merge_polygons(output.0)
    }
}

impl<G> BufferStrategy<G> for MultiPolygonBuffer
where
    G: MultiPolygonTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        polygons: &G,
        settings: BufferSettings,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let mut output = MultiPolygon::new();
        for polygon in polygons.polygons() {
            output.0.extend(PolygonBuffer.apply(polygon, settings)?.0);
        }
        crate::merge::merge_polygons(output.0)
    }
}

/// Buffer a point by `distance`, producing the disc (or square)
/// approximation.
///
/// Mirrors the point arm of `boost::geometry::buffer` with a
/// `point_circle` / `point_square` strategy
/// (`strategies/buffer/buffer_point_circle.hpp`).
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_overlay::buffer::{buffer_point, PointStrategy};
/// use geometry_algorithm::ring_area;
///
/// type P = Point2D<f64, Cartesian>;
/// let disc = buffer_point(&P::new(0.0, 0.0), 1.0, PointStrategy::Circle { points_per_circle: 360 });
/// // Area of the 360-gon closely approximates π.
/// assert!((ring_area(&disc).abs() - core::f64::consts::PI).abs() < 1e-3);
/// ```
#[inline]
#[must_use]
pub fn buffer_point<P>(center: &P, distance: f64, strategy: PointStrategy) -> Ring<P>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let cx: f64 = center.get::<0>().into();
    let cy: f64 = center.get::<1>().into();
    match strategy {
        PointStrategy::Circle { points_per_circle } => {
            circle_ring(cx, cy, distance, points_per_circle.max(3))
        }
        PointStrategy::Square => {
            let d = distance;
            // Fully-qualified `alloc::vec!`: only the `Vec` *type* is
            // imported (line 33), and the bare `vec!` macro is not in the
            // `no_std` prelude — matches the crate idiom in `assemble.rs`
            // / `traverse/state.rs`.
            Ring::from_vec(alloc::vec![
                make_point(cx - d, cy - d),
                make_point(cx - d, cy + d),
                make_point(cx + d, cy + d),
                make_point(cx + d, cy - d),
                make_point(cx - d, cy - d),
            ])
        }
    }
}

/// Buffer a **convex** polygon outward by a positive `distance`, rounding
/// the corners per `join`.
///
/// Each vertex of a convex polygon becomes a circular arc of radius
/// `distance` in the offset boundary; the arcs are joined by the offset
/// edges. Mirrors the convex case of `boost::geometry::buffer`
/// (`algorithms/buffer.hpp`) with a `join_round` strategy.
///
/// # Panics
///
/// Does not panic; a polygon with fewer than 3 exterior vertices returns
/// an empty ring's polygon.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::buffer::{buffer_convex_polygon, JoinStrategy};
/// use geometry_algorithm::ring_area;
/// use geometry_trait::Polygon as _;
///
/// type P = Point2D<f64, Cartesian>;
/// let sq: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
/// let grown = buffer_convex_polygon(&sq, 1.0, JoinStrategy::Round { points_per_circle: 720 });
/// // Area = s² + 4·s·d + π·d² = 4 + 8 + π.
/// let expected = 4.0 + 8.0 + core::f64::consts::PI;
/// assert!((ring_area(grown.exterior()).abs() - expected).abs() < 5e-2);
/// ```
#[inline]
#[must_use]
pub fn buffer_convex_polygon<G, P>(polygon: &G, distance: f64, join: JoinStrategy) -> Polygon<P>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let strategy = match join {
        JoinStrategy::Round { points_per_circle } => {
            BufferJoinStrategy::Round { points_per_circle }
        }
        JoinStrategy::Miter => BufferJoinStrategy::Miter {
            limit: f64::INFINITY,
        },
    };
    offset_ring(polygon.exterior(), distance, strategy, true)
        .map_or_else(|| Polygon::new(Ring::new()), Polygon::new)
}

fn offset_ring<R, P>(
    ring: &R,
    distance: f64,
    join: BufferJoinStrategy,
    clockwise: bool,
) -> Option<Ring<P>>
where
    R: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: Into<f64> + FromF64,
{
    let mut vertices = distinct_vertices(ring);
    if vertices.len() < 3 || !distance.is_finite() || distance == 0.0 {
        return None;
    }
    if signed_area_ccw_positive(&vertices) < 0.0 {
        vertices.reverse();
    }

    let count = vertices.len();
    let mut boundary = Vec::new();
    for index in 0..count {
        let previous = vertices[(index + count - 1) % count];
        let vertex = vertices[index];
        let next = vertices[(index + 1) % count];
        let incoming = (vertex.0 - previous.0, vertex.1 - previous.1);
        let outgoing = (next.0 - vertex.0, next.1 - vertex.1);
        let incoming_normal = outward_normal(incoming.0, incoming.1);
        let outgoing_normal = outward_normal(outgoing.0, outgoing.1);
        let before = (
            vertex.0 + incoming_normal.0 * distance,
            vertex.1 + incoming_normal.1 * distance,
        );
        let after = (
            vertex.0 + outgoing_normal.0 * distance,
            vertex.1 + outgoing_normal.1 * distance,
        );
        let intersection = line_intersection(before, incoming, after, outgoing);
        let cross = incoming.0 * outgoing.1 - incoming.1 * outgoing.0;
        let exterior_join = cross * distance > 0.0;

        if !exterior_join {
            if let Some(point) = intersection {
                boundary.push(point);
            } else {
                boundary.push(after);
            }
            continue;
        }

        match join {
            BufferJoinStrategy::Round { points_per_circle } => {
                boundary.push(before);
                push_arc_between(
                    &mut boundary,
                    vertex,
                    before,
                    after,
                    distance.abs(),
                    points_per_circle.max(4),
                    true,
                );
                boundary.push(after);
            }
            BufferJoinStrategy::Miter { limit } => {
                if let Some(point) = intersection {
                    let miter_length = hypot(point.0 - vertex.0, point.1 - vertex.1);
                    if point.0.is_finite()
                        && point.1.is_finite()
                        && miter_length <= limit.max(1.0) * distance.abs()
                    {
                        boundary.push(point);
                    } else {
                        boundary.push(before);
                        boundary.push(after);
                    }
                } else {
                    boundary.push(before);
                    boundary.push(after);
                }
            }
        }
    }

    boundary.dedup();
    if boundary.len() < 3 || signed_area_ccw_positive(&boundary).abs() <= f64::EPSILON {
        return None;
    }
    if distance < 0.0 {
        let clearance = distance.abs();
        let tolerance = mul_add(clearance, 1e-9, f64::EPSILON * 16.0);
        if boundary.iter().any(|point| {
            !point_in_or_on_ring(*point, &vertices)
                || minimum_boundary_distance(*point, &vertices) + tolerance < clearance
        }) {
            return None;
        }
    }
    if clockwise == (signed_area_ccw_positive(&boundary) > 0.0) {
        boundary.reverse();
    }
    boundary.push(boundary[0]);
    Some(Ring::from_vec(
        boundary
            .into_iter()
            .map(|(x, y)| make_point(x, y))
            .collect(),
    ))
}

fn buffer_linestring<L, P>(
    line: &L,
    left: f64,
    right: f64,
    join: BufferJoinStrategy,
    end: BufferEndStrategy,
) -> Result<Polygon<P>, OverlayError>
where
    L: LinestringTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: Into<f64> + FromF64,
{
    let mut vertices: Vec<(f64, f64)> = Vec::new();
    for point in line.points() {
        let value = (point.get::<0>().into(), point.get::<1>().into());
        if vertices.last().copied() != Some(value) {
            vertices.push(value);
        }
    }
    if vertices.len() < 2 {
        return Err(OverlayError::Unsupported);
    }

    let left_path = offset_path(&vertices, left, true, join);
    let right_path = offset_path(&vertices, right, false, join);
    if left_path.is_empty() || right_path.is_empty() {
        return Err(OverlayError::Unsupported);
    }
    let mut boundary = left_path;
    match end {
        BufferEndStrategy::Flat => {}
        BufferEndStrategy::Round { points_per_circle } => {
            let center = *vertices.last().expect("linestring has an endpoint");
            let from = *boundary.last().expect("left path has an endpoint");
            let to = *right_path.last().expect("right path has an endpoint");
            push_end_arc(
                &mut boundary,
                center,
                from,
                to,
                points_per_circle.max(4),
                true,
            );
        }
    }
    boundary.extend(right_path.iter().rev().copied());
    if let BufferEndStrategy::Round { points_per_circle } = end {
        let to = boundary[0];
        push_end_arc(
            &mut boundary,
            vertices[0],
            right_path[0],
            to,
            points_per_circle.max(4),
            true,
        );
    }
    if let Some(first) = boundary.first().copied() {
        boundary.push(first);
    }
    Ok(Polygon::new(Ring::from_vec(
        boundary
            .into_iter()
            .map(|(x, y)| make_point(x, y))
            .collect(),
    )))
}

fn offset_path(
    vertices: &[(f64, f64)],
    distance: f64,
    left: bool,
    join: BufferJoinStrategy,
) -> Vec<(f64, f64)> {
    let side = if left { 1.0 } else { -1.0 };
    let normals: Vec<(f64, f64)> = vertices
        .windows(2)
        .map(|edge| {
            let dx = edge[1].0 - edge[0].0;
            let dy = edge[1].1 - edge[0].1;
            let length = hypot(dx, dy);
            (-dy / length * side, dx / length * side)
        })
        .collect();
    let mut path = Vec::with_capacity(vertices.len());
    path.push((
        vertices[0].0 + normals[0].0 * distance,
        vertices[0].1 + normals[0].1 * distance,
    ));
    for index in 1..vertices.len() - 1 {
        let vertex = vertices[index];
        let previous = vertices[index - 1];
        let next = vertices[index + 1];
        let before = (
            vertex.0 + normals[index - 1].0 * distance,
            vertex.1 + normals[index - 1].1 * distance,
        );
        let after = (
            vertex.0 + normals[index].0 * distance,
            vertex.1 + normals[index].1 * distance,
        );
        let intersection = line_intersection(
            before,
            (vertex.0 - previous.0, vertex.1 - previous.1),
            after,
            (next.0 - vertex.0, next.1 - vertex.1),
        );
        match (join, intersection) {
            (BufferJoinStrategy::Miter { limit }, Some(point))
                if point.0.is_finite() && point.1.is_finite() =>
            {
                let miter_length = hypot(point.0 - vertex.0, point.1 - vertex.1);
                if distance == 0.0 || miter_length <= limit.max(1.0) * distance.abs() {
                    path.push(point);
                } else {
                    path.push(before);
                    path.push(after);
                }
            }
            (BufferJoinStrategy::Round { points_per_circle }, _) => {
                path.push(before);
                push_arc_between(
                    &mut path,
                    vertex,
                    before,
                    after,
                    distance.abs(),
                    points_per_circle.max(4),
                    left,
                );
                path.push(after);
            }
            _ => {
                path.push(before);
                path.push(after);
            }
        }
    }
    let last = vertices.len() - 1;
    path.push((
        vertices[last].0 + normals[last - 1].0 * distance,
        vertices[last].1 + normals[last - 1].1 * distance,
    ));
    path
}

fn line_intersection(
    first_origin: (f64, f64),
    first_direction: (f64, f64),
    second_origin: (f64, f64),
    second_direction: (f64, f64),
) -> Option<(f64, f64)> {
    let denominator =
        first_direction.0 * second_direction.1 - first_direction.1 * second_direction.0;
    if denominator.abs() <= f64::EPSILON {
        return None;
    }
    let delta = (
        second_origin.0 - first_origin.0,
        second_origin.1 - first_origin.1,
    );
    let factor = (delta.0 * second_direction.1 - delta.1 * second_direction.0) / denominator;
    Some((
        first_origin.0 + factor * first_direction.0,
        first_origin.1 + factor * first_direction.1,
    ))
}

fn push_arc_between(
    output: &mut Vec<(f64, f64)>,
    center: (f64, f64),
    from: (f64, f64),
    to: (f64, f64),
    radius: f64,
    points_per_circle: usize,
    counterclockwise: bool,
) {
    if radius == 0.0 {
        return;
    }
    let start = atan2(from.1 - center.1, from.0 - center.0);
    let mut end = atan2(to.1 - center.1, to.0 - center.0);
    if counterclockwise {
        while end < start {
            end += core::f64::consts::TAU;
        }
    } else {
        while end > start {
            end -= core::f64::consts::TAU;
        }
    }
    let sweep = end - start;
    let steps =
        ceil((sweep.abs() / core::f64::consts::TAU) * points_per_circle as f64).max(1.0) as usize;
    for step in 1..steps {
        let angle = start + sweep * step as f64 / steps as f64;
        output.push((
            center.0 + radius * cos(angle),
            center.1 + radius * sin(angle),
        ));
    }
}

fn push_end_arc(
    output: &mut Vec<(f64, f64)>,
    center: (f64, f64),
    from: (f64, f64),
    to: (f64, f64),
    points_per_circle: usize,
    clockwise: bool,
) {
    let radius =
        hypot(from.0 - center.0, from.1 - center.1).max(hypot(to.0 - center.0, to.1 - center.1));
    push_arc_between(
        output,
        center,
        from,
        to,
        radius,
        points_per_circle,
        !clockwise,
    );
}

/// Materialise an output point from the `f64` kernel coordinates.
fn make_point<P>(x: f64, y: f64) -> P
where
    P: PointMut + Default,
    P::Scalar: FromF64,
{
    let mut p = P::default();
    p.set::<0>(P::Scalar::from_f64(x));
    p.set::<1>(P::Scalar::from_f64(y));
    p
}

/// A regular-polygon approximation of a circle, clockwise and closed.
fn circle_ring<P>(cx: f64, cy: f64, r: f64, segments: usize) -> Ring<P>
where
    P: PointMut + Default + Copy,
    P::Scalar: FromF64,
{
    let mut pts = Vec::with_capacity(segments + 1);
    let step = core::f64::consts::TAU / segments as f64;
    for k in 0..segments {
        let a = -step * k as f64;
        pts.push(make_point(cx + r * cos(a), cy + r * sin(a)));
    }
    pts.push(pts[0]);
    Ring::from_vec(pts)
}

/// Distinct consecutive vertices of a ring as `f64` pairs (drops the
/// closing repeat).
fn distinct_vertices<R>(ring: &R) -> Vec<(f64, f64)>
where
    R: RingTrait,
    <R::Point as Point>::Scalar: Into<f64>,
{
    let mut pts: Vec<(f64, f64)> = ring
        .points()
        .map(|p| (p.get::<0>().into(), p.get::<1>().into()))
        .collect();
    if pts.len() >= 2 {
        let first = pts[0];
        let last = pts[pts.len() - 1];
        if first == last {
            pts.pop();
        }
    }
    pts
}

/// The standard math signed area of the vertex ring (counter-clockwise
/// positive), via the shoelace sum over the closed loop. Used only to
/// detect winding for normalisation.
fn signed_area_ccw_positive(verts: &[(f64, f64)]) -> f64 {
    let n = verts.len();
    let mut acc = 0.0;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        acc += a.0 * b.1 - b.0 * a.1;
    }
    acc * 0.5
}

fn minimum_boundary_distance(point: (f64, f64), vertices: &[(f64, f64)]) -> f64 {
    let mut minimum = f64::INFINITY;
    for index in 0..vertices.len() {
        let start = vertices[index];
        let end = vertices[(index + 1) % vertices.len()];
        let delta = (end.0 - start.0, end.1 - start.1);
        let length_squared = delta.0 * delta.0 + delta.1 * delta.1;
        let fraction = if length_squared == 0.0 {
            0.0
        } else {
            (((point.0 - start.0) * delta.0 + (point.1 - start.1) * delta.1) / length_squared)
                .clamp(0.0, 1.0)
        };
        let nearest = (start.0 + fraction * delta.0, start.1 + fraction * delta.1);
        minimum = minimum.min(hypot(point.0 - nearest.0, point.1 - nearest.1));
    }
    minimum
}

fn point_in_or_on_ring(point: (f64, f64), vertices: &[(f64, f64)]) -> bool {
    let scale = vertices.iter().fold(1.0_f64, |acc, vertex| {
        acc.max(vertex.0.abs()).max(vertex.1.abs())
    });
    if minimum_boundary_distance(point, vertices) <= scale * 1e-12 {
        return true;
    }

    let mut inside = false;
    for index in 0..vertices.len() {
        let start = vertices[index];
        let end = vertices[(index + 1) % vertices.len()];
        if (start.1 > point.1) != (end.1 > point.1)
            && point.0 < (end.0 - start.0) * (point.1 - start.1) / (end.1 - start.1) + start.0
        {
            inside = !inside;
        }
    }
    inside
}

/// The outward unit normal of a directed CCW edge with delta
/// `(dx, dy)` (pointing to the edge's right).
fn outward_normal(dx: f64, dy: f64) -> (f64, f64) {
    let len = (dx * dx + dy * dy).sqrt();
    if len == 0.0 {
        return (0.0, 0.0);
    }
    // Right-hand normal of (dx, dy) is (dy, -dx).
    (dy / len, -dx / len)
}

#[cfg(test)]
mod tests {
    //! OVL7 done-when: buffered areas match the closed-form values.
    //! Mirrors `test/algorithms/buffer/`.

    use super::{JoinStrategy, PointStrategy, buffer, buffer_convex_polygon, buffer_point};
    use geometry_algorithm::ring_area;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};
    use geometry_trait::{MultiPolygon as _, Polygon as _};

    type P = Point2D<f64, Cartesian>;

    fn close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "expected {b}, got {a}");
    }

    #[test]
    fn point_circle_area_approximates_pi_r_squared() {
        let disc = buffer_point(
            &P::new(0.0, 0.0),
            2.0,
            PointStrategy::Circle {
                points_per_circle: 720,
            },
        );
        // π·r² = π·4.
        close(ring_area(&disc).abs(), core::f64::consts::PI * 4.0, 1e-2);
    }

    #[test]
    fn point_square_area() {
        let sq = buffer_point(&P::new(0.0, 0.0), 3.0, PointStrategy::Square);
        // A square of half-side 3 → side 6 → area 36.
        close(ring_area(&sq).abs(), 36.0, 1e-9);
    }

    #[test]
    fn convex_square_round_buffer_area() {
        let sq: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let grown = buffer_convex_polygon(
            &sq,
            1.0,
            JoinStrategy::Round {
                points_per_circle: 720,
            },
        );
        // s² + 4·s·d + π·d² = 4 + 8 + π.
        let expected = 4.0 + 8.0 + core::f64::consts::PI;
        close(ring_area(grown.exterior()).abs(), expected, 1e-2);
    }

    #[test]
    fn convex_triangle_round_buffer_grows() {
        let tri: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (0.0, 0.0)]];
        let base = ring_area(tri.exterior()).abs(); // 6
        let grown = buffer_convex_polygon(
            &tri,
            0.5,
            JoinStrategy::Round {
                points_per_circle: 360,
            },
        );
        // The buffered area must exceed the original.
        assert!(ring_area(grown.exterior()).abs() > base);
    }

    #[test]
    fn buffer_is_winding_independent() {
        // Regression: the same square listed clockwise and counter-
        // clockwise must buffer to the same grown area. The winding
        // normalisation makes the outward offset direction correct for
        // both.
        let ccw: Polygon<P> =
            polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let cw: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]];
        let j = JoinStrategy::Round {
            points_per_circle: 720,
        };
        let expected = 4.0 + 8.0 + core::f64::consts::PI;
        let grown_from_counterclockwise =
            ring_area(buffer_convex_polygon(&ccw, 1.0, j).exterior()).abs();
        let grown_from_clockwise = ring_area(buffer_convex_polygon(&cw, 1.0, j).exterior()).abs();
        close(grown_from_counterclockwise, expected, 5e-2);
        close(grown_from_clockwise, expected, 5e-2);
    }

    #[test]
    fn miter_square_area_is_16() {
        // Regression: the old Miter arm placed the corner point at
        // distance d along the bisector (ON the round arc), yielding
        // 14.83 — smaller than even the round buffer. A true miter
        // corner is the offset-edge intersection at √2·d, so the
        // buffered 2×2 square is s² + 4·s·d + 4·d² = 16 exactly.
        let sq: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let grown = buffer_convex_polygon(&sq, 1.0, JoinStrategy::Miter);
        close(ring_area(grown.exterior()).abs(), 16.0, 1e-9);
    }

    #[test]
    fn miter_contains_near_corner_probe() {
        // A point at distance 0.99 < d from the input corner, in the
        // 22.5° direction, was EXCLUDED by the old chord-cut corner.
        use geometry_algorithm::within;
        let sq: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let grown = buffer_convex_polygon(&sq, 1.0, JoinStrategy::Miter);
        let ang = 22.5_f64.to_radians();
        let probe = P::new(2.0 + 0.99 * ang.cos(), 2.0 + 0.99 * ang.sin());
        assert!(
            within(&probe, &grown),
            "buffer must contain points within d"
        );
    }

    #[test]
    fn miter_is_superset_of_round_by_area() {
        // A miter fills the wedge beyond the round arc, so its area
        // can never be below the round join's.
        let j_round = JoinStrategy::Round {
            points_per_circle: 720,
        };
        let square: Polygon<P> =
            polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let triangle: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (0.0, 0.0)]];
        for pg in [square, triangle] {
            let m =
                ring_area(buffer_convex_polygon(&pg, 1.0, JoinStrategy::Miter).exterior()).abs();
            let r = ring_area(buffer_convex_polygon(&pg, 1.0, j_round).exterior()).abs();
            assert!(m >= r - 1e-9, "miter {m} must not be below round {r}");
        }
    }

    #[test]
    fn non_model_polygon_buffers_like_the_model_polygon() {
        // The generic signature accepts any `Polygon` trait impl — a
        // hand-rolled type must buffer to the same area as the same
        // shape held in a model polygon.
        use geometry_model::Ring;
        use geometry_tag::PolygonTag;
        use geometry_trait::{Geometry, Polygon as PolygonTrait};

        struct Parcel {
            outer: Ring<P>,
        }
        impl Geometry for Parcel {
            type Kind = PolygonTag;
            type Point = P;
        }
        impl PolygonTrait for Parcel {
            type Ring = Ring<P>;
            fn exterior(&self) -> &Ring<P> {
                &self.outer
            }
            fn interiors(&self) -> impl ExactSizeIterator<Item = &Ring<P>> {
                core::iter::empty()
            }
        }

        let pts = vec![
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
            P::new(2.0, 2.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ];
        let parcel = Parcel {
            outer: Ring::from_vec(pts.clone()),
        };
        let model: Polygon<P> = Polygon::new(Ring::from_vec(pts));
        let j = JoinStrategy::Round {
            points_per_circle: 360,
        };
        let parcel_buffer = buffer(&parcel, 1.0, j, PointStrategy::Square).unwrap();
        let model_buffer = buffer(&model, 1.0, j, PointStrategy::Square).unwrap();
        let a = ring_area(parcel_buffer.polygons().next().unwrap().exterior()).abs();
        let b = ring_area(model_buffer.polygons().next().unwrap().exterior()).abs();
        close(a, b, 1e-12);
    }

    #[test]
    fn miter_is_winding_independent() {
        // Same square listed CW and CCW buffers to the same miter area.
        let ccw: Polygon<P> =
            polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let cw: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]];
        close(
            ring_area(buffer_convex_polygon(&ccw, 1.0, JoinStrategy::Miter).exterior()).abs(),
            16.0,
            1e-9,
        );
        close(
            ring_area(buffer_convex_polygon(&cw, 1.0, JoinStrategy::Miter).exterior()).abs(),
            16.0,
            1e-9,
        );
    }
}
