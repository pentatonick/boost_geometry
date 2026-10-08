//! OVL7 — `buffer`: grow a geometry outward by a fixed distance.
//!
//! Mirrors `boost/geometry/algorithms/buffer.hpp` and the buffer
//! strategies under `strategies/buffer/`. A buffer offsets every part of
//! the input outward by `distance`, rounding or mitering the corners,
//! and unions the offset pieces into an output polygon.
//!
//! Cartesian dispatch covers every static single and homogeneous multi kind.
//! Spherical and geographic inputs are projected into a local tangent plane,
//! buffered by the same Cartesian engine, and transformed back. The angular
//! path is intended for local buffers: unlike Boost's per-segment geodesic
//! offset formulas, its error grows with the geometry's angular extent and it
//! rejects projection centers at the poles. This deliberate approximation is
//! recorded in the project feature-parity map for later reassessment.
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
    math::{atan2, ceil, cos, hypot, mul_add, sin, sqrt},
};
use geometry_cs::{
    AngleUnit, Cartesian, CartesianFamily, CoordinateSystem, FromF64, Geographic, GeographicFamily,
    Spherical, SphericalFamily,
};
use geometry_model::{
    Box as ModelBox, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
    Segment,
};
use geometry_strategy::buffer::{
    BufferDistanceStrategy, BufferEndStrategy, BufferJoinStrategy, BufferPointStrategy,
    BufferSettings, CartesianBuffer, DefaultBuffer, DefaultBufferStrategy, GeographicBuffer,
    SphericalBuffer,
};
use geometry_strategy::{DouglasPeucker, PointToSegment, Pythagoras, SimplifyStrategy};
use geometry_tag::{
    BoxTag, LinestringTag, MultiLinestringTag, MultiPointTag, MultiPolygonTag, PointTag,
    PolygonTag, RingTag, SameAs, SegmentTag,
};
use geometry_trait::{
    Box as BoxTrait, Closure, Geometry, Linestring as LinestringTrait,
    MultiLinestring as MultiLinestringTrait, MultiPoint as MultiPointTrait,
    MultiPolygon as MultiPolygonTrait, Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait,
    Segment as SegmentTrait, box_max, box_min, segment_end, segment_start,
};

use crate::operation::{OverlayError, difference_multi, union_multi};
use crate::predicate::segment_intersection::{SegmentIntersection, segment_intersection};

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
pub trait BufferStrategy<G: Geometry, CoordinateStrategy> {
    fn apply(
        &self,
        geometry: &G,
        settings: BufferSettings,
        coordinate_strategy: &CoordinateStrategy,
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
/// `boost/geometry/algorithms/detail/buffer/interface.hpp:246-273`. Cartesian,
/// spherical, and geographic dispatch supports point, segment, linestring,
/// ring, polygon, box, and all three homogeneous multi-geometry kinds. Point
/// inputs use `point`, linear inputs use all five strategy roles, and areal
/// inputs use signed distance and join policies.
///
/// # Errors
///
/// Returns [`OverlayError::Unsupported`] for non-finite distances or
/// asymmetric areal distances. A linear or areal input that simplifies to a
/// single point is buffered as that point, as Boost buffers it.
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
    <<G::Point as Point>::Cs as CoordinateSystem>::Family:
        DefaultBuffer<<<G::Point as Point>::Cs as CoordinateSystem>::Family>,
    <G::Kind as BufferStrategyForKind>::S: BufferStrategy<G, DefaultBufferStrategy<G>>,
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
/// policies, among them an asymmetric distance negative on one side only.
#[inline]
#[must_use = "buffering can fail and the generated geometry should be used"]
pub fn buffer_with<G>(
    geometry: &G,
    settings: BufferSettings,
) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError>
where
    G: Geometry,
    G::Kind: BufferStrategyForKind,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family:
        DefaultBuffer<<<G::Point as Point>::Cs as CoordinateSystem>::Family>,
    <G::Kind as BufferStrategyForKind>::S: BufferStrategy<G, DefaultBufferStrategy<G>>,
{
    buffer_with_strategy(geometry, settings, DefaultBufferStrategy::<G>::default())
}

/// Buffer a geometry with explicit coordinate-system and five-role strategy
/// bundles.
///
/// Mirrors the explicit strategy overload of `boost::geometry::buffer` from
/// `algorithms/detail/buffer/interface.hpp:246-273`, together with the
/// Cartesian, spherical, and geographic umbrella strategies under
/// `strategies/buffer/`.
///
/// [`SphericalBuffer`] and [`GeographicBuffer`] use a geometry-centered local
/// tangent projection before invoking the Cartesian offset engine. This keeps
/// distance units explicit and `no_std` compatible, but is a local-extent
/// approximation rather than Boost's per-segment geodesic construction.
///
/// # Errors
///
/// Returns [`OverlayError::Unsupported`] for invalid strategy values or
/// non-finite/inapplicable distances.
#[inline]
#[must_use = "buffering can fail and the generated geometry should be used"]
#[allow(
    clippy::needless_pass_by_value,
    reason = "Boost buffer coordinate strategies are small value objects passed explicitly"
)]
pub fn buffer_with_strategy<G, CoordinateStrategy>(
    geometry: &G,
    settings: BufferSettings,
    coordinate_strategy: CoordinateStrategy,
) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError>
where
    G: Geometry,
    G::Kind: BufferStrategyForKind,
    <G::Kind as BufferStrategyForKind>::S: BufferStrategy<G, CoordinateStrategy>,
{
    <<G::Kind as BufferStrategyForKind>::S as Default>::default().apply(
        geometry,
        settings,
        &coordinate_strategy,
    )
}

/// A polygon buffered at a distance of zero.
///
/// C++: `buffer_inserter` builds an offsetted ring per input ring and then
/// finds the turns between them, discards those inside the original, and
/// traverses what is left. Where the offsetted rings do not meet each other
/// there are no turns, nothing is discarded and nothing is traversed, and the
/// rings themselves are the answer — which is the case
/// `repair_one_polygon` needs and the case this arm answers.
///
/// A ring that does meet itself needs `check_turn_in_original` and the buffer
/// traversal, which are not ported; that asks for something this arm cannot
/// answer, and it says so rather than guessing.
fn zero_width_polygon_buffer<G>(
    polygon: &G,
) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError>
where
    G: PolygonTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
{
    use crate::piece_collection::{ZeroWidthOutcome, zero_width_outcome, zero_width_rings};

    let rings = zero_width_rings(polygon);
    match zero_width_outcome(&rings) {
        ZeroWidthOutcome::RingsStand => Ok(MultiPolygon(
            rings.into_iter().map(Polygon::new).collect::<Vec<_>>(),
        )),
        ZeroWidthOutcome::NeedsTraversal => Err(OverlayError::Unsupported),
    }
}

/// Implements the point arm selected by `buffer_all` at
/// `algorithms/detail/buffer/interface.hpp:269-273`.
impl<G> BufferStrategy<G, CartesianBuffer> for PointBuffer
where
    G: Point + PointMut + Default + Copy,
    G::Scalar: CoordinateScalar<Measure = G::Scalar> + Into<f64> + FromF64,
    <G::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        point_geometry: &G,
        settings: BufferSettings,
        _coordinate_strategy: &CartesianBuffer,
    ) -> Result<MultiPolygon<Polygon<G>>, OverlayError> {
        let BufferDistanceStrategy::Symmetric(distance) = settings.distance else {
            return Err(OverlayError::Unsupported);
        };
        if !distance.is_finite() {
            return Err(OverlayError::Unsupported);
        }
        // C++: `distance_symmetric::apply` hands every side the distance's
        // magnitude; only an areal geometry reads its sign, as a deflation.
        let distance = distance.abs();
        if distance == 0.0 {
            return Ok(MultiPolygon(alloc::vec![]));
        }
        Ok(point_buffer(
            (
                point_geometry.get::<0>().into(),
                point_geometry.get::<1>().into(),
            ),
            distance,
            settings.point,
        ))
    }
}

/// Implements the polygon arm selected by `buffer_all` at
/// `algorithms/detail/buffer/interface.hpp:269-273`.
impl<G> BufferStrategy<G, CartesianBuffer> for PolygonBuffer
where
    G: PolygonTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        polygon: &G,
        settings: BufferSettings,
        _coordinate_strategy: &CartesianBuffer,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let BufferDistanceStrategy::Symmetric(distance) = settings.distance else {
            return Err(OverlayError::Unsupported);
        };
        if !distance.is_finite() {
            return Err(OverlayError::Unsupported);
        }
        if distance == 0.0 {
            // C++: a zero-width buffer is not a no-op and not a special case
            // either — `buffer_inserter` runs its whole pipeline, and every
            // side simply offsets onto itself. It is what `repair_one_polygon`
            // falls back on, so it has to answer.
            return zero_width_polygon_buffer(polygon);
        }
        // C++: `buffer_inserter_ring` offsets each ring as `simplify_input`
        // leaves it, and an exterior left with fewer points than a ring needs
        // is buffered as its first point, which simplifying keeps; deflated,
        // that leaves nothing. A hole left that short encloses nothing.
        let Some(exterior) = simplified_ring(polygon.exterior(), distance) else {
            return Ok(match polygon.exterior().points().next() {
                Some(point) if distance > 0.0 => point_buffer(
                    (point.get::<0>().into(), point.get::<1>().into()),
                    distance,
                    settings.point,
                ),
                _ => MultiPolygon::new(),
            });
        };
        let simplified = Polygon::with_inners(
            exterior,
            polygon
                .interiors()
                .filter_map(|ring| simplified_ring(ring, distance))
                .collect(),
        );
        let outer = offset_ring(simplified.exterior(), distance, settings.join, true);
        let inners = simplified
            .interiors()
            .map(|ring| offset_ring(ring, -distance, settings.join, false))
            .collect::<Option<Vec<_>>>();
        let (Some(outer), Some(inners)) = (outer, inners) else {
            return dissolve_offset(&simplified, distance, settings.join);
        };
        if offset_rings_cross(&outer, &inners, distance) {
            return dissolve_offset(&simplified, distance, settings.join);
        }
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

impl<G> BufferStrategy<G, CartesianBuffer> for LinestringBuffer
where
    G: LinestringTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        line: &G,
        settings: BufferSettings,
        _coordinate_strategy: &CartesianBuffer,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        // C++: `distance_symmetric::apply` and `distance_asymmetric::apply`
        // hand each side the distance's magnitude when the distance is
        // negative — both sides of it, for the asymmetric one. One side
        // negative and the other not pushes that side's offset across the
        // line, which is not ported.
        let (left, right) = match settings.distance {
            BufferDistanceStrategy::Symmetric(distance) => (distance.abs(), distance.abs()),
            BufferDistanceStrategy::Asymmetric { left, right } if left < 0.0 && right < 0.0 => {
                (left.abs(), right.abs())
            }
            BufferDistanceStrategy::Asymmetric { left, right } => (left, right),
        };
        if !left.is_finite() || !right.is_finite() || left < 0.0 || right < 0.0 {
            return Err(OverlayError::Unsupported);
        }
        if left == 0.0 && right == 0.0 {
            return Ok(MultiPolygon(alloc::vec![]));
        }
        buffer_linestring(line, left, right, settings)
    }
}

impl<G> BufferStrategy<G, CartesianBuffer> for SegmentBuffer
where
    G: SegmentTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        segment: &G,
        settings: BufferSettings,
        coordinate_strategy: &CartesianBuffer,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let line: Linestring<G::Point> =
            Linestring::from_vec(alloc::vec![segment_start(segment), segment_end(segment)]);
        LinestringBuffer.apply(&line, settings, coordinate_strategy)
    }
}

impl<G> BufferStrategy<G, CartesianBuffer> for RingBuffer
where
    G: RingTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        ring: &G,
        settings: BufferSettings,
        coordinate_strategy: &CartesianBuffer,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let BufferDistanceStrategy::Symmetric(distance) = settings.distance else {
            return Err(OverlayError::Unsupported);
        };
        if !distance.is_finite() {
            return Err(OverlayError::Unsupported);
        }
        // C++: `buffer_inserter<ring_tag>` is the polygon inserter over one
        // ring. Its offset can cross itself just as a polygon's can, so it
        // takes the polygon arm — and with it the dissolve. The copy is a
        // closed ring, so an open one is closed on the way.
        let mut points: Vec<G::Point> = ring.points().copied().collect();
        if ring.closure() == Closure::Open {
            if let Some(&first) = points.first() {
                points.push(first);
            }
        }
        let polygon: Polygon<G::Point> = Polygon::new(Ring::from_vec(points));
        PolygonBuffer.apply(&polygon, settings, coordinate_strategy)
    }
}

impl<G> BufferStrategy<G, CartesianBuffer> for BoxBuffer
where
    G: BoxTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        bounds: &G,
        settings: BufferSettings,
        coordinate_strategy: &CartesianBuffer,
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
        RingBuffer.apply(&ring, settings, coordinate_strategy)
    }
}

impl<G> BufferStrategy<G, CartesianBuffer> for MultiPointBuffer
where
    G: MultiPointTrait<ItemPoint = <G as Geometry>::Point>,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        points: &G,
        settings: BufferSettings,
        coordinate_strategy: &CartesianBuffer,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let mut output = MultiPolygon::new();
        for point in points.points() {
            output
                .0
                .extend(PointBuffer.apply(point, settings, coordinate_strategy)?.0);
        }
        crate::merge::merge_polygons(output.0)
    }
}

impl<G> BufferStrategy<G, CartesianBuffer> for MultiLinestringBuffer
where
    G: MultiLinestringTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        lines: &G,
        settings: BufferSettings,
        coordinate_strategy: &CartesianBuffer,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let mut output = MultiPolygon::new();
        for line in lines.linestrings() {
            output.0.extend(
                LinestringBuffer
                    .apply(line, settings, coordinate_strategy)?
                    .0,
            );
        }
        crate::merge::merge_polygons(output.0)
    }
}

impl<G> BufferStrategy<G, CartesianBuffer> for MultiPolygonBuffer
where
    G: MultiPolygonTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(
        &self,
        polygons: &G,
        settings: BufferSettings,
        coordinate_strategy: &CartesianBuffer,
    ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
        let mut output = MultiPolygon::new();
        for polygon in polygons.polygons() {
            output.0.extend(
                PolygonBuffer
                    .apply(polygon, settings, coordinate_strategy)?
                    .0,
            );
        }
        crate::merge::merge_polygons(output.0)
    }
}

trait AngularCoordinateSystem {
    type Units: AngleUnit;
}

impl<Units: AngleUnit> AngularCoordinateSystem for Spherical<Units> {
    type Units = Units;
}

impl<Units: AngleUnit> AngularCoordinateSystem for Geographic<Units> {
    type Units = Units;
}

#[derive(Debug, Clone, Copy)]
struct LocalProjection {
    longitude: f64,
    latitude: f64,
    east_scale: f64,
    north_scale: f64,
}

impl LocalProjection {
    fn project(self, longitude: f64, latitude: f64) -> (f64, f64) {
        let mut delta_longitude = longitude - self.longitude;
        if delta_longitude > core::f64::consts::PI {
            delta_longitude -= 2.0 * core::f64::consts::PI;
        } else if delta_longitude < -core::f64::consts::PI {
            delta_longitude += 2.0 * core::f64::consts::PI;
        }
        (
            delta_longitude * self.east_scale,
            (latitude - self.latitude) * self.north_scale,
        )
    }

    fn unproject(self, x: f64, y: f64) -> (f64, f64) {
        let mut longitude = self.longitude + x / self.east_scale;
        if longitude > core::f64::consts::PI {
            longitude -= 2.0 * core::f64::consts::PI;
        } else if longitude < -core::f64::consts::PI {
            longitude += 2.0 * core::f64::consts::PI;
        }
        (longitude, self.latitude + y / self.north_scale)
    }
}

trait AngularBufferProjection {
    fn projection(&self, longitude: f64, latitude: f64) -> Result<LocalProjection, OverlayError>;
}

impl AngularBufferProjection for SphericalBuffer {
    fn projection(&self, longitude: f64, latitude: f64) -> Result<LocalProjection, OverlayError> {
        if !self.radius.is_finite() || self.radius <= 0.0 {
            return Err(OverlayError::Unsupported);
        }
        let longitude_scale = cos(latitude);
        if longitude_scale.abs() <= f64::EPSILON {
            return Err(OverlayError::Unsupported);
        }
        let east_scale = self.radius * longitude_scale;
        Ok(LocalProjection {
            longitude,
            latitude,
            east_scale,
            north_scale: self.radius,
        })
    }
}

impl AngularBufferProjection for GeographicBuffer {
    fn projection(&self, longitude: f64, latitude: f64) -> Result<LocalProjection, OverlayError> {
        let spheroid = self.spheroid;
        if !spheroid.equatorial_radius.is_finite()
            || spheroid.equatorial_radius <= 0.0
            || !spheroid.flattening.is_finite()
            || !(0.0..1.0).contains(&spheroid.flattening)
        {
            return Err(OverlayError::Unsupported);
        }

        let eccentricity_squared = spheroid.eccentricity_squared();
        let sin_latitude = sin(latitude);
        let denominator = sqrt(1.0 - eccentricity_squared * sin_latitude * sin_latitude);
        let prime_vertical = spheroid.equatorial_radius / denominator;
        let meridional = spheroid.equatorial_radius * (1.0 - eccentricity_squared)
            / (denominator * denominator * denominator);
        let longitude_scale = cos(latitude);
        if longitude_scale.abs() <= f64::EPSILON {
            return Err(OverlayError::Unsupported);
        }
        let east_scale = prime_vertical * longitude_scale;
        Ok(LocalProjection {
            longitude,
            latitude,
            east_scale,
            north_scale: meridional,
        })
    }
}

fn angular_coordinates<P>(point: &P) -> (f64, f64)
where
    P: Point,
    P::Scalar: Into<f64>,
    P::Cs: AngularCoordinateSystem,
{
    let longitude = <P::Cs as AngularCoordinateSystem>::Units::to_radians(point.get::<0>().into());
    let latitude = <P::Cs as AngularCoordinateSystem>::Units::to_radians(point.get::<1>().into());
    (longitude, latitude)
}

fn angular_point<P>(longitude: f64, latitude: f64) -> P
where
    P: PointMut + Default,
    P::Scalar: FromF64,
    P::Cs: AngularCoordinateSystem,
{
    let mut point = P::default();
    let longitude = <P::Cs as AngularCoordinateSystem>::Units::from_radians(longitude);
    let latitude = <P::Cs as AngularCoordinateSystem>::Units::from_radians(latitude);
    point.set::<0>(P::Scalar::from_f64(longitude));
    point.set::<1>(P::Scalar::from_f64(latitude));
    point
}

fn projection_center(coordinates: &[(f64, f64)]) -> Result<(f64, f64), OverlayError> {
    if coordinates.is_empty() {
        return Err(OverlayError::Unsupported);
    }
    let mut longitude_sine = 0.0;
    let mut longitude_cosine = 0.0;
    let mut latitude = 0.0;
    for &(longitude, point_latitude) in coordinates {
        longitude_sine += sin(longitude);
        longitude_cosine += cos(longitude);
        latitude += point_latitude;
    }
    let count = coordinates.len() as f64;
    Ok((atan2(longitude_sine, longitude_cosine), latitude / count))
}

type ProjectedPoint = Point2D<f64, Cartesian>;

fn projected_point<P>(point: &P, projection: LocalProjection) -> ProjectedPoint
where
    P: Point,
    P::Scalar: Into<f64>,
    P::Cs: AngularCoordinateSystem,
{
    let (longitude, latitude) = angular_coordinates(point);
    let (x, y) = projection.project(longitude, latitude);
    ProjectedPoint::new(x, y)
}

fn projected_ring<R>(ring: &R, projection: LocalProjection) -> Ring<ProjectedPoint>
where
    R: RingTrait,
    R::Point: Point,
    <R::Point as Point>::Scalar: Into<f64>,
    <R::Point as Point>::Cs: AngularCoordinateSystem,
{
    Ring::from_vec(
        ring.points()
            .map(|point| projected_point(point, projection))
            .collect(),
    )
}

fn projected_polygon<G>(polygon: &G, projection: LocalProjection) -> Polygon<ProjectedPoint>
where
    G: PolygonTrait,
    G::Point: Point,
    <G::Point as Point>::Scalar: Into<f64>,
    <G::Point as Point>::Cs: AngularCoordinateSystem,
{
    Polygon::with_inners(
        projected_ring(polygon.exterior(), projection),
        polygon
            .interiors()
            .map(|ring| projected_ring(ring, projection))
            .collect(),
    )
}

fn unprojected_buffer<P>(
    polygons: MultiPolygon<Polygon<ProjectedPoint>>,
    projection: LocalProjection,
) -> MultiPolygon<Polygon<P>>
where
    P: PointMut + Default,
    P::Scalar: FromF64,
    P::Cs: AngularCoordinateSystem,
{
    MultiPolygon::from_vec(
        polygons
            .0
            .into_iter()
            .map(|polygon| {
                let outer = Ring::from_vec(
                    polygon
                        .outer
                        .0
                        .into_iter()
                        .map(|point| {
                            let (longitude, latitude) = projection.unproject(point.x(), point.y());
                            angular_point(longitude, latitude)
                        })
                        .collect(),
                );
                let inners = polygon
                    .inners
                    .into_iter()
                    .map(|ring| {
                        Ring::from_vec(
                            ring.0
                                .into_iter()
                                .map(|point| {
                                    let (longitude, latitude) =
                                        projection.unproject(point.x(), point.y());
                                    angular_point(longitude, latitude)
                                })
                                .collect(),
                        )
                    })
                    .collect();
                Polygon::with_inners(outer, inners)
            })
            .collect(),
    )
}

fn projection_for_points<'a, P>(
    points: impl IntoIterator<Item = &'a P>,
    strategy: &impl AngularBufferProjection,
) -> Result<LocalProjection, OverlayError>
where
    P: Point + 'a,
    P::Scalar: Into<f64>,
    P::Cs: AngularCoordinateSystem,
{
    let coordinates: Vec<_> = points.into_iter().map(angular_coordinates).collect();
    let (longitude, latitude) = projection_center(&coordinates)?;
    strategy.projection(longitude, latitude)
}

fn projected_point_apply<P>(
    point: &P,
    settings: BufferSettings,
    strategy: &impl AngularBufferProjection,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    P: Point + PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64> + FromF64,
    P::Cs: AngularCoordinateSystem,
{
    let projection = projection_for_points(core::iter::once(point), strategy)?;
    let point = projected_point(point, projection);
    let output = PointBuffer.apply(&point, settings, &CartesianBuffer)?;
    Ok(unprojected_buffer(output, projection))
}

fn projected_linestring_apply<L>(
    line: &L,
    settings: BufferSettings,
    strategy: &impl AngularBufferProjection,
) -> Result<MultiPolygon<Polygon<L::Point>>, OverlayError>
where
    L: LinestringTrait,
    L::Point: PointMut + Default + Copy,
    <L::Point as Point>::Scalar:
        CoordinateScalar<Measure = <L::Point as Point>::Scalar> + Into<f64> + FromF64,
    <L::Point as Point>::Cs: AngularCoordinateSystem,
{
    let projection = projection_for_points(line.points(), strategy)?;
    let projected = Linestring::from_vec(
        line.points()
            .map(|point| projected_point(point, projection))
            .collect(),
    );
    let output = LinestringBuffer.apply(&projected, settings, &CartesianBuffer)?;
    Ok(unprojected_buffer(output, projection))
}

fn projected_ring_apply<R>(
    ring: &R,
    settings: BufferSettings,
    strategy: &impl AngularBufferProjection,
) -> Result<MultiPolygon<Polygon<R::Point>>, OverlayError>
where
    R: RingTrait,
    R::Point: PointMut + Default + Copy,
    <R::Point as Point>::Scalar:
        CoordinateScalar<Measure = <R::Point as Point>::Scalar> + Into<f64> + FromF64,
    <R::Point as Point>::Cs: AngularCoordinateSystem,
{
    let projection = projection_for_points(ring.points(), strategy)?;
    let output = RingBuffer.apply(
        &projected_ring(ring, projection),
        settings,
        &CartesianBuffer,
    )?;
    Ok(unprojected_buffer(output, projection))
}

fn projected_polygon_apply<G>(
    polygon: &G,
    settings: BufferSettings,
    strategy: &impl AngularBufferProjection,
) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError>
where
    G: PolygonTrait,
    G::Point: PointMut + Default + Copy,
    <G::Point as Point>::Scalar:
        CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
    <G::Point as Point>::Cs: AngularCoordinateSystem,
{
    let mut coordinates = polygon
        .exterior()
        .points()
        .map(angular_coordinates)
        .collect::<Vec<_>>();
    for ring in polygon.interiors() {
        coordinates.extend(ring.points().map(angular_coordinates));
    }
    let (longitude, latitude) = projection_center(&coordinates)?;
    let projection = strategy.projection(longitude, latitude)?;
    let output = PolygonBuffer.apply(
        &projected_polygon(polygon, projection),
        settings,
        &CartesianBuffer,
    )?;
    Ok(unprojected_buffer(output, projection))
}

macro_rules! impl_angular_buffer_strategy {
    ($strategy:ty, $family:ty) => {
        impl<G> BufferStrategy<G, $strategy> for PointBuffer
        where
            G: Point + PointMut + Default + Copy,
            G::Scalar: CoordinateScalar<Measure = G::Scalar> + Into<f64> + FromF64,
            G::Cs: AngularCoordinateSystem,
            <G::Cs as CoordinateSystem>::Family: SameAs<$family>,
        {
            fn apply(
                &self,
                geometry: &G,
                settings: BufferSettings,
                coordinate_strategy: &$strategy,
            ) -> Result<MultiPolygon<Polygon<G>>, OverlayError> {
                projected_point_apply(geometry, settings, coordinate_strategy)
            }
        }

        impl<G> BufferStrategy<G, $strategy> for LinestringBuffer
        where
            G: LinestringTrait,
            G::Point: PointMut + Default + Copy,
            <G::Point as Point>::Scalar:
                CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
            <G::Point as Point>::Cs: AngularCoordinateSystem,
            <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<$family>,
        {
            fn apply(
                &self,
                geometry: &G,
                settings: BufferSettings,
                coordinate_strategy: &$strategy,
            ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
                projected_linestring_apply(geometry, settings, coordinate_strategy)
            }
        }

        impl<G> BufferStrategy<G, $strategy> for SegmentBuffer
        where
            G: SegmentTrait,
            G::Point: PointMut + Default + Copy,
            <G::Point as Point>::Scalar:
                CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
            <G::Point as Point>::Cs: AngularCoordinateSystem,
            <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<$family>,
        {
            fn apply(
                &self,
                geometry: &G,
                settings: BufferSettings,
                coordinate_strategy: &$strategy,
            ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
                let line = Linestring::from_vec(alloc::vec![
                    segment_start(geometry),
                    segment_end(geometry),
                ]);
                projected_linestring_apply(&line, settings, coordinate_strategy)
            }
        }

        impl<G> BufferStrategy<G, $strategy> for RingBuffer
        where
            G: RingTrait,
            G::Point: PointMut + Default + Copy,
            <G::Point as Point>::Scalar:
                CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
            <G::Point as Point>::Cs: AngularCoordinateSystem,
            <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<$family>,
        {
            fn apply(
                &self,
                geometry: &G,
                settings: BufferSettings,
                coordinate_strategy: &$strategy,
            ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
                projected_ring_apply(geometry, settings, coordinate_strategy)
            }
        }

        impl<G> BufferStrategy<G, $strategy> for PolygonBuffer
        where
            G: PolygonTrait,
            G::Point: PointMut + Default + Copy,
            <G::Point as Point>::Scalar:
                CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
            <G::Point as Point>::Cs: AngularCoordinateSystem,
            <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<$family>,
        {
            fn apply(
                &self,
                geometry: &G,
                settings: BufferSettings,
                coordinate_strategy: &$strategy,
            ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
                projected_polygon_apply(geometry, settings, coordinate_strategy)
            }
        }

        impl<G> BufferStrategy<G, $strategy> for BoxBuffer
        where
            G: BoxTrait,
            G::Point: PointMut + Default + Copy,
            <G::Point as Point>::Scalar:
                CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
            <G::Point as Point>::Cs: AngularCoordinateSystem,
            <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<$family>,
        {
            fn apply(
                &self,
                geometry: &G,
                settings: BufferSettings,
                coordinate_strategy: &$strategy,
            ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
                let minimum = box_min(geometry);
                let maximum = box_max(geometry);
                let projection = projection_for_points([&minimum, &maximum], coordinate_strategy)?;
                let projected = ModelBox::from_corners(
                    projected_point(&minimum, projection),
                    projected_point(&maximum, projection),
                );
                let output = BoxBuffer.apply(&projected, settings, &CartesianBuffer)?;
                Ok(unprojected_buffer(output, projection))
            }
        }

        impl<G> BufferStrategy<G, $strategy> for MultiPointBuffer
        where
            G: MultiPointTrait<ItemPoint = <G as Geometry>::Point>,
            G::Point: PointMut + Default + Copy,
            <G::Point as Point>::Scalar:
                CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
            <G::Point as Point>::Cs: AngularCoordinateSystem,
            <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<$family>,
        {
            fn apply(
                &self,
                geometry: &G,
                settings: BufferSettings,
                coordinate_strategy: &$strategy,
            ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
                let projection = projection_for_points(geometry.points(), coordinate_strategy)?;
                let projected = MultiPoint::from_vec(
                    geometry
                        .points()
                        .map(|point| projected_point(point, projection))
                        .collect(),
                );
                let output = MultiPointBuffer.apply(&projected, settings, &CartesianBuffer)?;
                Ok(unprojected_buffer(output, projection))
            }
        }

        impl<G> BufferStrategy<G, $strategy> for MultiLinestringBuffer
        where
            G: MultiLinestringTrait,
            G::Point: PointMut + Default + Copy,
            <G::Point as Point>::Scalar:
                CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
            <G::Point as Point>::Cs: AngularCoordinateSystem,
            <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<$family>,
        {
            fn apply(
                &self,
                geometry: &G,
                settings: BufferSettings,
                coordinate_strategy: &$strategy,
            ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
                let coordinates = geometry
                    .linestrings()
                    .flat_map(|line| line.points().map(angular_coordinates))
                    .collect::<Vec<_>>();
                let (longitude, latitude) = projection_center(&coordinates)?;
                let projection = coordinate_strategy.projection(longitude, latitude)?;
                let projected = MultiLinestring::from_vec(
                    geometry
                        .linestrings()
                        .map(|line| {
                            Linestring::from_vec(
                                line.points()
                                    .map(|point| projected_point(point, projection))
                                    .collect(),
                            )
                        })
                        .collect(),
                );
                let output = MultiLinestringBuffer.apply(&projected, settings, &CartesianBuffer)?;
                Ok(unprojected_buffer(output, projection))
            }
        }

        impl<G> BufferStrategy<G, $strategy> for MultiPolygonBuffer
        where
            G: MultiPolygonTrait,
            G::Point: PointMut + Default + Copy,
            <G::Point as Point>::Scalar:
                CoordinateScalar<Measure = <G::Point as Point>::Scalar> + Into<f64> + FromF64,
            <G::Point as Point>::Cs: AngularCoordinateSystem,
            <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<$family>,
        {
            fn apply(
                &self,
                geometry: &G,
                settings: BufferSettings,
                coordinate_strategy: &$strategy,
            ) -> Result<MultiPolygon<Polygon<G::Point>>, OverlayError> {
                let coordinates = geometry
                    .polygons()
                    .flat_map(|polygon| {
                        polygon
                            .exterior()
                            .points()
                            .chain(polygon.interiors().flat_map(RingTrait::points))
                            .map(angular_coordinates)
                    })
                    .collect::<Vec<_>>();
                let (longitude, latitude) = projection_center(&coordinates)?;
                let projection = coordinate_strategy.projection(longitude, latitude)?;
                let projected = MultiPolygon::from_vec(
                    geometry
                        .polygons()
                        .map(|polygon| projected_polygon(polygon, projection))
                        .collect(),
                );
                let output = MultiPolygonBuffer.apply(&projected, settings, &CartesianBuffer)?;
                Ok(unprojected_buffer(output, projection))
            }
        }
    };
}

impl_angular_buffer_strategy!(SphericalBuffer, SphericalFamily);
impl_angular_buffer_strategy!(GeographicBuffer, GeographicFamily);

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
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64> + FromF64,
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
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64> + FromF64,
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

    let dot = |one: (f64, f64), two: (f64, f64)| one.0 * two.0 + one.1 * two.1;
    let count = vertices.len();
    let mut boundary = Vec::new();
    // Where the outline leaves the incoming side's offset and joins the
    // outgoing side's, corner by corner.
    let mut corners = Vec::with_capacity(count);
    let mut held = true;
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
            let point = intersection.unwrap_or(after);
            // C++: a concave corner adds no piece; the sides' pieces close it
            // only while each reaches past the end edge of the other.
            held &= dot((before.0 - vertex.0, before.1 - vertex.1), outgoing)
                <= dot(outgoing, outgoing)
                && dot(
                    (after.0 - vertex.0, after.1 - vertex.1),
                    (-incoming.0, -incoming.1),
                ) <= dot(incoming, incoming);
            corners.push((point, point));
            boundary.push(point);
            continue;
        }

        corners.push((before, after));
        push_join_points(
            join,
            vertex,
            before,
            after,
            intersection,
            distance,
            distance > 0.0,
            &mut boundary,
        );
    }
    // Each side's offset runs from where the outline joins it to where it
    // leaves it; a concave cut past a short side runs it backwards, over
    // ground the pieces do not bound.
    held &= (0..count).all(|index| {
        let next = (index + 1) % count;
        let side = (
            vertices[next].0 - vertices[index].0,
            vertices[next].1 - vertices[index].1,
        );
        let run = (
            corners[next].0.0 - corners[index].1.0,
            corners[next].0.1 - corners[index].1.1,
        );
        dot(run, side) >= 0.0
    });
    if !held {
        return None;
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

/// The points the join strategy contributes at a corner the offset turns
/// away from: from `before`, the end of the incoming side's offset, to
/// `after`, the start of the outgoing side's. `intersection` is where the two
/// offset lines meet, the miter point.
///
/// C++: `join_round::apply` and `join_miter::apply`, whose output range the
/// caller appends. Shared by the offsetted ring and the join piece the
/// dissolve builds for the same corner, so the two describe one offset.
/// `counterclockwise` is the way round the corner the offset turns: a ring
/// walked counter-clockwise rounds an outward offset's convex corner
/// counter-clockwise and an inward offset's reflex corner the other way,
/// and forcing one direction would sweep the long way through the material
/// at the other.
#[allow(
    clippy::too_many_arguments,
    reason = "the corner, its two offset ends, the miter point, the distance and the turn are what `join_strategy.apply` receives"
)]
fn push_join_points(
    join: BufferJoinStrategy,
    vertex: (f64, f64),
    before: (f64, f64),
    after: (f64, f64),
    intersection: Option<(f64, f64)>,
    distance: f64,
    counterclockwise: bool,
    boundary: &mut Vec<(f64, f64)>,
) {
    // C++: a join that cannot be made — its two ends one point, or no
    // miter point — adds nothing, and the outline runs on from `before`.
    let same =
        |one: (f64, f64), two: (f64, f64)| one.0.tolerant_eq(two.0) && one.1.tolerant_eq(two.1);
    boundary.push(before);
    if same(before, after) {
        return;
    }
    match join {
        BufferJoinStrategy::Round { points_per_circle } => {
            // C++: `join_round` sweeps clockwise from the end of the
            // incoming offset, its walk keeping the offset on its left; a
            // walk the other way round meets the same corner from the far
            // end of the arc.
            let (from, to) = if counterclockwise {
                (after, before)
            } else {
                (before, after)
            };
            let mut arc = round_join_points(vertex, from, to, distance.abs(), points_per_circle);
            if counterclockwise {
                arc.reverse();
            }
            boundary.extend(arc);
        }
        BufferJoinStrategy::Miter { limit } => {
            let Some(point) = intersection.filter(|point| {
                point.0.is_finite() && point.1.is_finite() && !same(*point, vertex)
            }) else {
                return;
            };
            // C++: a miter longer than the limit is not cut to a bevel; its
            // point is drawn back along the miter to the limit.
            let (dx, dy) = (point.0 - vertex.0, point.1 - vertex.1);
            let miter_length = sqrt(dx * dx + dy * dy);
            let max_length = limit.max(1.0) * distance.abs();
            if miter_length > max_length {
                let proportion = max_length / miter_length;
                boundary.push((vertex.0 + dx * proportion, vertex.1 + dy * proportion));
            } else {
                boundary.push(point);
            }
        }
    }
    boundary.push(after);
}

/// Whether the offsetted rings cross, so that the offset has to be rebuilt
/// from its pieces.
///
/// C++: `buffered_piece_collection` never trusts an offsetted ring — it finds
/// the turns between every piece and traverses them whatever the input. This
/// port keeps the offsetted rings wherever they are already the answer, which
/// is whenever `offset_ring` kept each one — every concave corner held and
/// every erosion kept its clearance — and no ring crosses itself or another.
/// It rebuilds the offset from the pieces only where a ring is not simple: a
/// notch narrower than twice the distance closes, a neck thinner than that
/// pinches off, a hole's arm fills in.
///
/// `outer` is the exterior's offsetted ring and `inners` the holes'.
fn offset_rings_cross<P>(outer: &Ring<P>, inners: &[Ring<P>], distance: f64) -> bool
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64>,
{
    if ring_crosses_itself(outer) || inners.iter().any(ring_crosses_itself) {
        return true;
    }
    if distance > 0.0 {
        // Growth moves the exterior outward and every hole inward, away from
        // one another; only erosion can run them into each other.
        return false;
    }
    inners.iter().enumerate().any(|(index, inner)| {
        rings_cross(outer, inner)
            || inners[index + 1..]
                .iter()
                .any(|other| rings_cross(inner, other))
    })
}

/// The sides of a closed ring as coordinate pairs, with the box of each.
fn ring_sides<P>(ring: &Ring<P>) -> Vec<(P, P, [f64; 4])>
where
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let points: Vec<P> = ring.points().copied().collect();
    points
        .windows(2)
        .map(|pair| {
            let (ax, ay): (f64, f64) = (pair[0].get::<0>().into(), pair[0].get::<1>().into());
            let (bx, by): (f64, f64) = (pair[1].get::<0>().into(), pair[1].get::<1>().into());
            (
                pair[0],
                pair[1],
                [ax.min(bx), ay.min(by), ax.max(bx), ay.max(by)],
            )
        })
        .collect()
}

/// Whether two sides meet, decided by the exact predicate behind a box test
/// that keeps the pass cheap for the long arcs a round join produces.
/// Coordinates the predicate cannot judge count as not meeting, which leaves
/// such a ring on the path it took before the dissolve existed.
fn sides_meet<P>(one: &(P, P, [f64; 4]), two: &(P, P, [f64; 4])) -> bool
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64>,
{
    let (a, b) = (&one.2, &two.2);
    if a[0] > b[2] || b[0] > a[2] || a[1] > b[3] || b[1] > a[3] {
        return false;
    }
    !matches!(
        segment_intersection::<Segment<P>, P>(
            &Segment::new(one.0, one.1),
            &Segment::new(two.0, two.1)
        ),
        SegmentIntersection::Disjoint | SegmentIntersection::OutOfRange
    )
}

/// Whether any two sides of the ring that are not neighbours meet.
fn ring_crosses_itself<P>(ring: &Ring<P>) -> bool
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64>,
{
    let sides = ring_sides(ring);
    let count = sides.len();
    (0..count).any(|first| {
        let last_neighbour = if first == 0 { count - 1 } else { count };
        (first + 2..last_neighbour).any(|second| sides_meet(&sides[first], &sides[second]))
    })
}

/// Whether any side of one ring meets any side of the other.
fn rings_cross<P>(one: &Ring<P>, two: &Ring<P>) -> bool
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64>,
{
    let one = ring_sides(one);
    let two = ring_sides(two);
    one.iter()
        .any(|side| two.iter().any(|other| sides_meet(side, other)))
}

/// The offset rebuilt from Boost's pieces, for a polygon whose offsetted
/// rings cannot be trusted.
///
/// C++: `buffer_inserter` cuts each ring into pieces — a `buffered_segment`
/// per side, offset by the distance, and a `buffered_join` at each corner the
/// join strategy rounds or miters — and `buffered_piece_collection` finds the
/// turns between them and traverses them, so that a stretch of one piece's
/// offset that ends up inside another piece never reaches the outline. The
/// pieces here are the same ones; in place of Boost's turn machinery they go
/// through the overlay engine: merged into one multi-polygon and then,
/// growing, unioned with the polygon or, eroding, taken away from it. Each
/// piece is the region within the distance of one side or one corner, so the
/// polygon with their union is exactly the grown shape and the polygon less
/// their union exactly the eroded one — a notch that closes, a neck that
/// pinches off into separate polygons, a hole that fills in part.
///
/// `polygon` is the one Boost offsets, its rings simplified: Boost traverses
/// only the pieces' offset edges, so where simplifying cut across a sliver of
/// the input, that sliver is no part of an erosion.
///
/// The work grows with the square of the result's vertex count, which is why
/// this is kept for the rings the offsetted ring gets wrong.
fn dissolve_offset<G, P>(
    polygon: &G,
    distance: f64,
    join: BufferJoinStrategy,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64> + FromF64,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let mut pieces: Vec<Polygon<P>> = Vec::new();
    push_ring_pieces(polygon.exterior(), distance, join, &mut pieces);
    for ring in polygon.interiors() {
        // A hole's outside is the polygon's inside, so it offsets the other
        // way round.
        push_ring_pieces(ring, -distance, join, &mut pieces);
    }
    let pieces = merge_pieces(pieces)?;
    if pieces.0.is_empty() {
        return Ok(MultiPolygon::new());
    }
    let original: MultiPolygon<Polygon<P>> = MultiPolygon(alloc::vec![Polygon::with_inners(
        Ring::from_vec(polygon.exterior().points().copied().collect()),
        polygon
            .interiors()
            .map(|ring| Ring::from_vec(ring.points().copied().collect()))
            .collect(),
    )]);
    if distance > 0.0 {
        union_multi(&original, &pieces)
    } else {
        difference_multi(&original, &pieces)
    }
}

/// The pieces one ring contributes, offset by `distance` along its outward
/// normals: a quadrilateral per side and, at each corner the offset turns
/// away from the ring, the join's wedge. A negative distance puts them on
/// the ring's inner side, which erodes an exterior and grows a hole.
///
/// C++: `buffer_range::iterate` — `add_side_piece` for every side and
/// `add_join` between consecutive sides, then the closing join. The corner
/// points are the ones `offset_ring` places, so the pieces and the offsetted
/// ring describe one offset.
fn push_ring_pieces<R, P>(
    ring: &R,
    distance: f64,
    join: BufferJoinStrategy,
    pieces: &mut Vec<Polygon<P>>,
) where
    R: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: Into<f64> + FromF64,
{
    let mut vertices = distinct_vertices(ring);
    vertices.dedup();
    while vertices.len() > 1 && vertices.last() == vertices.first() {
        vertices.pop();
    }
    if vertices.len() < 3 || distance == 0.0 {
        return;
    }
    if signed_area_ccw_positive(&vertices) < 0.0 {
        vertices.reverse();
    }

    let count = vertices.len();
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
        let far = (
            next.0 + outgoing_normal.0 * distance,
            next.1 + outgoing_normal.1 * distance,
        );
        push_piece(pieces, alloc::vec![vertex, next, far, after]);

        let cross = incoming.0 * outgoing.1 - incoming.1 * outgoing.0;
        if cross * distance > 0.0 {
            let intersection = line_intersection(before, incoming, after, outgoing);
            let mut wedge = alloc::vec![vertex, before];
            push_join_points(
                join,
                vertex,
                before,
                after,
                intersection,
                distance,
                distance > 0.0,
                &mut wedge,
            );
            wedge.push(after);
            push_piece(pieces, wedge);
        }
    }
}

/// One piece as a closed ring, dropped when it encloses nothing.
fn push_piece<P>(pieces: &mut Vec<Polygon<P>>, mut boundary: Vec<(f64, f64)>)
where
    P: PointMut + Default + Copy,
    P::Scalar: FromF64,
{
    boundary.dedup();
    if boundary.len() < 3 || signed_area_ccw_positive(&boundary).abs() <= f64::EPSILON {
        return;
    }
    boundary.push(boundary[0]);
    pieces.push(Polygon::new(Ring::from_vec(
        boundary
            .into_iter()
            .map(|(x, y)| make_point(x, y))
            .collect(),
    )));
}

/// The union of the pieces, merged neighbour with neighbour and then pair
/// with pair, so each union is between parts of like size and the work grows
/// with the result rather than with the number of pieces.
fn merge_pieces<P>(pieces: Vec<Polygon<P>>) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let mut merged: Vec<MultiPolygon<Polygon<P>>> = pieces
        .into_iter()
        .map(|piece| MultiPolygon(alloc::vec![piece]))
        .collect();
    while merged.len() > 1 {
        let mut next = Vec::with_capacity(merged.len().div_ceil(2));
        let mut pairs = merged.into_iter();
        while let Some(left) = pairs.next() {
            match pairs.next() {
                Some(right) => next.push(union_multi(&left, &right)?),
                None => next.push(left),
            }
        }
        merged = next;
    }
    Ok(merged.pop().unwrap_or_default())
}

/// A linestring buffered `left` and `right` of it.
///
/// C++: `buffer_inserter<linestring_tag>` — the input simplified first
/// (`simplify_input`), each side then walked as `buffer_range::iterate`
/// walks it and capped at the end it walks to, and a linestring that
/// simplifies to one point buffered as that point. Each side is the offset
/// to the left of the direction it is walked in: the linestring itself for
/// the left side, the linestring reversed for the right. The outline those
/// walks trace stands where it does not cross itself; where it does — a
/// turn sharper than its segments are long, a line doubling back or
/// crossing itself — the buffer is the union of the pieces Boost builds: a
/// side piece per segment and side, a join at each convex corner and a cap
/// at each end and each spike.
fn buffer_linestring<L, P>(
    line: &L,
    left: f64,
    right: f64,
    settings: BufferSettings,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    L: LinestringTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64> + FromF64,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let points: Vec<(f64, f64)> = line
        .points()
        .map(|point| (point.get::<0>().into(), point.get::<1>().into()))
        .collect();
    let vertices = simplified_input(&points, left.min(right) / 1000.0);
    let [.., penultimate, ultimate] = vertices.as_slice() else {
        // C++: a linestring that simplifies to one point is buffered as that
        // point, at the left distance; an empty one has no buffer.
        return Ok(match vertices.first() {
            Some(&point) if left > 0.0 => point_buffer(point, left, settings.point),
            _ => MultiPolygon::new(),
        });
    };
    let reversed: Vec<(f64, f64)> = vertices.iter().rev().copied().collect();
    let left_side = offset_side(&vertices, left, right, settings.join, settings.end);
    let right_side = offset_side(&reversed, right, left, settings.join, settings.end);
    let (left_start, left_end) = (
        left_side.outline[0],
        left_side.outline[left_side.outline.len() - 1],
    );
    let (right_start, right_end) = (
        right_side.outline[0],
        right_side.outline[right_side.outline.len() - 1],
    );
    let end = end_cap(
        *penultimate,
        *ultimate,
        left_end,
        right_start,
        left,
        right,
        settings.end,
    );
    let start = end_cap(
        vertices[1],
        vertices[0],
        right_end,
        left_start,
        right,
        left,
        settings.end,
    );

    if !left_side.folds && !right_side.folds {
        // C++: a cap's first point stands for the end of the side before it
        // and is not added, and its last is overwritten by the start of the
        // side after it (`update_last_point`, and the closing point
        // `finish_ring` makes equal to the first), so that the two do not
        // differ by a rounding error.
        let mut boundary = left_side.outline;
        boundary.extend(&end[1..end.len() - 1]);
        boundary.extend(right_side.outline);
        boundary.extend(&start[1..start.len() - 1]);
        boundary.dedup();
        let first = boundary[0];
        boundary.push(first);
        let outline: Ring<P> = Ring::from_vec(
            boundary
                .into_iter()
                .map(|(x, y)| make_point(x, y))
                .collect(),
        );
        if !ring_crosses_itself(&outline) {
            return Ok(MultiPolygon(alloc::vec![Polygon::new(outline)]));
        }
    }

    let mut pieces: Vec<Polygon<P>> = Vec::new();
    for piece in left_side.pieces.into_iter().chain(right_side.pieces) {
        push_piece(&mut pieces, piece);
    }
    // C++: a flat cap is the straight line across the end, with nothing
    // inside it; a round one is the half disc about the end.
    if let BufferEndStrategy::Round { .. } = settings.end {
        for (cap, at) in [(end, *ultimate), (start, vertices[0])] {
            let mut piece = alloc::vec![at];
            piece.extend(cap);
            push_piece(&mut pieces, piece);
        }
    }
    merge_pieces(pieces)
}

/// A point buffered by `distance` with the point strategy.
///
/// C++: `detail::buffer::buffer_point`, which a linear or areal inserter
/// falls back on for an input that simplifies to fewer points than it needs.
fn point_buffer<P>(
    (x, y): (f64, f64),
    distance: f64,
    strategy: BufferPointStrategy,
) -> MultiPolygon<Polygon<P>>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar> + Into<f64> + FromF64,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let strategy = match strategy {
        BufferPointStrategy::Circle { points_per_circle } => {
            PointStrategy::Circle { points_per_circle }
        }
        BufferPointStrategy::Square => PointStrategy::Square,
    };
    let ring = buffer_point(&make_point::<P>(x, y), distance, strategy);
    MultiPolygon(alloc::vec![Polygon::new(ring)])
}

/// A ring as Boost offsets it: simplified at a thousandth of the distance
/// and closed, or `None` when that leaves fewer points than a ring needs.
///
/// C++: `buffer_inserter_ring::apply` — `simplify_input` over the ring as
/// declared, held to `minimum_ring_size`, then walked through
/// `closed_clockwise_view`.
fn simplified_ring<R, P>(ring: &R, distance: f64) -> Option<Ring<P>>
where
    R: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: Into<f64> + FromF64,
{
    let points: Vec<(f64, f64)> = ring
        .points()
        .map(|point| (point.get::<0>().into(), point.get::<1>().into()))
        .collect();
    let mut simplified = simplified_input(&points, distance.abs() / 1000.0);
    let open = ring.closure() == Closure::Open;
    if simplified.len() < if open { 3 } else { 4 } {
        return None;
    }
    if open {
        simplified.push(simplified[0]);
    }
    Some(Ring::from_vec(
        simplified
            .into_iter()
            .map(|(x, y)| make_point(x, y))
            .collect(),
    ))
}

/// The vertices of a linestring or ring as Boost buffers them.
///
/// C++: `simplify_input` — Douglas–Peucker at `tolerance`, a thousandth of
/// the smaller buffer distance, which also drops repeated points, so that
/// a feature too small to see in the buffer cannot blow up into one.
fn simplified_input(points: &[(f64, f64)], tolerance: f64) -> Vec<(f64, f64)> {
    let line: Linestring<Point2D<f64, Cartesian>> =
        Linestring::from_vec(points.iter().map(|&(x, y)| Point2D::new(x, y)).collect());
    DouglasPeucker::<PointToSegment<Pythagoras>>::default()
        .simplify(&line, tolerance)
        .points()
        .map(|point| (point.get::<0>(), point.get::<1>()))
        .collect()
}

/// One side of a linestring's buffer, offset to the left of the walk.
struct OffsetSide {
    /// The side's outline from the start of its first segment's offset to
    /// the end of its last.
    outline: Vec<(f64, f64)>,
    /// The pieces Boost builds for the side: one per segment, convex
    /// corner and spike.
    pieces: Vec<Vec<(f64, f64)>>,
    /// Whether the outline folds back over itself: at a spike, or at a
    /// concave corner whose offsets do not cross.
    folds: bool,
}

/// The offset to the left of `walk` by `own`, `other` being the distance
/// on the far side.
///
/// C++: `buffer_range::iterate` with `side_straight`, and `add_join` at each
/// corner, which reads the turn with Boost's side test, `side_by_triangle`:
/// a corner the walk turns right at is convex and takes the join strategy,
/// one it turns left at is concave and is cut where the two offsets cross,
/// one it runs straight on through needs nothing, and one it doubles back
/// at is a spike and takes the end strategy.
fn offset_side(
    walk: &[(f64, f64)],
    own: f64,
    other: f64,
    join: BufferJoinStrategy,
    end: BufferEndStrategy,
) -> OffsetSide {
    let mut outline: Vec<(f64, f64)> = Vec::with_capacity(walk.len() * 2);
    let mut pieces = Vec::with_capacity(walk.len());
    let mut folds = false;
    let mut previous_offset = ((0.0, 0.0), (0.0, 0.0));
    for (index, segment) in walk.windows(2).enumerate() {
        let (start, end_point) = (segment[0], segment[1]);
        let normal = left_normal(start, end_point);
        let after = offset_point(start, normal, own);
        let far = offset_point(end_point, normal, own);
        pieces.push(alloc::vec![start, end_point, far, after]);
        if index == 0 {
            outline.push(after);
        } else {
            let previous = walk[index - 1];
            let before = previous_offset.1;
            let incoming = (start.0 - previous.0, start.1 - previous.1);
            let outgoing = (end_point.0 - start.0, end_point.1 - start.1);
            match f64::side_by_triangle(previous, start, end_point) {
                core::cmp::Ordering::Less => {
                    if let Some(miter) = miter_point(previous_offset, (after, far), start) {
                        let mut corner = Vec::new();
                        push_join_points(
                            join,
                            start,
                            before,
                            after,
                            Some(miter),
                            own,
                            false,
                            &mut corner,
                        );
                        outline.extend(corner.iter().skip(1));
                        let mut piece = alloc::vec![start];
                        piece.extend(corner);
                        pieces.push(piece);
                    }
                }
                core::cmp::Ordering::Greater => {
                    // C++: the two side pieces overlap at a concave corner,
                    // and the traversal cuts both offsets where they cross.
                    // That hands the corner of each piece past the crossing
                    // to the other segment's two pieces, which hold it only
                    // where it lies within that segment's reach: a segment
                    // shorter than the corner leaves it sticking out.
                    let dot = |one: (f64, f64), two: (f64, f64)| one.0 * two.0 + one.1 * two.1;
                    let back = (-incoming.0, -incoming.1);
                    let held = dot((before.0 - start.0, before.1 - start.1), outgoing)
                        <= dot(outgoing, outgoing)
                        && dot((after.0 - start.0, after.1 - start.1), back) <= dot(back, back)
                        && own * dot(left_normal(previous, start), normal) >= -other;
                    let kept = outline[outline.len() - 2];
                    if let Some(crossing) =
                        segment_crossing((kept, before), (after, far)).filter(|_| held)
                    {
                        outline.pop();
                        outline.push(crossing);
                    } else {
                        folds = true;
                        outline.extend([start, after]);
                    }
                }
                core::cmp::Ordering::Equal => {
                    if incoming.0 * outgoing.0 + incoming.1 * outgoing.1 <= 0.0 {
                        folds = true;
                        let cap = end_cap(previous, start, before, after, own, other, end);
                        outline.extend(cap.iter().skip(1));
                        if let BufferEndStrategy::Round { .. } = end {
                            let mut piece = alloc::vec![start];
                            piece.extend(cap);
                            pieces.push(piece);
                        }
                    }
                }
            }
        }
        outline.push(far);
        previous_offset = (after, far);
    }
    OffsetSide {
        outline,
        pieces,
        folds,
    }
}

/// The unit normal to the left of `start → end`.
///
/// C++: the perpendicular `side_straight::apply` builds, `(−dy, dx)` over
/// the segment's length.
fn left_normal(start: (f64, f64), end: (f64, f64)) -> (f64, f64) {
    let (dx, dy) = (end.0 - start.0, end.1 - start.1);
    let length = sqrt(dx * dx + dy * dy);
    (-dy / length, dx / length)
}

/// `point` moved `distance` along `normal`.
fn offset_point(point: (f64, f64), normal: (f64, f64), distance: f64) -> (f64, f64) {
    (point.0 + normal.0 * distance, point.1 + normal.1 * distance)
}

/// Where the offsets meeting at a convex corner meet: the miter point.
///
/// C++: `line_line_intersection::apply` for an equidistant side strategy —
/// the line through the incoming offset met with the line through the
/// outgoing one or, where that is the worse conditioned of the two, with the
/// line from the corner through the midpoint between the offsets' ends; no
/// point where both are parallel.
fn miter_point(
    incoming: ((f64, f64), (f64, f64)),
    outgoing: ((f64, f64), (f64, f64)),
    vertex: (f64, f64),
) -> Option<(f64, f64)> {
    // C++: `make_infinite_line`, the line `a·x + b·y + c = 0`.
    let line = |from: (f64, f64), to: (f64, f64)| {
        let a = from.1 - to.1;
        let b = to.0 - from.0;
        (a, b, -a * from.0 - b * from.1)
    };
    let p = line(incoming.0, incoming.1);
    let q = line(outgoing.0, outgoing.1);
    let between = (
        f64::midpoint(incoming.1.0, outgoing.0.0),
        f64::midpoint(incoming.1.1, outgoing.0.1),
    );
    let r = line(vertex, between);
    // C++: `denominator_pq` and `denominator_pr`.
    let with_outgoing = p.0 * q.1 - p.1 * q.0;
    let with_between = p.0 * r.1 - p.1 * r.0;
    if with_outgoing.tolerant_eq(0.0) && with_between.tolerant_eq(0.0) {
        return None;
    }
    let (other, denominator) = if with_outgoing.abs() > with_between.abs() {
        (q, with_outgoing)
    } else {
        (r, with_between)
    };
    Some((
        (p.1 * other.2 - p.2 * other.1) / denominator,
        (p.2 * other.0 - p.0 * other.2) / denominator,
    ))
}

/// Where the segment `one` crosses the segment `two`, if it does.
fn segment_crossing(
    one: ((f64, f64), (f64, f64)),
    two: ((f64, f64), (f64, f64)),
) -> Option<(f64, f64)> {
    let first = (one.1.0 - one.0.0, one.1.1 - one.0.1);
    let second = (two.1.0 - two.0.0, two.1.1 - two.0.1);
    let denominator = first.0 * second.1 - first.1 * second.0;
    if denominator == 0.0 {
        return None;
    }
    let delta = (two.0.0 - one.0.0, two.0.1 - one.0.1);
    let along_one = (delta.0 * second.1 - delta.1 * second.0) / denominator;
    let along_two = (delta.0 * first.1 - delta.1 * first.0) / denominator;
    ((0.0..=1.0).contains(&along_one) && (0.0..=1.0).contains(&along_two))
        .then_some((one.0.0 + along_one * first.0, one.0.1 + along_one * first.1))
}

/// The cap at `ultimate`, the end of the segment from `penultimate`: from
/// `own_perp` on the walking side, `own` from the line, round to
/// `other_perp` on the far side, `other` from it.
///
/// C++: `end_round::apply` — half a circle of `points_per_circle` (at least
/// four) points from the walking side clockwise, centred between the two
/// sides, with the far side's point added when the count is odd — and
/// `end_flat::apply`, the two sides' points.
fn end_cap(
    penultimate: (f64, f64),
    ultimate: (f64, f64),
    own_perp: (f64, f64),
    other_perp: (f64, f64),
    own: f64,
    other: f64,
    end: BufferEndStrategy,
) -> Vec<(f64, f64)> {
    let BufferEndStrategy::Round { points_per_circle } = end else {
        return alloc::vec![own_perp, other_perp];
    };
    let count = points_per_circle.max(4);
    let mut alpha = atan2(penultimate.1 - ultimate.1, penultimate.0 - ultimate.0)
        - core::f64::consts::FRAC_PI_2;
    let (center, radius) = if own.tolerant_eq(other) {
        (ultimate, own)
    } else {
        let half = (own - other) / 2.0;
        (
            (
                ultimate.0 + half * cos(alpha),
                ultimate.1 + half * sin(alpha),
            ),
            f64::midpoint(own, other),
        )
    };
    let diff = core::f64::consts::TAU / count as f64;
    let mut cap = Vec::with_capacity(count / 2 + 2);
    for _ in 0..=count / 2 {
        cap.push((
            center.0 + radius * cos(alpha),
            center.1 + radius * sin(alpha),
        ));
        alpha -= diff;
    }
    if count % 2 == 1 {
        cap.push(other_perp);
    }
    cap
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

/// The points strictly between `from` and `to` on the circle of `radius`
/// about `vertex`, clockwise from `from`.
///
/// C++: `join_round::generate_points` — the sweep cut into as many equal
/// steps as `points_per_circle` (at least four) puts in that much of a
/// circle, rounded up.
fn round_join_points(
    vertex: (f64, f64),
    from: (f64, f64),
    to: (f64, f64),
    radius: f64,
    points_per_circle: usize,
) -> Vec<(f64, f64)> {
    let two_pi = core::f64::consts::TAU;
    let angle1 = atan2(from.1 - vertex.1, from.0 - vertex.0);
    let mut angle2 = atan2(to.1 - vertex.1, to.0 - vertex.0);
    while angle2 > angle1 {
        angle2 -= two_pi;
    }
    let angle_diff = angle1 - angle2;
    let count = (ceil(points_per_circle.max(4) as f64 * angle_diff / two_pi) as usize).max(1);
    let diff = angle_diff / count as f64;
    let mut angle = angle1 - diff;
    let mut points = Vec::with_capacity(count - 1);
    for _ in 1..count {
        points.push((
            vertex.0 + radius * cos(angle),
            vertex.1 + radius * sin(angle),
        ));
        angle -= diff;
    }
    points
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

    use super::{
        BufferJoinStrategy, dissolve_offset, offset_rings_cross, push_piece, push_ring_pieces,
    };
    use super::{JoinStrategy, PointStrategy, buffer, buffer_convex_polygon, buffer_point};
    use alloc::vec::Vec;
    use geometry_algorithm::ring_area;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, Ring, polygon};
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

    // ---- The pieces path, at the edges the shaped inputs never reach ----
    //
    // `dissolve_offset` is only entered for a ring the offsetted ring gets
    // wrong, and the area assertions above all go through the simple path.
    // These drive the pieces builder and its gatekeeper directly, because
    // the degenerate inputs they guard against cannot be produced by a
    // well-shaped polygon — which is exactly why a missing guard here would
    // surface as a malformed ring far downstream in the overlay engine.

    const JOIN: BufferJoinStrategy = BufferJoinStrategy::Miter { limit: 5.0 };

    fn ring(points: &[(f64, f64)]) -> Ring<P> {
        Ring::from_vec(points.iter().map(|&(x, y)| P::new(x, y)).collect())
    }

    fn unit_square() -> Ring<P> {
        ring(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)])
    }

    /// A ring below three distinct vertices encloses nothing, and a zero
    /// distance moves nothing. Either way the ring must contribute no
    /// piece rather than a zero-area sliver for the engine to dissolve.
    #[test]
    fn a_ring_that_encloses_nothing_contributes_no_piece() {
        let mut pieces: Vec<Polygon<P>> = Vec::new();

        push_ring_pieces(
            &ring(&[(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)]),
            1.0,
            JOIN,
            &mut pieces,
        );
        assert!(pieces.is_empty(), "a two-vertex ring produced a piece");

        push_ring_pieces(&ring(&[(0.0, 0.0)]), 1.0, JOIN, &mut pieces);
        assert!(pieces.is_empty(), "a single vertex produced a piece");

        push_ring_pieces(&unit_square(), 0.0, JOIN, &mut pieces);
        assert!(pieces.is_empty(), "a zero distance produced a piece");
    }

    /// A ring may close on a repeated vertex more than once. Only one
    /// repeat is dropped as the closure, so the rest have to be trimmed —
    /// otherwise the last side is zero-length and its piece degenerate.
    /// The trimmed ring must give exactly the pieces the clean one does.
    #[test]
    fn repeated_closing_vertices_are_trimmed_before_the_sides_are_cut() {
        let mut clean: Vec<Polygon<P>> = Vec::new();
        push_ring_pieces(&unit_square(), 1.0, JOIN, &mut clean);

        let mut repeated: Vec<Polygon<P>> = Vec::new();
        push_ring_pieces(
            &ring(&[
                (0.0, 0.0),
                (1.0, 0.0),
                (1.0, 1.0),
                (0.0, 1.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ]),
            1.0,
            JOIN,
            &mut repeated,
        );

        assert!(!clean.is_empty(), "the square should cut into pieces");
        assert_eq!(clean, repeated);
    }

    /// A piece is kept only if it encloses area. A boundary of fewer than
    /// three points, or one whose points are collinear, bounds nothing and
    /// must be dropped at the source.
    #[test]
    fn a_piece_bounding_no_area_is_dropped() {
        let mut pieces: Vec<Polygon<P>> = Vec::new();

        push_piece::<P>(&mut pieces, alloc::vec![(0.0, 0.0), (1.0, 0.0)]);
        assert!(pieces.is_empty(), "a two-point boundary was kept");

        push_piece::<P>(&mut pieces, alloc::vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]);
        assert!(pieces.is_empty(), "a collinear boundary was kept");

        // A boundary that does enclose area is kept and closed.
        push_piece::<P>(&mut pieces, alloc::vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]);
        assert_eq!(pieces.len(), 1);
        assert_eq!(pieces[0].exterior().0.len(), 4);
    }

    /// A hole whose own offsetted ring crosses itself is not a usable
    /// answer whichever way the buffer runs, so it goes to the pieces —
    /// the hole-side counterpart of the exterior's self-crossing check.
    #[test]
    fn a_self_crossing_hole_forces_the_pieces_path() {
        let outer = unit_square();
        // A bow tie: the two diagonals cross.
        let bow_tie = ring(&[(0.0, 0.0), (1.0, 1.0), (1.0, 0.0), (0.0, 1.0), (0.0, 0.0)]);
        assert!(offset_rings_cross(
            &outer,
            core::slice::from_ref(&bow_tie),
            -0.1
        ));
        assert!(offset_rings_cross(&outer, &[bow_tie], 0.1));
    }

    /// When every ring of the polygon is degenerate there are no pieces to
    /// merge, and the dissolve must answer with an empty multi-polygon
    /// rather than unioning the original back in — a zero-distance buffer
    /// of a square is the square, but its *pieces* are nothing.
    #[test]
    fn a_dissolve_with_no_pieces_is_empty() {
        let square: Polygon<P> =
            polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let dissolved = dissolve_offset(&square, 0.0, JOIN).expect("no pieces is not an error");
        assert_eq!(dissolved.0.len(), 0);
    }
}
