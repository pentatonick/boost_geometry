//! OVL6.T1 / OVL6.T2 — the DE-9IM relate matrix and the spatial
//! predicates derived from it.
//!
//! Mirrors `boost/geometry/algorithms/relate.hpp`,
//! `algorithms/relation.hpp`, and `detail/relate/`. The DE-9IM matrix
//! records, for each pair drawn from {Interior, Boundary, Exterior} of
//! two geometries, the **dimension** of their intersection. The
//! `crosses` / `overlaps` / `touches` predicates
//! (`algorithms/{crosses,overlaps,touches}.hpp`) are then thin tests on
//! that matrix.
//!
//! Cartesian points, segments, linestrings, rings, boxes, polygons,
//! homogeneous multis, runtime geometries, and heterogeneous geometry
//! collections are dispatched through the same matrix interface. Union
//! topology follows OGC boundary rules, including the mod-2 boundary of a
//! multilinestring and absorption of members covered by areal interiors. The
//! public mask-string interface consumes the completed matrix.

#![allow(
    clippy::float_cmp,
    reason = "exact equality identifies stored endpoint identity for DE-9IM boundary classification"
)]

use alloc::vec::Vec;

use geometry_coords::{CoordinateScalar, precise_math};
use geometry_cs::{Cartesian, CartesianFamily, CoordinateSystem};
use geometry_model::{DynGeometry, Point2D, Polygon, Ring};
use geometry_tag::{
    BoxTag, DynamicGeometryTag, GeometryCollectionTag, LinestringTag, MultiLinestringTag,
    MultiPointTag, MultiPolygonTag, PointTag, PolygonTag, RingTag, SameAs, SegmentTag,
};
use geometry_trait::{
    Box as BoxTrait, Geometry, GeometryCollection, Linestring as LinestringTrait, MultiLinestring,
    MultiPoint, MultiPolygon, Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait,
    Segment as SegmentTrait, corner,
};

use crate::operation::OverlayError;
use crate::predicate::range_guard::{SAFE_ABS_MAX, polygon_in_range};

/// The dimension of an intersection cell in a [`De9im`] matrix.
///
/// Mirrors the per-cell value of Boost's relate matrix: `F` (empty) or
/// a dimension digit `0` / `1` / `2` (`detail/relate/result.hpp`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dimension {
    /// Empty intersection — Boost's `F`.
    Empty,
    /// Point (0-dimensional) intersection — Boost's `0`.
    Point,
    /// Curve (1-dimensional) intersection — Boost's `1`.
    Curve,
    /// Area (2-dimensional) intersection — Boost's `2`.
    Area,
}

impl Dimension {
    /// Whether the cell is non-empty (Boost's `T` — "true, any
    /// dimension").
    #[must_use]
    pub fn is_set(self) -> bool {
        self != Dimension::Empty
    }
}

/// A DE-9IM 3×3 intersection matrix between two geometries.
///
/// Rows are the first geometry's {Interior, Boundary, Exterior}, columns
/// the second's. `m[r][c]` is the dimension of the intersection of the
/// first's feature `r` with the second's feature `c`. Mirrors Boost's
/// `relate::matrix` (`detail/relate/result.hpp`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct De9im {
    /// `[row][col]` over `[Interior, Boundary, Exterior]`.
    pub m: [[Dimension; 3]; 3],
}

/// Row / column indices into a [`De9im`] matrix.
pub mod feature {
    /// Interior row/column index.
    pub const INTERIOR: usize = 0;
    /// Boundary row/column index.
    pub const BOUNDARY: usize = 1;
    /// Exterior row/column index.
    pub const EXTERIOR: usize = 2;
}

impl De9im {
    /// Swap the two related geometries by transposing the matrix.
    #[must_use]
    pub fn transposed(self) -> Self {
        let mut result = [[Dimension::Empty; 3]; 3];
        for (row, cells) in self.m.iter().enumerate() {
            for (column, dimension) in cells.iter().enumerate() {
                result[column][row] = *dimension;
            }
        }
        Self { m: result }
    }

    /// The `[Interior][Interior]` cell — do the two interiors meet, and
    /// in what dimension.
    #[must_use]
    pub fn interior_interior(&self) -> Dimension {
        self.m[feature::INTERIOR][feature::INTERIOR]
    }

    /// The `[Boundary][Boundary]` cell.
    #[must_use]
    pub fn boundary_boundary(&self) -> Dimension {
        self.m[feature::BOUNDARY][feature::BOUNDARY]
    }

    /// The `[Interior][Exterior]` cell — is part of the first's interior
    /// outside the second.
    #[must_use]
    pub fn interior_exterior(&self) -> Dimension {
        self.m[feature::INTERIOR][feature::EXTERIOR]
    }

    /// The `[Exterior][Interior]` cell.
    #[must_use]
    pub fn exterior_interior(&self) -> Dimension {
        self.m[feature::EXTERIOR][feature::INTERIOR]
    }

    /// Test this matrix against a nine-character DE-9IM mask.
    ///
    /// Mirrors Boost's `de9im::mask` matching from
    /// `algorithms/detail/relate/result.hpp:425-505`: `*` accepts any cell, `T`
    /// accepts any non-empty cell, `F` accepts an empty cell, and `0`/`1`/`2`
    /// require an exact point/curve/area dimension.
    ///
    /// # Errors
    ///
    /// Returns [`RelateError::InvalidMask`] unless `mask` contains exactly
    /// nine valid ASCII mask characters.
    pub fn matches(&self, mask: &str) -> Result<bool, RelateError> {
        let bytes = mask.as_bytes();
        if bytes.len() != 9 {
            return Err(RelateError::InvalidMask);
        }

        let mut result = true;
        for (dimension, expected) in self.m.iter().flatten().zip(bytes) {
            result &= match expected {
                b'*' => true,
                b'T' => dimension.is_set(),
                b'F' => *dimension == Dimension::Empty,
                b'0' => *dimension == Dimension::Point,
                b'1' => *dimension == Dimension::Curve,
                b'2' => *dimension == Dimension::Area,
                _ => return Err(RelateError::InvalidMask),
            };
        }
        Ok(result)
    }
}

/// Failure while evaluating a DE-9IM relate mask.
///
/// Rust error adaptation for `boost::geometry::relate(g1, g2, mask)` from
/// `algorithms/detail/relate/interface.hpp:347-382`; Boost reports a boolean,
/// while this port preserves unsupported-kernel and malformed-mask failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RelateError {
    /// The relation matrix could not be computed for the supplied geometry
    /// pair.
    Overlay(OverlayError),
    /// The mask was not exactly nine characters drawn from `*TF012`.
    InvalidMask,
}

/// Converts the overlay failure into the Rust relate-error adaptation for
/// `algorithms/detail/relate/interface.hpp:347-382`.
impl From<OverlayError> for RelateError {
    fn from(error: OverlayError) -> Self {
        Self::Overlay(error)
    }
}

/// Per-pair DE-9IM relation strategy.
///
/// Mirrors Boost's tag-dispatched implementations under
/// `algorithms/detail/relate/`.
#[doc(hidden)]
pub trait RelateStrategy<A, B> {
    /// Compute the relation matrix for the ordered pair.
    fn relate(&self, first: &A, second: &B) -> Result<De9im, OverlayError>;
}

/// Select a relation strategy from an ordered geometry-tag pair.
#[doc(hidden)]
pub trait RelatePairStrategy<Other> {
    /// Strategy implementing this ordered pair.
    type Strategy: Default;
}

#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelatePointPoint;
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelatePointLinestring;
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelatePointPolygon;
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelateLinestringPoint;
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelateLinestringLinestring;
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelateLinestringPolygon;
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelatePolygonPoint;
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelatePolygonLinestring;
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelatePolygonPolygon;
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RelateTopology;

impl RelatePairStrategy<PointTag> for PointTag {
    type Strategy = RelatePointPoint;
}
impl RelatePairStrategy<LinestringTag> for PointTag {
    type Strategy = RelatePointLinestring;
}
impl RelatePairStrategy<PolygonTag> for PointTag {
    type Strategy = RelatePointPolygon;
}
impl RelatePairStrategy<PointTag> for LinestringTag {
    type Strategy = RelateLinestringPoint;
}
impl RelatePairStrategy<LinestringTag> for LinestringTag {
    type Strategy = RelateLinestringLinestring;
}
impl RelatePairStrategy<PolygonTag> for LinestringTag {
    type Strategy = RelateLinestringPolygon;
}
impl RelatePairStrategy<PointTag> for PolygonTag {
    type Strategy = RelatePolygonPoint;
}
impl RelatePairStrategy<LinestringTag> for PolygonTag {
    type Strategy = RelatePolygonLinestring;
}
impl RelatePairStrategy<PolygonTag> for PolygonTag {
    type Strategy = RelatePolygonPolygon;
}

trait TopologyKind {}

impl TopologyKind for PointTag {}
impl TopologyKind for LinestringTag {}
impl TopologyKind for PolygonTag {}
impl TopologyKind for SegmentTag {}
impl TopologyKind for RingTag {}
impl TopologyKind for BoxTag {}
impl TopologyKind for MultiPointTag {}
impl TopologyKind for MultiLinestringTag {}
impl TopologyKind for MultiPolygonTag {}
impl TopologyKind for DynamicGeometryTag {}
impl TopologyKind for GeometryCollectionTag {}

macro_rules! topology_pair_for_single {
    ($single:ty, $($other:ty),+ $(,)?) => {
        $(
            impl RelatePairStrategy<$other> for $single {
                type Strategy = RelateTopology;
            }
        )+
    };
}

topology_pair_for_single!(
    PointTag,
    SegmentTag,
    RingTag,
    BoxTag,
    MultiPointTag,
    MultiLinestringTag,
    MultiPolygonTag,
    DynamicGeometryTag,
    GeometryCollectionTag,
);
topology_pair_for_single!(
    LinestringTag,
    SegmentTag,
    RingTag,
    BoxTag,
    MultiPointTag,
    MultiLinestringTag,
    MultiPolygonTag,
    DynamicGeometryTag,
    GeometryCollectionTag,
);
topology_pair_for_single!(
    PolygonTag,
    SegmentTag,
    RingTag,
    BoxTag,
    MultiPointTag,
    MultiLinestringTag,
    MultiPolygonTag,
    DynamicGeometryTag,
    GeometryCollectionTag,
);

impl<Other: TopologyKind> RelatePairStrategy<Other> for SegmentTag {
    type Strategy = RelateTopology;
}
impl<Other: TopologyKind> RelatePairStrategy<Other> for RingTag {
    type Strategy = RelateTopology;
}
impl<Other: TopologyKind> RelatePairStrategy<Other> for BoxTag {
    type Strategy = RelateTopology;
}
impl<Other: TopologyKind> RelatePairStrategy<Other> for MultiPointTag {
    type Strategy = RelateTopology;
}
impl<Other: TopologyKind> RelatePairStrategy<Other> for MultiLinestringTag {
    type Strategy = RelateTopology;
}
impl<Other: TopologyKind> RelatePairStrategy<Other> for MultiPolygonTag {
    type Strategy = RelateTopology;
}
impl<Other: TopologyKind> RelatePairStrategy<Other> for DynamicGeometryTag {
    type Strategy = RelateTopology;
}
impl<Other: TopologyKind> RelatePairStrategy<Other> for GeometryCollectionTag {
    type Strategy = RelateTopology;
}

type PairStrategy<A, B> =
    <<A as Geometry>::Kind as RelatePairStrategy<<B as Geometry>::Kind>>::Strategy;

/// Compute the DE-9IM matrix for a supported pointlike, linear, or areal pair.
///
/// Mirrors the pair dispatch in
/// `algorithms/detail/relate/interface.hpp:275-382`, including the
/// geometry-collection path in `detail/relate/implementation_gc.hpp`.
///
/// # Errors
///
/// Returns [`OverlayError::Unsupported`] when coordinates exceed the
/// Cartesian predicate range.
#[inline]
#[must_use = "relation computation can fail and the matrix should be used"]
pub fn relate<A, B>(first: &A, second: &B) -> Result<De9im, OverlayError>
where
    A: Geometry,
    B: Geometry,
    A::Kind: RelatePairStrategy<B::Kind>,
    PairStrategy<A, B>: RelateStrategy<A, B> + Default,
{
    PairStrategy::<A, B>::default().relate(first, second)
}

impl<A, B> RelateStrategy<A, B> for RelatePointPoint
where
    A: Point,
    B: Point<Scalar = A::Scalar>,
    <A::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    <B::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn relate(&self, first: &A, second: &B) -> Result<De9im, OverlayError> {
        Ok(relate_point_point(first, second))
    }
}

impl<P, L> RelateStrategy<P, L> for RelatePointLinestring
where
    P: Point,
    L: LinestringTrait<Point = P>,
    P::Scalar: Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn relate(&self, point: &P, line: &L) -> Result<De9im, OverlayError> {
        Ok(relate_point_linestring(point, line))
    }
}

impl<P, G> RelateStrategy<P, G> for RelatePointPolygon
where
    P: Point + Copy,
    G: PolygonTrait<Point = P>,
    P::Scalar: Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn relate(&self, point: &P, polygon: &G) -> Result<De9im, OverlayError> {
        Ok(relate_point_polygon(point, polygon))
    }
}

impl<L, P> RelateStrategy<L, P> for RelateLinestringPoint
where
    P: Point,
    L: LinestringTrait<Point = P>,
    P::Scalar: Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn relate(&self, line: &L, point: &P) -> Result<De9im, OverlayError> {
        Ok(relate_point_linestring(point, line).transposed())
    }
}

impl<A, B, P> RelateStrategy<A, B> for RelateLinestringLinestring
where
    A: LinestringTrait<Point = P>,
    B: LinestringTrait<Point = P>,
    P: Point,
    P::Scalar: Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn relate(&self, first: &A, second: &B) -> Result<De9im, OverlayError> {
        Ok(relate_linestring_linestring(first, second))
    }
}

impl<L, G, P> RelateStrategy<L, G> for RelateLinestringPolygon
where
    L: LinestringTrait<Point = P>,
    G: PolygonTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn relate(&self, line: &L, polygon: &G) -> Result<De9im, OverlayError> {
        Ok(relate_linestring_polygon(line, polygon))
    }
}

impl<G, P> RelateStrategy<G, P> for RelatePolygonPoint
where
    G: PolygonTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn relate(&self, polygon: &G, point: &P) -> Result<De9im, OverlayError> {
        Ok(relate_point_polygon(point, polygon).transposed())
    }
}

impl<G, L, P> RelateStrategy<G, L> for RelatePolygonLinestring
where
    G: PolygonTrait<Point = P>,
    L: LinestringTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn relate(&self, polygon: &G, line: &L) -> Result<De9im, OverlayError> {
        Ok(relate_linestring_polygon(line, polygon).transposed())
    }
}

impl<A, B, P> RelateStrategy<A, B> for RelatePolygonPolygon
where
    A: PolygonTrait<Point = P>,
    B: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn relate(&self, first: &A, second: &B) -> Result<De9im, OverlayError> {
        relate_polygon_polygon(first, second)
    }
}

type TopologyPointModel = Point2D<f64, Cartesian>;

#[derive(Debug, Default, Clone)]
struct Topology {
    points: Vec<[f64; 2]>,
    lines: Vec<Vec<[f64; 2]>>,
    polygons: Vec<Polygon<TopologyPointModel>>,
}

impl Topology {
    fn in_range(&self) -> bool {
        let in_range = |point: [f64; 2]| {
            point[0].is_finite()
                && point[1].is_finite()
                && point[0].abs() <= SAFE_ABS_MAX
                && point[1].abs() <= SAFE_ABS_MAX
        };
        if !self
            .points
            .iter()
            .chain(self.lines.iter().flatten())
            .copied()
            .all(in_range)
        {
            return false;
        }
        self.polygons.iter().all(|polygon| {
            polygon
                .outer
                .0
                .iter()
                .chain(polygon.inners.iter().flat_map(|ring| ring.0.iter()))
                .all(|point| in_range([point.x(), point.y()]))
        })
    }
}

trait TopologyBuilder<G> {
    fn append(&self, geometry: &G, topology: &mut Topology);
}

trait TopologyBuilderForKind {
    type Strategy: Default;
}

#[derive(Debug, Default, Clone, Copy)]
struct TopologyPoint;
#[derive(Debug, Default, Clone, Copy)]
struct TopologyLinestring;
#[derive(Debug, Default, Clone, Copy)]
struct TopologyPolygon;
#[derive(Debug, Default, Clone, Copy)]
struct TopologySegment;
#[derive(Debug, Default, Clone, Copy)]
struct TopologyRing;
#[derive(Debug, Default, Clone, Copy)]
struct TopologyBox;
#[derive(Debug, Default, Clone, Copy)]
struct TopologyMultiPoint;
#[derive(Debug, Default, Clone, Copy)]
struct TopologyMultiLinestring;
#[derive(Debug, Default, Clone, Copy)]
struct TopologyMultiPolygon;
#[derive(Debug, Default, Clone, Copy)]
struct TopologyDynamic;
#[derive(Debug, Default, Clone, Copy)]
struct TopologyCollection;

impl TopologyBuilderForKind for PointTag {
    type Strategy = TopologyPoint;
}
impl TopologyBuilderForKind for LinestringTag {
    type Strategy = TopologyLinestring;
}
impl TopologyBuilderForKind for PolygonTag {
    type Strategy = TopologyPolygon;
}
impl TopologyBuilderForKind for SegmentTag {
    type Strategy = TopologySegment;
}
impl TopologyBuilderForKind for RingTag {
    type Strategy = TopologyRing;
}
impl TopologyBuilderForKind for BoxTag {
    type Strategy = TopologyBox;
}
impl TopologyBuilderForKind for MultiPointTag {
    type Strategy = TopologyMultiPoint;
}
impl TopologyBuilderForKind for MultiLinestringTag {
    type Strategy = TopologyMultiLinestring;
}
impl TopologyBuilderForKind for MultiPolygonTag {
    type Strategy = TopologyMultiPolygon;
}
impl TopologyBuilderForKind for DynamicGeometryTag {
    type Strategy = TopologyDynamic;
}
impl TopologyBuilderForKind for GeometryCollectionTag {
    type Strategy = TopologyCollection;
}

type TopologyBuilderStrategy<G> = <<G as Geometry>::Kind as TopologyBuilderForKind>::Strategy;

impl<G> TopologyBuilder<G> for TopologyPoint
where
    G: Point,
    G::Scalar: Into<f64>,
    <G::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        topology.points.push(xy(geometry));
    }
}

impl<G> TopologyBuilder<G> for TopologyLinestring
where
    G: LinestringTrait,
    <G::Point as Point>::Scalar: Into<f64>,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        append_topology_line(geometry.points().map(xy).collect(), topology);
    }
}

impl<G> TopologyBuilder<G> for TopologyPolygon
where
    G: PolygonTrait,
    <G::Point as Point>::Scalar: Into<f64>,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        append_topology_polygon(geometry, topology);
    }
}

impl<G> TopologyBuilder<G> for TopologySegment
where
    G: SegmentTrait,
    <G::Point as Point>::Scalar: Into<f64>,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        append_topology_line(
            alloc::vec![
                [
                    geometry.get_indexed::<0, 0>().into(),
                    geometry.get_indexed::<0, 1>().into(),
                ],
                [
                    geometry.get_indexed::<1, 0>().into(),
                    geometry.get_indexed::<1, 1>().into(),
                ],
            ],
            topology,
        );
    }
}

impl<G> TopologyBuilder<G> for TopologyRing
where
    G: RingTrait,
    <G::Point as Point>::Scalar: Into<f64>,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        topology
            .polygons
            .push(Polygon::new(topology_ring(geometry)));
    }
}

impl<G> TopologyBuilder<G> for TopologyBox
where
    G: BoxTrait,
    <G::Point as Point>::Scalar: Into<f64>,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        let minimum = [
            geometry.get_indexed::<{ corner::MIN }, 0>().into(),
            geometry.get_indexed::<{ corner::MIN }, 1>().into(),
        ];
        let maximum = [
            geometry.get_indexed::<{ corner::MAX }, 0>().into(),
            geometry.get_indexed::<{ corner::MAX }, 1>().into(),
        ];
        topology
            .polygons
            .push(Polygon::new(Ring::from_vec(alloc::vec![
                topology_point(minimum),
                topology_point([minimum[0], maximum[1]]),
                topology_point(maximum),
                topology_point([maximum[0], minimum[1]]),
                topology_point(minimum),
            ])));
    }
}

impl<G> TopologyBuilder<G> for TopologyMultiPoint
where
    G: MultiPoint,
    <G::ItemPoint as Point>::Scalar: Into<f64>,
    <<G::ItemPoint as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        topology.points.extend(geometry.points().map(xy));
    }
}

impl<G> TopologyBuilder<G> for TopologyMultiLinestring
where
    G: MultiLinestring,
    <G::Point as Point>::Scalar: Into<f64>,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        for line in geometry.linestrings() {
            append_topology_line(line.points().map(xy).collect(), topology);
        }
    }
}

impl<G> TopologyBuilder<G> for TopologyMultiPolygon
where
    G: MultiPolygon,
    <G::Point as Point>::Scalar: Into<f64>,
    <<G::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        for polygon in geometry.polygons() {
            append_topology_polygon(polygon, topology);
        }
    }
}

impl<Scalar, Cs> TopologyBuilder<DynGeometry<Scalar, Cs>> for TopologyDynamic
where
    Scalar: CoordinateScalar + Into<f64>,
    Cs: CoordinateSystem,
    Cs::Family: SameAs<CartesianFamily>,
{
    fn append(&self, geometry: &DynGeometry<Scalar, Cs>, topology: &mut Topology) {
        append_dynamic_topology(geometry, topology);
    }
}

impl<G> TopologyBuilder<G> for TopologyCollection
where
    G: GeometryCollection,
    G::Item: Geometry,
    <G::Item as Geometry>::Kind: TopologyBuilderForKind,
    TopologyBuilderStrategy<G::Item>: TopologyBuilder<G::Item> + Default,
{
    fn append(&self, geometry: &G, topology: &mut Topology) {
        for item in geometry.items() {
            TopologyBuilderStrategy::<G::Item>::default().append(item, topology);
        }
    }
}

impl<A, B> RelateStrategy<A, B> for RelateTopology
where
    A: Geometry,
    B: Geometry,
    A::Kind: TopologyBuilderForKind,
    B::Kind: TopologyBuilderForKind,
    TopologyBuilderStrategy<A>: TopologyBuilder<A> + Default,
    TopologyBuilderStrategy<B>: TopologyBuilder<B> + Default,
{
    fn relate(&self, first: &A, second: &B) -> Result<De9im, OverlayError> {
        let mut first_topology = Topology::default();
        TopologyBuilderStrategy::<A>::default().append(first, &mut first_topology);
        let mut second_topology = Topology::default();
        TopologyBuilderStrategy::<B>::default().append(second, &mut second_topology);
        relate_topologies(&first_topology, &second_topology)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Location {
    Interior,
    Boundary,
    Exterior,
}

impl Location {
    fn index(self) -> usize {
        match self {
            Self::Interior => feature::INTERIOR,
            Self::Boundary => feature::BOUNDARY,
            Self::Exterior => feature::EXTERIOR,
        }
    }
}

fn empty_matrix() -> De9im {
    let mut matrix = De9im {
        m: [[Dimension::Empty; 3]; 3],
    };
    matrix.m[feature::EXTERIOR][feature::EXTERIOR] = Dimension::Area;
    matrix
}

fn relate_point_point<A, B>(first: &A, second: &B) -> De9im
where
    A: Point,
    B: Point<Scalar = A::Scalar>,
{
    let mut matrix = empty_matrix();
    if point_equal(first, second) {
        matrix.m[feature::INTERIOR][feature::INTERIOR] = Dimension::Point;
    } else {
        matrix.m[feature::INTERIOR][feature::EXTERIOR] = Dimension::Point;
        matrix.m[feature::EXTERIOR][feature::INTERIOR] = Dimension::Point;
    }
    matrix
}

fn relate_point_linestring<P, L>(point: &P, line: &L) -> De9im
where
    P: Point,
    L: LinestringTrait<Point = P>,
    P::Scalar: Into<f64>,
{
    let mut matrix = empty_matrix();
    let location = point_location_linestring(point, line);
    matrix.m[feature::INTERIOR][location.index()] = Dimension::Point;
    if line_has_curve(line) {
        matrix.m[feature::EXTERIOR][feature::INTERIOR] = Dimension::Curve;
    }
    let boundaries = line_boundary_points(line);
    if boundaries
        .iter()
        .any(|boundary| !point_equal(point, *boundary))
    {
        matrix.m[feature::EXTERIOR][feature::BOUNDARY] = Dimension::Point;
    }
    matrix
}

fn relate_point_polygon<P, G>(point: &P, polygon: &G) -> De9im
where
    P: Point + Copy,
    G: PolygonTrait<Point = P>,
    P::Scalar: Into<f64>,
{
    let mut matrix = empty_matrix();
    let location = point_location_polygon(point, polygon);
    matrix.m[feature::INTERIOR][location.index()] = Dimension::Point;
    matrix.m[feature::EXTERIOR][feature::INTERIOR] = Dimension::Area;
    matrix.m[feature::EXTERIOR][feature::BOUNDARY] = Dimension::Curve;
    matrix
}

fn relate_linestring_linestring<A, B, P>(first: &A, second: &B) -> De9im
where
    A: LinestringTrait<Point = P>,
    B: LinestringTrait<Point = P>,
    P: Point,
    P::Scalar: Into<f64>,
{
    let mut matrix = empty_matrix();
    let mut first_segments = Vec::new();
    for_each_line_segment(first, |start, end| {
        first_segments.push((xy(start), xy(end)));
    });
    let mut second_segments = Vec::new();
    for_each_line_segment(second, |start, end| {
        second_segments.push((xy(start), xy(end)));
    });
    for point in line_boundary_points(first) {
        let location = point_location_linestring(point, second);
        matrix.m[feature::BOUNDARY][location.index()] = Dimension::Point;
    }
    for point in line_boundary_points(second) {
        let location = point_location_linestring(point, first);
        matrix.m[location.index()][feature::BOUNDARY] = Dimension::Point;
    }

    for &first_segment in &first_segments {
        for &second_segment in &second_segments {
            match segment_relation(
                first_segment.0,
                first_segment.1,
                second_segment.0,
                second_segment.1,
            ) {
                SegmentRelation::Disjoint => {}
                SegmentRelation::Point(point) => {
                    let first_location = xy_location_linestring(point, first);
                    let second_location = xy_location_linestring(point, second);
                    matrix.m[first_location.index()][second_location.index()] = Dimension::Point;
                }
                SegmentRelation::Overlap => {
                    matrix.m[feature::INTERIOR][feature::INTERIOR] = Dimension::Curve;
                }
            }
        }
        if !xy_equal(first_segment.0, first_segment.1) {
            for interval in segment_parameters(first_segment, &second_segments, &[]).windows(2) {
                debug_assert!(interval[1] - interval[0] > f64::EPSILON);
                let sample = interpolate(
                    first_segment.0,
                    first_segment.1,
                    (interval[0] + interval[1]) * 0.5,
                );
                let location = xy_location_linestring(sample, second);
                matrix.m[feature::INTERIOR][location.index()] = Dimension::Curve;
            }
        }
    }
    for &second_segment in &second_segments {
        if !xy_equal(second_segment.0, second_segment.1) {
            for interval in segment_parameters(second_segment, &first_segments, &[]).windows(2) {
                debug_assert!(interval[1] - interval[0] > f64::EPSILON);
                let sample = interpolate(
                    second_segment.0,
                    second_segment.1,
                    (interval[0] + interval[1]) * 0.5,
                );
                let location = xy_location_linestring(sample, first);
                matrix.m[location.index()][feature::INTERIOR] = Dimension::Curve;
            }
        }
    }
    matrix
}

fn relate_linestring_polygon<L, G, P>(line: &L, polygon: &G) -> De9im
where
    L: LinestringTrait<Point = P>,
    G: PolygonTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: Into<f64>,
{
    let mut matrix = empty_matrix();
    matrix.m[feature::EXTERIOR][feature::INTERIOR] = Dimension::Area;
    matrix.m[feature::EXTERIOR][feature::BOUNDARY] = Dimension::Curve;
    for point in line_boundary_points(line) {
        let location = point_location_polygon(point, polygon);
        matrix.m[feature::BOUNDARY][location.index()] = Dimension::Point;
    }

    let mut boundary_crossings = 0usize;
    for_each_line_segment(line, |line1, line2| {
        for fraction in [0.125, 0.375, 0.625, 0.875] {
            match xy_location_polygon(interpolate(xy(line1), xy(line2), fraction), polygon) {
                Location::Interior => {
                    matrix.m[feature::INTERIOR][feature::INTERIOR] = Dimension::Curve;
                }
                Location::Boundary => {
                    matrix.m[feature::INTERIOR][feature::BOUNDARY] = Dimension::Point;
                }
                Location::Exterior => {
                    matrix.m[feature::INTERIOR][feature::EXTERIOR] = Dimension::Curve;
                }
            }
        }
        for_each_polygon_segment(polygon, |polygon1, polygon2| {
            if let SegmentRelation::Point(point) =
                segment_relation(xy(line1), xy(line2), xy(polygon1), xy(polygon2))
            {
                boundary_crossings += 1;
                let line_location = xy_location_linestring(point, line);
                matrix.m[line_location.index()][feature::BOUNDARY] = Dimension::Point;
            }
        });
    });
    if boundary_crossings >= 2 {
        matrix.m[feature::INTERIOR][feature::INTERIOR] = Dimension::Curve;
    }
    matrix
}

fn point_equal<A, B>(first: &A, second: &B) -> bool
where
    A: Point,
    B: Point<Scalar = A::Scalar>,
{
    first.get::<0>() == second.get::<0>() && first.get::<1>() == second.get::<1>()
}

fn line_has_curve<L>(line: &L) -> bool
where
    L: LinestringTrait,
    L::Point: Point,
{
    let mut points = line.points();
    let Some(mut previous) = points.next() else {
        return false;
    };
    for current in points {
        if !point_equal(previous, current) {
            return true;
        }
        previous = current;
    }
    false
}

fn line_boundary_points<L>(line: &L) -> alloc::vec::Vec<&L::Point>
where
    L: LinestringTrait,
    L::Point: Point,
{
    let points = line.points();
    let Some(first) = points.clone().next() else {
        return alloc::vec::Vec::new();
    };
    let last = points
        .last()
        .expect("a non-empty iterator has a last point");
    if point_equal(first, last) {
        alloc::vec::Vec::new()
    } else {
        alloc::vec![first, last]
    }
}

fn point_location_linestring<P, L>(point: &P, line: &L) -> Location
where
    P: Point,
    L: LinestringTrait<Point = P>,
    P::Scalar: Into<f64>,
{
    xy_location_linestring(xy(point), line)
}

fn xy_location_linestring<L>(point: [f64; 2], line: &L) -> Location
where
    L: LinestringTrait,
    L::Point: Point,
    <L::Point as Point>::Scalar: Into<f64>,
{
    for boundary in line_boundary_points(line) {
        if xy_equal(point, xy(boundary)) {
            return Location::Boundary;
        }
    }
    let mut interior = false;
    for_each_line_segment(line, |first, second| {
        if point_on_segment(point, xy(first), xy(second)) {
            interior = true;
        }
    });
    if interior {
        Location::Interior
    } else {
        Location::Exterior
    }
}

fn point_location_polygon<P, G>(point: &P, polygon: &G) -> Location
where
    P: Point + Copy,
    G: PolygonTrait<Point = P>,
    P::Scalar: Into<f64>,
{
    xy_location_polygon(xy(point), polygon)
}

fn xy_location_polygon<G>(point: [f64; 2], polygon: &G) -> Location
where
    G: PolygonTrait,
    G::Point: Point,
    <G::Point as Point>::Scalar: Into<f64>,
{
    let mut boundary = false;
    for_each_polygon_segment(polygon, |first, second| {
        if point_on_segment(point, xy(first), xy(second)) {
            boundary = true;
        }
    });
    if boundary {
        return Location::Boundary;
    }
    if point_in_ring_xy(point, polygon.exterior())
        && !polygon
            .interiors()
            .any(|ring| point_in_ring_xy(point, ring))
    {
        Location::Interior
    } else {
        Location::Exterior
    }
}

fn point_in_ring_xy<R>(point: [f64; 2], ring: &R) -> bool
where
    R: RingTrait,
    R::Point: Point,
    <R::Point as Point>::Scalar: Into<f64>,
{
    let mut inside = false;
    for_each_ring_segment(ring, |first, second| {
        let first = xy(first);
        let second = xy(second);
        if (first[1] > point[1]) != (second[1] > point[1])
            && point[0]
                < (second[0] - first[0]) * (point[1] - first[1]) / (second[1] - first[1]) + first[0]
        {
            inside = !inside;
        }
    });
    inside
}

fn for_each_line_segment<L>(line: &L, mut function: impl FnMut(&L::Point, &L::Point))
where
    L: LinestringTrait,
{
    let mut points = line.points();
    let Some(mut previous) = points.next() else {
        return;
    };
    for current in points {
        function(previous, current);
        previous = current;
    }
}

fn for_each_ring_segment<R>(ring: &R, mut function: impl FnMut(&R::Point, &R::Point))
where
    R: RingTrait,
{
    let mut points = ring.points();
    let Some(first) = points.next() else {
        return;
    };
    let mut previous = first;
    for current in points {
        function(previous, current);
        previous = current;
    }
    if !point_equal(previous, first) {
        function(previous, first);
    }
}

fn for_each_polygon_segment<G>(polygon: &G, mut function: impl FnMut(&G::Point, &G::Point))
where
    G: PolygonTrait,
{
    for_each_ring_segment(polygon.exterior(), &mut function);
    for ring in polygon.interiors() {
        for_each_ring_segment(ring, &mut function);
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum SegmentRelation {
    Disjoint,
    Point([f64; 2]),
    Overlap,
}

fn segment_relation(
    first1: [f64; 2],
    first2: [f64; 2],
    second1: [f64; 2],
    second2: [f64; 2],
) -> SegmentRelation {
    let d1 = precise_math::orient2d(second1, second2, first1);
    let d2 = precise_math::orient2d(second1, second2, first2);
    let d3 = precise_math::orient2d(first1, first2, second1);
    let d4 = precise_math::orient2d(first1, first2, second2);
    if opposite(d1, d2) && opposite(d3, d4) {
        return SegmentRelation::Point(line_cross(first1, first2, second1, second2));
    }
    if d1 == 0.0 && d2 == 0.0 && d3 == 0.0 && d4 == 0.0 {
        let overlap = collinear_overlap_length(first1, first2, second1, second2);
        return if overlap > 0.0 {
            SegmentRelation::Overlap
        } else if overlap == 0.0 {
            [first1, first2, second1, second2]
                .into_iter()
                .find(|point| {
                    point_on_segment(*point, first1, first2)
                        && point_on_segment(*point, second1, second2)
                })
                .map_or(SegmentRelation::Disjoint, SegmentRelation::Point)
        } else {
            SegmentRelation::Disjoint
        };
    }
    for (value, point, start, end) in [
        (d1, first1, second1, second2),
        (d2, first2, second1, second2),
        (d3, second1, first1, first2),
        (d4, second2, first1, first2),
    ] {
        if value == 0.0 && point_on_segment(point, start, end) {
            return SegmentRelation::Point(point);
        }
    }
    SegmentRelation::Disjoint
}

fn xy<P>(point: &P) -> [f64; 2]
where
    P: Point,
    P::Scalar: Into<f64>,
{
    [point.get::<0>().into(), point.get::<1>().into()]
}

fn xy_equal(first: [f64; 2], second: [f64; 2]) -> bool {
    first[0] == second[0] && first[1] == second[1]
}

fn point_on_segment(point: [f64; 2], start: [f64; 2], end: [f64; 2]) -> bool {
    precise_math::orient2d(start, end, point) == 0.0
        && point[0] >= start[0].min(end[0])
        && point[0] <= start[0].max(end[0])
        && point[1] >= start[1].min(end[1])
        && point[1] <= start[1].max(end[1])
}

fn opposite(first: f64, second: f64) -> bool {
    (first > 0.0 && second < 0.0) || (first < 0.0 && second > 0.0)
}

fn line_cross(
    first1: [f64; 2],
    first2: [f64; 2],
    second1: [f64; 2],
    second2: [f64; 2],
) -> [f64; 2] {
    let denominator = (first1[0] - first2[0]) * (second1[1] - second2[1])
        - (first1[1] - first2[1]) * (second1[0] - second2[0]);
    let first_cross = first1[0] * first2[1] - first1[1] * first2[0];
    let second_cross = second1[0] * second2[1] - second1[1] * second2[0];
    [
        (first_cross * (second1[0] - second2[0]) - (first1[0] - first2[0]) * second_cross)
            / denominator,
        (first_cross * (second1[1] - second2[1]) - (first1[1] - first2[1]) * second_cross)
            / denominator,
    ]
}

fn collinear_overlap_length(
    first1: [f64; 2],
    first2: [f64; 2],
    second1: [f64; 2],
    second2: [f64; 2],
) -> f64 {
    let use_x = (first1[0] - first2[0]).abs() >= (first1[1] - first2[1]).abs();
    let index = usize::from(!use_x);
    first1[index]
        .max(first2[index])
        .min(second1[index].max(second2[index]))
        - first1[index]
            .min(first2[index])
            .max(second1[index].min(second2[index]))
}

fn interpolate(first: [f64; 2], second: [f64; 2], fraction: f64) -> [f64; 2] {
    [
        first[0] + (second[0] - first[0]) * fraction,
        first[1] + (second[1] - first[1]) * fraction,
    ]
}

fn topology_point(coordinates: [f64; 2]) -> TopologyPointModel {
    TopologyPointModel::new(coordinates[0], coordinates[1])
}

fn topology_ring<R>(ring: &R) -> Ring<TopologyPointModel>
where
    R: RingTrait,
    <R::Point as Point>::Scalar: Into<f64>,
{
    Ring::from_vec(
        ring.points()
            .map(|point| topology_point(xy(point)))
            .collect(),
    )
}

fn append_topology_line(mut points: Vec<[f64; 2]>, topology: &mut Topology) {
    points.dedup_by(|first, second| xy_equal(*first, *second));
    if points.len() >= 2 {
        topology.lines.push(points);
    } else if let Some(point) = points.first() {
        topology.points.push(*point);
    }
}

fn append_topology_polygon<G>(polygon: &G, topology: &mut Topology)
where
    G: PolygonTrait,
    <G::Point as Point>::Scalar: Into<f64>,
{
    let outer = topology_ring(polygon.exterior());
    if outer.0.len() < 3 {
        append_topology_line(outer.0.iter().map(xy).collect(), topology);
        return;
    }
    topology.polygons.push(Polygon::with_inners(
        outer,
        polygon.interiors().map(topology_ring).collect(),
    ));
}

fn append_dynamic_topology<Scalar, Cs>(geometry: &DynGeometry<Scalar, Cs>, topology: &mut Topology)
where
    Scalar: CoordinateScalar + Into<f64>,
    Cs: CoordinateSystem,
    Cs::Family: SameAs<CartesianFamily>,
{
    match geometry {
        DynGeometry::Point(point) => topology.points.push(xy(point)),
        DynGeometry::LineString(line) => {
            append_topology_line(line.points().map(xy).collect(), topology);
        }
        DynGeometry::Polygon(polygon) => append_topology_polygon(polygon, topology),
        DynGeometry::MultiPoint(points) => topology.points.extend(points.points().map(xy)),
        DynGeometry::MultiLineString(lines) => {
            for line in lines.linestrings() {
                append_topology_line(line.points().map(xy).collect(), topology);
            }
        }
        DynGeometry::MultiPolygon(polygons) => {
            for polygon in polygons.polygons() {
                append_topology_polygon(polygon, topology);
            }
        }
        DynGeometry::GeometryCollection(items) => {
            for item in items {
                append_dynamic_topology(item, topology);
            }
        }
    }
}

fn topology_segments(topology: &Topology) -> Vec<([f64; 2], [f64; 2])> {
    let mut segments = Vec::new();
    for line in &topology.lines {
        for points in line.windows(2) {
            debug_assert!(!xy_equal(points[0], points[1]));
            segments.push((points[0], points[1]));
        }
    }
    for polygon in &topology.polygons {
        append_boundary_segments(&polygon.outer, &mut segments);
        for ring in &polygon.inners {
            append_boundary_segments(ring, &mut segments);
        }
    }
    segments
}

fn topology_location(topology: &Topology, point: [f64; 2]) -> Location {
    let mut polygon_boundary = false;
    for polygon in &topology.polygons {
        match xy_location_polygon(point, polygon) {
            Location::Interior => return Location::Interior,
            Location::Boundary => polygon_boundary = true,
            Location::Exterior => {}
        }
    }

    let mut on_line = false;
    let mut endpoint_count = 0usize;
    for line in &topology.lines {
        for segment in line.windows(2) {
            if point_on_segment(point, segment[0], segment[1]) {
                on_line = true;
            }
        }
        let first = *line
            .first()
            .expect("topology lines have at least two points");
        let last = *line
            .last()
            .expect("topology lines have at least two points");
        if !xy_equal(first, last) {
            endpoint_count += usize::from(xy_equal(point, first));
            endpoint_count += usize::from(xy_equal(point, last));
        }
    }
    if on_line {
        return if endpoint_count % 2 == 1 && !polygon_boundary {
            Location::Boundary
        } else {
            Location::Interior
        };
    }
    if topology
        .points
        .iter()
        .any(|candidate| xy_equal(*candidate, point))
    {
        return Location::Interior;
    }
    if polygon_boundary {
        Location::Boundary
    } else {
        Location::Exterior
    }
}

fn dimension_rank(dimension: Dimension) -> u8 {
    match dimension {
        Dimension::Empty => 0,
        Dimension::Point => 1,
        Dimension::Curve => 2,
        Dimension::Area => 3,
    }
}

fn set_dimension(matrix: &mut De9im, row: Location, column: Location, dimension: Dimension) {
    let cell = &mut matrix.m[row.index()][column.index()];
    if dimension_rank(dimension) > dimension_rank(*cell) {
        *cell = dimension;
    }
}

fn segment_parameter(point: [f64; 2], start: [f64; 2], end: [f64; 2]) -> f64 {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    if dx.abs() >= dy.abs() {
        debug_assert_ne!(dx, 0.0);
        (point[0] - start[0]) / dx
    } else {
        debug_assert_ne!(dy, 0.0);
        (point[1] - start[1]) / dy
    }
}

fn segment_parameters(
    segment: ([f64; 2], [f64; 2]),
    all_segments: &[([f64; 2], [f64; 2])],
    split_points: &[[f64; 2]],
) -> Vec<f64> {
    let mut parameters = alloc::vec![0.0, 1.0];
    for &(start, end) in all_segments {
        match segment_relation(segment.0, segment.1, start, end) {
            SegmentRelation::Point(point) => {
                parameters.push(segment_parameter(point, segment.0, segment.1));
            }
            SegmentRelation::Overlap => {
                for point in [start, end] {
                    if point_on_segment(point, segment.0, segment.1) {
                        parameters.push(segment_parameter(point, segment.0, segment.1));
                    }
                }
            }
            SegmentRelation::Disjoint => {}
        }
    }
    for &point in split_points {
        if point_on_segment(point, segment.0, segment.1) {
            parameters.push(segment_parameter(point, segment.0, segment.1));
        }
    }
    parameters.retain(|parameter| (-f64::EPSILON..=1.0 + f64::EPSILON).contains(parameter));
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|first, second| (*first - *second).abs() <= f64::EPSILON);
    parameters
}

fn record_segment_cells(
    matrix: &mut De9im,
    first: &Topology,
    second: &Topology,
    segment: ([f64; 2], [f64; 2]),
    all_segments: &[([f64; 2], [f64; 2])],
) {
    let parameters = segment_parameters(segment, all_segments, &second.points);
    for interval in parameters.windows(2) {
        debug_assert!(interval[1] - interval[0] > f64::EPSILON);
        let midpoint = interpolate(segment.0, segment.1, (interval[0] + interval[1]) * 0.5);
        let first_location = topology_location(first, midpoint);
        let second_location = topology_location(second, midpoint);
        debug_assert_ne!(first_location, Location::Exterior);
        set_dimension(matrix, first_location, second_location, Dimension::Curve);
    }
}

fn append_topology_candidates(
    topology: &Topology,
    segments: &[([f64; 2], [f64; 2])],
    output: &mut Vec<[f64; 2]>,
) {
    output.extend(topology.points.iter().copied());
    for &(start, end) in segments {
        output.push(start);
        output.push(end);
    }
}

fn areas_intersect(first: &Topology, second: &Topology) -> Result<bool, OverlayError> {
    for first_polygon in &first.polygons {
        for second_polygon in &second.polygons {
            if !crate::operation::intersection(first_polygon, second_polygon)?
                .0
                .is_empty()
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn has_area_outside(first: &Topology, second: &Topology) -> Result<bool, OverlayError> {
    for polygon in &first.polygons {
        let mut pieces = alloc::vec![polygon.clone()];
        for clip in &second.polygons {
            let mut remainder = Vec::new();
            for piece in pieces {
                remainder.extend(crate::operation::difference(&piece, clip)?.0);
            }
            pieces = remainder;
            if pieces.is_empty() {
                break;
            }
        }
        if !pieces.is_empty() {
            return Ok(true);
        }
    }
    Ok(false)
}

fn relate_topologies(first: &Topology, second: &Topology) -> Result<De9im, OverlayError> {
    if !first.in_range() || !second.in_range() {
        return Err(OverlayError::Unsupported);
    }

    let first_segments = topology_segments(first);
    let second_segments = topology_segments(second);
    let all_segments = first_segments
        .iter()
        .chain(&second_segments)
        .copied()
        .collect::<Vec<_>>();
    let mut matrix = empty_matrix();

    if areas_intersect(first, second)? {
        matrix.m[feature::INTERIOR][feature::INTERIOR] = Dimension::Area;
    }
    if has_area_outside(first, second)? {
        matrix.m[feature::INTERIOR][feature::EXTERIOR] = Dimension::Area;
    }
    if has_area_outside(second, first)? {
        matrix.m[feature::EXTERIOR][feature::INTERIOR] = Dimension::Area;
    }

    for &segment in &first_segments {
        record_segment_cells(&mut matrix, first, second, segment, &all_segments);
    }
    for &segment in &second_segments {
        for interval in segment_parameters(segment, &all_segments, &first.points).windows(2) {
            debug_assert!(interval[1] - interval[0] > f64::EPSILON);
            let midpoint = interpolate(segment.0, segment.1, (interval[0] + interval[1]) * 0.5);
            let first_location = topology_location(first, midpoint);
            let second_location = topology_location(second, midpoint);
            debug_assert_ne!(second_location, Location::Exterior);
            set_dimension(
                &mut matrix,
                first_location,
                second_location,
                Dimension::Curve,
            );
        }
    }

    let mut candidates = Vec::new();
    append_topology_candidates(first, &first_segments, &mut candidates);
    append_topology_candidates(second, &second_segments, &mut candidates);
    for &(first_start, first_end) in &first_segments {
        for &(second_start, second_end) in &second_segments {
            match segment_relation(first_start, first_end, second_start, second_end) {
                SegmentRelation::Point(point) => candidates.push(point),
                SegmentRelation::Overlap => {
                    for point in [first_start, first_end, second_start, second_end] {
                        if point_on_segment(point, first_start, first_end)
                            && point_on_segment(point, second_start, second_end)
                        {
                            candidates.push(point);
                        }
                    }
                }
                SegmentRelation::Disjoint => {}
            }
        }
    }
    candidates.sort_by(|first, second| {
        first[0]
            .total_cmp(&second[0])
            .then_with(|| first[1].total_cmp(&second[1]))
    });
    candidates.dedup_by(|first, second| xy_equal(*first, *second));
    for point in candidates {
        let first_location = topology_location(first, point);
        let second_location = topology_location(second, point);
        debug_assert!(
            first_location != Location::Exterior || second_location != Location::Exterior
        );
        set_dimension(
            &mut matrix,
            first_location,
            second_location,
            Dimension::Point,
        );
    }

    Ok(matrix)
}

/// Compute the DE-9IM matrix relating two polygons.
///
/// Fills the matrix from Boolean interior regions and exact segment-pair
/// boundary dimensions. Mirrors `boost::geometry::relation`
/// (`algorithms/relation.hpp`) for the areal × areal case.
///
/// # Errors
///
/// Returns [`OverlayError::Unsupported`] when either polygon leaves the exact
/// predicate range.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::relate::{relate, Dimension};
///
/// type P = Point2D<f64, Cartesian>;
/// let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
/// let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
/// let matrix = relate(&a, &b).unwrap();
/// // Overlapping squares: their interiors meet in an area.
/// assert_eq!(matrix.interior_interior(), Dimension::Area);
/// ```
fn relate_polygon_polygon<G1, G2, P>(g1: &G1, g2: &G2) -> Result<De9im, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    if !polygon_in_range(g1) || !polygon_in_range(g2) {
        return Err(OverlayError::Unsupported);
    }

    let interiors_overlap = !crate::operation::intersection(g1, g2)?.0.is_empty();
    let first_outside = !crate::operation::difference(g1, g2)?.0.is_empty();
    let second_outside = !crate::operation::difference(g2, g1)?.0.is_empty();
    let boundary_boundary = polygon_boundary_dimension(g1, g2);

    let mut matrix = empty_matrix();
    if interiors_overlap {
        matrix.m[feature::INTERIOR][feature::INTERIOR] = Dimension::Area;
    }
    if first_outside {
        matrix.m[feature::INTERIOR][feature::EXTERIOR] = Dimension::Area;
        matrix.m[feature::BOUNDARY][feature::EXTERIOR] = Dimension::Curve;
    }
    if second_outside {
        matrix.m[feature::EXTERIOR][feature::INTERIOR] = Dimension::Area;
        matrix.m[feature::EXTERIOR][feature::BOUNDARY] = Dimension::Curve;
    }
    matrix.m[feature::BOUNDARY][feature::BOUNDARY] = boundary_boundary;

    if interiors_overlap {
        match (first_outside, second_outside) {
            (true, true) => {
                matrix.m[feature::INTERIOR][feature::BOUNDARY] = Dimension::Curve;
                matrix.m[feature::BOUNDARY][feature::INTERIOR] = Dimension::Curve;
            }
            (true, false) => {
                matrix.m[feature::INTERIOR][feature::BOUNDARY] = Dimension::Curve;
            }
            (false, true) => {
                matrix.m[feature::BOUNDARY][feature::INTERIOR] = Dimension::Curve;
            }
            (false, false) => {}
        }
    }

    Ok(matrix)
}

fn polygon_boundary_dimension<G1, G2, P>(first: &G1, second: &G2) -> Dimension
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: Point,
    P::Scalar: Into<f64>,
{
    let first_segments = polygon_boundary_segments(first);
    let second_segments = polygon_boundary_segments(second);
    let mut dimension = Dimension::Empty;
    for (first_start, first_end) in &first_segments {
        for (second_start, second_end) in &second_segments {
            match segment_relation(*first_start, *first_end, *second_start, *second_end) {
                SegmentRelation::Overlap => return Dimension::Curve,
                SegmentRelation::Point(_) => dimension = Dimension::Point,
                SegmentRelation::Disjoint => {}
            }
        }
    }
    dimension
}

fn polygon_boundary_segments<G, P>(polygon: &G) -> alloc::vec::Vec<([f64; 2], [f64; 2])>
where
    G: PolygonTrait<Point = P>,
    P: Point,
    P::Scalar: Into<f64>,
{
    let mut segments = alloc::vec::Vec::new();
    append_boundary_segments(polygon.exterior(), &mut segments);
    for ring in polygon.interiors() {
        append_boundary_segments(ring, &mut segments);
    }
    segments
}

fn append_boundary_segments<R>(ring: &R, output: &mut alloc::vec::Vec<([f64; 2], [f64; 2])>)
where
    R: RingTrait,
    <R::Point as Point>::Scalar: Into<f64>,
{
    let points: alloc::vec::Vec<_> = ring.points().map(xy).collect();
    for pair in points.windows(2) {
        if !xy_equal(pair[0], pair[1]) {
            output.push((pair[0], pair[1]));
        }
    }
    if let (Some(first), Some(last)) = (points.first(), points.last())
        && !xy_equal(*first, *last)
    {
        output.push((*last, *first));
    }
}

/// Test whether two polygons satisfy a DE-9IM mask.
///
/// Mirrors `boost::geometry::relate(g1, g2, mask)` from
/// `boost/geometry/algorithms/detail/relate/interface.hpp:347-382`. Use the
/// crate-root `relation(g1, g2)` entry to obtain the matrix itself, matching
/// `boost::geometry::relation` from `algorithms/relation.hpp`.
///
/// # Errors
///
/// Returns [`RelateError::Overlay`] when the areal relation is unsupported,
/// or [`RelateError::InvalidMask`] for a malformed mask.
#[inline]
#[must_use = "relate can fail and its predicate result should be used"]
pub fn relate_mask<G1, G2>(g1: &G1, g2: &G2, mask: &str) -> Result<bool, RelateError>
where
    G1: Geometry,
    G2: Geometry,
    G1::Kind: RelatePairStrategy<G2::Kind>,
    PairStrategy<G1, G2>: RelateStrategy<G1, G2> + Default,
{
    relate(g1, g2)?.matches(mask)
}

/// `contains_properly`: the second geometry lies strictly inside the first.
///
/// Evaluates the OGC DE-9IM mask `T**FF*FF*`: the interiors intersect, and
/// neither the interior nor boundary of the second geometry intersects the
/// boundary or exterior of the first.
///
/// # Errors
///
/// Propagates [`OverlayError::Unsupported`] from [`relate`].
#[inline]
#[must_use = "contains_properly can fail and its predicate result should be used"]
pub fn contains_properly<G1, G2>(g1: &G1, g2: &G2) -> Result<bool, OverlayError>
where
    G1: Geometry,
    G2: Geometry,
    G1::Kind: RelatePairStrategy<G2::Kind>,
    PairStrategy<G1, G2>: RelateStrategy<G1, G2> + Default,
{
    let matrix = relate(g1, g2)?;
    Ok(matrix.interior_interior().is_set()
        && !matrix.m[feature::BOUNDARY][feature::INTERIOR].is_set()
        && !matrix.m[feature::BOUNDARY][feature::BOUNDARY].is_set()
        && !matrix.m[feature::EXTERIOR][feature::INTERIOR].is_set()
        && !matrix.m[feature::EXTERIOR][feature::BOUNDARY].is_set())
}

/// `touches`: the boundaries meet but the interiors do not.
///
/// Mirrors `boost::geometry::touches` (`algorithms/touches.hpp`) for the
/// areal × areal case: `II = F` and the boundaries have non-empty
/// intersection.
///
/// # Errors
///
/// Propagates [`OverlayError::Unsupported`] from [`relate`].
#[inline]
#[must_use = "touches can fail and its predicate result should be used"]
pub fn touches<G1, G2>(g1: &G1, g2: &G2) -> Result<bool, OverlayError>
where
    G1: Geometry,
    G2: Geometry,
    G1::Kind: RelatePairStrategy<G2::Kind>,
    PairStrategy<G1, G2>: RelateStrategy<G1, G2> + Default,
{
    let matrix = relate(g1, g2)?;
    Ok(!matrix.interior_interior().is_set()
        && (matrix.m[feature::INTERIOR][feature::BOUNDARY].is_set()
            || matrix.m[feature::BOUNDARY][feature::INTERIOR].is_set()
            || matrix.boundary_boundary().is_set()))
}

/// `overlaps`: the interiors intersect, and each geometry has interior
/// points outside the other, at the same dimension.
///
/// Mirrors `boost::geometry::overlaps` (`algorithms/overlaps.hpp`) for
/// areal × areal: `II = 2`, `IE = 2`, and `EI = 2`.
///
/// # Errors
///
/// Propagates [`OverlayError::Unsupported`] from [`relate`].
#[inline]
#[must_use = "overlaps can fail and its predicate result should be used"]
pub fn overlaps<G1, G2>(g1: &G1, g2: &G2) -> Result<bool, OverlayError>
where
    G1: Geometry,
    G2: Geometry,
    G1::Kind: RelatePairStrategy<G2::Kind>,
    PairStrategy<G1, G2>: RelateStrategy<G1, G2> + Default,
{
    let matrix = relate(g1, g2)?;
    let dimension = matrix.interior_interior();
    Ok(matches!(
        dimension,
        Dimension::Point | Dimension::Curve | Dimension::Area
    ) && matrix.interior_exterior() == dimension
        && matrix.exterior_interior() == dimension)
}

/// `crosses`: test the DE-9IM crossing masks for supported pairs.
///
/// Mirrors `boost::geometry::crosses` (`algorithms/crosses.hpp`); the
/// areal × areal arm returns `false` by definition, while line/line and
/// line/areal pairs use their corresponding dimensional masks.
///
/// # Errors
///
/// Propagates [`OverlayError::Unsupported`] from [`relate`].
#[inline]
#[must_use = "crosses can fail and its predicate result should be used"]
pub fn crosses<G1, G2>(g1: &G1, g2: &G2) -> Result<bool, OverlayError>
where
    G1: Geometry,
    G2: Geometry,
    G1::Kind: RelatePairStrategy<G2::Kind>,
    PairStrategy<G1, G2>: RelateStrategy<G1, G2> + Default,
{
    let matrix = relate(g1, g2)?;
    Ok((matrix.interior_interior() == Dimension::Point
        && matrix.interior_exterior() == Dimension::Curve
        && matrix.exterior_interior() == Dimension::Curve)
        || (matrix.interior_interior() == Dimension::Curve
            && (matrix.interior_exterior() == Dimension::Curve
                || matrix.exterior_interior() == Dimension::Curve)))
}

#[cfg(test)]
mod tests {
    //! OVL6.T1 / T2 done-when: matrix + predicate values. Mirrors the
    //! case families in `test/algorithms/relate/` and the
    //! `touches` / `overlaps` test files.

    use super::{Dimension, contains_properly, crosses, overlaps, relate, touches};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};

    type P = Point2D<f64, Cartesian>;

    fn square(x: f64, y: f64, s: f64) -> Polygon<P> {
        polygon![[(x, y), (x + s, y), (x + s, y + s), (x, y + s), (x, y)]]
    }

    #[test]
    fn overlapping_squares_overlap() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        assert_eq!(relate(&a, &b).unwrap().interior_interior(), Dimension::Area);
        assert!(overlaps(&a, &b).unwrap());
        assert!(!touches(&a, &b).unwrap());
        assert!(!crosses(&a, &b).unwrap());
    }

    #[test]
    fn proper_containment_excludes_boundary_contact() {
        let container = square(0.0, 0.0, 5.0);
        assert!(contains_properly(&container, &square(1.0, 1.0, 1.0)).unwrap());
        assert!(!contains_properly(&container, &square(0.0, 1.0, 1.0)).unwrap());
    }

    #[test]
    fn edge_touching_squares_have_curve_boundary_intersection() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(2.0, 0.0, 2.0);
        assert_eq!(
            relate(&a, &b).unwrap().boundary_boundary(),
            Dimension::Curve
        );
        assert!(touches(&a, &b).unwrap());
        assert!(!overlaps(&a, &b).unwrap());
    }

    #[test]
    fn edge_aligned_overlap_is_detected() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (3.0, 0.0), (3.0, 1.0), (0.0, 1.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(2.0, 0.0), (5.0, 0.0), (5.0, 1.0), (2.0, 1.0), (2.0, 0.0)]];
        assert!(overlaps(&a, &b).unwrap());
    }

    #[test]
    fn out_of_range_coordinates_are_unsupported() {
        // Regression: past ±2^26 the turn kernel silently drops
        // intersections, so an emptied turn graph would be misread as
        // disjoint (II=Empty) for genuinely overlapping huge polygons.
        // relate must refuse rather than return that wrong matrix.
        use crate::operation::OverlayError;
        let a: Polygon<P> = polygon![[
            (0.0, 0.0),
            (2e14, 0.0),
            (2e14, 2e14),
            (0.0, 2e14),
            (0.0, 0.0)
        ]];
        let b: Polygon<P> = polygon![[
            (1e14, 1e14),
            (3e14, 1e14),
            (3e14, 3e14),
            (1e14, 3e14),
            (1e14, 1e14)
        ]];
        assert_eq!(relate(&a, &b), Err(OverlayError::Unsupported));
        assert_eq!(overlaps(&a, &b), Err(OverlayError::Unsupported));
    }

    #[test]
    fn disjoint_squares_neither() {
        let a = square(0.0, 0.0, 1.0);
        let b = square(5.0, 5.0, 1.0);
        assert!(!touches(&a, &b).unwrap());
        assert!(!overlaps(&a, &b).unwrap());
        assert_eq!(
            relate(&a, &b).unwrap().interior_interior(),
            Dimension::Empty
        );
    }

    #[test]
    fn contained_square_does_not_overlap_or_touch() {
        // small ⊂ big: interiors meet (II = area) but small has no
        // interior outside big, so it is containment, not overlap. The
        // rep-point containment makes this decidable (no boundary touch),
        // so it is answered normally.
        let big = square(0.0, 0.0, 10.0);
        let small = square(3.0, 3.0, 2.0);
        assert_eq!(
            relate(&big, &small).unwrap().interior_interior(),
            Dimension::Area
        );
        assert!(!overlaps(&big, &small).unwrap());
        assert!(!touches(&big, &small).unwrap());
    }
}
