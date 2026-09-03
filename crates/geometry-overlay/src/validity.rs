//! OVL6.T4 — `is_valid` for rings and polygons.
//!
//! Mirrors `boost/geometry/algorithms/is_valid.hpp` and the failure
//! taxonomy in `boost/geometry/algorithms/validity_failure_type.hpp`.
//! A geometry is valid when it satisfies the OGC simple-feature rules:
//! finite, in-range coordinates; enough points; a closed boundary; no
//! spikes; no self-intersections; the expected ring orientation; and,
//! for polygons, every interior ring covered by the exterior.
//!
//! Polygon-level ring pairs and distinct multi-polygon members are checked
//! after their individual rings pass, including nested holes, disconnected
//! interiors, and intersecting member interiors.
//!
//! Scope: `Ring`, `Polygon`, and `MultiPolygon` validation.
//! Coordinate validity (NaN / infinity) is checked because the robustness
//! gate depends on finite input.
//!
//! [`is_valid`] preserves this crate's strict behavior for compatibility.
//! [`is_valid_with`] accepts [`ValidityOptions`], including
//! [`ValidityOptions::BOOST_DEFAULT`] which permits consecutive repeated
//! points like `policies/is_valid/default_policy.hpp:26-61`.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::Segment;
use geometry_strategy::{AreaStrategy, ShoelaceArea, WithinRing, WithinStrategy};
use geometry_tag::{MultiPolygonTag, PolygonTag, RingTag, SameAs};
use geometry_trait::{
    Geometry, MultiPolygon as MultiPolygonTrait, Point, PointMut, Polygon as PolygonTrait,
    Ring as RingTrait,
};

use crate::predicate::range_guard::coordinate_in_range;
use crate::predicate::segment_intersection::{SegmentIntersection, segment_intersection};

/// Why a geometry failed [`is_valid_ring`] / [`is_valid_polygon`].
///
/// Mirrors Boost's complete `validity_failure_type` taxonomy
/// (`algorithms/validity_failure_type.hpp:33-113`). The current areal
/// validator produces the relevant ring/polygon variants; retaining the
/// remaining categories keeps reporting stable as kind dispatch expands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidityFailure {
    /// Fewer than the 4 points a closed ring needs (3 distinct + the
    /// repeated closing vertex). Boost's `failure_few_points`.
    FewPoints,
    /// Two consecutive vertices are equal. Boost's
    /// `failure_duplicate_points`.
    DuplicatePoints,
    /// The ring's first and last vertices differ — it is not closed.
    /// Boost's `failure_not_closed`.
    NotClosed,
    /// Two non-adjacent edges of the ring cross, or an edge touches a
    /// non-adjacent vertex. Boost's `failure_self_intersections`.
    SelfIntersection,
    /// A coordinate is NaN or infinite. Boost's
    /// `failure_invalid_coordinate`.
    InvalidCoordinate,
    /// An interior ring is not contained in the exterior ring. Boost's
    /// `failure_interior_rings_outside`.
    InteriorRingOutside,
    /// A coordinate lies outside the safe arithmetic range
    /// ([`SAFE_ABS_MAX`](crate::predicate::range_guard::SAFE_ABS_MAX)).
    /// Past that magnitude the segment-intersection kernel yields
    /// `OutOfRange` and the self-intersection test would silently miss a
    /// real crossing, so validity cannot be confirmed. Reported as a
    /// distinct failure rather than a bogus "valid" (there is no Boost
    /// analogue — Boost's rescaling policy sidesteps the range limit this
    /// no-rescale port trades for).
    CoordinateOutOfRange,
    /// A vertex triple folds back on itself along one line — the ring
    /// carries a spike. Boost's `failure_spikes`.
    Spikes,
    /// The ring's stored vertex order contradicts its declared
    /// [`PointOrder`](geometry_trait::PointOrder) (or the ring has zero
    /// area, which admits no orientation). Exterior rings must traverse
    /// in their declared order (strategy-level signed area positive);
    /// interior rings the opposite. Boost's `failure_wrong_orientation`.
    WrongOrientation,
    /// One interior ring is contained by another interior ring. Boost's
    /// `failure_nested_interior_rings`.
    NestedInteriorRings,
    /// Ring contacts split the polygon's filled interior into disconnected
    /// pieces. Boost's `failure_disconnected_interior`.
    DisconnectedInterior,
    /// Distinct multi-polygon members overlap in area or share a boundary
    /// curve. Boost's `failure_intersecting_interiors`.
    IntersectingInteriors,
    /// The geometry collapses below its declared topological dimension.
    /// Boost's `failure_wrong_topological_dimension`.
    WrongTopologicalDimension,
    /// A box's maximum corner is lexicographically before its minimum corner.
    /// Boost's `failure_wrong_corner_order`.
    WrongCornerOrder,
    /// Collinear vertices occur on one polyhedral-surface face. Boost's
    /// `failure_collinear_points_on_face`.
    CollinearPointsOnFace,
    /// Vertices of one polyhedral-surface face are not coplanar. Boost's
    /// `failure_non_coplanar_points_on_face`.
    NonCoplanarPointsOnFace,
    /// A polyhedral-surface face contains too few vertices. Boost's
    /// `failure_few_points_on_face`.
    FewPointsOnFace,
    /// A polyhedral-surface edge has inconsistent face orientation. Boost's
    /// `failure_inconsistent_orientation`.
    InconsistentOrientation,
    /// Polyhedral-surface faces intersect away from a shared edge. Boost's
    /// `failure_invalid_intersection`.
    InvalidIntersection,
    /// Polyhedral-surface faces do not form a connected surface. Boost's
    /// `failure_disconnected_surface`.
    DisconnectedSurface,
}

impl ValidityFailure {
    /// Return the stable reason prefix for this failure.
    ///
    /// The areal, linear, box, and coordinate strings are byte-for-byte the
    /// messages returned by `validity_failure_type_message` in
    /// `policies/is_valid/failing_reason_policy.hpp:32-63`. Surface messages
    /// extend that table for the surface failure values added later in
    /// `algorithms/validity_failure_type.hpp:91-113`.
    #[must_use]
    pub const fn message(self) -> &'static str {
        match self {
            Self::FewPoints => "Geometry has too few points",
            Self::WrongTopologicalDimension => "Geometry has wrong topological dimension",
            Self::Spikes => "Geometry has spikes",
            Self::DuplicatePoints => "Geometry has duplicate (consecutive) points",
            Self::NotClosed => "Geometry is defined as closed but is open",
            Self::SelfIntersection => "Geometry has invalid self-intersections",
            Self::WrongOrientation => "Geometry has wrong orientation",
            Self::InteriorRingOutside => {
                "Geometry has interior rings defined outside the outer boundary"
            }
            Self::NestedInteriorRings => "Geometry has nested interior rings",
            Self::DisconnectedInterior => "Geometry has disconnected interior",
            Self::IntersectingInteriors => "Multi-polygon has intersecting interiors",
            Self::WrongCornerOrder => "Box has corners in wrong order",
            Self::InvalidCoordinate => "Geometry has point(s) with invalid coordinate(s)",
            Self::CoordinateOutOfRange => {
                "Geometry has coordinate(s) outside the supported arithmetic range"
            }
            Self::CollinearPointsOnFace => "Geometry has collinear points on a face",
            Self::NonCoplanarPointsOnFace => "Geometry has non-coplanar points on a face",
            Self::FewPointsOnFace => "Geometry has too few points on a face",
            Self::InconsistentOrientation => "Geometry has inconsistent surface orientation",
            Self::InvalidIntersection => "Geometry has invalid face intersections",
            Self::DisconnectedSurface => "Geometry has a disconnected surface",
        }
    }
}

impl core::fmt::Display for ValidityFailure {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str(self.message())
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ValidityFailure {}

/// Behavior switches applied by [`is_valid_with`].
///
/// Mirrors the `AllowDuplicates` and `AllowSpikes` template parameters of
/// `policies/is_valid/default_policy.hpp:26-61`. The current validator covers
/// areal geometries, so `allow_spikes_for_linear` is recorded for API parity
/// but only becomes observable when linear validity dispatch is added.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ValidityOptions {
    allow_duplicates: bool,
    allow_spikes_for_linear: bool,
}

impl ValidityOptions {
    /// Existing Rust behavior: report duplicates and spikes.
    pub const STRICT: Self = Self::new(false, false);

    /// Boost's default validity behavior: permit duplicate points and spikes
    /// in linear geometries.
    pub const BOOST_DEFAULT: Self = Self::new(true, true);

    /// Construct validity behavior from Boost's two policy switches.
    #[must_use]
    pub const fn new(allow_duplicates: bool, allow_spikes_for_linear: bool) -> Self {
        Self {
            allow_duplicates,
            allow_spikes_for_linear,
        }
    }

    /// Whether consecutive duplicate points are accepted.
    #[must_use]
    pub const fn allows_duplicates(self) -> bool {
        self.allow_duplicates
    }

    /// Whether spikes are accepted when linear validity dispatch is used.
    #[must_use]
    pub const fn allows_spikes_for_linear(self) -> bool {
        self.allow_spikes_for_linear
    }
}

impl Default for ValidityOptions {
    fn default() -> Self {
        Self::STRICT
    }
}

/// Per-kind validity implementation selected by [`is_valid`].
///
/// Rust tag-dispatch adapter for `boost::geometry::is_valid` from
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
#[doc(hidden)]
pub trait ValidityStrategy<G> {
    fn apply(&self, geometry: &G, options: ValidityOptions) -> Result<(), ValidityFailure>;
}

/// Tag-to-validity implementation picker.
///
/// Rust counterpart to the geometry-kind resolution behind
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
#[doc(hidden)]
pub trait ValidityStrategyForKind {
    type S: Default;
}

/// Ring validity implementation.
///
/// Implements the ring arm selected by the entry at
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct RingValidity;

/// Polygon validity implementation.
///
/// Implements the polygon arm selected by the entry at
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct PolygonValidity;

/// Multi-polygon validity implementation.
///
/// Implements the multi-polygon arm selected by the entry at
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
#[doc(hidden)]
#[derive(Debug, Default, Clone, Copy)]
pub struct MultiPolygonValidity;

/// Selects Boost's ring validity dispatch behind
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
impl ValidityStrategyForKind for RingTag {
    type S = RingValidity;
}

/// Selects Boost's polygon validity dispatch behind
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
impl ValidityStrategyForKind for PolygonTag {
    type S = PolygonValidity;
}

/// Selects Boost's multi-polygon validity dispatch behind
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
impl ValidityStrategyForKind for MultiPolygonTag {
    type S = MultiPolygonValidity;
}

/// Validate an areal geometry through its public geometry-kind tag.
///
/// Mirrors `boost::geometry::is_valid` from
/// `boost/geometry/algorithms/detail/is_valid/interface.hpp:155-202`.
/// Rings, polygons, and
/// multi-polygons use their corresponding validators; unsupported kinds fail
/// at compile time instead of returning an uninformative runtime value.
///
/// # Errors
///
/// Returns the first [`ValidityFailure`] detected by the selected areal
/// validator.
#[inline]
#[must_use = "validity failures must be handled"]
pub fn is_valid<G>(geometry: &G) -> Result<(), ValidityFailure>
where
    G: Geometry,
    G::Kind: ValidityStrategyForKind,
    <G::Kind as ValidityStrategyForKind>::S: ValidityStrategy<G>,
{
    is_valid_with(geometry, ValidityOptions::STRICT)
}

/// Validate an areal geometry with explicit validity behavior.
///
/// Mirrors the policy-taking overload behind
/// `algorithms/detail/is_valid/interface.hpp:155-202`. Use
/// [`ValidityOptions::BOOST_DEFAULT`] to select Boost's default handling of
/// consecutive duplicates, or [`ValidityOptions::STRICT`] for the behavior of
/// [`is_valid`].
///
/// # Errors
///
/// Returns the first [`ValidityFailure`] not accepted by `options`.
///
/// # Panics
///
/// Panics if a custom ring implementation passes validation with a non-empty
/// point iterator but yields no point when iterated again immediately after.
#[inline]
#[must_use = "validity failures must be handled"]
pub fn is_valid_with<G>(geometry: &G, options: ValidityOptions) -> Result<(), ValidityFailure>
where
    G: Geometry,
    G::Kind: ValidityStrategyForKind,
    <G::Kind as ValidityStrategyForKind>::S: ValidityStrategy<G>,
{
    <<G::Kind as ValidityStrategyForKind>::S as Default>::default().apply(geometry, options)
}

/// Return Boost's human-readable reason for strict validation.
///
/// This is the allocation-free Rust counterpart to the string-output overload
/// driven by `policies/is_valid/failing_reason_policy.hpp`.
#[inline]
#[must_use]
pub fn validity_reason<G>(geometry: &G) -> &'static str
where
    G: Geometry,
    G::Kind: ValidityStrategyForKind,
    <G::Kind as ValidityStrategyForKind>::S: ValidityStrategy<G>,
{
    validity_reason_with(geometry, ValidityOptions::STRICT)
}

/// Return Boost's human-readable reason using explicit validity behavior.
#[inline]
#[must_use]
pub fn validity_reason_with<G>(geometry: &G, options: ValidityOptions) -> &'static str
where
    G: Geometry,
    G::Kind: ValidityStrategyForKind,
    <G::Kind as ValidityStrategyForKind>::S: ValidityStrategy<G>,
{
    match is_valid_with(geometry, options) {
        Ok(()) => "Geometry is valid",
        Err(failure) => failure.message(),
    }
}

/// Implements the ring validity arm selected by
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
impl<G, P> ValidityStrategy<G> for RingValidity
where
    G: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(&self, ring: &G, options: ValidityOptions) -> Result<(), ValidityFailure> {
        is_valid_ring_with(ring, options)
    }
}

/// Implements the polygon validity arm selected by
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
impl<G, P> ValidityStrategy<G> for PolygonValidity
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(&self, polygon: &G, options: ValidityOptions) -> Result<(), ValidityFailure> {
        is_valid_polygon_with(polygon, options)
    }
}

/// Implements the multi-polygon validity arm selected by
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
impl<G, P> ValidityStrategy<G> for MultiPolygonValidity
where
    G: MultiPolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn apply(&self, multi_polygon: &G, options: ValidityOptions) -> Result<(), ValidityFailure> {
        let polygons: Vec<_> = multi_polygon.polygons().collect();
        for polygon in &polygons {
            is_valid_polygon_with(*polygon, options)?;
        }
        for first in 0..polygons.len() {
            for second in (first + 1)..polygons.len() {
                let matrix = crate::relate::relate(polygons[first], polygons[second])
                    .map_err(|_| ValidityFailure::SelfIntersection)?;
                if matrix.interior_interior() == crate::relate::Dimension::Area
                    || matrix.boundary_boundary() == crate::relate::Dimension::Curve
                {
                    return Err(ValidityFailure::IntersectingInteriors);
                }
            }
        }
        Ok(())
    }
}

/// Validate a single ring.
///
/// Checks point count, closure, coordinate finiteness, that no two
/// non-adjacent edges intersect, that no vertex triple is a spike, and
/// that the ring is wound in its declared order. Returns `Ok(())` for a
/// valid ring.
///
/// Mirrors the ring arm of `boost::geometry::is_valid`
/// (`algorithms/is_valid.hpp`, via `detail/is_valid/ring.hpp`).
///
/// # Errors
///
/// Returns a [`ValidityFailure`] describing the first rule the ring violates,
/// including [`ValidityFailure::Spikes`] and
/// [`ValidityFailure::WrongOrientation`].
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{Point2D, Ring};
/// use geometry_overlay::validity::is_valid_ring;
///
/// type P = Point2D<f64, Cartesian>;
/// let square: Ring<P> = Ring::from_vec(vec![
///     P::new(0.0, 0.0), P::new(0.0, 1.0), P::new(1.0, 1.0), P::new(1.0, 0.0), P::new(0.0, 0.0),
/// ]);
/// assert!(is_valid_ring(&square).is_ok());
/// ```
#[inline]
#[must_use = "validity failures must be handled"]
pub fn is_valid_ring<R, P>(ring: &R) -> Result<(), ValidityFailure>
where
    R: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    is_valid_ring_with(ring, ValidityOptions::STRICT)
}

/// Validate one ring with explicit validity behavior.
///
/// # Errors
///
/// Returns the first [`ValidityFailure`] not accepted by `options`.
#[inline]
#[must_use = "validity failures must be handled"]
pub fn is_valid_ring_with<R, P>(ring: &R, options: ValidityOptions) -> Result<(), ValidityFailure>
where
    R: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    validate_ring(ring, false, options)
}

/// Shared ring validation. `is_interior` flips the orientation
/// expectation: an exterior ring traverses in its declared order
/// (strategy-level `ShoelaceArea` positive); an interior ring winds
/// opposite (negative) — mirroring Boost's
/// `is_properly_oriented<Ring, IsInteriorRing>`.
fn validate_ring<R, P>(
    ring: &R,
    is_interior: bool,
    options: ValidityOptions,
) -> Result<(), ValidityFailure>
where
    R: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let mut pts: Vec<P> = ring.points().copied().collect();

    // Coordinate finiteness.
    for p in &pts {
        let x: f64 = p.get::<0>().into();
        let y: f64 = p.get::<1>().into();
        if !x.is_finite() || !y.is_finite() {
            return Err(ValidityFailure::InvalidCoordinate);
        }
    }

    // Boost's default policy accepts consecutive duplicates. Remove them
    // before count, spike, intersection, and orientation checks so a
    // zero-length edge cannot create a secondary failure.
    if options.allows_duplicates() {
        let mut deduplicated = Vec::with_capacity(pts.len());
        for point in pts {
            if !deduplicated
                .last()
                .is_some_and(|previous| same_point(previous, &point))
            {
                deduplicated.push(point);
            }
        }
        pts = deduplicated;
    }

    // Out-of-range coordinates: the self-intersection test below routes
    // each edge pair through the segment-intersection kernel, which drops
    // any crossing at coordinates past ±SAFE_ABS_MAX as `OutOfRange`. A
    // genuinely self-intersecting ring would then pass unnoticed, so
    // validity cannot be confirmed — refuse rather than claim valid.
    for p in &pts {
        if !coordinate_in_range(p) {
            return Err(ValidityFailure::CoordinateOutOfRange);
        }
    }

    // A closed ring needs at least 4 vertices (triangle + closing point).
    if pts.len() < 4 {
        return Err(ValidityFailure::FewPoints);
    }

    // First and last vertex must coincide.
    if !same_point(&pts[0], &pts[pts.len() - 1]) {
        return Err(ValidityFailure::NotClosed);
    }

    if !options.allows_duplicates() && pts.windows(2).any(|pair| same_point(&pair[0], &pair[1])) {
        return Err(ValidityFailure::DuplicatePoints);
    }

    // A vertex triple that folds back on itself along one line — Boost's
    // failure_spikes. Checked before self-intersection so spike inputs
    // report a deterministic error (matches Boost's check order).
    if has_spike(&pts) {
        return Err(ValidityFailure::Spikes);
    }

    // Orientation — Boost's failure_wrong_orientation. The strategy area
    // already folds the declared PointOrder: a correctly wound exterior
    // is positive, a correctly wound hole negative. Zero area
    // (degenerate) fails either way.
    //
    // Checked *before* self-intersection, because that is the order Boost
    // reports in: a counter-clockwise ring that also crosses itself comes
    // back `failure_wrong_orientation`, and only a correctly wound ring
    // goes on to be reported as `failure_self_intersections`. Spikes still
    // come first — a wrongly wound ring carrying a spike is
    // `failure_spikes`. Verified against Boost 1.83; see
    // `orientation_is_reported_before_self_intersection`.
    let area = ShoelaceArea.area(ring);
    let zero = <P::Scalar as CoordinateScalar>::ZERO;
    let properly_oriented = if is_interior {
        area < zero
    } else {
        area > zero
    };
    if !properly_oriented {
        return Err(ValidityFailure::WrongOrientation);
    }

    // No non-adjacent edge may intersect another.
    if has_self_intersection(&pts) {
        return Err(ValidityFailure::SelfIntersection);
    }

    Ok(())
}

/// Validate a polygon: its exterior ring (with exterior orientation
/// expectations), each interior ring (with interior orientation
/// expectations), and all exterior/interior and interior/interior ring-pair
/// topology constraints.
///
/// Mirrors the polygon arm of `boost::geometry::is_valid`
/// (`detail/is_valid/polygon.hpp`).
///
/// # Errors
///
/// Returns a [`ValidityFailure`] describing the first rule the polygon
/// violates, including failures for outside, nested, crossing, or
/// disconnecting interior rings.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::validity::is_valid_polygon;
///
/// type P = Point2D<f64, Cartesian>;
/// let pg: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)]];
/// assert!(is_valid_polygon(&pg).is_ok());
/// ```
#[inline]
#[must_use = "validity failures must be handled"]
pub fn is_valid_polygon<G, P>(polygon: &G) -> Result<(), ValidityFailure>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    is_valid_polygon_with(polygon, ValidityOptions::STRICT)
}

/// Validate one polygon with explicit validity behavior.
///
/// # Errors
///
/// Returns the first [`ValidityFailure`] not accepted by `options`.
///
/// # Panics
///
/// Panics if a custom ring implementation passes validation with a non-empty
/// point iterator but yields no point when iterated again immediately after.
#[inline]
#[must_use = "validity failures must be handled"]
pub fn is_valid_polygon_with<G, P>(
    polygon: &G,
    options: ValidityOptions,
) -> Result<(), ValidityFailure>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    validate_ring(polygon.exterior(), false, options)?;
    let inners: Vec<_> = polygon.interiors().collect();
    for inner in &inners {
        validate_ring(*inner, true, options)?;
        let rep = inner
            .points()
            .next()
            .expect("a validated ring contains at least four points");
        if !WithinRing.covered_by(rep, polygon.exterior()) {
            return Err(ValidityFailure::InteriorRingOutside);
        }
        let interaction = ring_pair_interaction(polygon.exterior(), *inner);
        if interaction.proper_crossing {
            return Err(ValidityFailure::SelfIntersection);
        }
        if interaction.overlap || interaction.contacts.len() > 1 {
            return Err(ValidityFailure::DisconnectedInterior);
        }
    }

    for first in 0..inners.len() {
        for second in (first + 1)..inners.len() {
            let interaction = ring_pair_interaction(inners[first], inners[second]);
            if interaction.proper_crossing || interaction.overlap {
                return Err(ValidityFailure::SelfIntersection);
            }
            if interaction.contacts.len() > 1 {
                return Err(ValidityFailure::DisconnectedInterior);
            }
            let nested = interaction.contacts.is_empty()
                && (ring_first_point_within(inners[first], inners[second])
                    || ring_first_point_within(inners[second], inners[first]));
            (!nested)
                .then_some(())
                .ok_or(ValidityFailure::NestedInteriorRings)?;
        }
    }
    Ok(())
}

#[derive(Default)]
struct RingPairInteraction<P> {
    proper_crossing: bool,
    overlap: bool,
    contacts: Vec<P>,
}

fn ring_pair_interaction<R1, R2, P>(first: &R1, second: &R2) -> RingPairInteraction<P>
where
    R1: RingTrait<Point = P>,
    R2: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let first_points: Vec<P> = first.points().copied().collect();
    let second_points: Vec<P> = second.points().copied().collect();
    let mut interaction = RingPairInteraction::default();
    for first_pair in first_points.windows(2) {
        let first_segment = Segment::new(first_pair[0], first_pair[1]);
        for second_pair in second_points.windows(2) {
            let second_segment = Segment::new(second_pair[0], second_pair[1]);
            match segment_intersection(&first_segment, &second_segment) {
                SegmentIntersection::Disjoint | SegmentIntersection::OutOfRange => {}
                SegmentIntersection::Collinear { .. } => interaction.overlap = true,
                SegmentIntersection::Single(point) => {
                    let endpoint_contact = first_pair.iter().any(|value| same_point(value, &point))
                        || second_pair.iter().any(|value| same_point(value, &point));
                    if !endpoint_contact {
                        interaction.proper_crossing = true;
                    }
                    if !interaction
                        .contacts
                        .iter()
                        .any(|value| same_point(value, &point))
                    {
                        interaction.contacts.push(point);
                    }
                }
            }
        }
    }
    interaction
}

fn ring_first_point_within<R1, R2, P>(inner: &R1, outer: &R2) -> bool
where
    R1: RingTrait<Point = P>,
    R2: RingTrait<Point = P>,
    P: Point,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    inner
        .points()
        .next()
        .is_some_and(|point| WithinRing.within(point, outer))
}

/// Whether any two non-adjacent edges of the vertex ring intersect. The
/// closing edge (last→first) is represented by the repeated final
/// vertex, so edges are the `pts[i] → pts[i+1]` pairs.
fn has_self_intersection<P>(pts: &[P]) -> bool
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let n = pts.len();
    // Edges: 0..n-1 (the last vertex repeats the first, closing the ring).
    let edges = n - 1;
    for i in 0..edges {
        let a = Segment::new(pts[i], pts[i + 1]);
        for j in (i + 1)..edges {
            // Skip edges that share a vertex (adjacent, or the
            // wrap-around pair of the first and last edge).
            if j == i + 1 {
                continue;
            }
            if i == 0 && j == edges - 1 {
                continue;
            }
            let b = Segment::new(pts[j], pts[j + 1]);
            match segment_intersection::<Segment<P>, P>(&a, &b) {
                SegmentIntersection::Disjoint | SegmentIntersection::OutOfRange => {}
                _ => return true,
            }
        }
    }
    false
}

fn same_point<P: Point>(a: &P, b: &P) -> bool
where
    P::Scalar: PartialEq,
{
    a.get::<0>() == b.get::<0>() && a.get::<1>() == b.get::<1>()
}

/// `true` iff `b` is a spike between `a` and `c`: collinear
/// (`cross == 0`) and folding back (`dot < 0`).
///
/// Deliberately **stricter** than
/// `geometry_algorithm::remove_spikes::is_spike_or_equal_2d`, which also
/// fires on a zero-length step. `remove_spikes` drops a repeated vertex;
/// `is_valid` does not reject one — Boost's default policy accepts
/// duplicates (see [`ValidityOptions::BOOST_DEFAULT`]), and a ring
/// carrying one is valid until `allow_duplicates` is turned off. The two
/// predicates answer different questions, so they are not shared.
fn is_spike_triple<P: Point>(a: &P, b: &P, c: &P) -> bool
where
    P::Scalar: CoordinateScalar,
{
    let ux = b.get::<0>() - a.get::<0>();
    let uy = b.get::<1>() - a.get::<1>();
    let vx = c.get::<0>() - b.get::<0>();
    let vy = c.get::<1>() - b.get::<1>();
    let zero = <P::Scalar as CoordinateScalar>::ZERO;
    ux * vy - uy * vx == zero && ux * vx + uy * vy < zero
}

/// Any spike anywhere on the closed ring cycle, seam included.
/// `pts` is the stored sequence (closing duplicate present — the
/// closure check has already passed). The walk drops the duplicate
/// and indexes the remaining cycle modularly, so triples
/// `(last-1, last, first)` and `(last, first, second)` are covered.
fn has_spike<P: Point + Copy>(pts: &[P]) -> bool
where
    P::Scalar: CoordinateScalar,
{
    // pts.len() >= 4 and pts[0] == pts[len-1] are guaranteed by the
    // earlier FewPoints / NotClosed checks.
    let cycle = &pts[..pts.len() - 1];
    let n = cycle.len(); // >= 3
    (0..n).any(|i| is_spike_triple(&cycle[(i + n - 1) % n], &cycle[i], &cycle[(i + 1) % n]))
}

#[cfg(test)]
mod tests {
    //! OVL6.T4 done-when: valid / invalid rings and polygons. Mirrors
    //! the case families in `test/algorithms/is_valid.cpp`.

    use super::{ValidityFailure, is_valid_polygon, is_valid_ring};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, Ring, polygon};

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn valid_square_ring() {
        let r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.0, 1.0),
            P::new(1.0, 1.0),
            P::new(1.0, 0.0),
            P::new(0.0, 0.0),
        ]);
        assert!(is_valid_ring(&r).is_ok());
    }

    #[test]
    fn too_few_points() {
        let r: Ring<P> = Ring::from_vec(vec![P::new(0.0, 0.0), P::new(1.0, 0.0), P::new(0.0, 0.0)]);
        assert_eq!(is_valid_ring(&r), Err(ValidityFailure::FewPoints));
    }

    #[test]
    fn out_of_range_self_intersection_is_not_reported_valid() {
        // Regression: a self-crossing "bow-tie" ring at coordinates past
        // ±2^26 had its crossing dropped as OutOfRange by the segment
        // kernel, so `has_self_intersection` returned false and the ring
        // was wrongly reported valid. The same shape in range is correctly
        // SelfIntersection; out of range it must be CoordinateOutOfRange,
        // never Ok.
        let s = 2.0e14;
        let huge_bowtie: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(s, s),
            P::new(s, 0.0),
            P::new(0.0, s),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(
            is_valid_ring(&huge_bowtie),
            Err(ValidityFailure::CoordinateOutOfRange)
        );
        // The in-range analogue is still caught — as WrongOrientation,
        // which is what Boost 1.83 reports for it: a bow-tie has zero
        // signed area, so it fails the orientation test before the
        // self-intersection test is ever reached.
        let small_bowtie: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(
            is_valid_ring(&small_bowtie),
            Err(ValidityFailure::WrongOrientation)
        );
    }

    #[test]
    fn not_closed() {
        let r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(1.0, 0.0),
            P::new(1.0, 1.0),
            P::new(0.0, 1.0),
        ]);
        assert_eq!(is_valid_ring(&r), Err(ValidityFailure::NotClosed));
    }

    #[test]
    fn self_intersecting_bowtie() {
        // A "bow-tie" quadrilateral whose diagonals cross. Its two lobes
        // cancel, so its signed area is zero and Boost reports the
        // orientation failure rather than the crossing:
        //
        //   bowtie (zero area)  valid=0 failure=22  area=+0
        let r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(is_valid_ring(&r), Err(ValidityFailure::WrongOrientation));
    }

    /// The order the ring checks report in, pinned against Boost 1.83.
    ///
    /// Boost runs spikes, then orientation, then self-intersection, and
    /// stops at the first failure. Getting the last two the wrong way round
    /// is not cosmetic: a caller that branches on the code — tilemaker's
    /// `buildWayGeometry` re-clips on `failure_self_intersections` but not
    /// on `failure_wrong_orientation` — takes a different path.
    #[test]
    fn orientation_is_reported_before_self_intersection() {
        // ccw + selfint no spike   valid=0 failure=22  area=-17
        let wound_wrong_and_crossing: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(4.0, 0.0),
            P::new(4.0, 4.0),
            P::new(0.0, 4.0),
            P::new(0.0, 0.0),
            P::new(1.0, -1.0),
            P::new(3.0, -3.0),
            P::new(1.0, -3.0),
            P::new(3.0, -1.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(
            is_valid_ring(&wound_wrong_and_crossing),
            Err(ValidityFailure::WrongOrientation)
        );

        // cw self-int, +area       valid=0 failure=21  area=+98
        let wound_right_and_crossing: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.0, 10.0),
            P::new(10.0, 10.0),
            P::new(10.0, 0.0),
            P::new(0.0, 0.0),
            P::new(3.0, -4.0),
            P::new(7.0, -4.0),
            P::new(3.0, -8.0),
            P::new(7.0, -8.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(
            is_valid_ring(&wound_right_and_crossing),
            Err(ValidityFailure::SelfIntersection)
        );

        // ccw + real spike         valid=0 failure=12  area=-16
        // Spikes still win over orientation.
        let wound_wrong_with_spike: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
            P::new(2.0, -2.0),
            P::new(2.0, 0.0),
            P::new(4.0, 0.0),
            P::new(4.0, 4.0),
            P::new(0.0, 4.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(
            is_valid_ring(&wound_wrong_with_spike),
            Err(ValidityFailure::Spikes)
        );
    }

    #[test]
    fn invalid_coordinate() {
        let r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(f64::NAN, 0.0),
            P::new(1.0, 1.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(is_valid_ring(&r), Err(ValidityFailure::InvalidCoordinate));
    }

    #[test]
    fn valid_polygon() {
        let pg: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)]];
        assert!(is_valid_polygon(&pg).is_ok());
    }

    #[test]
    fn valid_polygon_with_hole() {
        let pg: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (0.0, 0.0)
            ],
            [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0), (2.0, 2.0)]
        ];
        assert!(is_valid_polygon(&pg).is_ok());
    }

    #[test]
    fn wrongly_oriented_ring_is_rejected() {
        // CW-declared ring stored counter-clockwise. Boost:
        // failure_wrong_orientation.
        let r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
            P::new(2.0, 2.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(is_valid_ring(&r), Err(ValidityFailure::WrongOrientation));
    }

    #[test]
    fn ccw_declared_ring_correctly_wound_is_ok() {
        // CCW-declared ring stored counter-clockwise: strategy-level
        // area positive, valid. Locks the convention shared with
        // `correct()` (spec correct-orientation).
        let r: Ring<P, false> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
            P::new(2.0, 2.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ]);
        assert!(is_valid_ring(&r).is_ok());
    }

    #[test]
    fn all_collinear_ring_is_spikes() {
        // The finding's repro: a "ring" that is a line. Every edge
        // pair is adjacent or the wrap pair, so the old validator
        // reported Ok. Boost: failure_spikes.
        let flat: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(4.0, 0.0),
            P::new(2.0, 0.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(is_valid_ring(&flat), Err(ValidityFailure::Spikes));
    }

    #[test]
    fn square_with_spike_is_spikes() {
        // A CW square with an out-and-back spur on its bottom edge.
        // Both Spikes and SelfIntersection are arguably present; the
        // pipeline order pins Spikes (matches Boost's check order).
        let r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.0, 4.0),
            P::new(4.0, 4.0),
            P::new(4.0, 0.0),
            P::new(2.0, 0.0),
            P::new(2.0, -2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(is_valid_ring(&r), Err(ValidityFailure::Spikes));
    }

    #[test]
    fn hole_outside_exterior_is_rejected() {
        // The finding's repro. Doc promised InteriorRingOutside; the
        // variant was unreachable. Boost:
        // failure_interior_rings_outside. (Exterior CW-stored, hole
        // CCW-stored — both correctly wound, so orientation passes and
        // containment is what fails.)
        let pg: Polygon<P> = polygon![
            [(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)],
            [
                (10.0, 10.0),
                (12.0, 10.0),
                (12.0, 12.0),
                (10.0, 12.0),
                (10.0, 10.0)
            ]
        ];
        assert_eq!(
            is_valid_polygon(&pg),
            Err(ValidityFailure::InteriorRingOutside)
        );
    }

    #[test]
    fn hole_touching_exterior_boundary_is_ok() {
        // The hole's first vertex lies ON the exterior boundary:
        // covered_by (not within) is the containment predicate, so an
        // isolated touch is permitted — matching Boost.
        let pg: Polygon<P> = polygon![
            [(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)],
            [(0.0, 2.0), (1.0, 1.0), (1.0, 3.0), (0.0, 2.0)]
        ];
        assert!(is_valid_polygon(&pg).is_ok());
    }

    #[test]
    fn wrongly_oriented_hole_is_rejected() {
        // Correct CW exterior, but the hole is ALSO CW-stored — holes
        // must wind opposite. Boost: failure_wrong_orientation.
        let pg: Polygon<P> = polygon![
            [(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)],
            [(1.0, 1.0), (1.0, 2.0), (2.0, 2.0), (2.0, 1.0), (1.0, 1.0)]
        ];
        assert_eq!(
            is_valid_polygon(&pg),
            Err(ValidityFailure::WrongOrientation)
        );
    }
}
