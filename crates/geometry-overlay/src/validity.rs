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
    Closure, Geometry, MultiPolygon as MultiPolygonTrait, Point, PointMut, Polygon as PolygonTrait,
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
/// This is the [`ValidityOptions::STRICT`] policy, stricter than Boost's
/// default: consecutive repeated points are reported as
/// [`ValidityFailure::DuplicatePoints`], where Boost's
/// `is_valid_default_policy` accepts them. [`is_valid_with`] with
/// [`ValidityOptions::BOOST_DEFAULT`] answers as Boost's `is_valid` does.
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
/// driven by `policies/is_valid/failing_reason_policy.hpp`. Like
/// [`is_valid`], it reports consecutive repeated points, which Boost's
/// default policy accepts; [`validity_reason_with`] takes the policy.
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
        // `is_valid_multipolygon` (`detail/is_valid/multipolygon.hpp`) runs
        // each phase over every member before starting the next: all rings,
        // then where rings meet — within a member or across two — then each
        // member's holes, then each member's interior, and only then whether
        // one member lies inside another. So a member with too few points
        // is reported ahead of an earlier member crossing itself, and a
        // nesting only once no pair of members meets badly. Boost:
        //
        //   overlapping members     failure=21  boundaries cross
        //   edge-touching members   failure=21  boundaries share a curve
        //   identical members       failure=21
        //   one inside another      failure=40  interiors overlap, boundaries do not meet
        //   point-touching members  valid
        //   disjoint members        valid
        //
        // tilemaker's `buildWayGeometry` branches on 21 against 40.
        let polygons: Vec<_> = multi_polygon.polygons().collect();
        let views = polygons
            .iter()
            .map(|polygon| polygon_views(*polygon, options))
            .collect::<Result<Vec<_>, _>>()?;
        let touching = views
            .iter()
            .map(|rings| polygon_contacts(rings))
            .collect::<Result<Vec<_>, _>>()?;
        let scaled: Vec<Vec<_>> = views
            .iter()
            .map(|rings| rings.iter().map(|ring| scaled_ring(ring)).collect())
            .collect();
        let mut pairs = Vec::new();
        for first in 0..views.len() {
            for second in (first + 1)..views.len() {
                let meeting = members_meet(
                    &views[first],
                    &scaled[first],
                    &views[second],
                    &scaled[second],
                )?;
                pairs.push((first, second, meeting));
            }
        }
        for (polygon, touching) in polygons.iter().zip(&touching) {
            holes_inside(*polygon, touching)?;
        }
        for (rings, touching) in views.iter().zip(&touching) {
            interior_connected(rings.len(), touching)?;
        }
        // Boost asks this only of members whose every turn with another
        // member is a touch; that is every pair left once the turns pass.
        let nested = pairs
            .into_iter()
            .any(|(first, second, meeting)| match meeting {
                Members::Touching { nested } => nested,
                Members::Apart => members_nested(
                    &views[first],
                    &scaled[first],
                    &views[second],
                    &scaled[second],
                ),
            });
        if nested {
            return Err(ValidityFailure::IntersectingInteriors);
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
    let view = ring_view(ring, options)?;
    // A ring on its own is searched for crossings before its orientation
    // is judged (`is_valid_ring<Ring, true>` in `detail/is_valid/ring.hpp`),
    // so a bow-tie, whose lobes cancel to zero area, is a self-intersection.
    // A polygon checks every ring's orientation first; see
    // `is_valid_polygon_with`.
    if has_self_intersection(&view) {
        return Err(ValidityFailure::SelfIntersection);
    }
    check_orientation(ring, false)
}

/// Run the checks a ring passes on its own, in Boost's order
/// (`detail/is_valid/ring.hpp`, `is_valid_ring`), and return the ring as
/// Boost's `closed_view` presents it, with consecutive repeats dropped.
///
/// Orientation and self-intersection are left to the caller: a lone ring
/// and a polygon's rings meet them in different orders.
fn ring_view<R, P>(ring: &R, options: ValidityOptions) -> Result<Vec<P>, ValidityFailure>
where
    R: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let mut view: Vec<P> = ring.points().copied().collect();

    // Coordinate finiteness.
    for p in &view {
        let x: f64 = p.get::<0>().into();
        let y: f64 = p.get::<1>().into();
        if !x.is_finite() || !y.is_finite() {
            return Err(ValidityFailure::InvalidCoordinate);
        }
    }

    // Out-of-range coordinates: the self-intersection test routes each
    // edge pair through the segment-intersection kernel, which drops any
    // crossing at coordinates past ±SAFE_ABS_MAX as `OutOfRange`. A
    // genuinely self-intersecting ring would then pass unnoticed, so
    // validity cannot be confirmed — refuse rather than claim valid.
    for p in &view {
        if !coordinate_in_range(p) {
            return Err(ValidityFailure::CoordinateOutOfRange);
        }
    }

    // `minimum_ring_size`: a triangle is three stored points when the
    // closing point is implicit, four when it is repeated.
    let open = matches!(ring.closure(), Closure::Open);
    if view.len() < if open { 3 } else { 4 } {
        return Err(ValidityFailure::FewPoints);
    }
    if open {
        view.push(view[0]);
    }

    // Fewer than four distinct consecutive points around the closed ring
    // enclose no area: the ring is a point or a line.
    let mut distinct = view.clone();
    distinct.dedup_by(|current, previous| same_point(previous, current));
    if distinct.len() < 4 {
        return Err(ValidityFailure::WrongTopologicalDimension);
    }

    // `is_topologically_closed`: an open ring is closed by its view.
    if !open && !same_point(&view[0], &view[view.len() - 1]) {
        return Err(ValidityFailure::NotClosed);
    }

    // Boost's default policy accepts consecutive duplicates, the closing
    // edge's included; the strict policy rejects them.
    if !options.allows_duplicates() && view.windows(2).any(|pair| same_point(&pair[0], &pair[1])) {
        return Err(ValidityFailure::DuplicatePoints);
    }

    // A vertex triple that folds back on itself along one line — Boost's
    // failure_spikes.
    if has_spike(&distinct) {
        return Err(ValidityFailure::Spikes);
    }

    Ok(distinct)
}

/// Boost's `is_properly_oriented<Ring, IsInteriorRing>`. The strategy area
/// already folds the declared `PointOrder`: a correctly wound exterior is
/// positive, a correctly wound hole negative. Zero area (degenerate) fails
/// either way.
fn check_orientation<R, P>(ring: &R, is_interior: bool) -> Result<(), ValidityFailure>
where
    R: RingTrait<Point = P>,
    P: Point,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let area = ShoelaceArea.area(ring);
    let zero = <P::Scalar as CoordinateScalar>::ZERO.to_measure();
    let properly_oriented = if is_interior {
        area < zero
    } else {
        area > zero
    };
    if properly_oriented {
        Ok(())
    } else {
        Err(ValidityFailure::WrongOrientation)
    }
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
    let views = polygon_views(polygon, options)?;
    let touching = polygon_contacts(&views)?;
    holes_inside(polygon, &touching)?;
    interior_connected(views.len(), &touching)
}

/// The views of a polygon's rings once each has passed its own checks —
/// the exterior's first, then each hole's in order.
///
/// Every ring is checked before any two are compared (`has_valid_rings` in
/// `detail/is_valid/polygon.hpp`), so a later hole with too few points is
/// reported ahead of an earlier hole crossing the exterior, and a wrongly
/// wound ring ahead of a crossing.
fn polygon_views<G, P>(
    polygon: &G,
    options: ValidityOptions,
) -> Result<Vec<Vec<P>>, ValidityFailure>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let exterior = polygon.exterior();
    let mut views = alloc::vec![ring_view(exterior, options)?];
    check_orientation(exterior, false)?;
    for inner in polygon.interiors() {
        views.push(ring_view(inner, options)?);
        check_orientation(inner, true)?;
    }
    Ok(views)
}

/// Pairs of one polygon's rings that touch, by index into its views
/// (exterior first), each with the points where they do.
type Touches<P> = Vec<(usize, usize, Vec<P>)>;

/// `has_valid_self_turns` for one polygon: where its rings meet.
///
/// A ring meeting itself, two rings crossing or sharing a stretch, and two
/// rings touching the wrong way round are all self-intersections. A hole
/// may touch the exterior only from inside it, and two holes only from
/// outside each other — a hole touching the exterior from outside, or
/// touching a hole it lies in, fails here rather than as an outside or a
/// nested ring. Boost, exterior (0,0)-(10,10) clockwise:
///
/// ```text
///   hole fully interior                          valid
///   hole touching the exterior at one point      valid
///   hole outside, touching it at one point       failure=21
///   hole sharing a segment of it                 failure=21
/// ```
fn polygon_contacts<P>(views: &[Vec<P>]) -> Result<Touches<P>, ValidityFailure>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    if views.iter().any(|view| has_self_intersection(view)) {
        return Err(ValidityFailure::SelfIntersection);
    }
    let scaled: Vec<_> = views.iter().map(|view| scaled_ring(view)).collect();
    let mut touching = Vec::new();
    for first in 0..views.len() {
        for second in (first + 1)..views.len() {
            let contacts = ring_contacts(&views[first], &views[second])?;
            if contacts.points.is_empty() {
                continue;
            }
            let side = |ring: usize, splits: &[Vec<P>], other: usize| {
                stretches_side(
                    [(views[ring].as_slice(), splits)],
                    core::slice::from_ref(&scaled[other]),
                )
            };
            let acceptable = if first == 0 {
                side(second, &contacts.second_splits, 0) == Some(true)
            } else {
                side(second, &contacts.second_splits, first) == Some(false)
                    && side(first, &contacts.first_splits, second) == Some(false)
            };
            if !acceptable {
                return Err(ValidityFailure::SelfIntersection);
            }
            touching.push((first, second, contacts.points));
        }
    }
    Ok(touching)
}

/// Whether rings `first` and `second` of one polygon touch.
fn touch<P>(touching: &Touches<P>, first: usize, second: usize) -> bool {
    touching
        .iter()
        .any(|(a, b, _)| (*a, *b) == (first, second) || (*a, *b) == (second, first))
}

/// `are_holes_inside`: every hole inside the exterior, and no hole inside
/// another.
///
/// A hole that meets the exterior nowhere lies wholly inside it or wholly
/// outside, so its first point decides. Two holes that do not meet are
/// nested when either one's first point lies inside the other. Boost asks
/// that only of holes that meet no ring at all, so it passes a hole nested
/// in another as long as the nested one touches some third ring; this asks
/// it of every pair that does not meet.
fn holes_inside<G, P>(polygon: &G, touching: &Touches<P>) -> Result<(), ValidityFailure>
where
    G: PolygonTrait<Point = P>,
    P: Point,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let exterior = polygon.exterior();
    let inners: Vec<_> = polygon.interiors().collect();
    for (index, inner) in inners.iter().enumerate() {
        if !touch(touching, 0, index + 1)
            && !inner
                .points()
                .next()
                .is_some_and(|point| WithinRing.covered_by(point, exterior))
        {
            return Err(ValidityFailure::InteriorRingOutside);
        }
    }
    for first in 0..inners.len() {
        for second in (first + 1)..inners.len() {
            if !touch(touching, first + 1, second + 1)
                && (ring_first_point_within(inners[first], inners[second])
                    || ring_first_point_within(inners[second], inners[first]))
            {
                return Err(ValidityFailure::NestedInteriorRings);
            }
        }
    }
    Ok(())
}

/// `has_connected_interior`: rings touching in a loop fence off part of the
/// interior — a hole touching the exterior twice, or three holes touching
/// pairwise.
fn interior_connected<P>(rings: usize, touching: &Touches<P>) -> Result<(), ValidityFailure>
where
    P: Point + Copy,
    P::Scalar: CoordinateScalar,
{
    if touches_close_a_loop(rings, touching) {
        Err(ValidityFailure::DisconnectedInterior)
    } else {
        Ok(())
    }
}

/// How two members of a multi-polygon that pass `members_meet` relate.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Members {
    /// Their boundaries do not meet.
    Apart,
    /// Their boundaries touch at points, with one member inside the other
    /// (`nested`) or each outside the other.
    Touching { nested: bool },
}

/// How two members of a multi-polygon meet, failing with a
/// self-intersection where their boundaries cross or share a stretch.
///
/// Boost accepts any turn between members at which the boundaries touch
/// without crossing (`touch_only` in `is_acceptable_turn.hpp`), from
/// outside or from inside alike, and leaves a member touching another from
/// inside to the nesting check. Every stretch of one member's rings between
/// contacts therefore lies on one side of the other member: mixed sides are
/// a crossing.
fn members_meet<P>(
    first: &[Vec<P>],
    first_scaled: &[geometry_model::Ring<P>],
    second: &[Vec<P>],
    second_scaled: &[geometry_model::Ring<P>],
) -> Result<Members, ValidityFailure>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let per_edge = |rings: &[Vec<P>]| -> Vec<Vec<Vec<P>>> {
        rings
            .iter()
            .map(|ring| alloc::vec![Vec::new(); ring.len() - 1])
            .collect()
    };
    let (mut first_splits, mut second_splits) = (per_edge(first), per_edge(second));
    let mut touching = false;
    for (first_ring, first_view) in first.iter().enumerate() {
        for (second_ring, second_view) in second.iter().enumerate() {
            let contacts = ring_contacts(first_view, second_view)?;
            if contacts.points.is_empty() {
                continue;
            }
            touching = true;
            for (held, found) in first_splits[first_ring]
                .iter_mut()
                .zip(contacts.first_splits)
            {
                held.extend(found);
            }
            for (held, found) in second_splits[second_ring]
                .iter_mut()
                .zip(contacts.second_splits)
            {
                held.extend(found);
            }
        }
    }
    if !touching {
        return Ok(Members::Apart);
    }
    let side = |rings: &[Vec<P>], splits: &[Vec<Vec<P>>], other| {
        stretches_side(
            rings
                .iter()
                .map(Vec::as_slice)
                .zip(splits.iter().map(Vec::as_slice)),
            other,
        )
        .ok_or(ValidityFailure::SelfIntersection)
    };
    let first_inside = side(first, &first_splits, second_scaled)?;
    let second_inside = side(second, &second_splits, first_scaled)?;
    Ok(Members::Touching {
        nested: first_inside || second_inside,
    })
}

/// Whether either of two members whose boundaries do not meet lies in the
/// other's interior. Then the first point of either exterior decides for
/// its whole member.
fn members_nested<P>(
    first: &[Vec<P>],
    first_scaled: &[geometry_model::Ring<P>],
    second: &[Vec<P>],
    second_scaled: &[geometry_model::Ring<P>],
) -> bool
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let inside = |rings: &[Vec<P>], other: &[geometry_model::Ring<P>]| {
        locate(&point_sum(&rings[0][0], &rings[0][0]), other) == Some(true)
    };
    inside(first, second_scaled) || inside(second, first_scaled)
}

/// Where two distinct rings of one polygon meet: Boost's turns between
/// them. A crossing or a shared stretch is a self-intersection outright;
/// what remains are isolated contacts, each recorded against the edges it
/// lies on so both rings can be cut into stretches that meet the other
/// ring only at their ends.
struct RingContacts<P> {
    /// The distinct contact points.
    points: Vec<P>,
    /// For each edge of the first ring, the contacts on it.
    first_splits: Vec<Vec<P>>,
    /// For each edge of the second ring, the contacts on it.
    second_splits: Vec<Vec<P>>,
}

/// Collect the contacts between two ring views (each closed, repeats
/// dropped).
fn ring_contacts<P>(first: &[P], second: &[P]) -> Result<RingContacts<P>, ValidityFailure>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let mut contacts = RingContacts {
        points: Vec::new(),
        first_splits: alloc::vec![Vec::new(); first.len() - 1],
        second_splits: alloc::vec![Vec::new(); second.len() - 1],
    };
    for (first_edge, first_pair) in first.windows(2).enumerate() {
        let first_segment = Segment::new(first_pair[0], first_pair[1]);
        for (second_edge, second_pair) in second.windows(2).enumerate() {
            let second_segment = Segment::new(second_pair[0], second_pair[1]);
            match segment_intersection(&first_segment, &second_segment) {
                SegmentIntersection::Disjoint | SegmentIntersection::OutOfRange => {}
                SegmentIntersection::Collinear { .. } => {
                    return Err(ValidityFailure::SelfIntersection);
                }
                SegmentIntersection::Single(point) => {
                    if !first_pair
                        .iter()
                        .chain(second_pair)
                        .any(|end| same_point(end, &point))
                    {
                        return Err(ValidityFailure::SelfIntersection);
                    }
                    contacts.first_splits[first_edge].push(point);
                    contacts.second_splits[second_edge].push(point);
                    if !contacts.points.iter().any(|seen| same_point(seen, &point)) {
                        contacts.points.push(point);
                    }
                }
            }
        }
    }
    Ok(contacts)
}

/// The side of `other` on which every stretch of `rings` between its
/// contacts with `other` lies: `Some(true)` strictly inside, `Some(false)`
/// strictly outside, and `None` when stretches lie on different sides — the
/// rings cross `other` there — or one runs along its boundary.
///
/// Each ring comes with its contacts per edge. `other` is a polygon's
/// rings, exterior first, scaled by two ([`scaled_ring`]). A stretch meets
/// `other` only at its ends, so its midpoint decides for all of it; the
/// midpoint is tested as the sum of the ends, which divides nothing and so
/// stays exact for integer coordinates as well.
fn stretches_side<'a, P>(
    rings: impl IntoIterator<Item = (&'a [P], &'a [Vec<P>])>,
    other: &[geometry_model::Ring<P>],
) -> Option<bool>
where
    P: PointMut + Default + Copy + 'a,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let mut side = None;
    for (ring, splits) in rings {
        for (edge, contacts) in ring.windows(2).zip(splits) {
            let mut stops = Vec::with_capacity(contacts.len() + 2);
            stops.push(edge[0]);
            stops.extend(sorted_along(&edge[0], &edge[1], contacts));
            stops.push(edge[1]);
            for stretch in stops.windows(2) {
                if same_point(&stretch[0], &stretch[1]) {
                    continue;
                }
                let here = locate(&point_sum(&stretch[0], &stretch[1]), other)?;
                if *side.get_or_insert(here) != here {
                    return None;
                }
            }
        }
    }
    side
}

/// A ring view with every coordinate doubled, for [`stretches_side`].
fn scaled_ring<P>(view: &[P]) -> geometry_model::Ring<P>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    geometry_model::Ring::from_vec(view.iter().map(|point| point_sum(point, point)).collect())
}

/// Where `point` lies against the polygon whose rings are `rings`, exterior
/// first: `Some(true)` in its interior, `Some(false)` outside it, `None` on
/// one of its rings.
fn locate<P>(point: &P, rings: &[geometry_model::Ring<P>]) -> Option<bool>
where
    P: Point,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let (exterior, holes) = rings.split_first()?;
    if !WithinRing.within(point, exterior) {
        return (!WithinRing.covered_by(point, exterior)).then_some(false);
    }
    for hole in holes {
        if WithinRing.covered_by(point, hole) {
            return WithinRing.within(point, hole).then_some(false);
        }
    }
    Some(true)
}

/// `points`, all on the segment from `start` to `end`, in order from
/// `start`.
fn sorted_along<P>(start: &P, end: &P, points: &[P]) -> Vec<P>
where
    P: Point + Copy,
    P::Scalar: CoordinateScalar,
{
    let along_x =
        (end.get::<0>() - start.get::<0>()).abs() >= (end.get::<1>() - start.get::<1>()).abs();
    let key = |point: &P| {
        if along_x {
            point.get::<0>()
        } else {
            point.get::<1>()
        }
    };
    let forward = key(end) >= key(start);
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| {
        let order = key(a)
            .partial_cmp(&key(b))
            .unwrap_or(core::cmp::Ordering::Equal);
        if forward { order } else { order.reverse() }
    });
    sorted
}

/// The coordinate-wise sum of two points.
fn point_sum<P>(a: &P, b: &P) -> P
where
    P: PointMut + Default,
    P::Scalar: CoordinateScalar,
{
    let mut sum = P::default();
    sum.set::<0>(a.get::<0>() + b.get::<0>());
    sum.set::<1>(a.get::<1>() + b.get::<1>());
    sum
}

/// Boost's `complement_graph::has_cycles`
/// (`detail/is_valid/complement_graph.hpp`): rings and contact points are
/// the nodes, each ring joined to every point where it touches another
/// ring. A cycle is a loop of touching rings, which fences off part of the
/// interior.
fn touches_close_a_loop<P>(rings: usize, touching: &[(usize, usize, Vec<P>)]) -> bool
where
    P: Point + Copy,
    P::Scalar: CoordinateScalar,
{
    let mut points: Vec<P> = Vec::new();
    let mut links = Vec::new();
    for (first, second, contacts) in touching {
        for contact in contacts {
            let node = rings
                + points
                    .iter()
                    .position(|seen| same_point(seen, contact))
                    .unwrap_or_else(|| {
                        points.push(*contact);
                        points.len() - 1
                    });
            links.push((*first, node));
            links.push((*second, node));
        }
    }
    links.sort_unstable();
    links.dedup();

    let mut parent: Vec<usize> = (0..rings + points.len()).collect();
    let root = |parent: &mut Vec<usize>, mut node: usize| {
        while parent[node] != node {
            parent[node] = parent[parent[node]];
            node = parent[node];
        }
        node
    };
    for (ring, point) in links {
        let (ring_root, point_root) = (root(&mut parent, ring), root(&mut parent, point));
        if ring_root == point_root {
            return true;
        }
        parent[ring_root] = point_root;
    }
    false
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

/// Whether any two non-adjacent edges of the ring view intersect. The
/// closing edge (last→first) is represented by the view's closing point,
/// so edges are the `pts[i] → pts[i+1]` pairs.
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

/// Boost's `equals_point_point`: both coordinates equal under
/// `math::equals`, the tolerance [`CoordinateScalar::tolerant_eq`] mirrors.
fn same_point<P: Point>(a: &P, b: &P) -> bool
where
    P::Scalar: CoordinateScalar,
{
    a.get::<0>().tolerant_eq(b.get::<0>()) && a.get::<1>().tolerant_eq(b.get::<1>())
}

/// `true` iff `b` is a spike between `a` and `c`: collinear by Boost's
/// side test and folding back (`dot < 0`).
///
/// C++: `has_spikes` calls `is_spike_or_equal(next, cur, prev)`, whose side
/// test is the relate strategy's `side_by_triangle`; the walk has already
/// skipped every neighbour `math::equals` to the vertex, so no step has
/// zero length and a fold back is a strictly negative dot product.
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
    let (a, b, c) = (
        (a.get::<0>(), a.get::<1>()),
        (b.get::<0>(), b.get::<1>()),
        (c.get::<0>(), c.get::<1>()),
    );
    P::Scalar::side_by_triangle(c, b, a) == core::cmp::Ordering::Equal
        && P::Scalar::dot_sign(a, b, b, c) == Some(core::cmp::Ordering::Less)
}

/// Any spike anywhere on the closed ring cycle, seam included.
/// `pts` is the ring view (closing point present, repeats dropped). The
/// walk drops the closing point and indexes the remaining cycle
/// modularly, so triples `(last-1, last, first)` and
/// `(last, first, second)` are covered.
fn has_spike<P: Point + Copy>(pts: &[P]) -> bool
where
    P::Scalar: CoordinateScalar,
{
    // pts.len() >= 4 and a closing point equal to the first are
    // guaranteed by the earlier topological-dimension and closure checks.
    let cycle = &pts[..pts.len() - 1];
    let n = cycle.len(); // >= 3
    (0..n).any(|i| is_spike_triple(&cycle[(i + n - 1) % n], &cycle[i], &cycle[(i + 1) % n]))
}

#[cfg(test)]
mod tests {
    //! OVL6.T4 done-when: valid / invalid rings and polygons. Mirrors
    //! the case families in `test/algorithms/is_valid.cpp`.

    use super::{
        ValidityFailure, ValidityOptions, is_valid_polygon, is_valid_ring, is_valid_ring_with,
    };
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
        // The in-range analogue is still caught, as the self-intersection
        // Boost reports for it.
        let small_bowtie: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(
            is_valid_ring(&small_bowtie),
            Err(ValidityFailure::SelfIntersection)
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
        // cancel to zero signed area, but a ring on its own is searched for
        // crossings before its orientation is judged; as a polygon's
        // exterior the orientation comes first. Boost:
        //
        //   bowtie ring     valid=0 failure=21
        //   bowtie polygon  valid=0 failure=22
        let r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(is_valid_ring(&r), Err(ValidityFailure::SelfIntersection));
        let pg: Polygon<P> = Polygon {
            outer: r,
            inners: vec![],
        };
        assert_eq!(
            is_valid_polygon(&pg),
            Err(ValidityFailure::WrongOrientation)
        );
    }

    /// The order the checks report in, pinned against Boost.
    ///
    /// Boost stops at the first failure. A lone ring runs spikes, then
    /// self-intersection, then orientation (`is_valid_ring<Ring, true>`); a
    /// polygon runs every ring's own checks, orientation included, before
    /// it looks for crossings (`has_valid_rings`, then
    /// `has_valid_self_turns`). Getting the order wrong is not cosmetic: a
    /// caller that branches on the code — tilemaker's `buildWayGeometry`
    /// re-clips on `failure_self_intersections` but not on
    /// `failure_wrong_orientation` — takes a different path.
    #[test]
    fn crossing_and_orientation_report_in_boost_order() {
        // ccw + selfint no spike   ring failure=21, polygon failure=22
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
            Err(ValidityFailure::SelfIntersection)
        );
        let pg: Polygon<P> = Polygon {
            outer: wound_wrong_and_crossing,
            inners: vec![],
        };
        assert_eq!(
            is_valid_polygon(&pg),
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

    /// A "hole" lying wholly outside the exterior and touching it at one
    /// vertex is a turn no valid polygon has — a hole may touch the
    /// exterior only from inside — and Boost checks turns before hole
    /// containment, so it reports a self-intersection.
    #[test]
    fn hole_outside_touching_the_exterior_at_a_vertex_is_invalid() {
        let pg: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (0.0, 0.0)
            ],
            [(10.0, 5.0), (12.0, 4.0), (12.0, 6.0), (10.0, 5.0)]
        ];
        assert_eq!(
            is_valid_polygon(&pg),
            Err(ValidityFailure::SelfIntersection)
        );
    }

    fn ring_of<const CW: bool, const CL: bool>(points: &[(f64, f64)]) -> Ring<P, CW, CL> {
        Ring::from_vec(points.iter().map(|&(x, y)| P::new(x, y)).collect())
    }

    fn square_exterior() -> Ring<P> {
        ring_of(&[
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0),
        ])
    }

    /// An open ring leaves its closing point implicit, so three stored
    /// points are a triangle and the closing edge is the view's
    /// (`closed_view`); Boost validates open rings and polygons through
    /// that view.
    #[test]
    fn open_rings_and_polygons_are_validated_closed() {
        let triangle: Ring<P, true, false> = ring_of(&[(0.0, 0.0), (0.0, 1.0), (1.0, 0.0)]);
        assert_eq!(is_valid_ring(&triangle), Ok(()));
        let line: Ring<P, true, false> = ring_of(&[(0.0, 0.0), (1.0, 0.0)]);
        assert_eq!(is_valid_ring(&line), Err(ValidityFailure::FewPoints));

        let polygon: Polygon<P, true, false> = Polygon {
            outer: ring_of(&[(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]),
            inners: vec![ring_of(&[(2.0, 2.0), (4.0, 2.0), (2.0, 4.0)])],
        };
        assert_eq!(is_valid_polygon(&polygon), Ok(()));

        // Repeating the first point in an open ring is a duplicate of the
        // view's closing point: accepted by Boost's default policy only.
        let repeated: Ring<P, true, false> =
            ring_of(&[(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0)]);
        assert_eq!(
            is_valid_ring_with(&repeated, ValidityOptions::BOOST_DEFAULT),
            Ok(())
        );
        assert_eq!(
            is_valid_ring(&repeated),
            Err(ValidityFailure::DuplicatePoints)
        );
    }

    /// Enough stored points but fewer than four distinct ones around the
    /// closed ring enclose no area. Boost: failure=11, ahead of the
    /// duplicate the ring also carries.
    #[test]
    fn ring_without_area_has_wrong_topological_dimension() {
        let flat: Ring<P> = ring_of(&[(0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (0.0, 0.0)]);
        for options in [ValidityOptions::STRICT, ValidityOptions::BOOST_DEFAULT] {
            assert_eq!(
                is_valid_ring_with(&flat, options),
                Err(ValidityFailure::WrongTopologicalDimension)
            );
        }
    }

    /// Points equal under `math::equals` are the same point to Boost's
    /// closure and duplicate checks.
    #[test]
    fn closure_and_duplicates_compare_with_tolerance() {
        let nearly_closed: Ring<P> = ring_of(&[
            (1.0, 1.0),
            (1.0, 11.0),
            (11.0, 11.0),
            (11.0, 1.0),
            (1.000_000_000_000_000_2, 1.0),
        ]);
        assert_eq!(is_valid_ring(&nearly_closed), Ok(()));
        let nearly_repeated: Ring<P> = ring_of(&[
            (0.0, 0.0),
            (0.0, 10.0),
            (0.0, 10.000_000_000_000_002),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0),
        ]);
        assert_eq!(
            is_valid_ring(&nearly_repeated),
            Err(ValidityFailure::DuplicatePoints)
        );
    }

    /// Every ring's own checks run before rings are compared: a later hole
    /// with too few points is reported ahead of an earlier hole crossing
    /// the exterior. Boost: failure=10.
    #[test]
    fn every_ring_is_checked_before_rings_are_compared() {
        let pg: Polygon<P> = Polygon {
            outer: square_exterior(),
            inners: vec![
                ring_of(&[
                    (8.0, 8.0),
                    (12.0, 8.0),
                    (12.0, 12.0),
                    (8.0, 12.0),
                    (8.0, 8.0),
                ]),
                ring_of(&[(2.0, 2.0), (3.0, 2.0), (2.0, 2.0)]),
            ],
        };
        assert_eq!(is_valid_polygon(&pg), Err(ValidityFailure::FewPoints));
    }

    /// Three holes touching pairwise, each at one point, fence off the
    /// triangle between them even though no two holes touch twice.
    /// Boost: failure=32.
    #[test]
    fn holes_touching_in_a_loop_disconnect_the_interior() {
        let pg: Polygon<P> = Polygon {
            outer: square_exterior(),
            inners: vec![
                ring_of(&[(4.0, 5.0), (6.0, 5.0), (6.0, 9.0), (4.0, 9.0), (4.0, 5.0)]),
                ring_of(&[(4.0, 4.0), (5.0, 4.0), (4.0, 5.0), (4.0, 4.0)]),
                ring_of(&[(5.0, 1.0), (6.0, 1.0), (5.0, 5.0), (5.0, 1.0)]),
            ],
        };
        assert_eq!(
            is_valid_polygon(&pg),
            Err(ValidityFailure::DisconnectedInterior)
        );
    }

    /// Two holes inside a third, touching each other but not it. Boost
    /// looks for nesting only among holes that touch nothing and reports
    /// this polygon valid; holes inside a hole are nested whatever else
    /// they touch.
    #[test]
    fn holes_nested_in_a_hole_are_nested_even_when_touching() {
        let pg: Polygon<P> = Polygon {
            outer: square_exterior(),
            inners: vec![
                ring_of(&[(1.0, 1.0), (9.0, 1.0), (9.0, 9.0), (1.0, 9.0), (1.0, 1.0)]),
                ring_of(&[(3.0, 3.0), (5.0, 3.0), (5.0, 5.0), (3.0, 5.0), (3.0, 3.0)]),
                ring_of(&[(5.0, 5.0), (7.0, 5.0), (7.0, 7.0), (5.0, 7.0), (5.0, 5.0)]),
            ],
        };
        assert_eq!(
            is_valid_polygon(&pg),
            Err(ValidityFailure::NestedInteriorRings)
        );
    }

    /// Multi-polygon members, against Boost: each phase runs over every
    /// member before the next, and a member touching another from inside
    /// is a nesting, not a crossing.
    #[test]
    fn multi_polygon_members_follow_boost_phases() {
        use geometry_model::MultiPolygon;
        let square: Polygon<P> = Polygon {
            outer: square_exterior(),
            inners: vec![],
        };
        let check = |members: Vec<Polygon<P>>| super::is_valid(&MultiPolygon(members));

        let touching_inside = Polygon {
            outer: ring_of(&[(2.0, 2.0), (2.0, 4.0), (10.0, 3.0), (2.0, 2.0)]),
            inners: vec![],
        };
        assert_eq!(
            check(vec![square.clone(), touching_inside]),
            Err(ValidityFailure::IntersectingInteriors)
        );

        let sharing_an_edge = Polygon {
            outer: ring_of(&[
                (10.0, 0.0),
                (10.0, 10.0),
                (20.0, 10.0),
                (20.0, 0.0),
                (10.0, 0.0),
            ]),
            inners: vec![],
        };
        assert_eq!(
            check(vec![square.clone(), sharing_an_edge]),
            Err(ValidityFailure::SelfIntersection)
        );

        let touching_a_corner = Polygon {
            outer: ring_of(&[
                (10.0, 10.0),
                (10.0, 20.0),
                (20.0, 20.0),
                (20.0, 10.0),
                (10.0, 10.0),
            ]),
            inners: vec![],
        };
        assert_eq!(check(vec![square.clone(), touching_a_corner]), Ok(()));

        // The first member's hole crosses its exterior, but the second
        // member's ring is checked first.
        let crossed = Polygon {
            outer: square_exterior(),
            inners: vec![ring_of(&[
                (8.0, 8.0),
                (12.0, 8.0),
                (12.0, 12.0),
                (8.0, 12.0),
                (8.0, 8.0),
            ])],
        };
        let too_short = Polygon {
            outer: ring_of(&[(20.0, 20.0), (20.0, 21.0), (20.0, 20.0)]),
            inners: vec![],
        };
        assert_eq!(
            check(vec![crossed, too_short]),
            Err(ValidityFailure::FewPoints)
        );
    }

    /// A ring with the right orientation that still crosses itself — a
    /// loop hanging off its bottom edge — fails as Boost's `failure=21`
    /// (`6b76894`).
    #[test]
    fn a_correctly_wound_self_crossing_exterior_is_a_self_intersection() {
        let looped: Polygon<P> = Polygon {
            outer: ring_of(&[
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (5.0, 0.0),
                (5.0, -2.0),
                (6.0, -2.0),
                (6.0, 1.0),
                (0.0, 0.0),
            ]),
            inners: vec![],
        };
        assert_eq!(
            is_valid_polygon(&looped),
            Err(ValidityFailure::SelfIntersection)
        );
    }

    /// Members meeting only at vertices can still overlap: a diamond
    /// through two corners of the square lies partly inside it and partly
    /// outside, so its boundary crosses there. Against Boost (`6b76894`):
    /// the diamond `failure=21`, an island in a hole valid, and a member in
    /// the holed polygon's material `failure=40`.
    #[test]
    fn multi_polygon_members_meeting_at_vertices_or_holes() {
        use geometry_model::MultiPolygon;
        let check = |members: Vec<Polygon<P>>| super::is_valid(&MultiPolygon(members));
        let square: Polygon<P> = Polygon {
            outer: square_exterior(),
            inners: vec![],
        };
        let diamond = Polygon {
            outer: ring_of(&[
                (0.0, 10.0),
                (5.0, 15.0),
                (10.0, 10.0),
                (5.0, 5.0),
                (0.0, 10.0),
            ]),
            inners: vec![],
        };
        assert_eq!(
            check(vec![square, diamond]),
            Err(ValidityFailure::SelfIntersection)
        );

        let holed: Polygon<P> = Polygon {
            outer: square_exterior(),
            inners: vec![ring_of(&[
                (2.0, 2.0),
                (8.0, 2.0),
                (8.0, 8.0),
                (2.0, 8.0),
                (2.0, 2.0),
            ])],
        };
        let island = Polygon {
            outer: ring_of(&[(3.0, 3.0), (3.0, 7.0), (7.0, 7.0), (7.0, 3.0), (3.0, 3.0)]),
            inners: vec![],
        };
        assert_eq!(check(vec![holed.clone(), island]), Ok(()));
        let in_material = Polygon {
            outer: ring_of(&[(0.5, 0.5), (0.5, 1.5), (1.5, 1.5), (1.5, 0.5), (0.5, 0.5)]),
            inners: vec![],
        };
        assert_eq!(
            check(vec![holed, in_material]),
            Err(ValidityFailure::IntersectingInteriors)
        );
    }
}
