//! OVL6.T4 — `is_valid` for rings and polygons.
//!
//! Mirrors `boost/geometry/algorithms/is_valid.hpp` and the failure
//! taxonomy in `boost/geometry/algorithms/validity_failure_type.hpp`.
//! A geometry is valid when it satisfies the OGC simple-feature rules:
//! finite, in-range coordinates; enough points; a closed boundary; no
//! spikes; no self-intersections; the expected ring orientation; and,
//! for polygons, every interior ring covered by the exterior.
//!
//! **Deferred:** ring×ring edge-crossing detection (a hole whose
//! representative vertex is covered by the exterior but whose edges
//! cross the exterior or another hole; Boost's polygon-level
//! `failure_self_intersections`) and intersections between distinct
//! members of a multi-polygon are not checked.
//!
//! v1 scope: `Ring`, `Polygon`, and per-member `MultiPolygon` validation.
//! Coordinate validity (NaN / infinity) is checked because the robustness
//! gate depends on finite input.
//!
//! Unlike Boost's default validity policy, which permits consecutive repeated
//! points, this Rust entry selects Boost's strict-policy behavior and returns
//! [`ValidityFailure::DuplicatePoints`]. Boost exercises that policy in
//! `test/algorithms/is_valid.cpp:1626-1634`.

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
/// Mirrors the subset of Boost's `validity_failure_type`
/// (`algorithms/validity_failure_type.hpp:33-106`) that the v1 areal validator
/// can produce. The numeric groupings (few-points, not-closed,
/// self-intersections, …) match Boost's categories.
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
}

/// Per-kind validity implementation selected by [`is_valid`].
///
/// Rust tag-dispatch adapter for `boost::geometry::is_valid` from
/// `algorithms/detail/is_valid/interface.hpp:153-203`.
#[doc(hidden)]
pub trait ValidityStrategy<G> {
    fn apply(&self, geometry: &G) -> Result<(), ValidityFailure>;
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
    <<G::Kind as ValidityStrategyForKind>::S as Default>::default().apply(geometry)
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
    fn apply(&self, ring: &G) -> Result<(), ValidityFailure> {
        is_valid_ring(ring)
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
    fn apply(&self, polygon: &G) -> Result<(), ValidityFailure> {
        is_valid_polygon(polygon)
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
    fn apply(&self, multi_polygon: &G) -> Result<(), ValidityFailure> {
        for polygon in multi_polygon.polygons() {
            is_valid_polygon(polygon)?;
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
/// A [`ValidityFailure`] describing the first rule the ring violates,
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
pub fn is_valid_ring<R, P>(ring: &R) -> Result<(), ValidityFailure>
where
    R: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    validate_ring(ring, false)
}

/// Shared ring validation. `is_interior` flips the orientation
/// expectation: an exterior ring traverses in its declared order
/// (strategy-level `ShoelaceArea` positive); an interior ring winds
/// opposite (negative) — mirroring Boost's
/// `is_properly_oriented<Ring, IsInteriorRing>`.
fn validate_ring<R, P>(ring: &R, is_interior: bool) -> Result<(), ValidityFailure>
where
    R: RingTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let pts: Vec<P> = ring.points().copied().collect();

    // Coordinate finiteness.
    for p in &pts {
        let x: f64 = p.get::<0>().into();
        let y: f64 = p.get::<1>().into();
        if !x.is_finite() || !y.is_finite() {
            return Err(ValidityFailure::InvalidCoordinate);
        }
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

    if pts.windows(2).any(|pair| same_point(&pair[0], &pair[1])) {
        return Err(ValidityFailure::DuplicatePoints);
    }

    // A vertex triple that folds back on itself along one line — Boost's
    // failure_spikes. Checked before self-intersection so spike inputs
    // report a deterministic error (matches Boost's check order).
    if has_spike(&pts) {
        return Err(ValidityFailure::Spikes);
    }

    // No non-adjacent edge may intersect another.
    if has_self_intersection(&pts) {
        return Err(ValidityFailure::SelfIntersection);
    }

    // Orientation — Boost's failure_wrong_orientation. The strategy area
    // already folds the declared PointOrder: a correctly wound exterior
    // is positive, a correctly wound hole negative. Zero area
    // (degenerate) fails either way.
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

    Ok(())
}

/// Validate a polygon: its exterior ring (with exterior orientation
/// expectations), each interior ring (with interior orientation
/// expectations), and that every interior ring's representative vertex
/// is covered by the exterior.
///
/// Mirrors the polygon arm of `boost::geometry::is_valid`
/// (`detail/is_valid/polygon.hpp`). Hole↔exterior and hole↔hole
/// *edge-crossing* detection, and consecutive duplicate points, remain
/// deferred with the rest of the multi/degenerate overlay — a hole
/// whose representative vertex is covered by the exterior but whose
/// edges cross the exterior boundary is not detected.
///
/// # Errors
///
/// A [`ValidityFailure`] describing the first rule the polygon
/// violates, including [`ValidityFailure::InteriorRingOutside`] when a
/// hole's representative vertex lies strictly outside the exterior.
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
pub fn is_valid_polygon<G, P>(polygon: &G) -> Result<(), ValidityFailure>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    validate_ring(polygon.exterior(), false)?;
    for inner in polygon.interiors() {
        validate_ring(inner, true)?;
        // Containment: the hole's first vertex must not lie strictly
        // outside the exterior. `covered_by` (interior OR boundary)
        // deliberately admits an isolated boundary touch — Boost
        // permits those; only `failure_interior_rings_outside` is
        // detectable without ring×ring turn analysis. A hole whose
        // first vertex is inside but whose edges cross the exterior
        // remains undetected — deferred (module docs).
        if let Some(rep) = inner.points().next() {
            if !WithinRing.covered_by(rep, polygon.exterior()) {
                return Err(ValidityFailure::InteriorRingOutside);
            }
        }
    }
    Ok(())
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
/// (`cross == 0`) and folding back (`dot < 0`). Same 2-D kernel as
/// `geometry_algorithm::remove_spikes::is_spike_2d` (private there);
/// duplicated locally rather than widening that crate's public
/// surface — if a third consumer appears, hoist the predicate into a
/// shared home per the aggregate-slicing rules.
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
        // The in-range analogue is still caught as a self-intersection.
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
        // A "bow-tie" quadrilateral whose diagonals cross.
        let r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(is_valid_ring(&r), Err(ValidityFailure::SelfIntersection));
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
