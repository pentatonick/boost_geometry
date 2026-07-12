//! OVL5 — the boolean overlay free functions.
//!
//! Thin orchestration over the pipeline
//! [`get_turns`](crate::turn) → [`enrich`](mod@crate::traverse::enrich) →
//! [`traverse`](fn@crate::traverse::traverse) →
//! [`assemble`](mod@crate::assemble). Mirrors
//! `boost/geometry/algorithms/intersection.hpp`, `union_.hpp`,
//! `difference.hpp`, and `sym_difference.hpp`.
//!
//! # Where these live
//!
//! The overlay plan (`phase_03-…-overlay.md` §OVL5) placed these in
//! `geometry-algorithm`. That would form a dependency cycle:
//! `geometry-overlay` already depends on `geometry-algorithm` (for
//! `within` / `ring_area`), so `geometry-algorithm` cannot depend back
//! on `geometry-overlay`. The functions therefore live here in
//! `geometry-overlay` and are re-exported by the `geometry` facade.
//! This is the same class of spec-stub cycle already corrected
//! elsewhere in the port.
//!
//! # Scope (v1)
//!
//! Polygon × polygon → `MultiPolygon`, for the clean areal case (simple
//! polygons, transversal crossings). The overlay operates on each
//! input's **exterior** ring; an input carrying interior rings (holes)
//! is refused with [`OverlayError::Unsupported`] rather than silently
//! treated as solid. Other degenerate inputs surface as
//! [`OverlayError::Unsupported`] too; non-overlapping inputs take the
//! documented fast paths.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{MultiPolygon, Polygon, Ring};
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait};

use crate::assemble::assemble_multipolygon;
use crate::predicate::range_guard::polygon_in_range;
use crate::traverse::{OverlayOp, TraversalError, enrich, traverse};
use crate::turn::{RingKind, get_turns_ring_ring};

/// Reject a polygon pair whose coordinates leave the safe arithmetic
/// range. Out of range, the turn collector silently drops intersections
/// (they surface as [`SegmentIntersection::OutOfRange`] and emit no turn),
/// so an emptied turn graph would be read as "disjoint" and yield a
/// silently wrong result. Refusing up front keeps the "never wrong
/// silently" contract (see [`crate::predicate::range_guard`]).
fn both_in_range<G1, G2, P>(g1: &G1, g2: &G2) -> bool
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: Point,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    polygon_in_range(g1) && polygon_in_range(g2)
}

/// Failure of a boolean overlay operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverlayError {
    /// The overlay hit a degenerate case the v1 clean areal engine does
    /// not handle (clustered turns, self-intersection, collinear shared
    /// edges). Propagated from [`TraversalError`].
    Unsupported,
}

impl From<TraversalError> for OverlayError {
    fn from(_: TraversalError) -> Self {
        OverlayError::Unsupported
    }
}

/// Intersection of two polygons — the region inside **both**.
///
/// Mirrors `boost::geometry::intersection`
/// (`algorithms/intersection.hpp`). Returns an empty `MultiPolygon`
/// when the polygons do not overlap.
///
/// # Errors
///
/// [`OverlayError::Unsupported`] for degenerate inputs (see the module
/// docs).
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::operation::intersection;
/// use geometry_trait::MultiPolygon as _;
///
/// type P = Point2D<f64, Cartesian>;
/// let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
/// let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
/// let out = intersection(&a, &b).unwrap();
/// assert_eq!(out.polygons().count(), 1);
/// ```
pub fn intersection<G1, G2, P>(g1: &G1, g2: &G2) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    if has_holes(g1) || has_holes(g2) || !both_in_range(g1, g2) {
        return Err(OverlayError::Unsupported);
    }
    let (r1, r2) = (g1.exterior(), g2.exterior());
    let turns = get_turns_ring_ring(r1, 0, RingKind::Exterior, r2, 1, RingKind::Exterior);

    if turns.is_empty() {
        // No boundary crossings: the intersection is empty unless one
        // polygon is wholly inside the other, in which case it is the
        // inner polygon.
        return Ok(containment_result(g1, g2, OverlayOp::Intersection));
    }

    let enriched = enrich(r1, r2, &turns);
    let rings = traverse(&enriched, &turns, OverlayOp::Intersection)?;
    Ok(assemble_multipolygon(&rings))
}

/// Whether a polygon carries any interior ring (hole). v1 overlay
/// operates on the exterior boundary only; an input with holes would be
/// silently treated as solid, so the operations refuse it rather than
/// return a wrong area.
fn has_holes<G, P>(g: &G) -> bool
where
    G: PolygonTrait<Point = P>,
    P: Point,
{
    g.interiors().next().is_some()
}

/// Union of two polygons — the region inside **either**.
///
/// Mirrors `boost::geometry::union_` (`algorithms/union_.hpp`; the C++
/// trailing underscore dodges the keyword — Rust needs no such dodge,
/// but `union` is reserved so the free function is named `union_poly`).
///
/// # Errors
///
/// [`OverlayError::Unsupported`] for degenerate inputs.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::operation::union_poly;
/// use geometry_trait::MultiPolygon as _;
///
/// type P = Point2D<f64, Cartesian>;
/// let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
/// let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
/// let out = union_poly(&a, &b).unwrap();
/// assert_eq!(out.polygons().count(), 1);
/// ```
pub fn union_poly<G1, G2, P>(g1: &G1, g2: &G2) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    if has_holes(g1) || has_holes(g2) || !both_in_range(g1, g2) {
        return Err(OverlayError::Unsupported);
    }
    let (r1, r2) = (g1.exterior(), g2.exterior());
    let turns = get_turns_ring_ring(r1, 0, RingKind::Exterior, r2, 1, RingKind::Exterior);

    if turns.is_empty() {
        // No crossings: either disjoint (two separate polygons) or one
        // contains the other (the outer polygon).
        return Ok(union_no_crossing(g1, g2));
    }

    let enriched = enrich(r1, r2, &turns);
    let rings = traverse(&enriched, &turns, OverlayOp::Union)?;
    Ok(assemble_multipolygon(&rings))
}

/// Difference of two polygons — the region inside the first but outside
/// the second (`A − B`).
///
/// Mirrors `boost::geometry::difference` (`algorithms/difference.hpp`).
/// The subtrahend `B`'s exterior ring is reversed, so a forward-only
/// traversal that keeps `A`'s outside arcs assembles `A − B`.
///
/// # Errors
///
/// [`OverlayError::Unsupported`] for degenerate inputs.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::operation::difference;
/// use geometry_trait::MultiPolygon as _;
///
/// type P = Point2D<f64, Cartesian>;
/// let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
/// let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
/// let out = difference(&a, &b).unwrap();
/// assert_eq!(out.polygons().count(), 1);
/// ```
pub fn difference<G1, G2, P>(g1: &G1, g2: &G2) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    if has_holes(g1) || has_holes(g2) || !both_in_range(g1, g2) {
        return Err(OverlayError::Unsupported);
    }
    let r1 = g1.exterior();
    let r2 = g2.exterior();
    let turns = get_turns_ring_ring(r1, 0, RingKind::Exterior, r2, 1, RingKind::Exterior);

    if turns.is_empty() {
        // No crossings: A inside B → empty; disjoint → A whole; B inside
        // A → A with B as a hole, which the exterior-only assembler
        // cannot yet build, so it is refused rather than returned wrong.
        return difference_no_crossing(g1, g2);
    }

    let enriched = enrich(r1, r2, &turns);
    let rings = traverse(&enriched, &turns, OverlayOp::Difference)?;
    Ok(assemble_multipolygon(&rings))
}

/// Symmetric difference of two polygons — the region inside exactly one
/// of them (`(A − B) ∪ (B − A)`).
///
/// Mirrors `boost::geometry::sym_difference`
/// (`algorithms/sym_difference.hpp`). Computed as the union of the two
/// one-sided differences.
///
/// # Errors
///
/// [`OverlayError::Unsupported`] for degenerate inputs.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::operation::sym_difference;
/// use geometry_trait::MultiPolygon as _;
///
/// type P = Point2D<f64, Cartesian>;
/// let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
/// let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
/// let out = sym_difference(&a, &b).unwrap();
/// assert!(out.polygons().count() >= 1);
/// ```
pub fn sym_difference<G1, G2, P>(g1: &G1, g2: &G2) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let a_minus_b = difference(g1, g2)?;
    let b_minus_a = difference(g2, g1)?;
    // Union the two disjoint difference results by concatenating their
    // polygons — `(A − B)` and `(B − A)` share no interior area, so no
    // further overlay is needed.
    let mut polygons: Vec<Polygon<P>> = a_minus_b.0;
    polygons.extend(b_minus_a.0);
    Ok(MultiPolygon(polygons))
}

/// Difference result when the two boundaries do not cross.
///
/// * `A` inside `B` → empty.
/// * `B` inside `A` → `A` with `B` as a hole — the exterior-only
///   assembler cannot build this, so it is refused with
///   [`OverlayError::Unsupported`] rather than returned as `A` whole
///   (which would over-report the area).
/// * disjoint → `A` whole.
fn difference_no_crossing<G1, G2, P>(
    g1: &G1,
    g2: &G2,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    if first_vertex_within(g1, g2) {
        // A inside B → nothing left.
        return Ok(MultiPolygon(Vec::new()));
    }
    if first_vertex_within(g2, g1) {
        // B inside A → A with a hole; deferred.
        return Err(OverlayError::Unsupported);
    }
    // Disjoint → A whole.
    Ok(MultiPolygon(Vec::from([clone_polygon(g1)])))
}

/// The intersection / containment result when the two boundaries do not
/// cross: the inner polygon if one is inside the other, else empty.
fn containment_result<G1, G2, P>(g1: &G1, g2: &G2, _op: OverlayOp) -> MultiPolygon<Polygon<P>>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    if first_vertex_within(g1, g2) {
        MultiPolygon(Vec::from([clone_polygon(g1)]))
    } else if first_vertex_within(g2, g1) {
        MultiPolygon(Vec::from([clone_polygon(g2)]))
    } else {
        MultiPolygon(Vec::new())
    }
}

/// The union result when boundaries do not cross: the outer polygon if
/// one contains the other, else both polygons side by side.
fn union_no_crossing<G1, G2, P>(g1: &G1, g2: &G2) -> MultiPolygon<Polygon<P>>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    if first_vertex_within(g1, g2) {
        MultiPolygon(Vec::from([clone_polygon(g2)]))
    } else if first_vertex_within(g2, g1) {
        MultiPolygon(Vec::from([clone_polygon(g1)]))
    } else {
        MultiPolygon(Vec::from([clone_polygon(g1), clone_polygon(g2)]))
    }
}

/// Whether the first vertex of `inner`'s exterior lies within `outer`.
fn first_vertex_within<GI, GO, P>(inner: &GI, outer: &GO) -> bool
where
    GI: PolygonTrait<Point = P>,
    GO: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let Some(v) = inner.exterior().points().next() else {
        return false;
    };
    // `within` is implemented for the concrete `model::Polygon`, not an
    // arbitrary `PolygonTrait`, so materialise `outer` first.
    let outer_model = clone_polygon(outer);
    geometry_algorithm::within(v, &outer_model)
}

/// Copy any [`PolygonTrait`] into a concrete `model::Polygon` by reading
/// its rings through the trait surface.
fn clone_polygon<G, P>(g: &G) -> Polygon<P>
where
    G: PolygonTrait<Point = P>,
    P: Point + Copy,
{
    let outer: Ring<P> = Ring::from_vec(g.exterior().points().copied().collect());
    let inners: Vec<Ring<P>> = g
        .interiors()
        .map(|r| Ring::from_vec(r.points().copied().collect()))
        .collect();
    Polygon::with_inners(outer, inners)
}

#[cfg(test)]
mod tests {
    use super::{OverlayError, intersection, union_poly};
    use geometry_algorithm::ring_area;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};
    use geometry_trait::{MultiPolygon as _, Polygon as _};

    type P = Point2D<f64, Cartesian>;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-5 * a.abs().max(b.abs()).max(1.0)
    }

    fn total_area(mp: &geometry_model::MultiPolygon<Polygon<P>>) -> f64 {
        mp.polygons().map(|pg| ring_area(pg.exterior()).abs()).sum()
    }

    #[test]
    fn intersection_of_offset_squares() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
        let out = intersection(&a, &b).unwrap();
        assert_eq!(out.polygons().count(), 1);
        assert!(close(total_area(&out), 1.0), "area {}", total_area(&out));
    }

    #[test]
    fn intersection_disjoint_is_empty() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(5.0, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 6.0), (5.0, 5.0)]];
        let out = intersection(&a, &b).unwrap();
        assert_eq!(out.polygons().count(), 0);
    }

    #[test]
    fn intersection_contained_is_inner() {
        let big: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        let small: Polygon<P> =
            polygon![[(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0), (2.0, 2.0)]];
        let out = intersection(&big, &small).unwrap();
        assert_eq!(out.polygons().count(), 1);
        assert!(close(total_area(&out), 4.0), "area {}", total_area(&out));
    }

    #[test]
    fn union_of_offset_squares_area() {
        // |A| + |B| - |A∩B| = 4 + 4 - 1 = 7.
        let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
        let out = union_poly(&a, &b).unwrap();
        assert_eq!(out.polygons().count(), 1);
        assert!(close(total_area(&out), 7.0), "area {}", total_area(&out));
    }

    #[test]
    fn union_disjoint_is_two_polygons() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(5.0, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 6.0), (5.0, 5.0)]];
        let out = union_poly(&a, &b).unwrap();
        assert_eq!(out.polygons().count(), 2);
        assert!(close(total_area(&out), 2.0), "area {}", total_area(&out));
    }

    #[test]
    fn difference_of_offset_squares_area() {
        // |A − B| = |A| − |A∩B| = 4 − 1 = 3.
        let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
        let out = super::difference(&a, &b).unwrap();
        assert_eq!(out.polygons().count(), 1);
        assert!(close(total_area(&out), 3.0), "area {}", total_area(&out));
    }

    #[test]
    fn difference_disjoint_is_first_whole() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(5.0, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 6.0), (5.0, 5.0)]];
        let out = super::difference(&a, &b).unwrap();
        assert_eq!(out.polygons().count(), 1);
        assert!(close(total_area(&out), 4.0), "area {}", total_area(&out));
    }

    #[test]
    fn difference_a_inside_b_is_empty() {
        let big: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        let small: Polygon<P> =
            polygon![[(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0), (2.0, 2.0)]];
        let out = super::difference(&small, &big).unwrap();
        assert_eq!(out.polygons().count(), 0);
    }

    #[test]
    fn difference_with_contained_subtrahend_is_refused_not_over_reported() {
        // B strictly inside A: A − B is A with a hole. The exterior-only
        // assembler cannot build the hole, so it must refuse rather than
        // return A whole (area 100 instead of the true 96).
        let big: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        let small: Polygon<P> =
            polygon![[(3.0, 3.0), (5.0, 3.0), (5.0, 5.0), (3.0, 5.0), (3.0, 3.0)]];
        assert_eq!(
            super::difference(&big, &small),
            Err(OverlayError::Unsupported)
        );
        assert_eq!(
            super::sym_difference(&big, &small),
            Err(OverlayError::Unsupported)
        );
    }

    #[test]
    fn input_with_holes_is_refused_not_silently_wrong() {
        // A polygon with an interior ring: the exterior-only overlay would
        // treat it as solid and return a wrong area. It must refuse.
        let donut: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0), (3.0, 3.0)]
        ];
        let sq: Polygon<P> = polygon![[(2.0, 2.0), (8.0, 2.0), (8.0, 8.0), (2.0, 8.0), (2.0, 2.0)]];
        assert_eq!(intersection(&donut, &sq), Err(OverlayError::Unsupported));
        assert_eq!(union_poly(&donut, &sq), Err(OverlayError::Unsupported));
        assert_eq!(
            super::difference(&donut, &sq),
            Err(OverlayError::Unsupported)
        );
        assert_eq!(
            super::sym_difference(&donut, &sq),
            Err(OverlayError::Unsupported)
        );
    }

    #[test]
    fn out_of_range_coordinates_are_refused_not_silently_wrong() {
        // Regression: two huge overlapping squares (~1e14, past the ±2^26
        // safe range) made the turn kernel silently drop every crossing as
        // OutOfRange; the emptied turn graph was misread as "B inside A",
        // over-reporting the intersection area ~4× as `Ok`. All ops must
        // refuse rather than return a silently wrong result.
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
        assert_eq!(intersection(&a, &b), Err(OverlayError::Unsupported));
        assert_eq!(union_poly(&a, &b), Err(OverlayError::Unsupported));
        assert_eq!(super::difference(&a, &b), Err(OverlayError::Unsupported));
        assert_eq!(
            super::sym_difference(&a, &b),
            Err(OverlayError::Unsupported)
        );
    }

    #[test]
    fn sym_difference_of_offset_squares_area() {
        // |A △ B| = |A| + |B| − 2|A∩B| = 4 + 4 − 2·1 = 6.
        let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
        let out = super::sym_difference(&a, &b).unwrap();
        assert!(close(total_area(&out), 6.0), "area {}", total_area(&out));
    }

    #[test]
    fn union_contained_is_outer() {
        let big: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        let small: Polygon<P> =
            polygon![[(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0), (2.0, 2.0)]];
        let out = union_poly(&big, &small).unwrap();
        assert_eq!(out.polygons().count(), 1);
        assert!(close(total_area(&out), 100.0), "area {}", total_area(&out));
    }
}
