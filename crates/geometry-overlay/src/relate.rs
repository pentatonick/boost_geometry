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
//! v1 scope: polygon × polygon (areal × areal), computed from the turn
//! graph plus interior sampling. The matrix is filled well enough for
//! the three predicates; the full mask-string interface is deferred.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{Polygon, Ring};
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait};

use crate::operation::OverlayError;
use crate::predicate::range_guard::polygon_in_range;
use crate::surface_point::point_on_surface;
use crate::turn::info::Method;
use crate::turn::{RingKind, get_turns_ring_ring};

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
}

/// Compute the DE-9IM matrix relating two polygons.
///
/// Fills the matrix from the turn graph (do the boundaries cross, and
/// where) and interior sampling (is each interior partly inside / partly
/// outside the other). Mirrors `boost::geometry::relation`
/// (`algorithms/relation.hpp`) for the areal × areal case.
///
/// # Errors
///
/// Returns [`OverlayError::Unsupported`] in two cases:
///
/// * Either polygon has a coordinate outside the safe arithmetic range
///   ([`SAFE_ABS_MAX`](crate::predicate::range_guard::SAFE_ABS_MAX)) — out
///   of range the turn collector silently drops intersections, so the
///   relation cannot be trusted.
/// * The boundaries meet but only *non-transversally* (edge-aligned/
///   collinear or vertex-only contact) **and** neither interior
///   representative point falls strictly inside the other. In that state
///   the turn graph cannot distinguish a pure boundary touch from a
///   genuine area overlap whose crossings all land on vertices/edges, so
///   the interior/interior cell is genuinely ambiguous — the same
///   degenerate class the boolean overlay operations refuse (see
///   [`crate::intersection`]).
///
/// Every transversal crossing, containment, or clean disjoint case is
/// answered normally.
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
pub fn relate<G1, G2, P>(g1: &G1, g2: &G2) -> Result<De9im, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    // Out-of-range coordinates make the turn collector silently drop
    // intersections, so an emptied turn graph would be misread as
    // disjoint (II = Empty) — a wrong answer. Refuse up front, matching
    // the boolean overlay operations (see `crate::predicate::range_guard`).
    if !polygon_in_range(g1) || !polygon_in_range(g2) {
        return Err(OverlayError::Unsupported);
    }

    let r1 = g1.exterior();
    let r2 = g2.exterior();
    let turns = get_turns_ring_ring(r1, 0, RingKind::Exterior, r2, 1, RingKind::Exterior);
    let boundaries_meet = !turns.is_empty();
    // A transversal crossing (not a mere touch/collinear contact) is the
    // signal that the interiors genuinely overlap in area.
    let boundaries_cross_transversally = turns.iter().any(|t| t.method == Method::Crosses);

    // Interior representatives.
    let rep1 = point_on_surface(g1);
    let rep2 = point_on_surface(g2);

    // Is g1's interior point strictly inside g2, and vice versa? (Boost's
    // `within` is strict-interior, which is what we want here.)
    let g2_model = clone_polygon(g2);
    let g1_model = clone_polygon(g1);
    let rep1_in_g2 = rep1.is_some_and(|p| geometry_algorithm::within(&p, &g2_model));
    let rep2_in_g1 = rep2.is_some_and(|p| geometry_algorithm::within(&p, &g1_model));

    // Ambiguous overlap: the boundaries touch, but neither a transversal
    // crossing nor a rep-point containment witnesses an interior overlap.
    // A single interior sample can miss an overlap region that lies off to
    // one side (e.g. two edge-aligned bands, or a diamond whose crossings
    // all land on B's vertices). We cannot soundly decide II here, so we
    // refuse rather than under-report. Note the order: transversal cross
    // OR either containment makes the relation decidable and is handled
    // below; only their joint absence *with* boundary contact is unsafe.
    if boundaries_meet && !boundaries_cross_transversally && !rep1_in_g2 && !rep2_in_g1 {
        return Err(OverlayError::Unsupported);
    }

    let interiors_overlap = boundaries_cross_transversally || rep1_in_g2 || rep2_in_g1;

    let mut m = [[Dimension::Empty; 3]; 3];

    // Interior/Interior: area when the interiors genuinely overlap.
    if interiors_overlap {
        m[feature::INTERIOR][feature::INTERIOR] = Dimension::Area;
    }

    // Boundary/Boundary: the boundaries meet at the turn points.
    if boundaries_meet {
        m[feature::BOUNDARY][feature::BOUNDARY] = Dimension::Point;
    }

    // Interior/Exterior: part of g1's interior lies outside g2 unless g1
    // is wholly contained in g2. It is contained only when its interior
    // point is inside g2 *and* the boundaries do not cross transversally.
    let g1_contained = rep1_in_g2 && !boundaries_cross_transversally;
    if !g1_contained {
        m[feature::INTERIOR][feature::EXTERIOR] = Dimension::Area;
    }
    let g2_contained = rep2_in_g1 && !boundaries_cross_transversally;
    if !g2_contained {
        m[feature::EXTERIOR][feature::INTERIOR] = Dimension::Area;
    }

    // Exterior/Exterior is always the unbounded plane outside both.
    m[feature::EXTERIOR][feature::EXTERIOR] = Dimension::Area;

    Ok(De9im { m })
}

/// `touches`: the boundaries meet but the interiors do not.
///
/// Mirrors `boost::geometry::touches` (`algorithms/touches.hpp`) for the
/// areal × areal case: `II = F` and the boundaries have non-empty
/// intersection.
///
/// # Errors
///
/// Propagates [`OverlayError::Unsupported`] from [`relate`] for the
/// ambiguous non-transversal-contact class (see [`relate`]'s docs).
pub fn touches<G1, G2, P>(g1: &G1, g2: &G2) -> Result<bool, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let matrix = relate(g1, g2)?;
    Ok(!matrix.interior_interior().is_set() && matrix.boundary_boundary().is_set())
}

/// `overlaps`: the interiors intersect, and each geometry has interior
/// points outside the other, at the same dimension.
///
/// Mirrors `boost::geometry::overlaps` (`algorithms/overlaps.hpp`) for
/// areal × areal: `II = 2`, `IE = 2`, and `EI = 2`.
///
/// # Errors
///
/// Propagates [`OverlayError::Unsupported`] from [`relate`] for the
/// ambiguous non-transversal-contact class (see [`relate`]'s docs). This
/// keeps `overlaps` from silently reporting `false` for two polygons that
/// genuinely overlap along an edge-aligned or vertex-only boundary.
pub fn overlaps<G1, G2, P>(g1: &G1, g2: &G2) -> Result<bool, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let matrix = relate(g1, g2)?;
    Ok(matrix.interior_interior() == Dimension::Area
        && matrix.interior_exterior() == Dimension::Area
        && matrix.exterior_interior() == Dimension::Area)
}

/// `crosses`: for two areal geometries this is always `false` — crossing
/// is defined only for geometries of differing dimension (e.g. a line
/// crossing an area).
///
/// Mirrors `boost::geometry::crosses` (`algorithms/crosses.hpp`); the
/// areal × areal arm returns `false` by definition. Provided for
/// completeness so callers get a uniform predicate surface. It never
/// inspects the (possibly ambiguous) turn graph, so it is infallible;
/// the `Result` return keeps its signature uniform with the sibling
/// predicates [`overlaps`] / [`touches`].
///
/// # Errors
///
/// Never returns an error; the `Ok(false)` is unconditional.
#[allow(
    clippy::unnecessary_wraps,
    reason = "The Result is intentional: it keeps `crosses` signature-compatible with the sibling fallible predicates `overlaps`/`touches`, so callers handle one uniform surface."
)]
pub fn crosses<G1, G2, P>(_g1: &G1, _g2: &G2) -> Result<bool, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: Point,
{
    Ok(false)
}

/// Copy any [`PolygonTrait`] into a concrete `model::Polygon`.
fn clone_polygon<G, P>(g: &G) -> Polygon<P>
where
    G: PolygonTrait<Point = P>,
    P: Point + Copy,
{
    let outer: Ring<P> = Ring::from_vec(g.exterior().points().copied().collect());
    let inners = g
        .interiors()
        .map(|r| Ring::from_vec(r.points().copied().collect()))
        .collect();
    Polygon::with_inners(outer, inners)
}

#[cfg(test)]
mod tests {
    //! OVL6.T1 / T2 done-when: matrix + predicate values. Mirrors the
    //! case families in `test/algorithms/relate/` and the
    //! `touches` / `overlaps` test files.

    use super::{Dimension, crosses, overlaps, relate, touches};
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
    fn edge_touching_squares_are_unsupported() {
        // Share the edge x = 2 but interiors are disjoint. The boundaries
        // meet only collinearly and neither interior point is inside the
        // other, so the turn graph cannot tell a pure edge-touch from an
        // edge-aligned overlap — the relation is reported unsupported
        // rather than a possibly-wrong boolean.
        use crate::operation::OverlayError;
        let a = square(0.0, 0.0, 2.0);
        let b = square(2.0, 0.0, 2.0);
        assert_eq!(relate(&a, &b), Err(OverlayError::Unsupported));
        assert_eq!(touches(&a, &b), Err(OverlayError::Unsupported));
        assert_eq!(overlaps(&a, &b), Err(OverlayError::Unsupported));
    }

    #[test]
    fn edge_aligned_overlap_is_unsupported_not_false() {
        // Regression: A = [0,3]x[0,1], B = [2,5]x[0,1] genuinely overlap
        // in [2,3]x[0,1], but all boundary contacts are collinear and both
        // interior samples land outside the other. The old code returned
        // `overlaps = false` here (a wrong answer); now it refuses.
        use crate::operation::OverlayError;
        let a: Polygon<P> = polygon![[(0.0, 0.0), (3.0, 0.0), (3.0, 1.0), (0.0, 1.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(2.0, 0.0), (5.0, 0.0), (5.0, 1.0), (2.0, 1.0), (2.0, 0.0)]];
        assert_eq!(overlaps(&a, &b), Err(OverlayError::Unsupported));
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
