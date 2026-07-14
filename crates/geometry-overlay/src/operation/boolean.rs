//! OVL5 — the boolean overlay free functions.
//!
//! The public entries route through a planar split-edge arrangement that
//! performs turn collection, colocation handling, boundary classification,
//! traversal, and [`assemble`](mod@crate::assemble). Mirrors
//! `boost/geometry/algorithms/intersection.hpp`, `union.hpp`,
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
//! Polygon × polygon → `MultiPolygon`, including interior rings, contained
//! holes/islands, shared edges, and colocated vertices. Coordinates outside
//! the exact-predicate range surface as [`OverlayError::Unsupported`].

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{MultiPolygon, Polygon};
use geometry_tag::SameAs;
use geometry_trait::{PointMut, Polygon as PolygonTrait};

use crate::traverse::TraversalError;

use super::areal::{ArealOp, overlay as areal_overlay};

/// Failure of a boolean overlay operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverlayError {
    /// Coordinates exceeded the predicate range or the input could not form
    /// a supported result boundary. Also propagated from legacy
    /// [`TraversalError`] callers.
    Unsupported,
}

impl From<TraversalError> for OverlayError {
    fn from(_: TraversalError) -> Self {
        OverlayError::Unsupported
    }
}

/// Intersection of two polygons — the region inside **both**.
///
/// Mirrors `boost::geometry::intersection` from
/// `algorithms/detail/intersection/interface.hpp:342-372`. Returns an empty
/// `MultiPolygon` when the polygons do not overlap.
///
/// # Errors
///
/// [`OverlayError::Unsupported`] when coordinates exceed the predicate range.
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
#[inline]
#[must_use = "intersection can fail and the resulting geometry should be used"]
pub fn intersection<G1, G2, P>(g1: &G1, g2: &G2) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    areal_overlay(g1, g2, ArealOp::Intersection)
}

/// Union of two polygons — the region inside **either**.
///
/// Mirrors `boost::geometry::union_` from `algorithms/union.hpp:851-881`; the
/// C++ trailing underscore dodges the keyword, while this compatibility entry
/// uses the unambiguous name `union_poly`.
///
/// # Errors
///
/// [`OverlayError::Unsupported`] when coordinates exceed the predicate range.
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
#[inline]
#[must_use = "union can fail and the resulting geometry should be used"]
pub fn union_poly<G1, G2, P>(g1: &G1, g2: &G2) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    areal_overlay(g1, g2, ArealOp::Union)
}

/// Union of two polygons — the region inside either input.
///
/// This is the Boost-style public spelling of [`union_poly`]. Rust reserves
/// `union`, so callers write the raw identifier `r#union(a, b)`; the exported
/// symbol is still named `union`.
///
/// Mirrors `boost::geometry::union_` from
/// `boost/geometry/algorithms/union.hpp:866-880`.
///
/// # Errors
///
/// Propagates [`OverlayError::Unsupported`] from [`union_poly`].
#[inline]
#[must_use = "union can fail and the resulting geometry should be used"]
pub fn r#union<G1, G2, P>(g1: &G1, g2: &G2) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    union_poly(g1, g2)
}

/// Difference of two polygons — the region inside the first but outside
/// the second (`A − B`).
///
/// Mirrors `boost::geometry::difference` from
/// `algorithms/difference.hpp:686-714`.
///
/// # Errors
///
/// [`OverlayError::Unsupported`] when coordinates exceed the predicate range.
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
#[inline]
#[must_use = "difference can fail and the resulting geometry should be used"]
pub fn difference<G1, G2, P>(g1: &G1, g2: &G2) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    areal_overlay(g1, g2, ArealOp::Difference)
}

/// Symmetric difference of two polygons — the region inside exactly one
/// of them (`(A − B) ∪ (B − A)`).
///
/// Mirrors `boost::geometry::sym_difference` from
/// `algorithms/sym_difference.hpp:795-824`.
///
/// # Errors
///
/// [`OverlayError::Unsupported`] when coordinates exceed the predicate range.
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
#[inline]
#[must_use = "symmetric difference can fail and the resulting geometry should be used"]
pub fn sym_difference<G1, G2, P>(g1: &G1, g2: &G2) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    G1: PolygonTrait<Point = P>,
    G2: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    areal_overlay(g1, g2, ArealOp::SymDifference)
}

#[cfg(test)]
mod tests {
    use super::{OverlayError, intersection, union_poly};
    use geometry_algorithm::area;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};
    use geometry_trait::{MultiPolygon as _, Polygon as _};

    type P = Point2D<f64, Cartesian>;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-5 * a.abs().max(b.abs()).max(1.0)
    }

    fn total_area(mp: &geometry_model::MultiPolygon<Polygon<P>>) -> f64 {
        mp.polygons().map(|pg| area(pg).abs()).sum()
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
    fn difference_with_contained_subtrahend_emits_a_hole() {
        let big: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        let small: Polygon<P> =
            polygon![[(3.0, 3.0), (5.0, 3.0), (5.0, 5.0), (3.0, 5.0), (3.0, 3.0)]];
        let difference = super::difference(&big, &small).unwrap();
        assert_eq!(difference.polygons().count(), 1);
        assert_eq!(difference.polygons().next().unwrap().interiors().count(), 1);
        assert!(close(total_area(&difference), 96.0));
        assert!(close(
            total_area(&super::sym_difference(&big, &small).unwrap()),
            96.0
        ));
    }

    #[test]
    fn input_with_holes_participates_in_all_operations() {
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
        assert!(close(total_area(&intersection(&donut, &sq).unwrap()), 20.0));
        assert!(close(total_area(&union_poly(&donut, &sq).unwrap()), 100.0));
        assert!(close(
            total_area(&super::difference(&donut, &sq).unwrap()),
            64.0
        ));
        assert!(close(
            total_area(&super::sym_difference(&donut, &sq).unwrap()),
            80.0
        ));
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
