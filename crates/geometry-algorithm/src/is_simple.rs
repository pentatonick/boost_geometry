//! `is_simple(&g) -> bool` — OGC Simple Feature predicate.
//!
//! Mirrors `boost::geometry::is_simple(g)` from
//! `boost/geometry/algorithms/is_simple.hpp`. The v1 linestring
//! implementation is brute-force `O(n²)`; a sweepline-based variant
//! lands once `phase_03`'s overlay infrastructure is in place.
//!
//! A linestring is *simple* iff no two non-adjacent segments intersect,
//! adjacent segments touch only at their shared vertex (no zero-length
//! edge, no collinear doubling-back), and it has no repeated
//! non-consecutive vertex. A closed linestring / ring is allowed to
//! share exactly its first and last vertex — that is the ring closure,
//! not a self-intersection.
//!
//! An areal geometry (polygon) is *simple* iff every ring is non-empty
//! and free of consecutive duplicate vertices — nothing more. This
//! mirrors Boost exactly (`algorithms/detail/is_simple/areal.hpp`):
//! "a Polygon is always a simple geometric object provided that it is
//! valid". Self-intersections, ring touches, and ring crossings are
//! validity concerns — see `geometry_overlay::validity::is_valid_polygon`.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_model::{Linestring, Polygon, Ring, Segment};
use geometry_strategy::{CartesianIntersects, IntersectsStrategy};
use geometry_trait::{Linestring as LinestringTrait, Point, Polygon as PolygonTrait};

/// `true` iff `g` satisfies the OGC "is simple" predicate.
///
/// Mirrors `boost::geometry::is_simple` from
/// `boost/geometry/algorithms/is_simple.hpp`. Dispatch happens through
/// the [`IsSimple`] trait so linestrings and polygons share one entry
/// point.
#[inline]
#[must_use]
pub fn is_simple<G: IsSimple>(g: &G) -> bool {
    g.is_simple()
}

/// Kind-keyed backing trait for [`is_simple`]. Hidden from the public
/// surface — callers reach it through the free function only.
#[doc(hidden)]
pub trait IsSimple {
    /// `true` iff `self` is simple.
    fn is_simple(&self) -> bool;
}

impl<P> IsSimple for Linestring<P>
where
    P: Point,
    P::Scalar: CoordinateScalar,
    CartesianIntersects: IntersectsStrategy<Segment<P>, Segment<P>>,
    P: geometry_trait::PointMut + Default + Copy,
{
    fn is_simple(&self) -> bool {
        let pts: Vec<P> = self.points().copied().collect();
        linestring_points_simple(&pts)
    }
}

impl<P, const CW: bool, const CL: bool> IsSimple for Polygon<P, CW, CL>
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    fn is_simple(&self) -> bool {
        // Boost's areal is_simple is deliberately shallow: every ring
        // must be non-empty and lack consecutive duplicate points —
        // nothing else. "A Polygon is always a simple geometric object
        // provided that it is valid"
        // (`algorithms/detail/is_simple/areal.hpp`); ring touches,
        // crossings, and self-intersections are `is_valid`'s business
        // (see `geometry_overlay::validity`), not simplicity's.
        ring_lacks_duplicates(self.exterior()) && self.interiors().all(|r| ring_lacks_duplicates(r))
    }
}

/// Boost's `is_simple_ring`: non-empty and no consecutive duplicate
/// vertices, walked over the closeable view — for an open ring the
/// implicit (last, first) closing pair is also checked; for a closed
/// ring the stored closing repetition is the ring closure, not a
/// duplicate. Mirrors `! detail::is_valid::has_duplicates<Ring>` in
/// `algorithms/detail/is_simple/areal.hpp`.
fn ring_lacks_duplicates<P, const CW: bool, const CL: bool>(r: &Ring<P, CW, CL>) -> bool
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let pts: &[P] = &r.0;
    if pts.is_empty() {
        return false;
    }
    for w in pts.windows(2) {
        if points_equal(&w[0], &w[1]) {
            return false;
        }
    }
    if matches!(
        geometry_trait::Ring::closure(r),
        geometry_trait::Closure::Open
    ) && pts.len() >= 2
        && points_equal(&pts[pts.len() - 1], &pts[0])
    {
        // Open ring whose stored last already equals the first: under
        // the closeable view that IS a consecutive duplicate.
        return false;
    }
    true
}

/// The shared linestring-simplicity walk over a point slice.
fn linestring_points_simple<P>(pts: &[P]) -> bool
where
    P: Point + geometry_trait::PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
    CartesianIntersects: IntersectsStrategy<Segment<P>, Segment<P>>,
{
    if pts.len() < 2 {
        return true;
    }

    // Is this a closed loop (first vertex coincides with the last)? Then
    // the first and last segments legitimately share that vertex.
    let closed = points_equal(&pts[0], &pts[pts.len() - 1]);

    let segs: Vec<Segment<P>> = pts.windows(2).map(|w| Segment::new(w[0], w[1])).collect();

    for i in 0..segs.len() {
        // Zero-length segment (a repeated consecutive vertex) is never
        // simple.
        if points_equal(&pts[i], &pts[i + 1]) {
            return false;
        }
        for j in (i + 1)..segs.len() {
            let adjacent = j == i + 1;
            if adjacent {
                // Adjacent segments share their join vertex — permitted.
                // Reject only a collinear doubling-back (the incoming
                // and outgoing directions point opposite ways).
                if doubles_back(&pts[i], &pts[i + 1], &pts[j + 1]) {
                    return false;
                }
            } else if closed && i == 0 && j == segs.len() - 1 {
                // First and last segment of a closed loop meet only at
                // the shared closing vertex — that is the ring closure,
                // not a self-intersection.
            } else if CartesianIntersects.intersects(&segs[i], &segs[j]) {
                return false;
            }
        }
    }
    true
}

/// Coordinate-wise 2D equality of two points.
#[inline]
fn points_equal<P: Point>(a: &P, b: &P) -> bool {
    a.get::<0>() == b.get::<0>() && a.get::<1>() == b.get::<1>()
}

/// `true` iff the edge `p0`-`p1` and the adjacent edge `p1`-`p2` are
/// collinear and fold back over one another (the turn reverses
/// direction along the same line).
#[inline]
fn doubles_back<P: Point>(p0: &P, p1: &P, p2: &P) -> bool
where
    P::Scalar: CoordinateScalar,
{
    let ax = p1.get::<0>() - p0.get::<0>();
    let ay = p1.get::<1>() - p0.get::<1>();
    let bx = p2.get::<0>() - p1.get::<0>();
    let by = p2.get::<1>() - p1.get::<1>();
    let cross = ax * by - ay * bx;
    if cross != P::Scalar::ZERO {
        return false;
    }
    // Collinear: fold-back iff the incoming and outgoing directions have
    // a negative dot product.
    ax * bx + ay * by < P::Scalar::ZERO
}

#[cfg(test)]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/is_simple.cpp:46-120`.

    use super::is_simple;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, Polygon, linestring, polygon};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn two_point_is_simple() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 2.)];
        assert!(is_simple(&ls));
    }

    #[test]
    fn three_point_is_simple() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 2.), (2., 3.)];
        assert!(is_simple(&ls));
    }

    #[test]
    fn consecutive_duplicate_not_simple() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (0., 0.), (1., 0.)];
        assert!(!is_simple(&ls));
    }

    #[test]
    fn figure_eight_not_simple() {
        let ls: Linestring<Pt> =
            linestring![(0., 0.), (1., 0.), (2., 0.), (1., 1.), (1., 0.), (1., -1.)];
        assert!(!is_simple(&ls));
    }

    #[test]
    fn bowtie_linestring_not_simple() {
        // (0,0)-(2,2)-(2,0)-(0,2): the first and third segments cross.
        let ls: Linestring<Pt> = linestring![(0., 0.), (2., 2.), (2., 0.), (0., 2.)];
        assert!(!is_simple(&ls));
    }

    #[test]
    fn closed_simple_quadrilateral() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (1., 1.), (0., 0.)];
        assert!(is_simple(&ls));
    }

    #[test]
    fn closed_square_is_simple() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)];
        assert!(is_simple(&ls));
    }

    #[test]
    fn unit_square_polygon_is_simple() {
        let pg: Polygon<Pt> = polygon![[(0., 0.), (4., 0.), (4., 3.), (0., 3.), (0., 0.)]];
        assert!(is_simple(&pg));
    }

    #[test]
    fn polygon_with_disjoint_hole_is_simple() {
        let pg: Polygon<Pt> = polygon![
            [(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)],
            [(2., 2.), (4., 2.), (4., 4.), (2., 4.), (2., 2.)],
        ];
        assert!(is_simple(&pg));
    }

    #[test]
    fn bowtie_polygon_is_simple_but_invalid() {
        // Boost parity: the bow-tie has no duplicate vertices, so it is
        // SIMPLE — and invalid (geometry_overlay::validity::is_valid_ring
        // reports SelfIntersection; not asserted here because algorithm
        // must not depend on overlay — that would be a dependency cycle).
        let pg: Polygon<Pt> = polygon![[(0., 0.), (2., 2.), (0., 2.), (2., 0.), (0., 0.)]];
        assert!(is_simple(&pg));
    }

    #[test]
    fn hole_touching_outer_is_simple() {
        // Boost parity: areal is_simple checks only per-ring duplicate
        // points ("a Polygon is always a simple geometric object provided
        // that it is valid", detail/is_simple/areal.hpp). A hole touching
        // the exterior is a VALIDITY question, not a simplicity one.
        let pg: Polygon<Pt> = polygon![
            [(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)],
            [(0., 0.), (5., 5.), (10., 0.), (5., 0.), (0., 0.)],
        ];
        assert!(is_simple(&pg));
    }

    #[test]
    fn polygon_with_consecutive_duplicate_is_not_simple() {
        // Boost's has_duplicates: a consecutive repeated vertex makes
        // the ring (and so the polygon) non-simple.
        let pg: Polygon<Pt> =
            polygon![[(0., 0.), (4., 0.), (4., 0.), (4., 4.), (0., 4.), (0., 0.)]];
        assert!(!is_simple(&pg));
    }

    #[test]
    fn polygon_with_duplicate_in_hole_is_not_simple() {
        let pg: Polygon<Pt> = polygon![
            [(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)],
            [(2., 2.), (4., 2.), (4., 2.), (4., 4.), (2., 4.), (2., 2.)],
        ];
        assert!(!is_simple(&pg));
    }

    #[test]
    fn polygon_with_empty_exterior_is_not_simple() {
        // Boost: "not empty and lacking duplicate points" — empty
        // fails the first clause.
        use geometry_model::Ring;
        let pg: Polygon<Pt> = Polygon::new(Ring::new());
        assert!(!is_simple(&pg));
    }
}
