//! `intersects(&a, &b)` — see
//! `boost/geometry/algorithms/intersects.hpp`.
//!
//! Cartesian-only in v1. Default strategy is
//! [`geometry_strategy::CartesianIntersects`], which implements every
//! pair in one canonical direction; the
//! [`geometry_strategy::intersects::Reversed`] blanket lifts each
//! pair to its swap.

use geometry_strategy::intersects::Reversed;
use geometry_strategy::{CartesianIntersects, IntersectsStrategy};

/// `true` iff `a` and `b` share at least one point.
///
/// Mirrors `boost::geometry::intersects(a, b)` from
/// `boost/geometry/algorithms/intersects.hpp`. Tries the canonical
/// pair direction first, falling back to the reversed direction
/// through the `Reversed<CartesianIntersects>` blanket so callers
/// never have to remember which argument order has an explicit impl.
#[inline]
#[must_use]
pub fn intersects<A, B>(a: &A, b: &B) -> bool
where
    CartesianIntersects: IntersectsStrategy<A, B>,
{
    CartesianIntersects.intersects(a, b)
}

/// Reversed-direction entry point — used when only `(B, A)` has an
/// explicit impl. Mirrors the Boost `reverse_dispatch` fallback in
/// `algorithms/detail/intersects/interface.hpp`.
#[inline]
#[must_use]
pub fn intersects_reversed<A, B>(a: &A, b: &B) -> bool
where
    Reversed<CartesianIntersects>: IntersectsStrategy<A, B>,
{
    Reversed(CartesianIntersects).intersects(a, b)
}

#[cfg(test)]
mod tests {
    //! Reference values from
    //! `geometry/test/algorithms/intersects/intersects.cpp:38-79`.
    //! Each test cites the C++ line it mirrors.

    use super::intersects;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, Polygon, Segment, linestring, polygon};

    type P = Point2D<f64, Cartesian>;
    type LS = Linestring<P>;

    fn pt(x: f64, y: f64) -> P {
        Point2D::new(x, y)
    }

    /// `intersects.cpp:38` — linestring crosses segment.
    #[test]
    fn ls_crosses_segment() {
        let ls: LS = linestring![(1.0, 1.0), (3.0, 3.0), (2.0, 5.0)];
        let s = Segment::new(pt(2.0, 0.0), pt(2.0, 6.0));
        assert!(intersects(&ls, &s));
    }

    /// `intersects.cpp:39` — linestring touches segment endpoint.
    #[test]
    fn ls_touches_segment_endpoint() {
        let ls: LS = linestring![(1.0, 1.0), (3.0, 3.0)];
        let s = Segment::new(pt(1.0, 0.0), pt(1.0, 1.0));
        assert!(intersects(&ls, &s));
    }

    /// `intersects.cpp:41` — disjoint linestring and segment.
    #[test]
    fn ls_disjoint_from_segment() {
        let ls: LS = linestring![(1.0, 1.0), (3.0, 3.0)];
        let s = Segment::new(pt(3.0, 0.0), pt(4.0, 1.0));
        assert!(!intersects(&ls, &s));
    }

    /// `intersects.cpp:50` — linestring/linestring proper crossing.
    #[test]
    fn ls_crosses_ls() {
        let a: LS = linestring![(0.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let b: LS = linestring![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)];
        assert!(intersects(&a, &b));
    }

    /// `intersects.cpp:55` — collinear overlap.
    #[test]
    fn ls_overlap_collinear() {
        let a: LS = linestring![(0.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let b: LS = linestring![(1.0, 0.0), (4.0, 0.0), (5.0, 0.0)];
        assert!(intersects(&a, &b));
    }

    /// `intersects.cpp:69` — linestring inside polygon.
    #[test]
    fn ls_inside_polygon() {
        let ls: LS = linestring![(1.0, 1.0), (2.0, 2.0)];
        let p: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        assert!(intersects(&ls, &p));
    }

    /// `intersects.cpp:71` — linestring outside polygon.
    #[test]
    fn ls_outside_polygon() {
        let ls: LS = linestring![(11.0, 0.0), (12.0, 12.0)];
        let p: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        assert!(!intersects(&ls, &p));
    }

    /// Reverse-direction call — `intersects(polygon, linestring)`
    /// resolves through the per-pair reverse impl on
    /// `CartesianIntersects` and agrees with the canonical direction.
    #[test]
    fn reversed_pair_agrees() {
        let ls: LS = linestring![(1.0, 1.0), (2.0, 2.0)];
        let p: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        assert_eq!(intersects(&ls, &p), intersects(&p, &ls));
    }

    fn square(shift: f64) -> Polygon<P> {
        polygon![[
            (shift, shift),
            (shift + 4.0, shift),
            (shift + 4.0, shift + 4.0),
            (shift, shift + 4.0),
            (shift, shift)
        ]]
    }

    /// Point × Point intersects iff the two points are coordinate-equal.
    #[test]
    fn point_point_intersects_iff_equal() {
        assert!(intersects(&pt(1.0, 2.0), &pt(1.0, 2.0)));
        assert!(!intersects(&pt(1.0, 2.0), &pt(1.0, 2.5)));
    }

    /// Point × Segment is true exactly when the point lies on the closed
    /// segment (interior, endpoint) and false off it.
    #[test]
    fn point_segment_membership() {
        let s = Segment::new(pt(0.0, 0.0), pt(4.0, 4.0));
        assert!(intersects(&pt(2.0, 2.0), &s)); // interior
        assert!(intersects(&pt(0.0, 0.0), &s)); // endpoint
        assert!(!intersects(&pt(2.0, 3.0), &s)); // off the line
        assert!(!intersects(&pt(5.0, 5.0), &s)); // collinear, past end
    }

    /// Segment × Segment detects a proper crossing and rejects a
    /// parallel offset pair.
    #[test]
    fn segment_segment_crossing_and_parallel() {
        let a = Segment::new(pt(0.0, 0.0), pt(4.0, 4.0));
        let cross = Segment::new(pt(0.0, 4.0), pt(4.0, 0.0));
        let parallel = Segment::new(pt(0.0, 1.0), pt(4.0, 5.0));
        assert!(intersects(&a, &cross));
        assert!(!intersects(&a, &parallel));
    }

    /// An empty linestring operand never intersects (the `it.next()`
    /// guard) — in either position.
    #[test]
    fn empty_linestring_operands_do_not_intersect() {
        let empty: LS = linestring![];
        let s = Segment::new(pt(0.0, 0.0), pt(1.0, 1.0));
        assert!(!intersects(&empty, &s));

        let non_empty: LS = linestring![(0.0, 0.0), (1.0, 1.0)];
        assert!(!intersects(&empty, &non_empty));
        assert!(!intersects(&non_empty, &empty));
    }

    /// Point × Polygon is `covered_by`: interior true, boundary true,
    /// exterior false.
    #[test]
    fn point_polygon_is_covered_by() {
        let sq = square(0.0);
        assert!(intersects(&pt(2.0, 2.0), &sq)); // interior
        assert!(intersects(&pt(0.0, 2.0), &sq)); // on edge
        assert!(!intersects(&pt(9.0, 9.0), &sq)); // outside
    }

    /// Polygon × Polygon: containment (either direction), edge crossing,
    /// and full disjointness are each classified correctly.
    #[test]
    fn polygon_polygon_containment_crossing_disjoint() {
        let a = square(0.0); // (0,0)-(4,4)
        let contained: Polygon<P> =
            polygon![[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)]];
        let overlapping = square(2.0); // (2,2)-(6,6): edges cross
        let disjoint = square(100.0);

        assert!(intersects(&a, &contained)); // B inside A
        assert!(intersects(&contained, &a)); // A inside B (other arm)
        assert!(intersects(&a, &overlapping)); // edge crossing
        assert!(!intersects(&a, &disjoint)); // fully apart
    }

    /// A polygon with a hole crosses another polygon's exterior through
    /// its hole edge — exercises the interior-ring crossing arms.
    #[test]
    fn polygon_polygon_hole_edges_are_tested() {
        // `a` is a 10x10 square with a central 4x4 hole (2..6).
        let a: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(2.0, 2.0), (6.0, 2.0), (6.0, 6.0), (2.0, 6.0), (2.0, 2.0)]
        ];
        // `b` sits inside the hole but its edges cross the hole boundary.
        let b: Polygon<P> = polygon![[(4.0, 4.0), (8.0, 4.0), (8.0, 8.0), (4.0, 8.0), (4.0, 4.0)]];
        assert!(intersects(&a, &b));
    }

    /// Every reverse-ordered pair agrees with its forward result — the
    /// reverse per-pair impls delegate to the forward kernels.
    #[test]
    fn reverse_pairs_delegate_to_forward() {
        let s = Segment::new(pt(0.0, 0.0), pt(4.0, 4.0));
        let p = pt(2.0, 2.0);
        assert_eq!(intersects(&s, &p), intersects(&p, &s));

        let ls: LS = linestring![(1.0, 1.0), (3.0, 3.0), (2.0, 5.0)];
        assert_eq!(intersects(&s, &ls), intersects(&ls, &s));

        let sq = square(0.0);
        assert_eq!(intersects(&sq, &p), intersects(&p, &sq));

        let ls_in: LS = linestring![(1.0, 1.0), (2.0, 2.0)];
        assert_eq!(intersects(&sq, &ls_in), intersects(&ls_in, &sq));
        // And the delegations are all `true` for these intersecting inputs.
        assert!(intersects(&s, &p));
        assert!(intersects(&sq, &p));
        assert!(intersects(&sq, &ls_in));
    }

    /// Point × Point compares all three ordinates for 3D points — the
    /// `2 =>` arm only a 3D point reaches.
    #[test]
    fn point_point_intersects_in_three_dimensions() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let a = P3::new(1.0, 2.0, 3.0);
        let same = P3::new(1.0, 2.0, 3.0);
        let diff_z = P3::new(1.0, 2.0, 9.0);
        assert!(intersects(&a, &same));
        assert!(!intersects(&a, &diff_z));
    }

    /// A linestring lying inside a polygon (no edge crossing) intersects
    /// via the vertex-`covered_by` fast path.
    #[test]
    fn linestring_polygon_inside_via_vertex_path() {
        let ls: LS = linestring![(1.0, 1.0), (2.0, 2.0)];
        assert!(intersects(&ls, &square(0.0)));
    }
}
