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
}
