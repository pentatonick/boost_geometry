//! `is_convex(&g) -> bool`.
//!
//! Mirrors `boost::geometry::is_convex(g)` from
//! `boost/geometry/algorithms/is_convex.hpp`. A ring is convex iff every
//! consecutive cross-product `((b − a) × (c − b))` has the same sign
//! (zero allowed — collinear interior vertices don't disqualify). A
//! polygon is convex iff its outer ring is convex AND it has no interior
//! rings.
//!
//! This is Cartesian-only: the cross-product predicate is the same in
//! every Cartesian coordinate system, and spherical / geographic
//! convexity is a different algorithm not yet in Boost, so no strategy
//! layer is needed — Boost itself ships `is_convex` as a plain free
//! function.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_model::{MultiPolygon, Polygon, Ring};
use geometry_trait::{Point as PointTrait, Polygon as _, Ring as RingTrait};

/// True iff `g` is convex.
///
/// Mirrors `boost::geometry::is_convex(g)` from
/// `boost/geometry/algorithms/is_convex.hpp`.
#[must_use]
pub fn is_convex<G: IsConvex>(g: &G) -> bool {
    g.is_convex()
}

/// Per-kind convexity dispatch. Implemented for [`Ring`], [`Polygon`],
/// and [`MultiPolygon`].
#[doc(hidden)]
pub trait IsConvex {
    /// True iff `self` is convex.
    fn is_convex(&self) -> bool;
}

/// Convexity test for a ring: every consecutive cross-product shares one
/// sign (zero permitted). Uses modular indexing so the closing edge of a
/// closed ring is checked too.
fn ring_is_convex<P: PointTrait, const CW: bool, const CL: bool>(ring: &Ring<P, CW, CL>) -> bool {
    let pts: Vec<&P> = ring.points().collect();
    let len = pts.len();
    if len < 3 {
        return true;
    }

    let zero = P::Scalar::ZERO;
    let mut sign: Option<bool> = None; // Some(true) = positive turn
    for index in 0..len {
        let prev = pts[index];
        let curr = pts[(index + 1) % len];
        let next = pts[(index + 2) % len];
        let edge_in_x = curr.get::<0>() - prev.get::<0>();
        let edge_in_y = curr.get::<1>() - prev.get::<1>();
        let edge_out_x = next.get::<0>() - curr.get::<0>();
        let edge_out_y = next.get::<1>() - curr.get::<1>();
        let cross = edge_in_x * edge_out_y - edge_in_y * edge_out_x;
        if cross == zero {
            continue;
        }
        let positive = cross > zero;
        match sign {
            None => sign = Some(positive),
            Some(previous) if previous != positive => return false,
            _ => {}
        }
    }
    true
}

impl<P: PointTrait, const CW: bool, const CL: bool> IsConvex for Ring<P, CW, CL> {
    fn is_convex(&self) -> bool {
        ring_is_convex(self)
    }
}

impl<P: PointTrait, const CW: bool, const CL: bool> IsConvex for Polygon<P, CW, CL> {
    fn is_convex(&self) -> bool {
        self.interiors().count() == 0 && ring_is_convex(self.exterior())
    }
}

impl<Pg: IsConvex + geometry_trait::Polygon> IsConvex for MultiPolygon<Pg> {
    fn is_convex(&self) -> bool {
        self.0.iter().all(IsConvex::is_convex)
    }
}

#[cfg(test)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/is_convex.cpp` — a triangle /
    //! square is convex; a reflex polygon and any polygon with a hole
    //! are not.

    use super::is_convex;
    use geometry_cs::Cartesian;
    use geometry_model::{MultiPolygon, Point2D, Polygon, Ring, polygon};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn triangle_is_convex() {
        let pg: Polygon<Pt> = polygon![[(0., 0.), (4., 0.), (2., 3.), (0., 0.)]];
        assert!(is_convex(&pg));
    }

    #[test]
    fn square_is_convex() {
        let pg: Polygon<Pt> = polygon![[(0., 0.), (4., 0.), (4., 4.), (0., 4.), (0., 0.)]];
        assert!(is_convex(&pg));
    }

    #[test]
    fn reflex_polygon_is_not_convex() {
        let pg: Polygon<Pt> =
            polygon![[(0., 0.), (4., 0.), (2., 1.), (4., 4.), (0., 4.), (0., 0.)]];
        assert!(!is_convex(&pg));
    }

    #[test]
    fn polygon_with_hole_is_not_convex() {
        let pg: Polygon<Pt> = polygon![
            [(0., 0.), (4., 0.), (4., 4.), (0., 4.), (0., 0.)],
            [(1., 1.), (2., 1.), (2., 2.), (1., 2.), (1., 1.)],
        ];
        assert!(!is_convex(&pg));
    }

    /// A ring with fewer than 3 distinct vertices is trivially convex
    /// (the `len < 3` guard).
    #[test]
    fn two_point_ring_is_trivially_convex() {
        let r: Ring<Pt> = Ring::from_vec(alloc::vec![Pt::new(0., 0.), Pt::new(1., 0.)]);
        assert!(is_convex(&r));
    }

    /// All vertices collinear: every cross-product is zero, no sign is
    /// ever set, and the ring counts as convex (matches Boost, where
    /// degenerate/collinear rings are not rejected as concave).
    #[test]
    fn collinear_ring_is_convex() {
        let r: Ring<Pt> = Ring::from_vec(alloc::vec![
            Pt::new(0., 0.),
            Pt::new(1., 1.),
            Pt::new(2., 2.)
        ]);
        assert!(is_convex(&r));
    }

    /// A convex (non-degenerate) `Ring` exercises the `Ring` impl's
    /// positive path directly, not via `Polygon`.
    #[test]
    fn convex_ring_direct() {
        let r: Ring<Pt> = Ring::from_vec(alloc::vec![
            Pt::new(0., 0.),
            Pt::new(4., 0.),
            Pt::new(4., 4.),
            Pt::new(0., 4.),
        ]);
        assert!(is_convex(&r));
    }

    /// `MultiPolygon` is convex iff *every* member is.
    #[test]
    fn multi_polygon_all_members_must_be_convex() {
        let convex: Polygon<Pt> = polygon![[(0., 0.), (4., 0.), (2., 3.), (0., 0.)]];
        let reflex: Polygon<Pt> =
            polygon![[(0., 0.), (4., 0.), (2., 1.), (4., 4.), (0., 4.), (0., 0.)]];
        let all_convex = MultiPolygon(alloc::vec![convex.clone(), convex.clone()]);
        assert!(is_convex(&all_convex));
        let mixed = MultiPolygon(alloc::vec![convex, reflex]);
        assert!(!is_convex(&mixed));
    }
}
