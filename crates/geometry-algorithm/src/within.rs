//! `within(&p, &g)` / `covered_by(&p, &g)` — point-in-geometry
//! containment, see `boost/geometry/algorithms/within.hpp` and
//! `boost/geometry/algorithms/covered_by.hpp`.
//!
//! Cartesian-only in v1, point-as-first-argument only. The default
//! strategy is the winding-number PIP from
//! `boost/geometry/strategies/cartesian/point_in_poly_winding.hpp`,
//! selected per geometry *kind* by the tag-keyed
//! [`geometry_strategy::WithinStrategyForKind`] picker, so any
//! concept-adapted foreign type resolves through the same path as the
//! equivalent `geometry-model` value (see `specs/open-tag-dispatch/`).
//! Spherical / geographic variants arrive alongside the Haversine /
//! Andoyer / Vincenty distance work in later tasks.

use geometry_strategy::{WithinStrategy, WithinStrategyForKind};
use geometry_trait::{Geometry, Point};

/// `true` iff `p` lies in the strict interior of `g`.
///
/// Mirrors `boost::geometry::within(p, g)` from
/// `boost/geometry/algorithms/within.hpp`. The default strategy is
/// selected per geometry kind by the tag-keyed
/// [`geometry_strategy::WithinStrategyForKind`] picker (see
/// [`geometry_strategy::within`]), mirroring Boost's
/// `services::default_strategy` resolution at
/// `strategies/cartesian/point_in_poly_winding.hpp:254-258`.
///
/// Boundary semantics follow Boost: a point on a segment, vertex, or
/// box face returns `false` from `within` and `true` from
/// [`covered_by`]. See `test/strategies/winding.cpp:34-43` for the
/// reference cases (all corners and sides "officially false").
#[inline]
#[must_use]
pub fn within<P, G>(p: &P, g: &G) -> bool
where
    P: Point,
    G: Geometry,
    G::Kind: WithinStrategyForKind,
    <G::Kind as WithinStrategyForKind>::S: WithinStrategy<P, G>,
{
    <<G::Kind as WithinStrategyForKind>::S as Default>::default().within(p, g)
}

/// `true` iff `p` lies in the strict interior **or** on the boundary
/// of `g`.
///
/// Mirrors `boost::geometry::covered_by(p, g)` from
/// `boost/geometry/algorithms/covered_by.hpp`. Same dispatch story as
/// [`within`]; the per-geometry impls differ only in how they treat
/// "result code 0" (boundary) — see the result-code table in
/// [`geometry_strategy::within`].
#[inline]
#[must_use]
pub fn covered_by<P, G>(p: &P, g: &G) -> bool
where
    P: Point,
    G: Geometry,
    G::Kind: WithinStrategyForKind,
    <G::Kind as WithinStrategyForKind>::S: WithinStrategy<P, G>,
{
    <<G::Kind as WithinStrategyForKind>::S as Default>::default().covered_by(p, g)
}

#[cfg(test)]
mod tests {
    //! Reference values from `geometry/test/strategies/winding.cpp:19-73`
    //! (the Cartesian section). Each test cites the C++ line(s) it
    //! mirrors.

    use super::{covered_by, within};
    use geometry_cs::Cartesian;
    use geometry_model::{Box, Point2D, Polygon, Ring, polygon};

    type P = Point2D<f64, Cartesian>;

    fn pt(x: f64, y: f64) -> P {
        Point2D::new(x, y)
    }

    fn box_polygon() -> Polygon<P> {
        polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]]
    }

    /// `winding.cpp:30` — `b1` interior point.
    #[test]
    fn box_b1_inside() {
        assert!(within(&pt(1.0, 1.0), &box_polygon()));
    }

    /// `winding.cpp:31` — `b2` exterior point.
    #[test]
    fn box_b2_outside() {
        assert!(!within(&pt(3.0, 3.0), &box_polygon()));
    }

    /// `winding.cpp:34-37` — every corner is "officially false".
    #[test]
    fn box_corners_excluded() {
        let p = box_polygon();
        for (x, y) in [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)] {
            assert!(!within(&pt(x, y), &p), "corner ({x},{y})");
        }
    }

    /// `winding.cpp:40-43` — every side is "officially false".
    #[test]
    fn box_sides_excluded() {
        let p = box_polygon();
        for (x, y) in [(0.0, 1.0), (1.0, 2.0), (2.0, 1.0), (1.0, 0.0)] {
            assert!(!within(&pt(x, y), &p), "side ({x},{y})");
        }
    }

    fn triangle() -> Polygon<P> {
        polygon![[(0.0, 0.0), (0.0, 4.0), (6.0, 0.0), (0.0, 0.0)]]
    }

    /// `winding.cpp:46`.
    #[test]
    fn triangle_t1_inside() {
        assert!(within(&pt(1.0, 1.0), &triangle()));
    }

    /// `winding.cpp:47`.
    #[test]
    fn triangle_t2_outside() {
        assert!(!within(&pt(3.0, 3.0), &triangle()));
    }

    fn with_hole() -> Polygon<P> {
        polygon![
            [(0.0, 0.0), (0.0, 3.0), (3.0, 3.0), (3.0, 0.0), (0.0, 0.0)],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)]
        ]
    }

    /// `winding.cpp:58` — between outer and hole.
    #[test]
    fn hole_h1_in_outer_not_hole() {
        assert!(within(&pt(0.5, 0.5), &with_hole()));
    }

    /// `winding.cpp:59` — strict interior of the hole.
    #[test]
    fn hole_h2a_in_hole_not_within() {
        assert!(!within(&pt(1.5, 1.5), &with_hole()));
    }

    /// `covered_by` inverts the boundary rule: corners and sides ARE
    /// covered, exterior points are not.
    #[test]
    fn covered_by_includes_boundary() {
        let p = box_polygon();
        assert!(covered_by(&pt(0.0, 0.0), &p)); // corner
        assert!(covered_by(&pt(0.0, 1.0), &p)); // side
        assert!(covered_by(&pt(1.0, 1.0), &p)); // interior
        assert!(!covered_by(&pt(3.0, 3.0), &p)); // outside
    }

    /// `quickstart.qbk` — point `(3.7, 2.0)` in the quickstart
    /// polygon is within. Adds doc-visible coverage at the algorithm
    /// layer.
    #[test]
    fn quickstart_polygon_3_7_2_0_is_within() {
        let q: Polygon<P> = polygon![[
            (2.0, 1.3),
            (2.4, 1.7),
            (2.8, 1.8),
            (3.4, 1.2),
            (3.7, 1.6),
            (3.4, 2.0),
            (4.1, 3.0),
            (5.3, 2.6),
            (5.4, 1.2),
            (4.9, 0.8),
            (2.9, 0.7),
            (2.0, 1.3)
        ]];
        assert!(within(&pt(3.7, 2.0), &q));
    }

    /// Box geometry directly (not a polygon): strict-vs-non-strict
    /// per-dimension.
    #[test]
    fn box_geometry_within_and_covered_by() {
        let b = Box::from_corners(pt(0.0, 0.0), pt(2.0, 2.0));
        assert!(within(&pt(1.0, 1.0), &b));
        assert!(!within(&pt(0.0, 0.0), &b));
        assert!(covered_by(&pt(0.0, 0.0), &b));
        assert!(!covered_by(&pt(3.0, 3.0), &b));
    }

    /// Ring geometry directly (no exterior/interior split).
    #[test]
    fn ring_within_and_covered_by() {
        let r: Ring<P> = Ring::from_vec(vec![
            pt(0.0, 0.0),
            pt(0.0, 2.0),
            pt(2.0, 2.0),
            pt(2.0, 0.0),
            pt(0.0, 0.0),
        ]);
        assert!(within(&pt(1.0, 1.0), &r));
        assert!(!within(&pt(0.0, 0.0), &r));
        assert!(covered_by(&pt(0.0, 0.0), &r));
    }
}
