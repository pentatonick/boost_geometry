//! Per-CS strategy for point-in-polygon containment (`within` /
//! `covered_by`).
//!
//! Mirrors the pieces of Boost.Geometry that collaborate to make
//! `boost::geometry::within(p, g)` / `boost::geometry::covered_by(p, g)`
//! work for any (point, polygonal | box) pair in any coordinate system:
//!
//! * `boost/geometry/strategies/within.hpp` — the per-CS
//!   `within`-strategy concept (apply/result two-phase),
//! * `boost/geometry/strategies/covered_by.hpp` — same concept reused,
//! * `boost/geometry/strategies/cartesian/point_in_poly_winding.hpp` —
//!   `cartesian_winding`, the default Cartesian PIP, implementing the
//!   classic winding-number algorithm with on-segment detection,
//! * `boost/geometry/strategies/cartesian/point_in_box.hpp` —
//!   `cartesian_point_in_box`, the per-corner strict / non-strict
//!   comparisons used to fold "is the point inside this axis-aligned
//!   box" into the dispatch.
//!
//! The Boost concept exposes a stateful three-step API — construct a
//! `state_type`, call `apply(point, s1, s2, state)` for every segment,
//! then call `result(state)` to read off `-1` / `0` / `+1` (outside /
//! boundary / interior). The Rust analogue collapses that three-step
//! shape into a single `within` / `covered_by` pair on
//! [`WithinStrategy`] because the per-segment walk is identical for
//! every CS — only the per-segment kernel changes.
//!
//! ## Coherence note
//!
//! Boost dispatches on the geometry's tag via partial template
//! specialisation — `dispatch::within<Point, Ring, _, ring_tag>` and
//! `dispatch::within<Point, Polygon, _, polygon_tag>` are mutually
//! exclusive because the C++ side can prove tags distinct. Rust's
//! trait system cannot prove a downstream type does not implement
//! several geometry traits at once, so two open blankets on one strategy
//! struct would collide (E0119). The port reproduces Boost's tag
//! dispatch instead: one **per-kind strategy struct** ([`WithinBox`],
//! [`WithinRing`], [`WithinPoly`]) carries a single concept-bounded
//! `WithinStrategy` impl — distinct `Self`, so no overlap — and the
//! tag-keyed [`WithinStrategyForKind`] picker routes `G::Kind` to the
//! right struct. Because the picker keys on the tag, any concept-adapted
//! foreign type resolves through the same path as the equivalent
//! `geometry-model` value (see `specs/open-tag-dispatch/`).
//!
//! [`crate::intersects`] reaches point-in-polygon containment through
//! the open [`WithinPoly`] strategy directly (not the algorithm-layer
//! `covered_by` free fn — that would be an upward crate dependency /
//! cycle), so both crates share the one open kernel.
//!
//! ## Result-code convention
//!
//! Mirrors Boost's `cartesian_winding::result` at
//! `strategy/cartesian/point_in_poly_winding.hpp:69-74`:
//!
//! | Boost code | Meaning            | `within` | `covered_by` |
//! |-----------:|--------------------|---------:|-------------:|
//! |       `-1` | outside            |  `false` |      `false` |
//! |        `0` | on the boundary    |  `false` |       `true` |
//! |       `+1` | strict interior    |   `true` |       `true` |
//!
//! ## Precision limit (Cartesian, `f64`)
//!
//! The winding kernel decides each point's side of a segment from the
//! sign of a cross product of coordinate *differences*. For `f64` that
//! sign is exact only while the operands stay within the mantissa: past
//! roughly `±2^26` (`67_108_864`) the products no longer fit in 53 bits
//! and the sign can flip, so a strict-interior / boundary / exterior
//! classification may be wrong for coordinates beyond that magnitude.
//! This is the same limit the overlay engine gates on with its
//! `SAFE_ABS_MAX` range guard, and it matches Boost: the non-rescaled
//! `cartesian_winding` shares the bound (Boost's rescaling only ever
//! applied at the overlay/turn layer, not here). `within` does **not**
//! reject out-of-range input — callers that work with coordinates beyond
//! `±2^26` must scale down first.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::{BoxTag, PolygonTag, RingTag, SameAs};
use geometry_trait::{
    Box as BoxTrait, Point as PointTrait, PointMut, Polygon as PolygonTrait, Ring as RingTrait,
    corner,
};

/// A strategy for point-in-geometry containment.
///
/// Mirrors the per-CS `within` strategy concept declared in
/// `boost/geometry/strategies/within.hpp` and refined per coordinate
/// system in `strategies/cartesian/point_in_poly_winding.hpp` /
/// `strategies/spherical/point_in_poly_winding.hpp`. The Boost concept
/// exposes a stateful `apply(point, s1, s2, state)` accumulator plus a
/// final `result(state)` reduction; the Rust analogue collapses the
/// two phases into a single `within` / `covered_by` pair keyed on the
/// geometry type, because the per-segment walk shape is identical for
/// every CS — only the per-segment kernel changes.
pub trait WithinStrategy<P: PointTrait, G> {
    /// `true` iff `p` lies in the strict interior of `g`.
    ///
    /// Mirrors `boost::geometry::within(p, g, strategy)` from
    /// `boost/geometry/algorithms/within.hpp` resolved through
    /// `cartesian_winding::result == 1` at
    /// `strategy/cartesian/point_in_poly_winding.hpp:69-74`.
    fn within(&self, p: &P, g: &G) -> bool;

    /// `true` iff `p` lies in the strict interior **or** on the
    /// boundary of `g`.
    ///
    /// Mirrors `boost::geometry::covered_by(p, g, strategy)` from
    /// `boost/geometry/algorithms/covered_by.hpp` resolved through
    /// `cartesian_winding::result >= 0` at the same lines.
    fn covered_by(&self, p: &P, g: &G) -> bool;
}

// =====================================================================
// Per-kind strategy structs + tag-keyed picker
// =====================================================================
//
// Each struct carries the kernel for one kind, bound on the *open*
// concept (`G: Box`/`Ring`/`Polygon`) so any adapted foreign type
// resolves. Distinct `Self` per kind ⇒ no overlap.
//
// * Box     — `strategy::within::cartesian_point_in_box::apply`
//             (`strategy/cartesian/point_in_box.hpp:55-93`).
// * Ring    — `cartesian_winding_base::apply`
//             (`strategy/cartesian/point_in_poly_winding.hpp:91-131`).
// * Polygon — `detail::within::point_in_polygon::apply`
//             (`algorithms/detail/within/point_in_geometry.hpp:200-244`):
//             within the exterior and not covered_by any hole.

/// Open point-in-box strategy. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct WithinBox;
/// Open point-in-ring (winding number) strategy. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct WithinRing;
/// Open point-in-polygon (winding number, hole-aware) strategy. See the
/// [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct WithinPoly;

impl<P, G> WithinStrategy<P, G> for WithinBox
where
    G: BoxTrait<Point = P>,
    P: PointMut,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn within(&self, p: &P, b: &G) -> bool {
        let x = p.get::<0>();
        let y = p.get::<1>();
        let xmin = b.get_indexed::<{ corner::MIN }, 0>();
        let ymin = b.get_indexed::<{ corner::MIN }, 1>();
        let xmax = b.get_indexed::<{ corner::MAX }, 0>();
        let ymax = b.get_indexed::<{ corner::MAX }, 1>();
        xmin < x && x < xmax && ymin < y && y < ymax
    }

    #[inline]
    fn covered_by(&self, p: &P, b: &G) -> bool {
        let x = p.get::<0>();
        let y = p.get::<1>();
        let xmin = b.get_indexed::<{ corner::MIN }, 0>();
        let ymin = b.get_indexed::<{ corner::MIN }, 1>();
        let xmax = b.get_indexed::<{ corner::MAX }, 0>();
        let ymax = b.get_indexed::<{ corner::MAX }, 1>();
        xmin <= x && x <= xmax && ymin <= y && y <= ymax
    }
}

impl<P, G> WithinStrategy<P, G> for WithinRing
where
    G: RingTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn within(&self, p: &P, r: &G) -> bool {
        winding_result(p, r) == InOut::Interior
    }

    #[inline]
    fn covered_by(&self, p: &P, r: &G) -> bool {
        !matches!(winding_result(p, r), InOut::Exterior)
    }
}

impl<P, G> WithinStrategy<P, G> for WithinPoly
where
    G: PolygonTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn within(&self, p: &P, pg: &G) -> bool {
        if !WithinRing.within(p, pg.exterior()) {
            return false;
        }
        for hole in pg.interiors() {
            if WithinRing.covered_by(p, hole) {
                return false;
            }
        }
        true
    }

    #[inline]
    fn covered_by(&self, p: &P, pg: &G) -> bool {
        if !WithinRing.covered_by(p, pg.exterior()) {
            return false;
        }
        for hole in pg.interiors() {
            if WithinRing.within(p, hole) {
                return false;
            }
        }
        true
    }
}

/// Type-level "which `WithinStrategy` struct does this geometry *kind*
/// use". One impl per [`geometry_tag`] kind tag, keyed on the tag (never a
/// concept blanket — that would overlap, E0119). The
/// [`crate::within`]/[`crate::covered_by`] free functions route
/// `G → G::Kind → S` through this trait.
#[doc(hidden)]
pub trait WithinStrategyForKind {
    /// The per-kind [`WithinStrategy`] struct this tag is computed with.
    type S: Default;
}

impl WithinStrategyForKind for BoxTag {
    type S = WithinBox;
}
impl WithinStrategyForKind for RingTag {
    type S = WithinRing;
}
impl WithinStrategyForKind for PolygonTag {
    type S = WithinPoly;
}

// ---- Winding-number kernel ------------------------------------------

/// Tri-state outcome of the winding-number walk.
///
/// Mirrors the integer return of `cartesian_winding::result` at
/// `strategy/cartesian/point_in_poly_winding.hpp:69-74`:
/// `-1` outside, `0` on the boundary, `+1` strict interior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InOut {
    Exterior,
    Boundary,
    Interior,
}

/// Walk the segments of `r` accumulating the winding count and the
/// "on a segment" flag, then collapse to an [`InOut`].
///
/// Mirrors `cartesian_winding_base::apply` together with `result` at
/// `strategy/cartesian/point_in_poly_winding.hpp:91-131, 69-74`. The
/// closing edge for an open ring is added explicitly here — matches
/// Boost's `closed_clockwise_view` wrap done one layer up at
/// `algorithms/detail/within/point_in_geometry.hpp`.
fn winding_result<P, R>(p: &P, r: &R) -> InOut
where
    P: PointTrait,
    P::Scalar: CoordinateScalar,
    R: RingTrait<Point = P>,
{
    let mut count: i32 = 0;
    let mut it = r.points();
    let Some(mut prev) = it.next() else {
        // Empty ring — outside by convention. Mirrors the
        // `boost::size(ring) < minimum_ring_size` guard at
        // `algorithms/detail/within/point_in_geometry.hpp:198`.
        return InOut::Exterior;
    };
    let first = prev;
    for curr in it {
        match apply_segment(p, prev, curr) {
            Step::Touches => return InOut::Boundary,
            Step::Count(c) => count += c,
        }
        prev = curr;
    }
    // Open ring: close the loop explicitly. For a closed ring `prev`
    // already equals `first`, so the kernel below returns `count = 0`
    // (eq1 && eq2 with equal y-spans → no touch, no contribution).
    match apply_segment(p, prev, first) {
        Step::Touches => return InOut::Boundary,
        Step::Count(c) => count += c,
    }
    if count == 0 {
        InOut::Exterior
    } else {
        InOut::Interior
    }
}

/// Per-segment outcome of the winding kernel.
#[derive(Debug, Clone, Copy)]
enum Step {
    /// The point lies on this segment; the walk can short-circuit.
    Touches,
    /// Contribution to the running winding count.
    Count(i32),
}

/// Single-segment contribution of the winding number, plus the
/// on-segment short-circuit.
///
/// Mirrors `cartesian_winding_base::apply` at
/// `strategy/cartesian/point_in_poly_winding.hpp:91-131` together
/// with its `check_segment` / `check_touch` / `calculate_count` /
/// `side_equal` helpers (lines 139-217). The cartesian side strategy
/// reduces to the cross-product sign
/// `(s2.x - s1.x) * (p.y - s1.y) - (s2.y - s1.y) * (p.x - s1.x)`
/// (`strategy/cartesian/side_by_triangle.hpp:178-200`).
fn apply_segment<P>(p: &P, s1: &P, s2: &P) -> Step
where
    P: PointTrait,
    P::Scalar: CoordinateScalar,
{
    let px = p.get::<0>();
    let py = p.get::<1>();
    let s1x = s1.get::<0>();
    let s2x = s2.get::<0>();
    let s1y = s1.get::<1>();
    let s2y = s2.get::<1>();

    let eq1 = s1x == px;
    let eq2 = s2x == px;

    // check_touch: vertical segment exactly on the point's x.
    // Mirrors lines 154-184 of point_in_poly_winding.hpp.
    if eq1 && eq2 {
        let (lo, hi) = if s1y <= s2y { (s1y, s2y) } else { (s2y, s1y) };
        if lo <= py && py <= hi {
            return Step::Touches;
        }
        return Step::Count(0);
    }

    // calculate_count: lines 186-203.
    let count = if eq1 {
        if s2x > px { 1 } else { -1 }
    } else if eq2 {
        if s1x > px { -1 } else { 1 }
    } else if s1x < px && s2x > px {
        2
    } else if s2x < px && s1x > px {
        -2
    } else {
        0
    };

    if count == 0 {
        return Step::Count(0);
    }

    // side: for ±1, side_equal; for ±2, cartesian side cross product.
    // Mirrors lines 100-110 of point_in_poly_winding.hpp.
    let side: i32 = if count == 1 || count == -1 {
        let se = if eq1 { s1 } else { s2 };
        let sey = se.get::<1>();
        if py == sey {
            0
        } else if py < sey {
            -count
        } else {
            count
        }
    } else {
        // Cartesian side: sign of (s2 - s1) × (p - s1).
        // Mirrors `side_by_triangle::side_value` at
        // strategy/cartesian/side_by_triangle.hpp:178-200.
        let cross = (s2x - s1x) * (py - s1y) - (s2y - s1y) * (px - s1x);
        if cross > P::Scalar::ZERO {
            1
        } else if cross < P::Scalar::ZERO {
            -1
        } else {
            0
        }
    };

    if side == 0 {
        return Step::Touches;
    }

    if side * count > 0 {
        Step::Count(count)
    } else {
        Step::Count(0)
    }
}

#[cfg(test)]
mod tests {
    //! Reference values from `geometry/test/strategies/winding.cpp:19-73`
    //! (the Cartesian section). Each test cites the C++ line(s) it
    //! mirrors.

    use super::{WithinBox, WithinPoly, WithinRing, WithinStrategy};
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
        assert!(WithinPoly.within(&pt(1.0, 1.0), &box_polygon()));
    }

    /// `winding.cpp:31` — `b2` exterior point.
    #[test]
    fn box_b2_outside() {
        assert!(!WithinPoly.within(&pt(3.0, 3.0), &box_polygon()));
    }

    /// `winding.cpp:34-37` — all four corners are "officially false".
    #[test]
    fn box_corners_are_not_within() {
        let p = box_polygon();
        for (x, y) in [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)] {
            assert!(!WithinPoly.within(&pt(x, y), &p), "corner ({x},{y})");
        }
    }

    /// `winding.cpp:40-43` — all four sides are "officially false".
    #[test]
    fn box_sides_are_not_within() {
        let p = box_polygon();
        for (x, y) in [(0.0, 1.0), (1.0, 2.0), (2.0, 1.0), (1.0, 0.0)] {
            assert!(!WithinPoly.within(&pt(x, y), &p), "side ({x},{y})");
        }
    }

    /// `winding.cpp:46-47` — triangle interior / exterior.
    #[test]
    fn triangle_interior_and_exterior() {
        let t: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 4.0), (6.0, 0.0), (0.0, 0.0)]];
        assert!(WithinPoly.within(&pt(1.0, 1.0), &t));
        assert!(!WithinPoly.within(&pt(3.0, 3.0), &t));
    }

    /// `winding.cpp:58-60` — polygon-with-hole semantics: inside the
    /// outer-but-outside the hole is within; inside the hole is not.
    #[test]
    fn hole_semantics() {
        let with_hole: Polygon<P> = polygon![
            [(0.0, 0.0), (0.0, 3.0), (3.0, 3.0), (3.0, 0.0), (0.0, 0.0)],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)]
        ];
        // h1
        assert!(WithinPoly.within(&pt(0.5, 0.5), &with_hole));
        // h2a — inside the hole
        assert!(!WithinPoly.within(&pt(1.5, 1.5), &with_hole));
    }

    /// `covered_by` inverts the boundary rule: corners and sides are
    /// covered, but external points are not. Mirrors the Boost
    /// `result >= 0` projection at
    /// `strategy/cartesian/point_in_poly_winding.hpp:69-74`.
    #[test]
    fn covered_by_includes_boundary() {
        let p = box_polygon();
        assert!(WithinPoly.covered_by(&pt(0.0, 0.0), &p));
        assert!(WithinPoly.covered_by(&pt(0.0, 1.0), &p));
        assert!(WithinPoly.covered_by(&pt(1.0, 1.0), &p));
        assert!(!WithinPoly.covered_by(&pt(3.0, 3.0), &p));
    }

    /// `Box`-as-geometry path: strict-vs-non-strict per-dimension.
    /// Mirrors `cartesian_point_in_box` at
    /// `strategy/cartesian/point_in_box.hpp:55-93`.
    #[test]
    fn box_geometry_strict_vs_non_strict() {
        let b = Box::from_corners(pt(0.0, 0.0), pt(2.0, 2.0));
        // strict interior
        assert!(WithinBox.within(&pt(1.0, 1.0), &b));
        // boundary: corner
        assert!(!WithinBox.within(&pt(0.0, 0.0), &b));
        assert!(WithinBox.covered_by(&pt(0.0, 0.0), &b));
        // boundary: side
        assert!(!WithinBox.within(&pt(0.0, 1.0), &b));
        assert!(WithinBox.covered_by(&pt(0.0, 1.0), &b));
        // outside
        assert!(!WithinBox.within(&pt(3.0, 3.0), &b));
        assert!(!WithinBox.covered_by(&pt(3.0, 3.0), &b));
    }

    /// Ring-only path — same kernel, no exterior/interior split.
    #[test]
    fn ring_within_smoke() {
        let r: Ring<P> = Ring::from_vec(vec![
            pt(0.0, 0.0),
            pt(0.0, 2.0),
            pt(2.0, 2.0),
            pt(2.0, 0.0),
            pt(0.0, 0.0),
        ]);
        assert!(WithinRing.within(&pt(1.0, 1.0), &r));
        assert!(!WithinRing.within(&pt(0.0, 0.0), &r));
        assert!(WithinRing.covered_by(&pt(0.0, 0.0), &r));
    }

    /// Open ring (no repeated closing vertex): the kernel must add
    /// the implicit `last -> first` edge so containment still works.
    #[test]
    fn open_ring_closes_implicitly() {
        let mut r = Ring::<P, true, false>::new();
        r.push(pt(0.0, 0.0));
        r.push(pt(0.0, 2.0));
        r.push(pt(2.0, 2.0));
        r.push(pt(2.0, 0.0));
        assert!(WithinRing.within(&pt(1.0, 1.0), &r));
        assert!(!WithinRing.within(&pt(3.0, 3.0), &r));
    }
}
