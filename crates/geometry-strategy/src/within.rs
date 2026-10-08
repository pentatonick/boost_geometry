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
//! `geometry-model` value.
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
//! ## Precision (Cartesian, floating point)
//!
//! The ring walk is the shared [`crate::winding`] kernel, so a point's side
//! of an edge is Boost's `side_by_triangle`, not an exact sign: a point
//! within an epsilon of an edge — scaled by the edge's extent — is on it,
//! as it is to Boost, at any magnitude.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::{BoxTag, PolygonTag, RingTag, SameAs};
use geometry_trait::{
    Box as BoxTrait, Point as PointTrait, PointMut, Polygon as PolygonTrait, Ring as RingTrait,
    corner, fold_dims, ordinate,
};

use crate::winding::{PointLocation, polygon_location, ring_location};

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
        fold_dims(true, p, |inside, p, d| {
            inside && box_dimension_contains(p, b, d, true)
        })
    }

    #[inline]
    fn covered_by(&self, p: &P, b: &G) -> bool {
        fold_dims(true, p, |inside, p, d| {
            inside && box_dimension_contains(p, b, d, false)
        })
    }
}

/// Does `p`'s ordinate `d` lie inside the box's `[min, max]` on that
/// axis — strictly (`within`) or inclusively (`covered_by`)? One arm per
/// dimension up to `MAX_DIM`, the per-dimension loop of
/// `strategy/cartesian/point_in_box.hpp:55-93`.
#[inline]
fn box_dimension_contains<P, G>(p: &P, b: &G, d: usize, strict: bool) -> bool
where
    G: BoxTrait<Point = P>,
    P: PointMut,
{
    let (min, max) = match d {
        0 => (
            b.get_indexed::<{ corner::MIN }, 0>(),
            b.get_indexed::<{ corner::MAX }, 0>(),
        ),
        1 => (
            b.get_indexed::<{ corner::MIN }, 1>(),
            b.get_indexed::<{ corner::MAX }, 1>(),
        ),
        2 => (
            b.get_indexed::<{ corner::MIN }, 2>(),
            b.get_indexed::<{ corner::MAX }, 2>(),
        ),
        3 => (
            b.get_indexed::<{ corner::MIN }, 3>(),
            b.get_indexed::<{ corner::MAX }, 3>(),
        ),
        _ => unreachable!("fold_dims caps at MAX_DIM"),
    };
    let value = ordinate(p, d);
    if strict {
        min < value && value < max
    } else {
        min <= value && value <= max
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
        ring_location(xy(p), r.points().map(xy)) == PointLocation::Interior
    }

    #[inline]
    fn covered_by(&self, p: &P, r: &G) -> bool {
        ring_location(xy(p), r.points().map(xy)) != PointLocation::Exterior
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
        location_in_polygon(p, pg) == PointLocation::Interior
    }

    #[inline]
    fn covered_by(&self, p: &P, pg: &G) -> bool {
        location_in_polygon(p, pg) != PointLocation::Exterior
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

// ---- Point location --------------------------------------------------

/// The planar ordinates of `p`: the walk is two-dimensional.
fn xy<P: PointTrait>(p: &P) -> (P::Scalar, P::Scalar) {
    (p.get::<0>(), p.get::<1>())
}

/// Where `p` lies relative to `pg`, by the [`crate::winding`] kernel.
///
/// Mirrors `point_in_geometry<Polygon>` driving `cartesian_winding` at
/// `algorithms/detail/within/point_in_geometry.hpp`.
fn location_in_polygon<P, G>(p: &P, pg: &G) -> PointLocation
where
    G: PolygonTrait<Point = P>,
    P: PointTrait,
{
    polygon_location(
        xy(p),
        pg.exterior().points().map(xy),
        pg.interiors().map(|hole| hole.points().map(xy)),
    )
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

    /// `point_in_box.hpp` loops over every dimension: a point above a
    /// 3-D box is outside it even when its `x`/`y` fall inside.
    #[test]
    fn box_containment_reads_the_third_dimension() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let b = Box::from_corners(P3::new(0.0, 0.0, 0.0), P3::new(2.0, 2.0, 2.0));
        assert!(WithinBox.within(&P3::new(1.0, 1.0, 1.0), &b));
        assert!(!WithinBox.within(&P3::new(1.0, 1.0, 10.0), &b));
        assert!(!WithinBox.covered_by(&P3::new(1.0, 1.0, 10.0), &b));
        assert!(WithinBox.covered_by(&P3::new(1.0, 1.0, 2.0), &b));
        assert!(!WithinBox.within(&P3::new(1.0, 1.0, 2.0), &b));
    }

    /// A 4-D point built ordinate-wise, since `Point::new` stops at
    /// three arguments.
    fn p4(v: [f64; 4]) -> geometry_model::Point<f64, 4> {
        use geometry_trait::set_ordinate;
        let mut p = geometry_model::Point::<f64, 4>::default();
        for (d, value) in v.into_iter().enumerate() {
            set_ordinate(&mut p, d, value);
        }
        p
    }

    /// `fold_dims` runs to the point's own arity, so the last row of the
    /// per-dimension lookup is only reached by a point of the largest
    /// arity the table supports. A point inside on x, y and z and
    /// outside on the fourth axis is the input that distinguishes a
    /// present row from a missing one — and the strict/inclusive split
    /// must hold on that axis exactly as it does on x.
    #[test]
    fn box_containment_reads_the_fourth_dimension() {
        let b = Box::from_corners(p4([0.0; 4]), p4([2.0; 4]));

        assert!(WithinBox.within(&p4([1.0; 4]), &b));
        assert!(!WithinBox.within(&p4([1.0, 1.0, 1.0, 10.0]), &b));
        assert!(!WithinBox.covered_by(&p4([1.0, 1.0, 1.0, 10.0]), &b));

        // On the boundary of the fourth axis only: covered, not within.
        assert!(WithinBox.covered_by(&p4([1.0, 1.0, 1.0, 2.0]), &b));
        assert!(!WithinBox.within(&p4([1.0, 1.0, 1.0, 2.0]), &b));
    }

    /// Past the last row the lookup must fail loudly rather than fall
    /// through to another axis, which would answer with a comparison
    /// the caller never asked for.
    #[test]
    #[should_panic(expected = "fold_dims caps at MAX_DIM")]
    fn box_dimension_contains_panics_past_max_dim() {
        let b = Box::from_corners(p4([0.0; 4]), p4([2.0; 4]));
        let _ = super::box_dimension_contains(&p4([1.0; 4]), &b, 4, true);
    }
}
