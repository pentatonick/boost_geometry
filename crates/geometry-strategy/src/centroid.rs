//! `CentroidStrategy<G>` — geometric centre of a geometry.
//!
//! Mirrors the per-CS centroid-strategy concept from
//! `boost/geometry/strategies/centroid/services.hpp` plus the Cartesian
//! implementations in `boost/geometry/strategies/cartesian/centroid_*.hpp`
//! and the per-kind dispatch in
//! `boost/geometry/algorithms/centroid.hpp`. Per-kind Cartesian formulas:
//!
//! * `Segment`, `Box`          → midpoint of endpoints / corners
//! * `Linestring`              → length-weighted midpoint of segments
//! * `Ring` (closed) / `Polygon` → area-weighted Bashein–Detmer formula
//! * `MultiPoint`              → arithmetic mean of points
//!
//! Each per-kind impl lives behind a different strategy unit-struct so
//! coherence stays disjoint — the same distinct-struct-per-kind trick as
//! `area` (see `strategies/cartesian/area.hpp` and the module docs of
//! [`crate::area`]). Rust cannot prove a single type is not both a
//! `Ring` and a `Polygon`, so a single strategy carrying overlapping
//! `impl CentroidStrategy<G>` blocks keyed off the open traits would be
//! rejected (E0119); the sibling unit-structs below each carry a single
//! concept-bounded impl (`impl<G: Ring> … for CartesianRingCentroid`, …)
//! — distinct `Self`, so no overlap. The
//! [`CentroidStrategyForKind`] picker then routes `G::Kind` (the tag
//! [`Geometry::Kind`] already carries) to the right struct, disjoint on
//! the tag. This opens every kind to any concept-adapted foreign type,
//! not just the `geometry-model` structs (see
//! `specs/open-tag-dispatch/`).
#![allow(
    clippy::similar_names,
    reason = "The centroid accumulators `sum_x`/`sum_y` are the natural, domain-standard names for the per-axis running sums."
)]

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::{BoxTag, LinestringTag, MultiPointTag, PolygonTag, RingTag, SameAs, SegmentTag};
use geometry_trait::{
    Box as BoxTrait, Geometry, Linestring as LinestringTrait, MultiPoint as MultiPointTrait,
    Point as PointTrait, PointMut, Polygon as PolygonTrait, Ring as RingTrait,
    Segment as SegmentTrait, box_max, box_min, segment_end, segment_start,
};

use crate::area::{AreaStrategy, ShoelaceArea};
use crate::cartesian::Pythagoras;
use crate::distance::DistanceStrategy;

/// A strategy for computing the centroid of `G`.
///
/// Mirrors the per-CS centroid-strategy concept from
/// `boost/geometry/strategies/centroid/services.hpp`. The Boost concept
/// exposes a stateful `apply(p1, p2, state)` accumulator plus a
/// `result(state)` reduction (see
/// `strategies/cartesian/centroid_bashein_detmer.hpp:173-231`); the Rust
/// analogue collapses the two phases into a single method
/// [`CentroidStrategy::centroid`] keyed on the geometry type.
pub trait CentroidStrategy<G: Geometry> {
    /// The output point type. Almost always `G::Point` — Boost picks the
    /// input point type by default
    /// (`strategies/default_centroid_result.hpp`).
    type Output: PointMut + Default;

    /// Compute the centroid of `g`.
    fn centroid(&self, g: &G) -> Self::Output;
}

/// Cartesian centroid for a [`geometry_trait::Ring`] — the Bashein–Detmer formula
/// (signed-area-weighted vertex pairs).
///
/// Mirrors `boost::geometry::strategy::centroid::bashein_detmer` from
/// `strategies/cartesian/centroid_bashein_detmer.hpp:173-231`, reached
/// through the `areal_tag` arm of
/// `boost/geometry/algorithms/centroid.hpp`.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianRingCentroid;

/// Cartesian centroid for a [`geometry_trait::Polygon`] — the [`CartesianRingCentroid`]
/// formula applied to every ring (exterior plus interiors), combined by
/// signed area.
///
/// Mirrors the polygon arm of
/// `boost/geometry/algorithms/centroid.hpp`: each interior ring's
/// (oppositely-wound, hence oppositely-signed) area-weighted centroid is
/// folded into the running sum, so a plain area-weighted combine already
/// performs the hole correction.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianPolygonCentroid;

/// Cartesian centroid for a [`geometry_trait::Linestring`] — length-weighted midpoint of
/// each segment, summed and divided by total length.
///
/// Mirrors the `linear_tag` arm of
/// `boost/geometry/algorithms/centroid.hpp` together with
/// `strategies/cartesian/centroid_average.hpp`, which averages segment
/// midpoints weighted by segment length.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianLinestringCentroid;

/// Cartesian centroid for a [`geometry_trait::Segment`] — `(start + end) / 2`.
///
/// Mirrors the `segment_tag` arm of
/// `boost/geometry/algorithms/centroid.hpp`, which returns the segment
/// midpoint.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianSegmentCentroid;

/// Cartesian centroid for a [`geometry_trait::Box`] — corner midpoint per dimension.
///
/// Mirrors the `box_tag` arm of
/// `boost/geometry/algorithms/centroid.hpp`
/// (`detail::centroid::centroid_box`), which returns the midpoint of the
/// min / max corners.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianBoxCentroid;

/// Cartesian centroid for a [`geometry_trait::MultiPoint`] — arithmetic mean of the
/// member points.
///
/// Mirrors the `pointlike_tag` arm of
/// `boost/geometry/algorithms/centroid.hpp` together with
/// `strategies/cartesian/centroid_average.hpp`.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianMultiPointCentroid;

// ---- helpers ---------------------------------------------------------

/// Build a 2-D point from its two coordinates via [`Default`] +
/// `set::<0>` / `set::<1>`. Shared by the areal (Bashein–Detmer) impls,
/// which are inherently 2-D — the C++ strategy reads only `get<0>` /
/// `get<1>` (`centroid_bashein_detmer.hpp:191-199`).
#[inline]
fn point_2d<P>(x: P::Scalar, y: P::Scalar) -> P
where
    P: PointTrait + PointMut + Default,
{
    let mut p = P::default();
    p.set::<0>(x);
    p.set::<1>(y);
    p
}

/// The scalar `2` (`ONE + ONE`) for the argument scalar type.
#[inline]
fn two<T: CoordinateScalar>() -> T {
    T::ONE + T::ONE
}

/// The scalar `3` for the argument scalar type — the `3 * sum_a2 = 6A`
/// divisor of `centroid_bashein_detmer.hpp:211-212`.
#[inline]
fn three<T: CoordinateScalar>() -> T {
    T::ONE + T::ONE + T::ONE
}

/// The Bashein–Detmer accumulator triple `(sum_a2, sum_x, sum_y)`, all in
/// the ring's scalar type.
type BasheinDetmerSums<R> = (
    <<R as Geometry>::Point as PointTrait>::Scalar,
    <<R as Geometry>::Point as PointTrait>::Scalar,
    <<R as Geometry>::Point as PointTrait>::Scalar,
);

/// Sum the Bashein–Detmer accumulators `(sum_a2, sum_x, sum_y)` over the
/// consecutive vertex pairs of `r`. Mirrors the per-segment `apply` at
/// `centroid_bashein_detmer.hpp:191-199`:
///
/// ```text
/// ai      = x1 * y2 - x2 * y1
/// sum_a2 += ai
/// sum_x  += ai * (x1 + x2)
/// sum_y  += ai * (y1 + y2)
/// ```
///
/// For an open ring the implicit `last -> first` closing pair is added
/// explicitly, mirroring the way [`crate::area`] closes an open ring.
fn bashein_detmer_sums<R>(r: &R) -> BasheinDetmerSums<R>
where
    R: RingTrait,
    R::Point: PointTrait,
{
    let zero = <R::Point as PointTrait>::Scalar::ZERO;
    let mut sum_a2 = zero;
    let mut sum_x = zero;
    let mut sum_y = zero;

    let mut acc = |a: &R::Point, b: &R::Point| {
        let x1 = a.get::<0>();
        let y1 = a.get::<1>();
        let x2 = b.get::<0>();
        let y2 = b.get::<1>();
        let ai = x1 * y2 - x2 * y1;
        sum_a2 = sum_a2 + ai;
        sum_x = sum_x + ai * (x1 + x2);
        sum_y = sum_y + ai * (y1 + y2);
    };

    let it = r.points();
    let next = it.clone().skip(1);
    for (a, b) in it.zip(next) {
        acc(a, b);
    }
    if matches!(r.closure(), geometry_trait::Closure::Open) {
        let mut first_it = r.points();
        if let Some(first) = first_it.next() {
            if let Some(last) = r.points().last() {
                acc(last, first);
            }
        }
    }

    (sum_a2, sum_x, sum_y)
}

// ---- Ring ------------------------------------------------------------
//
// Mirrors `strategy::centroid::bashein_detmer::result` at
// `centroid_bashein_detmer.hpp:202-231`: `Cx = sum_x / (3 * sum_a2)`,
// `Cy = sum_y / (3 * sum_a2)`. When `sum_a2 == 0` (a degenerate, zero-
// area ring) Boost's `result` returns `false` and the higher-level
// `centroid_polygon` falls back to the first ring vertex
// (`test/algorithms/centroid.cpp:50-57`); we mirror that fallback here.

impl<G> CentroidStrategy<G> for CartesianRingCentroid
where
    G: RingTrait,
    G::Point: PointTrait + PointMut + Default + Copy,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    ShoelaceArea: AreaStrategy<G, Out = <G::Point as PointTrait>::Scalar>,
{
    type Output = G::Point;

    fn centroid(&self, r: &G) -> G::Point {
        let (sum_a2, sum_x, sum_y) = bashein_detmer_sums(r);
        let zero = <G::Point as PointTrait>::Scalar::ZERO;
        if sum_a2 == zero {
            // Degenerate ring: fall back to the first vertex
            // (`centroid.cpp:50-57`). An empty ring yields the origin
            // (Default), matching a zero-init result point.
            return r.points().next().copied().unwrap_or_default();
        }
        let a3 = three::<<G::Point as PointTrait>::Scalar>() * sum_a2;
        point_2d::<G::Point>(sum_x / a3, sum_y / a3)
    }
}

// ---- Polygon ---------------------------------------------------------
//
// Mirrors the polygon arm of `algorithms/centroid.hpp`. Each ring
// contributes `signed_area_k * centroid_k`; the interior rings arrive
// with the opposite sign under `ShoelaceArea` (Boost's signed-area
// convention winds holes opposite the exterior), so a plain sum performs
// the hole subtraction. The result is `sum_c / sum_area`, degenerating
// to the exterior ring's first vertex when the total signed area is 0.

impl<G> CentroidStrategy<G> for CartesianPolygonCentroid
where
    G: PolygonTrait,
    G::Point: PointTrait + PointMut + Default + Copy,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    ShoelaceArea: AreaStrategy<G::Ring, Out = <G::Point as PointTrait>::Scalar>,
    CartesianRingCentroid: CentroidStrategy<G::Ring, Output = G::Point>,
{
    type Output = G::Point;

    fn centroid(&self, pg: &G) -> G::Point {
        let zero = <G::Point as PointTrait>::Scalar::ZERO;
        let mut sum_area = zero;
        let mut sum_x = zero;
        let mut sum_y = zero;

        let mut fold_ring = |ring: &G::Ring| {
            let area = ShoelaceArea.area(ring);
            let c = CartesianRingCentroid.centroid(ring);
            sum_area = sum_area + area;
            sum_x = sum_x + area * c.get::<0>();
            sum_y = sum_y + area * c.get::<1>();
        };

        fold_ring(pg.exterior());
        for inner in pg.interiors() {
            fold_ring(inner);
        }

        if sum_area == zero {
            return pg.exterior().points().next().copied().unwrap_or_default();
        }
        point_2d::<G::Point>(sum_x / sum_area, sum_y / sum_area)
    }
}

// ---- Linestring ------------------------------------------------------
//
// Mirrors the linear arm of `algorithms/centroid.hpp`: each segment
// contributes `seg_length * midpoint`, summed and divided by the total
// length. Degenerate (total length 0) falls back to the first point
// (`centroid.cpp:81-82`).

impl<G> CentroidStrategy<G> for CartesianLinestringCentroid
where
    G: LinestringTrait,
    G::Point: PointTrait + PointMut + Default + Copy,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    Pythagoras: DistanceStrategy<G::Point, G::Point, Out = <G::Point as PointTrait>::Scalar>,
{
    type Output = G::Point;

    fn centroid(&self, ls: &G) -> G::Point {
        let zero = <G::Point as PointTrait>::Scalar::ZERO;
        let half =
            <G::Point as PointTrait>::Scalar::ONE / two::<<G::Point as PointTrait>::Scalar>();
        let mut total_len = zero;
        let mut sum_x = zero;
        let mut sum_y = zero;

        let it = ls.points();
        let next = it.clone().skip(1);
        for (a, b) in it.zip(next) {
            let seg_len = Pythagoras.distance(a, b);
            let mid_x = (a.get::<0>() + b.get::<0>()) * half;
            let mid_y = (a.get::<1>() + b.get::<1>()) * half;
            total_len = total_len + seg_len;
            sum_x = sum_x + seg_len * mid_x;
            sum_y = sum_y + seg_len * mid_y;
        }

        if total_len == zero {
            return ls.points().next().copied().unwrap_or_default();
        }
        point_2d::<G::Point>(sum_x / total_len, sum_y / total_len)
    }
}

// ---- Segment ---------------------------------------------------------
//
// Mirrors the segment arm of `algorithms/centroid.hpp`: the midpoint of
// the two endpoints, per dimension.

impl<G> CentroidStrategy<G> for CartesianSegmentCentroid
where
    G: SegmentTrait,
    G::Point: PointTrait + PointMut + Default + Copy,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = G::Point;

    fn centroid(&self, s: &G) -> G::Point {
        let a = segment_start(s);
        let b = segment_end(s);
        midpoint(&a, &b)
    }
}

// ---- Box -------------------------------------------------------------
//
// Mirrors `detail::centroid::centroid_box` in
// `algorithms/centroid.hpp`: the midpoint of the min / max corners, per
// dimension.

impl<G> CentroidStrategy<G> for CartesianBoxCentroid
where
    G: BoxTrait,
    G::Point: PointTrait + PointMut + Default + Copy,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = G::Point;

    fn centroid(&self, b: &G) -> G::Point {
        let lo = box_min(b);
        let hi = box_max(b);
        midpoint(&lo, &hi)
    }
}

// ---- MultiPoint ------------------------------------------------------
//
// Mirrors the pointlike arm of `algorithms/centroid.hpp`: the arithmetic
// mean of the member points, per dimension. Degenerate (no points) falls
// back to the origin (a zero-init default point).

impl<G> CentroidStrategy<G> for CartesianMultiPointCentroid
where
    G: MultiPointTrait,
    G::ItemPoint: PointTrait + PointMut + Default + Copy,
    <<G::ItemPoint as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = G::ItemPoint;

    fn centroid(&self, mp: &G) -> G::ItemPoint {
        let zero = <G::ItemPoint as PointTrait>::Scalar::ZERO;
        let mut count = zero;
        let mut sum_x = zero;
        let mut sum_y = zero;
        for p in mp.points() {
            sum_x = sum_x + p.get::<0>();
            sum_y = sum_y + p.get::<1>();
            count = count + <G::ItemPoint as PointTrait>::Scalar::ONE;
        }
        if count == zero {
            return G::ItemPoint::default();
        }
        point_2d::<G::ItemPoint>(sum_x / count, sum_y / count)
    }
}

/// The per-dimension midpoint of two points — `(a + b) / 2` on
/// dimensions `0` and `1`. Shared by the [`geometry_trait::Segment`] and [`geometry_trait::Box`] impls,
/// which are both a two-corner midpoint. 2-D only, matching the rest of
/// this module and the reference test coverage.
#[inline]
fn midpoint<P>(a: &P, b: &P) -> P
where
    P: PointTrait + PointMut + Default,
{
    let half = P::Scalar::ONE / two::<P::Scalar>();
    let x = (a.get::<0>() + b.get::<0>()) * half;
    let y = (a.get::<1>() + b.get::<1>()) * half;
    point_2d::<P>(x, y)
}

/// Type-level "which centroid strategy does this geometry *kind* use".
///
/// One impl per [`geometry_tag`] kind tag, mapping each tag to its
/// per-kind [`CentroidStrategy`] struct above. Keyed on the **tag**
/// (`impl CentroidStrategyForKind for RingTag`) rather than on a concept
/// blanket (`impl<G: Ring> … for G`, which would overlap its `Polygon`
/// sibling — E0119) or on the concrete `geometry-model` structs (which
/// would keep `centroid` model-bound). Distinct tags never conflict, so
/// the picker is coherent; a concept-adapted foreign type resolves to the
/// same struct as the equivalent model value because they share a
/// `Kind`. The `geometry-algorithm::centroid` free function routes
/// `G → G::Kind → S` through this trait, staying strategy-less while
/// leaving room for the explicit-strategy `centroid_with`.
///
/// # Spherical / geographic centroid — DEFERRED (LA8.T3)
///
/// The per-kind impls above are all gated on
/// `<…::Cs>::Family: SameAs<CartesianFamily>`, so `centroid(&g)` is a
/// compile error for a spherical or geographic geometry — that is
/// intentional. Boost's *area* and *azimuth* have exact, published
/// reference values (which LA8.T1/T2/T4 reproduce), but Boost ships **no
/// dedicated spherical / geographic centroid test values**: its
/// `strategies/centroid/spherical.hpp` merely marks `Box` / `Segment`
/// "not applicable" and otherwise inherits the Cartesian
/// `centroid_average` (an arithmetic mean of lon/lat, *not* a true
/// on-sphere centroid). The LA8.T3 stub instead sketches a different
/// algorithm (project to 3-D unit normals, area-weight, normalise, map
/// back) with **no reference rows to validate against**.
///
/// Per the task's "prefer correctness over coverage — skip + document
/// rather than ship wrong math" directive, the non-Cartesian centroid is
/// deferred until a validated reference exists. Callers who need it today
/// can supply an explicit strategy through
/// `geometry_algorithm::centroid_with`. The `DefaultLength` /
/// `DefaultArea` / `DefaultAzimuth` family-keyed dispatch traits added in
/// LA8 give the eventual family impl a ready-made shape to follow.
#[doc(hidden)]
pub trait CentroidStrategyForKind {
    /// The per-kind [`CentroidStrategy`] struct this tag is computed with.
    type S: Default;
}

impl CentroidStrategyForKind for RingTag {
    type S = CartesianRingCentroid;
}

impl CentroidStrategyForKind for PolygonTag {
    type S = CartesianPolygonCentroid;
}

impl CentroidStrategyForKind for LinestringTag {
    type S = CartesianLinestringCentroid;
}

impl CentroidStrategyForKind for SegmentTag {
    type S = CartesianSegmentCentroid;
}

impl CentroidStrategyForKind for BoxTag {
    type S = CartesianBoxCentroid;
}

impl CentroidStrategyForKind for MultiPointTag {
    type S = CartesianMultiPointCentroid;
}

#[cfg(test)]
mod tests {
    //! Reference values from `geometry/test/algorithms/centroid.cpp`.
    //! `BOOST_CHECK_CLOSE` there uses a 0.0001 % tolerance; the exact
    //! reference doubles are reproduced with `1e-9` absolute tolerance.
    #![allow(
        clippy::float_cmp,
        reason = "centroids are compared with an explicit absolute tolerance, not `==`"
    )]

    use super::{
        CartesianBoxCentroid, CartesianLinestringCentroid, CartesianMultiPointCentroid,
        CartesianPolygonCentroid, CartesianRingCentroid, CartesianSegmentCentroid,
        CentroidStrategy,
    };
    use geometry_cs::Cartesian;
    use geometry_model::{Box, MultiPoint, Point2D, Polygon, Ring, Segment, linestring, polygon};
    use geometry_trait::Point as _;

    type Pt = Point2D<f64, Cartesian>;

    fn close_pt(got: &Pt, x: f64, y: f64, tol: f64) -> bool {
        (got.get::<0>() - x).abs() < tol && (got.get::<1>() - y).abs() < tol
    }

    // centroid.cpp:139 — ring "POLYGON((1 1, 1 2, 2 2, 2 1, 1 1))" → (1.5, 1.5)
    #[test]
    fn ring_centroid_unit_square_shift() {
        let r: Ring<Pt> = Ring::from_vec(vec![
            Pt::new(1., 1.),
            Pt::new(1., 2.),
            Pt::new(2., 2.),
            Pt::new(2., 1.),
            Pt::new(1., 1.),
        ]);
        let c = CartesianRingCentroid.centroid(&r);
        assert!(close_pt(&c, 1.5, 1.5, 1e-9));
    }

    // centroid.cpp:111-114 — the Bashein/Detmer reference ring →
    // (4.06923363095238, 1.65055803571429).
    #[test]
    fn ring_bashein_detmer_reference() {
        let r: Ring<Pt> = Ring::from_vec(vec![
            Pt::new(2., 1.3),
            Pt::new(2.4, 1.7),
            Pt::new(2.8, 1.8),
            Pt::new(3.4, 1.2),
            Pt::new(3.7, 1.6),
            Pt::new(3.4, 2.),
            Pt::new(4.1, 3.),
            Pt::new(5.3, 2.6),
            Pt::new(5.4, 1.2),
            Pt::new(4.9, 0.8),
            Pt::new(2.9, 0.7),
            Pt::new(2., 1.3),
        ]);
        let c = CartesianRingCentroid.centroid(&r);
        assert!(close_pt(
            &c,
            4.069_233_630_952_38,
            1.650_558_035_714_29,
            1e-9
        ));
    }

    // centroid.cpp:46 — POLYGON((0 0,0 10,10 10,10 0,0 0)) → (5, 5)
    #[test]
    fn polygon_10x10_square_centroid_is_5_5() {
        let pg: Polygon<Pt> = polygon![[(0., 0.), (0., 10.), (10., 10.), (10., 0.), (0., 0.)]];
        let c = CartesianPolygonCentroid.centroid(&pg);
        assert!(close_pt(&c, 5.0, 5.0, 1e-9));
    }

    // centroid.cpp:191-192 — POLYGON((0 0, 1 0, 1 1, 0 1, 0 0), ()) → (0.5, 0.5).
    // (Unit square, plus an empty interior ring is a no-op.)
    #[test]
    fn polygon_unit_square_centroid_is_half_half() {
        let pg: Polygon<Pt> = polygon![[(0., 0.), (1., 0.), (1., 1.), (0., 1.), (0., 0.)]];
        let c = CartesianPolygonCentroid.centroid(&pg);
        assert!(close_pt(&c, 0.5, 0.5, 1e-9));
    }

    // centroid.cpp:40-44 — the Bashein/Detmer reference polygon *with a
    // hole*. The C++ test asserts SQL Server's constant
    // `(4.0466264962959677, 1.6348996057331333)` with a 0.0001 %
    // `BOOST_CHECK_CLOSE` tolerance. Boost's own Bashein/Detmer kernel
    // (which this mirrors) produces the PostGIS / Oracle value
    // `(4.0466265060241, 1.63489959839357)` quoted at
    // `centroid_bashein_detmer.hpp:99` — the two agree to ~1e-8, well
    // inside 0.0001 %. We assert the value the algorithm actually
    // computes (PostGIS / Oracle) so the tolerance can stay tight.
    #[test]
    fn polygon_with_hole_reference() {
        let pg: Polygon<Pt> = polygon![
            [
                (2., 1.3),
                (2.4, 1.7),
                (2.8, 1.8),
                (3.4, 1.2),
                (3.7, 1.6),
                (3.4, 2.),
                (4.1, 3.),
                (5.3, 2.6),
                (5.4, 1.2),
                (4.9, 0.8),
                (2.9, 0.7),
                (2., 1.3)
            ],
            [(4., 2.), (4.2, 1.4), (4.8, 1.9), (4.4, 2.2), (4., 2.)]
        ];
        let c = CartesianPolygonCentroid.centroid(&pg);
        assert!(close_pt(
            &c,
            4.046_626_506_024_1,
            1.634_899_598_393_57,
            1e-9
        ));
    }

    // centroid.cpp:50 — invalid, self-intersecting (area = 0) polygon →
    // fall back to first vertex (1, 1).
    #[test]
    fn degenerate_zero_area_polygon_returns_first_vertex() {
        let pg: Polygon<Pt> = polygon![[
            (1., 1.),
            (4., -2.),
            (4., 2.),
            (10., 0.),
            (1., 0.),
            (10., 1.),
            (1., 1.)
        ]];
        let c = CartesianPolygonCentroid.centroid(&pg);
        assert!(close_pt(&c, 1.0, 1.0, 1e-9));
    }

    // centroid.cpp:73 — LINESTRING(1 1, 2 2, 3 3) → (2, 2)
    #[test]
    fn linestring_centroid_diagonal() {
        let ls = linestring![(1., 1.), (2., 2.), (3., 3.)];
        let c = CartesianLinestringCentroid.centroid(&ls);
        assert!(close_pt(&c, 2.0, 2.0, 1e-9));
    }

    // centroid.cpp:74 — LINESTRING(0 0,0 4, 4 4) → (1, 3)
    #[test]
    fn linestring_centroid_bent() {
        let ls = linestring![(0., 0.), (0., 4.), (4., 4.)];
        let c = CartesianLinestringCentroid.centroid(&ls);
        assert!(close_pt(&c, 1.0, 3.0, 1e-9));
    }

    // centroid.cpp:81 — degenerate (length 0) linestring → first point.
    #[test]
    fn linestring_degenerate_returns_first_point() {
        let ls = linestring![(1., 1.), (1., 1.)];
        let c = CartesianLinestringCentroid.centroid(&ls);
        assert!(close_pt(&c, 1.0, 1.0, 1e-9));
    }

    // centroid.cpp:109 — segment (1 1) → (3 3) → midpoint (2, 2)
    #[test]
    fn segment_midpoint() {
        let s = Segment::new(Pt::new(1., 1.), Pt::new(3., 3.));
        let c = CartesianSegmentCentroid.centroid(&s);
        assert!(close_pt(&c, 2.0, 2.0, 1e-12));
    }

    // centroid.cpp:131 — box "POLYGON((1 2,3 4))" → (2, 3)
    #[test]
    fn box_centroid() {
        let b: Box<Pt> = Box::from_corners(Pt::new(1., 2.), Pt::new(3., 4.));
        let c = CartesianBoxCentroid.centroid(&b);
        assert!(close_pt(&c, 2.0, 3.0, 1e-12));
    }

    // MultiPoint {(0,0),(2,0),(0,2)} → arithmetic mean (2/3, 2/3).
    #[test]
    fn multipoint_mean() {
        let mp: MultiPoint<Pt> =
            MultiPoint::from_vec(vec![Pt::new(0., 0.), Pt::new(2., 0.), Pt::new(0., 2.)]);
        let c = CartesianMultiPointCentroid.centroid(&mp);
        assert!(close_pt(&c, 2.0 / 3.0, 2.0 / 3.0, 1e-9));
    }
}
