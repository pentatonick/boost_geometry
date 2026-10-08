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
//! not just the `geometry-model` structs.
#![allow(
    clippy::similar_names,
    reason = "The centroid accumulators `sum_x`/`sum_y` are the natural, domain-standard names for the per-axis running sums."
)]

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::{
    BoxTag, LinestringTag, MultiPointTag, MultiPolygonTag, PolygonTag, RingTag, SameAs, SegmentTag,
};
use geometry_trait::{
    Box as BoxTrait, Geometry, Linestring as LinestringTrait, MultiPoint as MultiPointTrait,
    MultiPolygon as MultiPolygonTrait, Point as PointTrait, PointMut, Polygon as PolygonTrait,
    Ring as RingTrait, Segment as SegmentTrait, box_max, box_min, fold_dims, ordinate, segment_end,
    segment_start, set_ordinate,
};

use crate::area::{AreaStrategy, ShoelaceArea};
use crate::cartesian::Pythagoras;
use crate::distance::DistanceStrategy;

/// The scalar a centroid of `P` coordinates is computed in: `f64` for
/// integer coordinates, which Boost likewise accumulates in `double`
/// before it converts the result back with `numeric_cast`.
type Measure<P> = <<P as PointTrait>::Scalar as CoordinateScalar>::Measure;

/// Largest `DIM` a point may have. Matches `geometry_trait`'s `MAX_DIM`,
/// the dimensions [`fold_dims`] visits.
const MAX_DIM: usize = 4;

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

/// Cartesian centroid for a [`geometry_trait::MultiPolygon`] — one
/// Bashein–Detmer accumulator over every ring of every member.
///
/// Mirrors the multi-polygon arm of
/// `boost/geometry/algorithms/centroid.hpp`.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianMultiPolygonCentroid;

/// Cartesian centroid for a [`geometry_trait::Linestring`] — length-weighted midpoint of
/// each segment, summed and divided by total length.
///
/// Mirrors the `linear_tag` arm of
/// `boost/geometry/algorithms/centroid.hpp` together with
/// `strategies/cartesian/centroid_weighted_length.hpp`, which averages
/// segment midpoints weighted by segment length.
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

/// `p − origin` in the measure: the translation Boost's
/// `translating_transformer` applies before it accumulates the centroid of
/// an areal geometry (`algorithms/detail/centroid/translating_transformer.hpp`).
/// The accumulators then hold products of the geometry's extent rather than
/// of its absolute position, which would cancel catastrophically far from
/// the coordinate origin. A linear or pointlike geometry is accumulated
/// untranslated, as Boost's identity transformer leaves it.
#[inline]
fn translated<P: PointTrait>(p: &P, origin: &P) -> (Measure<P>, Measure<P>) {
    (
        p.get::<0>().to_measure() - origin.get::<0>().to_measure(),
        p.get::<1>().to_measure() - origin.get::<1>().to_measure(),
    )
}

/// Build a 2-D point from its two coordinates via [`Default`] +
/// `set::<0>` / `set::<1>`, each converted to the point's scalar — Boost's
/// `numeric_cast`.
#[inline]
fn point_2d<P>(x: Measure<P>, y: Measure<P>) -> P
where
    P: PointTrait + PointMut + Default,
{
    let mut p = P::default();
    p.set::<0>(P::Scalar::from_measure(x));
    p.set::<1>(P::Scalar::from_measure(y));
    p
}

/// [`point_2d`] for a [`translated`] centroid, moved back by `origin` once
/// converted — `translating_transformer::apply_reverse`, in Boost's order.
/// Shared by the areal (Bashein–Detmer) impls, which are inherently 2-D —
/// the C++ strategy reads only `get<0>` / `get<1>`
/// (`centroid_bashein_detmer.hpp:191-199`).
#[inline]
fn translated_back<P>(x: Measure<P>, y: Measure<P>, origin: &P) -> P
where
    P: PointTrait + PointMut + Default,
{
    let mut p = point_2d::<P>(x, y);
    p.set::<0>(p.get::<0>() + origin.get::<0>());
    p.set::<1>(p.get::<1>() + origin.get::<1>());
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

/// Boost's Bashein–Detmer state, `bashein_detmer::sums`
/// (`strategies/cartesian/centroid_bashein_detmer.hpp:139-167`): one count
/// and three running sums that every ring of an areal geometry adds to, in
/// order, once translated by `origin`.
struct BasheinDetmer<'a, P: PointTrait> {
    origin: &'a P,
    count: usize,
    sum_a2: Measure<P>,
    sum_x: Measure<P>,
    sum_y: Measure<P>,
}

impl<'a, P> BasheinDetmer<'a, P>
where
    P: PointTrait + PointMut + Default,
{
    fn new(origin: &'a P) -> Self {
        let zero = <Measure<P> as CoordinateScalar>::ZERO;
        Self {
            origin,
            count: 0,
            sum_a2: zero,
            sum_x: zero,
            sum_y: zero,
        }
    }

    /// Add every edge of `ring`, the closing edge of an open ring included:
    /// `centroid_range_state` over the ring's closed view
    /// (`algorithms/centroid.hpp:163-195`).
    fn add_ring<R: RingTrait<Point = P>>(&mut self, ring: &R) {
        let mut points = ring.points();
        let Some(first) = points.next() else {
            return;
        };
        let first = translated(first, self.origin);
        let mut previous = first;
        for point in points {
            let point = translated(point, self.origin);
            self.add_edge(previous, point);
            previous = point;
        }
        if matches!(ring.closure(), geometry_trait::Closure::Open) {
            self.add_edge(previous, first);
        }
    }

    /// `bashein_detmer::apply` (`centroid_bashein_detmer.hpp:173-200`).
    fn add_edge(&mut self, (x1, y1): (Measure<P>, Measure<P>), (x2, y2): (Measure<P>, Measure<P>)) {
        let ai = x1 * y2 - y1 * x2;
        self.count += 1;
        self.sum_a2 = self.sum_a2 + ai;
        self.sum_x = self.sum_x + ai * (x1 + x2);
        self.sum_y = self.sum_y + ai * (y1 + y2);
    }

    /// The centroid, moved back by `origin`: `bashein_detmer::result`
    /// (`centroid_bashein_detmer.hpp:203-231`) and `apply_reverse`. `None`
    /// where Boost's `result` fails — for no edge, for an area of zero by
    /// `math::equals`, or for an area whose triple is not finite.
    fn result(&self) -> Option<P> {
        if self.count == 0
            || self
                .sum_a2
                .tolerant_eq(<Measure<P> as CoordinateScalar>::ZERO)
        {
            return None;
        }
        let a3 = three::<Measure<P>>() * self.sum_a2;
        a3.is_finite()
            .then(|| translated_back::<P>(self.sum_x / a3, self.sum_y / a3, self.origin))
    }
}

// ---- Ring ------------------------------------------------------------
//
// Mirrors `centroid_range` under `centroid_linear_areal`
// (`algorithms/centroid.hpp:134-157,197-226,357-369`): an empty ring is an
// error (Boost's `centroid_exception`), a ring of one point is that point,
// and a ring Boost's `result` fails on falls back to its first point,
// `point_on_border`.
impl<G> CentroidStrategy<G> for CartesianRingCentroid
where
    G: RingTrait,
    G::Point: PointTrait + PointMut + Default + Copy,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    ShoelaceArea: AreaStrategy<G, Out = Measure<G::Point>>,
{
    type Output = G::Point;

    fn centroid(&self, r: &G) -> G::Point {
        let mut points = r.points();
        let first = *points.next().expect("centroid of an empty ring");
        if points.next().is_none() {
            return first;
        }
        let mut state = BasheinDetmer::new(&first);
        state.add_ring(r);
        state.result().unwrap_or(first)
    }
}

// ---- Polygon ---------------------------------------------------------
//
// Mirrors `centroid_polygon` (`algorithms/centroid.hpp:234-288`): the
// exterior decides as a ring does, then the exterior and every interior ring
// add to one state. The interior rings arrive with the opposite sign — Boost
// winds holes opposite the exterior — so the plain sum subtracts them.
impl<G> CentroidStrategy<G> for CartesianPolygonCentroid
where
    G: PolygonTrait,
    G::Point: PointTrait + PointMut + Default + Copy,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    ShoelaceArea: AreaStrategy<G::Ring, Out = Measure<G::Point>>,
    CartesianRingCentroid: CentroidStrategy<G::Ring, Output = G::Point>,
{
    type Output = G::Point;

    fn centroid(&self, pg: &G) -> G::Point {
        let mut exterior = pg.exterior().points();
        let first = *exterior
            .next()
            .expect("centroid of a polygon with an empty exterior ring");
        if exterior.next().is_none() {
            return first;
        }
        let mut state = BasheinDetmer::new(&first);
        state.add_ring(pg.exterior());
        for inner in pg.interiors() {
            state.add_ring(inner);
        }
        state.result().unwrap_or(first)
    }
}

// ---- MultiPolygon ----------------------------------------------------
//
// Mirrors `centroid_multi<centroid_polygon_state>`
// (`algorithms/centroid.hpp:314-354`), which runs one state over every ring
// of every member and divides once. That is also why it accumulates rather
// than combining per-part centroids: a member with zero area drops out of an
// area-weighted combine but still adds to the running numerator, and Boost
// keeps that contribution. The translation origin is the multi-polygon's
// first point; a failed `result` falls back to the first point of the first
// exterior ring that has one, `point_on_border`.
impl<G> CentroidStrategy<G> for CartesianMultiPolygonCentroid
where
    G: MultiPolygonTrait,
    G::Point: PointTrait + PointMut + Default + Copy,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = G::Point;

    fn centroid(&self, mp: &G) -> G::Point {
        let rings = || {
            mp.polygons()
                .flat_map(|polygon| core::iter::once(polygon.exterior()).chain(polygon.interiors()))
        };
        let origin = *rings()
            .find_map(|ring| ring.points().next())
            .expect("centroid of an empty multi-polygon");
        let mut state = BasheinDetmer::new(&origin);
        for ring in rings() {
            state.add_ring(ring);
        }
        state.result().unwrap_or_else(|| {
            mp.polygons()
                .find_map(|polygon| polygon.exterior().points().next().copied())
                .unwrap_or(origin)
        })
    }
}

// ---- Linestring ------------------------------------------------------
//
// Mirrors `centroid_range` with `weighted_length`
// (`strategies/cartesian/centroid_weighted_length.hpp:97-141`) under
// `centroid_linear_areal`: each segment adds its length and its midpoint
// weighted by it, in every dimension, and the sums are divided by the total
// length. An empty linestring is an error; a total length of zero by
// `math::equals`, or one that is not finite, falls back to the first point.
impl<G> CentroidStrategy<G> for CartesianLinestringCentroid
where
    G: LinestringTrait,
    G::Point: PointTrait + PointMut + Default + Copy,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    Pythagoras: DistanceStrategy<G::Point, G::Point, Out = Measure<G::Point>>,
{
    type Output = G::Point;

    fn centroid(&self, ls: &G) -> G::Point {
        let first = *ls.points().next().expect("centroid of an empty linestring");
        let zero = <Measure<G::Point> as CoordinateScalar>::ZERO;
        let two = two::<Measure<G::Point>>();
        let mut length = zero;
        let mut sums = [zero; MAX_DIM];
        for (a, b) in ls.points().zip(ls.points().skip(1)) {
            let d = Pythagoras.distance(a, b);
            length = length + d;
            let d_half = d / two;
            fold_dims((), a, |(), a, dimension| {
                let weighted_median = (ordinate(a, dimension).to_measure()
                    + ordinate(b, dimension).to_measure())
                    * d_half;
                sums[dimension] = sums[dimension] + weighted_median;
            });
        }
        if length.tolerant_eq(zero) || !length.is_finite() {
            return first;
        }
        let mut centroid = G::Point::default();
        fold_dims((), &first, |(), _, dimension| {
            set_ordinate(
                &mut centroid,
                dimension,
                <G::Point as PointTrait>::Scalar::from_measure(sums[dimension] / length),
            );
        });
        centroid
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
// mean of the member points, per dimension. An empty multi-point is an
// error, Boost's `centroid_exception` (`algorithms/centroid.hpp:320-328`).

impl<G> CentroidStrategy<G> for CartesianMultiPointCentroid
where
    G: MultiPointTrait,
    G::ItemPoint: PointTrait + PointMut + Default + Copy,
    <<G::ItemPoint as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = G::ItemPoint;

    fn centroid(&self, mp: &G) -> G::ItemPoint {
        let zero = <Measure<G::ItemPoint> as CoordinateScalar>::ZERO;
        let mut count = zero;
        // One sum per dimension, so every dimension of the mean is
        // covered, not just the first two.
        let mut sums = [zero; MAX_DIM];
        let mut dimensions = None;
        for p in mp.points() {
            fold_dims((), p, |(), p, d| {
                sums[d] = sums[d] + ordinate(p, d).to_measure();
            });
            count = count + <Measure<G::ItemPoint> as CoordinateScalar>::ONE;
            dimensions = Some(*p);
        }
        let dimensions = dimensions.expect("centroid of an empty multi-point");
        let mut mean = G::ItemPoint::default();
        fold_dims((), &dimensions, |(), _, d| {
            set_ordinate(
                &mut mean,
                d,
                <G::ItemPoint as PointTrait>::Scalar::from_measure(sums[d] / count),
            );
        });
        mean
    }
}

/// The midpoint of two points, `(a + b) / 2` in every dimension: Boost's
/// `centroid_indexed` (`algorithms/centroid.hpp:110-128`). Shared by the
/// [`geometry_trait::Segment`] and [`geometry_trait::Box`] impls, which are
/// both a two-corner midpoint.
#[inline]
fn midpoint<P>(a: &P, b: &P) -> P
where
    P: PointTrait + PointMut + Default,
{
    let two = two::<Measure<P>>();
    let mut midpoint = P::default();
    fold_dims((), a, |(), a, dimension| {
        let sum = ordinate(a, dimension).to_measure() + ordinate(b, dimension).to_measure();
        set_ordinate(&mut midpoint, dimension, P::Scalar::from_measure(sum / two));
    });
    midpoint
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

impl CentroidStrategyForKind for MultiPolygonTag {
    type S = CartesianMultiPolygonCentroid;
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
        CartesianMultiPolygonCentroid, CartesianPolygonCentroid, CartesianRingCentroid,
        CartesianSegmentCentroid, CentroidStrategy,
    };
    use geometry_cs::Cartesian;
    use geometry_model::{
        Box, MultiPoint, MultiPolygon, Point2D, Polygon, Ring, Segment, linestring, polygon,
    };
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

    /// A part with zero area still contributes to the running numerator.
    ///
    /// Combining per-part centroids weighted by area drops it — its weight is
    /// zero — and lands somewhere else. Boost 1.83 on a clockwise
    /// `model::polygon` / `model::multi_polygon`:
    ///
    /// ```text
    /// bowtie exterior + hole  -> (10.6667, 11)  area=-4
    /// zero-area MP            -> (0, 0)         area=0
    /// mixed MP                -> (11.3333, 11)  area=4
    /// ```
    #[test]
    fn a_zero_area_part_still_moves_the_centroid() {
        // Exterior is a bow-tie: zero area, non-zero numerator.
        let bowtie_with_hole: Polygon<Pt> = polygon![
            [(0.0, 0.0), (2.0, 2.0), (2.0, 0.0), (0.0, 2.0), (0.0, 0.0)],
            [
                (10.0, 10.0),
                (12.0, 10.0),
                (12.0, 12.0),
                (10.0, 12.0),
                (10.0, 10.0)
            ]
        ];
        let c = CartesianPolygonCentroid.centroid(&bowtie_with_hole);
        assert!(close_pt(&c, 32.0 / 3.0, 11.0, 1e-9), "{c:?}");

        let bowtie: Polygon<Pt> =
            polygon![[(0.0, 0.0), (2.0, 2.0), (2.0, 0.0), (0.0, 2.0), (0.0, 0.0)]];
        let other_bowtie: Polygon<Pt> = polygon![[
            (10.0, 10.0),
            (12.0, 12.0),
            (12.0, 10.0),
            (10.0, 12.0),
            (10.0, 10.0)
        ]];
        let square: Polygon<Pt> = polygon![[
            (10.0, 10.0),
            (10.0, 12.0),
            (12.0, 12.0),
            (12.0, 10.0),
            (10.0, 10.0)
        ]];

        // Every member degenerate: the first vertex of the first member.
        let all_degenerate = MultiPolygon(vec![bowtie.clone(), other_bowtie]);
        let c = CartesianMultiPolygonCentroid.centroid(&all_degenerate);
        assert!(close_pt(&c, 0.0, 0.0, 1e-9), "{c:?}");

        // One degenerate member beside a real one: it still pulls the result.
        let mixed = MultiPolygon(vec![bowtie, square]);
        let c = CartesianMultiPolygonCentroid.centroid(&mixed);
        assert!(close_pt(&c, 34.0 / 3.0, 11.0, 1e-9), "{c:?}");
    }

    /// Every ring adds to one running state, in order, as Boost's
    /// `centroid_polygon_state` adds them; per-ring sums added afterwards
    /// round differently. Boost (`aed7bc3`): (2.8222947186999883,
    /// 3.8762772374738392).
    #[test]
    fn rings_add_to_one_running_state() {
        let pg: Polygon<Pt> = polygon![
            [(0., 0.), (0., 7.), (6., 8.), (5., 0.), (0., 0.)],
            [(1.1, 2.1), (2.2, 2.1), (2.2, 2.8), (1.1, 2.1)]
        ];
        let c = CartesianPolygonCentroid.centroid(&pg);
        assert_eq!(
            (c.get::<0>(), c.get::<1>()),
            (2.822_294_718_699_988_3, 3.876_277_237_473_839_2)
        );
    }

    /// Boost's `result` calls an area of zero by `math::equals` degenerate,
    /// so a triangle a nanometre across falls back to its first point.
    /// Boost (`aed7bc3`): (1e-9, 1e-9).
    #[test]
    fn an_area_zero_by_math_equals_falls_back_to_the_first_point() {
        let pg: Polygon<Pt> = polygon![[(1e-9, 1e-9), (1e-9, 2e-9), (2e-9, 2e-9), (1e-9, 1e-9)]];
        let c = CartesianPolygonCentroid.centroid(&pg);
        assert_eq!((c.get::<0>(), c.get::<1>()), (1e-9, 1e-9));
    }

    /// A length that overflows is not finite, and Boost's `result` refuses
    /// to divide by it: the centroid falls back to the first point.
    /// Boost (`aed7bc3`): the first point.
    #[test]
    fn an_overflowing_length_falls_back_to_the_first_point() {
        let ls: geometry_model::Linestring<Pt> = linestring![
            (-9.535_516_924_438_917e159, 4.820_435_616_002_894e159),
            (-8.230_722_080_934_48e159, 6.172_257_949_105_479_4e159)
        ];
        let c = CartesianLinestringCentroid.centroid(&ls);
        assert_eq!(
            (c.get::<0>(), c.get::<1>()),
            (-9.535_516_924_438_917e159, 4.820_435_616_002_894e159)
        );
    }

    /// Boost's segment, box and linestring centroids cover every dimension
    /// (`centroid_indexed`, `weighted_length`). Boost (`aed7bc3`): (1, 2, 4)
    /// twice, then (1.4, 2.8, 5.6).
    #[test]
    fn indexed_and_linear_centroids_cover_the_third_dimension() {
        use geometry_model::{Linestring, Point3D};
        type P3 = Point3D<f64, Cartesian>;
        let xyz = |p: P3| (p.get::<0>(), p.get::<1>(), p.get::<2>());
        let (a, b) = (P3::new(0., 0., 2.), P3::new(2., 4., 6.));
        assert_eq!(
            xyz(CartesianSegmentCentroid.centroid(&Segment::new(a, b))),
            (1., 2., 4.)
        );
        assert_eq!(
            xyz(CartesianBoxCentroid.centroid(&Box::from_corners(a, b))),
            (1., 2., 4.)
        );
        let ls = Linestring::from_vec(vec![a, b, P3::new(2., 4., 10.)]);
        assert_eq!(
            xyz(CartesianLinestringCentroid.centroid(&ls)),
            (1.4, 2.8, 5.6)
        );
    }

    /// The pointlike arm averages *per dimension*: a 3-D multi-point's
    /// centroid carries the mean `z`.
    #[test]
    fn multipoint_mean_covers_the_third_dimension() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let mp: MultiPoint<P3> =
            MultiPoint::from_vec(vec![P3::new(0., 0., 10.), P3::new(2., 2., 12.)]);
        let c = CartesianMultiPointCentroid.centroid(&mp);
        assert!((c.get::<0>() - 1.0).abs() < 1e-12);
        assert!((c.get::<1>() - 1.0).abs() < 1e-12);
        let z = c.get::<2>();
        assert!((z - 11.0).abs() < 1e-12, "z mean should be 11, got {z}");
    }

    /// A ring, or a polygon's exterior, of one point has that point for
    /// its centroid.
    #[test]
    fn a_one_point_ring_is_its_own_centroid() {
        let r: Ring<Pt> = Ring::from_vec(vec![Pt::new(3., 4.)]);
        assert!(close_pt(&CartesianRingCentroid.centroid(&r), 3., 4., 1e-12));
        let pg: Polygon<Pt> = Polygon::new(r);
        assert!(close_pt(
            &CartesianPolygonCentroid.centroid(&pg),
            3.,
            4.,
            1e-12
        ));
    }

    /// An empty interior ring adds nothing: the polygon's centroid is its
    /// exterior's.
    #[test]
    fn an_empty_interior_ring_adds_nothing() {
        let square = || {
            Ring::from_vec(vec![
                Pt::new(0., 0.),
                Pt::new(0., 2.),
                Pt::new(2., 2.),
                Pt::new(2., 0.),
                Pt::new(0., 0.),
            ])
        };
        let pg: Polygon<Pt> = Polygon::with_inners(square(), vec![Ring::from_vec(vec![])]);
        assert!(close_pt(
            &CartesianPolygonCentroid.centroid(&pg),
            1.,
            1.,
            1e-12
        ));
    }
}
