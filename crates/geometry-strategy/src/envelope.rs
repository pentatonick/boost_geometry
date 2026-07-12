//! Per-CS strategy for the axis-aligned bounding box (envelope).
//!
//! Mirrors three pieces of Boost.Geometry that collaborate to make
//! `boost::geometry::envelope(g, mbr)` work for any geometry in any
//! coordinate system:
//!
//! * `boost/geometry/strategies/envelope/services.hpp` — the
//!   `services::default_strategy<G>` metafunction that picks the
//!   per-CS envelope strategy.
//! * `boost/geometry/strategies/envelope/cartesian.hpp` —
//!   `strategies::envelope::cartesian<>`, whose per-tag dispatch
//!   selects `envelope::cartesian_point` / `cartesian_box` /
//!   `cartesian_segment` / `expand::point` / `expand::box_` and so
//!   on. The Cartesian case for non-trivial geometries is a plain
//!   per-coordinate min/max walk — no great-circle bookkeeping,
//!   unlike the spherical / geographic strategies.
//! * `boost/geometry/algorithms/detail/envelope/range.hpp:33-66` —
//!   `envelope_range_of_boxes` and `envelope_range_of_points`, which
//!   walk a point sequence and grow a `Box` by component-wise
//!   `min` / `max`. The Rust port collapses the C++ template
//!   recursion `dimension_one` / `dimension_two` of
//!   `algorithms/detail/envelope/initialize.hpp` into the
//!   [`seed_from_point`] / [`grow_from_point`] helpers below, written
//!   against [`fold_dims`].
//!
//! ## Coherence note
//!
//! Boost dispatches on the geometry's tag via partial template
//! specialisation — `dispatch::envelope<G, point_tag>` and
//! `dispatch::envelope<G, linestring_tag>` are mutually exclusive
//! because the C++ side can ask the compiler to prove tags are
//! distinct. Rust's trait system cannot prove that a downstream type
//! does *not* implement both [`geometry_trait::Point`] and (say)
//! [`geometry_trait::Linestring`] simultaneously, so two open blankets
//! on one strategy struct would collide (E0119). The port reproduces
//! Boost's tag dispatch instead: one **per-kind strategy struct**
//! ([`EnvelopePoint`], [`EnvelopeLinestring`], …) carries a single
//! concept-bounded `EnvelopeStrategy` impl — distinct `Self`, so no
//! overlap — and the tag-keyed [`EnvelopeStrategyForKind`] picker routes
//! `G::Kind` to the right struct (disjoint on the tag). Because the
//! picker keys on the tag [`geometry_trait::Geometry::Kind`] already
//! carries, any concept-adapted foreign type resolves through the same
//! path as the equivalent `geometry-model` value (see
//! `specs/open-tag-dispatch/`).
//!
//! T35 lands the Cartesian implementation only — Boost's
//! Spherical/Geographic envelope strategies arrive alongside the
//! Haversine / Andoyer / Vincenty distance strategies in later
//! tasks (T40+).

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::Box as ModelBox;
use geometry_tag::{
    BoxTag, LinestringTag, MultiLinestringTag, MultiPointTag, MultiPolygonTag, PointTag,
    PolygonTag, RingTag, SameAs, SegmentTag,
};
use geometry_trait::{
    Box as BoxTrait, IndexedAccess, Linestring as LinestringTrait,
    MultiLinestring as MultiLinestringTrait, MultiPoint as MultiPointTrait,
    MultiPolygon as MultiPolygonTrait, Point as PointTrait, PointMut, Polygon as PolygonTrait,
    Ring as RingTrait, Segment as SegmentTrait, fold_dims,
};

/// A strategy for computing the axis-aligned bounding box of a
/// geometry.
///
/// Mirrors the per-CS envelope-strategy concept declared in
/// `boost/geometry/strategies/envelope/services.hpp` and refined per
/// coordinate system in `strategies/envelope/{cartesian,spherical,
/// geographic}.hpp`. The Boost concept exposes a family of per-tag
/// sub-strategies (`envelope::cartesian_point`,
/// `envelope::cartesian_box`, …) keyed off the geometry's tag; the
/// Rust analogue keeps that family as one per-kind strategy struct each
/// ([`EnvelopePoint`], [`EnvelopeBox`], …), selected by the tag-keyed
/// [`EnvelopeStrategyForKind`] picker.
///
/// # Associated items
///
/// * [`Self::Output`] — the bounding-box type. For the
///   Cartesian implementation this is always
///   `geometry_model::Box<G::Point>`, matching Boost's
///   `default_envelope_result<Geometry>::type`
///   (`strategies/default_envelope_result.hpp`).
pub trait EnvelopeStrategy<G> {
    /// The output box type. Mirrors
    /// `boost::geometry::default_envelope_result<G>::type` from
    /// `strategies/default_envelope_result.hpp`.
    type Output;

    /// Compute the axis-aligned bounding box of `g`.
    ///
    /// Mirrors `boost::geometry::dispatch::envelope<G, Tag>::apply`
    /// from `algorithms/detail/envelope/interface.hpp`, with the
    /// `Box` returned by value instead of mutated by reference.
    fn envelope(&self, g: &G) -> Self::Output;
}

/// Cartesian envelope, per geometry kind: component-wise min / max across
/// every coordinate of every point in the geometry.
///
/// Mirrors the per-tag sub-strategies of
/// `boost::geometry::strategies::envelope::cartesian<>`
/// (`strategies/envelope/cartesian.hpp`). Each struct is a stateless ZST
/// carrying exactly one concept-bounded [`EnvelopeStrategy`] impl, so
/// distinct kinds never overlap; the [`EnvelopeStrategyForKind`] picker
/// routes a tag to its struct.
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvelopePoint;
/// Cartesian envelope of a [`geometry_trait::Linestring`]. See
/// [`EnvelopePoint`].
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvelopeLinestring;
/// Cartesian envelope of a [`geometry_trait::Ring`]. See [`EnvelopePoint`].
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvelopeRing;
/// Cartesian envelope of a [`geometry_trait::Polygon`]. See
/// [`EnvelopePoint`].
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvelopePolygon;
/// Cartesian envelope of a [`geometry_trait::Segment`]. See
/// [`EnvelopePoint`].
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvelopeSegment;
/// Cartesian envelope of a [`geometry_trait::Box`]. See [`EnvelopePoint`].
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvelopeBox;
/// Cartesian envelope of a [`geometry_trait::MultiPoint`]. See
/// [`EnvelopePoint`].
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvelopeMultiPoint;
/// Cartesian envelope of a [`geometry_trait::MultiLinestring`]. See
/// [`EnvelopePoint`].
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvelopeMultiLinestring;
/// Cartesian envelope of a [`geometry_trait::MultiPolygon`]. See
/// [`EnvelopePoint`].
#[derive(Debug, Default, Clone, Copy)]
pub struct EnvelopeMultiPolygon;

// ---- Helpers ---------------------------------------------------------
//
// The shared shape of the per-tag dispatch is "seed the box from the
// first point, then grow it by each remaining point". `seed_from_point`
// writes both corners equal to `p`; `grow_from_point` widens each
// corner per-dimension by `min` / `max` against `p`. Both helpers are
// written against [`fold_dims`] so the per-dimension recursion is
// shared with Pythagoras and the indexed-access materialiser.

/// Initialise `out` so both its corners equal `p`.
///
/// Mirrors `detail::envelope::initialize<dimension_one, …>::apply` at
/// `algorithms/detail/envelope/initialize.hpp:36-51` — the per-tag
/// "envelope of a single point" base case.
#[inline]
pub fn seed_from_point<P>(out: &mut ModelBox<P>, p: &P)
where
    P: PointMut,
{
    fold_dims((), p, |(), p, d| {
        // `fold_dims` hands us `d` as a `usize`; the four arms map to
        // the const-generic `D` that `Point::get` / `Box::set_indexed`
        // require. Dimensions past `MAX_DIM` are rejected by
        // `fold_dims` itself.
        match d {
            0 => write_both::<P, 0>(out, p),
            1 => write_both::<P, 1>(out, p),
            2 => write_both::<P, 2>(out, p),
            3 => write_both::<P, 3>(out, p),
            _ => unreachable!("fold_dims: dimension out of MAX_DIM range"),
        }
    });
}

#[inline]
fn write_both<P, const D: usize>(out: &mut ModelBox<P>, p: &P)
where
    P: PointMut,
{
    let v = p.get::<D>();
    out.set_indexed::<0, D>(v);
    out.set_indexed::<1, D>(v);
}

/// Widen `out` per-dimension by `p` — `min` into the min corner,
/// `max` into the max corner.
///
/// Mirrors `detail::expand::point::apply` at
/// `algorithms/detail/expand/point.hpp:34-55`, the per-point growth
/// step used by every range envelope.
#[inline]
pub fn grow_from_point<P>(out: &mut ModelBox<P>, p: &P)
where
    P: PointMut,
{
    fold_dims((), p, |(), p, d| match d {
        0 => grow_one::<P, 0>(out, p),
        1 => grow_one::<P, 1>(out, p),
        2 => grow_one::<P, 2>(out, p),
        3 => grow_one::<P, 3>(out, p),
        _ => unreachable!("fold_dims: dimension out of MAX_DIM range"),
    });
}

#[inline]
fn grow_one<P, const D: usize>(out: &mut ModelBox<P>, p: &P)
where
    P: PointMut,
{
    let v = p.get::<D>();
    let lo = out.get_indexed::<0, D>();
    let hi = out.get_indexed::<1, D>();
    // `PartialOrd` is the only ordering bound `CoordinateScalar`
    // guarantees, so we compare with `<` rather than calling
    // `Ord::min` / `Ord::max` — the latter would force a total order
    // we deliberately do not require (NaN coordinates remain caller
    // error, exactly as on the Boost side).
    if v < lo {
        out.set_indexed::<0, D>(v);
    }
    if v > hi {
        out.set_indexed::<1, D>(v);
    }
}

/// Walk an iterator of points seeding the box from the first and
/// growing by the rest.
///
/// For a **non-empty** range the box is seeded from the first real point
/// (not from zero), so all-negative coordinates envelope correctly. For
/// an **empty** range it returns `Box::default()` = `((0,0),(0,0))` — a
/// degenerate box at the origin. Note this differs from Boost, which
/// leaves an empty envelope *inverted* (`min = +∞`, `max = −∞`) so
/// emptiness is detectable (`algorithms/detail/envelope/initialize.hpp`);
/// callers that must distinguish "empty" from "a point at the origin"
/// should check the source geometry for emptiness first. Representing an
/// inverted box needs an infinity sentinel the v1 `Box` does not carry;
/// deferred.
#[inline]
pub fn envelope_of_points<'a, P, I>(it: I) -> ModelBox<P>
where
    P: PointMut + Default + 'a,
    P::Scalar: CoordinateScalar,
    I: IntoIterator<Item = &'a P>,
{
    let mut out = ModelBox::<P>::default();
    let mut it = it.into_iter();
    if let Some(first) = it.next() {
        seed_from_point(&mut out, first);
        for p in it {
            grow_from_point(&mut out, p);
        }
    }
    out
}

// ---- Per-kind concept-bounded impls ---------------------------------
//
// Each impl below carries a single open concept bound (`G: Point`,
// `G: Linestring`, …) on a distinct struct, so coherence stays trivial —
// distinct `Self` types never overlap. The shape mirrors the Boost
// per-tag dispatch in `algorithms/detail/envelope/`:
//
// * `point.hpp:33-58`             → `EnvelopePoint`
// * `range.hpp:46-66`             → `EnvelopeLinestring` / `EnvelopeRing`
// * `areal.hpp` (polygon arm)     → `EnvelopePolygon`
// * `segment.hpp`                 → `EnvelopeSegment`
// * `box.hpp`                     → `EnvelopeBox`
// * `multipoint.hpp`              → `EnvelopeMultiPoint`
// * `multilinestring.hpp`         → `EnvelopeMultiLinestring`
// * `multipolygon.hpp`            → `EnvelopeMultiPolygon`
//
// The output box is always `ModelBox<G::Point>` — Boost's
// `default_envelope_result<G>::type`.

// ---- Point ----------------------------------------------------------

impl<G> EnvelopeStrategy<G> for EnvelopePoint
where
    G: PointTrait + PointMut + Default,
    <G::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = ModelBox<G>;

    #[inline]
    fn envelope(&self, g: &G) -> Self::Output {
        let mut out = ModelBox::<G>::default();
        seed_from_point(&mut out, g);
        out
    }
}

// ---- Linestring -----------------------------------------------------

impl<G> EnvelopeStrategy<G> for EnvelopeLinestring
where
    G: LinestringTrait,
    G::Point: PointMut + Default,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = ModelBox<G::Point>;

    #[inline]
    fn envelope(&self, g: &G) -> Self::Output {
        envelope_of_points::<G::Point, _>(g.points())
    }
}

// ---- Ring -----------------------------------------------------------
//
// The four `(ClockWise, Closed)` combinations all reduce to "walk every
// vertex, take componentwise extremes" — the closing edge does not
// add new extremes, and orientation doesn't affect bounds. The C++
// test `envelope.cpp:48-51` exercises exactly this insensitivity by
// running the same `(4 1)(0 7)(7 9)` triangle in all four
// combinations and expecting the same box.

impl<G> EnvelopeStrategy<G> for EnvelopeRing
where
    G: RingTrait,
    G::Point: PointMut + Default,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = ModelBox<G::Point>;

    #[inline]
    fn envelope(&self, g: &G) -> Self::Output {
        envelope_of_points::<G::Point, _>(g.points())
    }
}

// ---- Polygon --------------------------------------------------------
//
// Only the exterior ring contributes — interior rings live strictly
// inside, so they can only ever narrow the box, never widen it.

impl<G> EnvelopeStrategy<G> for EnvelopePolygon
where
    G: PolygonTrait,
    G::Point: PointMut + Default,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = ModelBox<G::Point>;

    #[inline]
    fn envelope(&self, g: &G) -> Self::Output {
        envelope_of_points::<G::Point, _>(g.exterior().points())
    }
}

// ---- Segment --------------------------------------------------------
//
// Cartesian segment envelope is per-coordinate min/max of the two
// endpoints — no great-circle handling, unlike the spherical /
// geographic segment strategies.

impl<G> EnvelopeStrategy<G> for EnvelopeSegment
where
    G: SegmentTrait,
    G::Point: PointMut + Default,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = ModelBox<G::Point>;

    #[inline]
    fn envelope(&self, g: &G) -> Self::Output {
        let mut out = ModelBox::<G::Point>::default();
        seed_from_point(&mut out, &geometry_trait::segment_start(g));
        grow_from_point(&mut out, &geometry_trait::segment_end(g));
        out
    }
}

// ---- Box ------------------------------------------------------------
//
// Envelope of a box is the box itself — produced by a fresh
// allocation (rather than `Clone`) so the Cartesian path stays
// uniform with the other geometry kinds, where the output is always a
// freshly seeded box.

impl<G> EnvelopeStrategy<G> for EnvelopeBox
where
    G: BoxTrait,
    G::Point: PointMut + Default,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = ModelBox<G::Point>;

    #[inline]
    fn envelope(&self, g: &G) -> Self::Output {
        let mut out = ModelBox::<G::Point>::default();
        seed_from_point(&mut out, &geometry_trait::box_min(g));
        grow_from_point(&mut out, &geometry_trait::box_max(g));
        out
    }
}

// ---- MultiPoint -----------------------------------------------------

impl<G> EnvelopeStrategy<G> for EnvelopeMultiPoint
where
    G: MultiPointTrait,
    G::ItemPoint: PointMut + Default,
    <<G::ItemPoint as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = ModelBox<G::ItemPoint>;

    #[inline]
    fn envelope(&self, g: &G) -> Self::Output {
        envelope_of_points::<G::ItemPoint, _>(g.points())
    }
}

// ---- MultiLinestring ------------------------------------------------
//
// The member concept is open (any `Linestring` whose point matches),
// widening past the pinned element type of the earlier model-bound impl —
// the intended BYO widening. Body walks every member point unchanged.

impl<G> EnvelopeStrategy<G> for EnvelopeMultiLinestring
where
    G: MultiLinestringTrait,
    G::Point: PointMut + Default,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = ModelBox<G::Point>;

    #[inline]
    fn envelope(&self, g: &G) -> Self::Output {
        let mut out = ModelBox::<G::Point>::default();
        let mut seeded = false;
        for ls in g.linestrings() {
            for p in ls.points() {
                if seeded {
                    grow_from_point(&mut out, p);
                } else {
                    seed_from_point(&mut out, p);
                    seeded = true;
                }
            }
        }
        out
    }
}

// ---- MultiPolygon ---------------------------------------------------
//
// The member concept is open (any `Polygon` whose point matches) — the
// same BYO widening as `EnvelopeMultiLinestring`.

impl<G> EnvelopeStrategy<G> for EnvelopeMultiPolygon
where
    G: MultiPolygonTrait,
    G::Point: PointMut + Default,
    <<G::Point as PointTrait>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = ModelBox<G::Point>;

    #[inline]
    fn envelope(&self, g: &G) -> Self::Output {
        let mut out = ModelBox::<G::Point>::default();
        let mut seeded = false;
        for poly in g.polygons() {
            for p in poly.exterior().points() {
                if seeded {
                    grow_from_point(&mut out, p);
                } else {
                    seed_from_point(&mut out, p);
                    seeded = true;
                }
            }
        }
        out
    }
}

/// Type-level "which `EnvelopeStrategy` struct does this geometry *kind*
/// use". One impl per [`geometry_tag`] kind tag, keyed on the tag (never a
/// concept blanket — that would overlap, E0119), so any concept-adapted
/// foreign type resolves to the same struct as the equivalent model value.
/// The [`crate::envelope`] free function routes `G → G::Kind → S` through
/// this trait.
#[doc(hidden)]
pub trait EnvelopeStrategyForKind {
    /// The per-kind [`EnvelopeStrategy`] struct this tag is computed with.
    type S: Default;
}

impl EnvelopeStrategyForKind for PointTag {
    type S = EnvelopePoint;
}
impl EnvelopeStrategyForKind for LinestringTag {
    type S = EnvelopeLinestring;
}
impl EnvelopeStrategyForKind for RingTag {
    type S = EnvelopeRing;
}
impl EnvelopeStrategyForKind for PolygonTag {
    type S = EnvelopePolygon;
}
impl EnvelopeStrategyForKind for SegmentTag {
    type S = EnvelopeSegment;
}
impl EnvelopeStrategyForKind for BoxTag {
    type S = EnvelopeBox;
}
impl EnvelopeStrategyForKind for MultiPointTag {
    type S = EnvelopeMultiPoint;
}
impl EnvelopeStrategyForKind for MultiLinestringTag {
    type S = EnvelopeMultiLinestring;
}
impl EnvelopeStrategyForKind for MultiPolygonTag {
    type S = EnvelopeMultiPolygon;
}

#[cfg(test)]
mod tests {
    //! Reference values come from
    //! `geometry/test/algorithms/envelope_expand/envelope.cpp:38-54`
    //! (the `test_2d` arm). Each test cites the line it mirrors.

    use super::{
        EnvelopeBox, EnvelopeLinestring, EnvelopePoint, EnvelopePolygon, EnvelopeSegment,
        EnvelopeStrategy,
    };
    use geometry_cs::Cartesian;
    use geometry_model::{Box, Linestring, Point2D, Polygon, Segment, linestring, polygon};
    use geometry_trait::IndexedAccess as _;

    type P = Point2D<f64, Cartesian>;

    fn assert_2d(b: &Box<P>, xmin: f64, xmax: f64, ymin: f64, ymax: f64) {
        assert_eq!(b.get_indexed::<0, 0>().to_bits(), xmin.to_bits());
        assert_eq!(b.get_indexed::<0, 1>().to_bits(), ymin.to_bits());
        assert_eq!(b.get_indexed::<1, 0>().to_bits(), xmax.to_bits());
        assert_eq!(b.get_indexed::<1, 1>().to_bits(), ymax.to_bits());
    }

    /// `envelope.cpp:38` — `POINT(1 1)` → `(1,1) (1,1)`.
    #[test]
    fn point_envelope_collapses() {
        let p = Point2D::<f64, Cartesian>::new(1.0, 1.0);
        assert_2d(&EnvelopePoint.envelope(&p), 1.0, 1.0, 1.0, 1.0);
    }

    /// `envelope.cpp:39` — `LINESTRING(1 1,2 2)` → `(1,1) (2,2)`.
    #[test]
    fn linestring_two_points() {
        let ls: Linestring<P> = linestring![(1.0, 1.0), (2.0, 2.0)];
        assert_2d(&EnvelopeLinestring.envelope(&ls), 1.0, 2.0, 1.0, 2.0);
    }

    /// `envelope.cpp:40` — square polygon, envelope = the polygon.
    #[test]
    fn polygon_axis_aligned_square() {
        let p: Polygon<P> = polygon![[(1.0, 1.0), (1.0, 3.0), (3.0, 3.0), (3.0, 1.0), (1.0, 1.0),]];
        assert_2d(&EnvelopePolygon.envelope(&p), 1.0, 3.0, 1.0, 3.0);
    }

    /// `envelope.cpp:43` — `BOX(1 1,3 3)` — envelope is the box.
    #[test]
    fn box_envelope_is_self() {
        let b = Box::from_corners(
            Point2D::<f64, Cartesian>::new(1.0, 1.0),
            Point2D::<f64, Cartesian>::new(3.0, 3.0),
        );
        assert_2d(&EnvelopeBox.envelope(&b), 1.0, 3.0, 1.0, 3.0);
    }

    /// `envelope.cpp:48` — non-convex closed CW ring; envelope
    /// tightens to the extremes `(0,1)-(7,9)`.
    #[test]
    fn ring_non_convex() {
        let p: Polygon<P> = polygon![[(4.0, 1.0), (0.0, 7.0), (7.0, 9.0), (4.0, 1.0)]];
        assert_2d(&EnvelopePolygon.envelope(&p), 0.0, 7.0, 1.0, 9.0);
    }

    /// `envelope.cpp:54` — `SEGMENT(1 1,3 3)`.
    #[test]
    fn segment_envelope() {
        let s = Segment::new(
            Point2D::<f64, Cartesian>::new(1.0, 1.0),
            Point2D::<f64, Cartesian>::new(3.0, 3.0),
        );
        assert_2d(&EnvelopeSegment.envelope(&s), 1.0, 3.0, 1.0, 3.0);
    }

    /// Segment with crossed coordinates — the start corner has to
    /// be `(min_x, min_y)`, not "the first endpoint".
    #[test]
    fn segment_envelope_crossed() {
        let s = Segment::new(
            Point2D::<f64, Cartesian>::new(3.0, 1.0),
            Point2D::<f64, Cartesian>::new(1.0, 3.0),
        );
        assert_2d(&EnvelopeSegment.envelope(&s), 1.0, 3.0, 1.0, 3.0);
    }
}
