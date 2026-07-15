//! Per-CS strategy for the `intersects` set-relation algorithm.
//!
//! Mirrors `boost::geometry::intersects(g1, g2)` from
//! `boost/geometry/algorithms/intersects.hpp` together with the
//! per-pair dispatch tables it pulls in from
//! `algorithms/detail/intersects/{interface,implementation}.hpp` and,
//! transitively, the Cartesian segment-segment / point-on-segment
//! kernels in `strategy/cartesian/`. Two geometries `intersects` iff
//! they are *not* `disjoint` — Boost's interface header is literally
//! `intersects(a, b) == !disjoint(a, b)`
//! (`algorithms/detail/intersects/interface.hpp:64-78`). The Rust port
//! flips that around: `intersects` is the primary kernel here and
//! [`crate::disjoint`] is the negation, because every constructive
//! per-pair test is naturally an "is there a shared point?" question.
//!
//! ## Coherence note
//!
//! Rust's trait system cannot prove a downstream type does not implement
//! several geometry traits at once, so two open blankets on one strategy
//! struct would collide (E0119). The port reproduces Boost's per-pair
//! tag dispatch instead: one **per-ordered-pair strategy struct**
//! ([`IxPointPoint`], [`IxLinestringPolygon`], …) carries a single
//! concept-pair-bounded `IntersectsStrategy` impl — distinct `Self`, so
//! no overlap — and the tag-keyed [`IntersectsPairStrategy`] picker
//! routes `(A::Kind, B::Kind)` to the right struct. Because it keys on
//! the tags, a concept-adapted foreign type resolves through the same
//! path as the equivalent `geometry-model` value.
//! [`CartesianIntersects`] remains as a thin
//! facade that routes through the picker, so `disjoint` / `is_simple` and
//! the `Reversed` symmetry adapter keep resolving unchanged.
//!
//! ## Symmetry
//!
//! `intersects` is symmetric: `intersects(a, b) == intersects(b, a)`.
//! Each pair appears in exactly one canonical direction here and the
//! [`Reversed`] blanket at the bottom lifts every
//! `IntersectsStrategy<A, B>` to an `IntersectsStrategy<B, A>` for
//! free — mirroring `boost::geometry::reverse_dispatch` at
//! `core/reverse_dispatch.hpp`.

extern crate alloc;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::{LinestringTag, PointTag, PolygonTag, SameAs, SegmentTag};
use geometry_trait::{
    Geometry, Linestring as LinestringTrait, Point as PointTrait, Polygon as PolygonTrait,
    Ring as RingTrait, Segment as SegmentTrait, segment_end, segment_start,
};

use crate::within::{WithinPoly, WithinStrategy};

pub use crate::reversal::Reversed;

/// A strategy for "do these two geometries share at least one point?".
///
/// Mirrors `boost::geometry::intersects(g1, g2)` from
/// `boost/geometry/algorithms/intersects.hpp`. The Boost API takes the
/// strategy implicitly through the algorithm's per-pair dispatch
/// table; the Rust analogue collapses dispatch and strategy onto one
/// trait keyed on the geometry pair.
pub trait IntersectsStrategy<A, B> {
    /// `true` iff `a` and `b` share at least one point.
    fn intersects(&self, a: &A, b: &B) -> bool;
}

/// The Cartesian intersection facade — Boost's default for the Cartesian
/// coordinate system.
///
/// Routes `(A, B)` through the tag-keyed [`IntersectsPairStrategy`] picker
/// to the matching per-pair strategy struct. Kept as a single type so
/// consumers that name it directly ([`crate::disjoint`],
/// `is_simple`, [`Reversed`]) resolve unchanged; the per-pair *bodies*
/// live on the [`IxPointPoint`] … structs below.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianIntersects;

impl<A, B> IntersectsStrategy<A, B> for CartesianIntersects
where
    A: Geometry,
    B: Geometry,
    A::Kind: IntersectsPairStrategy<B::Kind>,
    <A::Kind as IntersectsPairStrategy<B::Kind>>::S: IntersectsStrategy<A, B>,
{
    #[inline]
    fn intersects(&self, a: &A, b: &B) -> bool {
        <<A::Kind as IntersectsPairStrategy<B::Kind>>::S as Default>::default().intersects(a, b)
    }
}

// ---- Per-pair strategy structs --------------------------------------
//
// Each struct carries a single concept-pair-bounded `IntersectsStrategy`
// impl; distinct structs never overlap. Bodies are the existing kernels,
// re-bound on the open concepts. The four `covered_by` cross-strategy
// calls go through the open [`WithinPoly`] strategy (not the algorithm
// free fn — that would be an upward crate dependency).

/// Point × Point. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxPointPoint;
/// Point × Segment. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxPointSegment;
/// Segment × Segment. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxSegmentSegment;
/// Linestring × Segment. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxLinestringSegment;
/// Linestring × Linestring. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxLinestringLinestring;
/// Point × Polygon. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxPointPolygon;
/// Linestring × Polygon. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxLinestringPolygon;
/// Polygon × Polygon. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxPolygonPolygon;
/// Segment × Point (reverse of [`IxPointSegment`]). See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxSegmentPoint;
/// Segment × Linestring (reverse of [`IxLinestringSegment`]). See the
/// [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxSegmentLinestring;
/// Polygon × Point (reverse of [`IxPointPolygon`]). See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxPolygonPoint;
/// Polygon × Linestring (reverse of [`IxLinestringPolygon`]). See the
/// [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct IxPolygonLinestring;

// ---- Point × Point ---------------------------------------------------
//
// Two points "intersect" iff they are coordinate-wise equal. Mirrors
// the pointlike/pointlike arm at
// `algorithms/detail/disjoint/point_point.hpp:36-66`.

impl<A, B> IntersectsStrategy<A, B> for IxPointPoint
where
    A: PointTrait,
    B: PointTrait<Scalar = A::Scalar>,
    <A::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, a: &A, b: &B) -> bool {
        points_equal::<A, B>(a, b, A::DIM)
    }
}

// ---- Point × Segment -------------------------------------------------
//
// A point lies on a segment iff it is collinear with the two
// endpoints and inside the bounding box of those endpoints.

impl<P, S> IntersectsStrategy<P, S> for IxPointSegment
where
    P: PointTrait,
    S: SegmentTrait<Point = P>,
    P: geometry_trait::PointMut + Default,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, p: &P, s: &S) -> bool {
        point_on_segment(p, &segment_start(s), &segment_end(s))
    }
}

// ---- Segment × Segment -----------------------------------------------

impl<A, B, P> IntersectsStrategy<A, B> for IxSegmentSegment
where
    A: SegmentTrait<Point = P>,
    B: SegmentTrait<Point = P>,
    P: PointTrait + geometry_trait::PointMut + Default,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, a: &A, b: &B) -> bool {
        segments_intersect(
            &segment_start(a),
            &segment_end(a),
            &segment_start(b),
            &segment_end(b),
        )
    }
}

// ---- Linestring × Segment --------------------------------------------

impl<L, S, P> IntersectsStrategy<L, S> for IxLinestringSegment
where
    L: LinestringTrait<Point = P>,
    S: SegmentTrait<Point = P>,
    P: PointTrait + geometry_trait::PointMut + Default,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, ls: &L, s: &S) -> bool {
        let s1 = segment_start(s);
        let s2 = segment_end(s);
        let mut it = ls.points();
        let Some(mut prev) = it.next() else {
            return false;
        };
        for curr in it {
            if segments_intersect(prev, curr, &s1, &s2) {
                return true;
            }
            prev = curr;
        }
        false
    }
}

// ---- Linestring × Linestring -----------------------------------------

impl<A, B, P> IntersectsStrategy<A, B> for IxLinestringLinestring
where
    A: LinestringTrait<Point = P>,
    B: LinestringTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, a: &A, b: &B) -> bool {
        let mut ia = a.points();
        let Some(mut pa) = ia.next() else {
            return false;
        };
        for qa in ia {
            let mut ib = b.points();
            let Some(mut pb) = ib.next() else {
                return false;
            };
            for qb in ib {
                if segments_intersect(pa, qa, pb, qb) {
                    return true;
                }
                pb = qb;
            }
            pa = qa;
        }
        false
    }
}

// ---- Point × Polygon -------------------------------------------------
//
// A point intersects a polygon iff it is covered_by (interior or
// boundary). Goes through the open `WithinPoly` strategy.

impl<P, G> IntersectsStrategy<P, G> for IxPointPolygon
where
    P: PointTrait,
    G: PolygonTrait<Point = P>,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, p: &P, pg: &G) -> bool {
        WithinPoly.covered_by(p, pg)
    }
}

// ---- Linestring × Polygon --------------------------------------------

impl<L, G, P> IntersectsStrategy<L, G> for IxLinestringPolygon
where
    L: LinestringTrait<Point = P>,
    G: PolygonTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn intersects(&self, ls: &L, pg: &G) -> bool {
        // A connected line with no polygon-boundary crossing cannot change
        // between polygon material and its complement, so one representative
        // point is sufficient for containment.
        let Some(first) = ls.points().next() else {
            return false;
        };
        if WithinPoly.covered_by(first, pg) {
            return true;
        }
        // Any sub-segment crossing any ring sub-segment.
        if linestring_crosses_ring(ls, pg.exterior()) {
            return true;
        }
        for hole in pg.interiors() {
            if linestring_crosses_ring(ls, hole) {
                return true;
            }
        }
        false
    }
}

// ---- Polygon × Polygon -----------------------------------------------

impl<A, B, P> IntersectsStrategy<A, B> for IxPolygonPolygon
where
    A: PolygonTrait<Point = P>,
    B: PolygonTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn intersects(&self, a: &A, b: &B) -> bool {
        // Containment fast path: a vertex of A inside B, or vice versa.
        if let Some(v) = a.exterior().points().next() {
            if WithinPoly.covered_by(v, b) {
                return true;
            }
        }
        if let Some(v) = b.exterior().points().next() {
            if WithinPoly.covered_by(v, a) {
                return true;
            }
        }
        // Any ring-edge crossing.
        if rings_cross(a.exterior(), b.exterior()) {
            return true;
        }
        for hole in a.interiors() {
            if rings_cross(hole, b.exterior()) {
                return true;
            }
        }
        for hole in b.interiors() {
            if rings_cross(a.exterior(), hole) {
                return true;
            }
        }
        false
    }
}

// ---- Reverse pairs ---------------------------------------------------
//
// Each asymmetric pair gets its own struct that swaps and delegates to
// the forward struct, so the per-pair picker can cover each ordered pair
// directly.

impl<S, P> IntersectsStrategy<S, P> for IxSegmentPoint
where
    S: SegmentTrait<Point = P>,
    P: PointTrait + geometry_trait::PointMut + Default,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, s: &S, p: &P) -> bool {
        IxPointSegment.intersects(p, s)
    }
}

impl<S, L, P> IntersectsStrategy<S, L> for IxSegmentLinestring
where
    S: SegmentTrait<Point = P>,
    L: LinestringTrait<Point = P>,
    P: PointTrait + geometry_trait::PointMut + Default,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, s: &S, ls: &L) -> bool {
        IxLinestringSegment.intersects(ls, s)
    }
}

impl<G, P> IntersectsStrategy<G, P> for IxPolygonPoint
where
    G: PolygonTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, pg: &G, p: &P) -> bool {
        IxPointPolygon.intersects(p, pg)
    }
}

impl<G, L, P> IntersectsStrategy<G, L> for IxPolygonLinestring
where
    G: PolygonTrait<Point = P>,
    L: LinestringTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn intersects(&self, pg: &G, ls: &L) -> bool {
        IxLinestringPolygon.intersects(ls, pg)
    }
}

/// Type-level "which `IntersectsStrategy` struct does this ordered pair
/// of geometry *kinds* use". A trait parameterised by the second tag
/// `K2`, keyed on the first tag `Self` — disjoint on the pair, so no
/// overlap. One entry per implemented ordered pair. The
/// [`crate::intersects`] free function routes `(A::Kind, B::Kind)`
/// through this trait.
#[doc(hidden)]
pub trait IntersectsPairStrategy<K2> {
    /// The per-pair [`IntersectsStrategy`] struct this tag pair uses.
    type S: Default;
}

impl IntersectsPairStrategy<PointTag> for PointTag {
    type S = IxPointPoint;
}
impl IntersectsPairStrategy<SegmentTag> for PointTag {
    type S = IxPointSegment;
}
impl IntersectsPairStrategy<SegmentTag> for SegmentTag {
    type S = IxSegmentSegment;
}
impl IntersectsPairStrategy<SegmentTag> for LinestringTag {
    type S = IxLinestringSegment;
}
impl IntersectsPairStrategy<LinestringTag> for LinestringTag {
    type S = IxLinestringLinestring;
}
impl IntersectsPairStrategy<PolygonTag> for PointTag {
    type S = IxPointPolygon;
}
impl IntersectsPairStrategy<PolygonTag> for LinestringTag {
    type S = IxLinestringPolygon;
}
impl IntersectsPairStrategy<PolygonTag> for PolygonTag {
    type S = IxPolygonPolygon;
}
impl IntersectsPairStrategy<PointTag> for SegmentTag {
    type S = IxSegmentPoint;
}
impl IntersectsPairStrategy<LinestringTag> for SegmentTag {
    type S = IxSegmentLinestring;
}
impl IntersectsPairStrategy<PointTag> for PolygonTag {
    type S = IxPolygonPoint;
}
impl IntersectsPairStrategy<LinestringTag> for PolygonTag {
    type S = IxPolygonLinestring;
}

// ---- Kernels ---------------------------------------------------------

/// Coordinate-wise point equality across the first `dim` dimensions.
#[inline]
fn points_equal<A, B>(a: &A, b: &B, dim: usize) -> bool
where
    A: PointTrait,
    B: PointTrait<Scalar = A::Scalar>,
{
    let mut i = 0;
    while i < dim {
        // const-generic indexed access — match on every dimension supported
        // by geometry_trait's stable-Rust dimension walkers.
        let eq = match i {
            0 => a.get::<0>() == b.get::<0>(),
            1 => a.get::<1>() == b.get::<1>(),
            2 => a.get::<2>() == b.get::<2>(),
            3 => a.get::<3>() == b.get::<3>(),
            _ => panic!("points_equal: dimension exceeds MAX_DIM (4)"),
        };
        if !eq {
            return false;
        }
        i += 1;
    }
    true
}

/// Point-on-segment test in 2D. Mirrors the per-segment short-circuit
/// in `cartesian_winding_base::apply` at
/// `strategy/cartesian/point_in_poly_winding.hpp:91-131`: the point
/// lies on `s1->s2` iff the side cross product is zero **and** the
/// point's parameter along the segment lies in `[0, 1]`.
fn point_on_segment<P>(p: &P, s1: &P, s2: &P) -> bool
where
    P: PointTrait,
    P::Scalar: CoordinateScalar,
{
    let px = p.get::<0>();
    let py = p.get::<1>();
    let ax = s1.get::<0>();
    let ay = s1.get::<1>();
    let bx = s2.get::<0>();
    let by = s2.get::<1>();
    let cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    if cross != P::Scalar::ZERO {
        return false;
    }
    let (xlo, xhi) = if ax <= bx { (ax, bx) } else { (bx, ax) };
    let (ylo, yhi) = if ay <= by { (ay, by) } else { (by, ay) };
    xlo <= px && px <= xhi && ylo <= py && py <= yhi
}

/// 2D segment-segment intersection test. Returns `true` iff the two
/// closed segments share at least one point — proper crossing,
/// endpoint touch, or collinear overlap. Mirrors the boolean
/// projection of `strategy/cartesian/intersection.hpp:139-260`.
fn segments_intersect<P>(p1: &P, p2: &P, p3: &P, p4: &P) -> bool
where
    P: PointTrait,
    P::Scalar: CoordinateScalar,
{
    let x1 = p1.get::<0>();
    let y1 = p1.get::<1>();
    let x2 = p2.get::<0>();
    let y2 = p2.get::<1>();
    let x3 = p3.get::<0>();
    let y3 = p3.get::<1>();
    let x4 = p4.get::<0>();
    let y4 = p4.get::<1>();

    let d1 = side_sign((x3, y3), (x4, y4), (x1, y1));
    let d2 = side_sign((x3, y3), (x4, y4), (x2, y2));
    let d3 = side_sign((x1, y1), (x2, y2), (x3, y3));
    let d4 = side_sign((x1, y1), (x2, y2), (x4, y4));

    // Proper crossing — the endpoints of each segment lie on
    // opposite sides of the other segment's line.
    if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
        return true;
    }

    // Endpoint-on-segment / collinear cases: a `side_sign` of zero
    // means the third point lies on the other segment's line; we
    // still have to check the bounding-box containment.
    if d1 == 0 && point_on_segment(p1, p3, p4) {
        return true;
    }
    if d2 == 0 && point_on_segment(p2, p3, p4) {
        return true;
    }
    if d3 == 0 && point_on_segment(p3, p1, p2) {
        return true;
    }
    if d4 == 0 && point_on_segment(p4, p1, p2) {
        return true;
    }
    false
}

/// Sign of the side cross product `(b - a) × (c - a)`. `+1` left,
/// `-1` right, `0` collinear. Mirrors `side_by_triangle::side_value`
/// at `strategy/cartesian/side_by_triangle.hpp:178-200`.
#[inline]
fn side_sign<T: CoordinateScalar>(a: (T, T), b: (T, T), c: (T, T)) -> i32 {
    let v = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    if v > T::ZERO {
        1
    } else if v < T::ZERO {
        -1
    } else {
        0
    }
}

/// Whether the closed axis-aligned bounds of two segments are disjoint.
///
/// Comparisons are written without `min`/`max` so a coordinate that is not
/// ordered (for example, `NaN`) falls through to the exact segment predicate.
#[inline]
fn segment_bounds_disjoint<P>(p1: &P, p2: &P, p3: &P, p4: &P) -> bool
where
    P: PointTrait,
{
    let x1 = p1.get::<0>();
    let y1 = p1.get::<1>();
    let x2 = p2.get::<0>();
    let y2 = p2.get::<1>();
    let x3 = p3.get::<0>();
    let y3 = p3.get::<1>();
    let x4 = p4.get::<0>();
    let y4 = p4.get::<1>();

    (x1 < x3 && x1 < x4 && x2 < x3 && x2 < x4)
        || (x3 < x1 && x3 < x2 && x4 < x1 && x4 < x2)
        || (y1 < y3 && y1 < y4 && y2 < y3 && y2 < y4)
        || (y3 < y1 && y3 < y2 && y4 < y1 && y4 < y2)
}

/// Does any sub-segment of `ls` cross any sub-segment of `r` (with
/// `r`'s closing edge added explicitly if the ring is open)?
fn linestring_crosses_ring<L, R, P>(ls: &L, r: &R) -> bool
where
    L: LinestringTrait<Point = P>,
    R: RingTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
{
    let mut ils = ls.points();
    let Some(mut pls) = ils.next() else {
        return false;
    };
    for qls in ils {
        if ring_edge_crosses_segment(r, pls, qls) {
            return true;
        }
        pls = qls;
    }
    false
}

/// Does the segment `pls -> qls` cross any edge of the ring `r`?
fn ring_edge_crosses_segment<R, P>(r: &R, pls: &P, qls: &P) -> bool
where
    R: RingTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
{
    let mut ir = r.points();
    let Some(mut pr) = ir.next() else {
        return false;
    };
    let first = pr;
    let mut has_edge = false;
    for qr in ir {
        has_edge = true;
        if !segment_bounds_disjoint(pls, qls, pr, qr) && segments_intersect(pls, qls, pr, qr) {
            return true;
        }
        pr = qr;
    }
    if has_edge && pr.get::<0>() == first.get::<0>() && pr.get::<1>() == first.get::<1>() {
        return false;
    }
    // Close an open coordinate sequence explicitly. A one-point ring keeps
    // its degenerate edge so its established point-like behavior is intact.
    !segment_bounds_disjoint(pls, qls, pr, first) && segments_intersect(pls, qls, pr, first)
}

/// Does any edge of ring `a` cross any edge of ring `b`? Both rings
/// are walked with an explicit closing edge appended for open rings;
/// for closed rings the closing edge degenerates to zero length and
/// contributes nothing.
fn rings_cross<Ra, Rb, P>(a: &Ra, b: &Rb) -> bool
where
    Ra: RingTrait<Point = P>,
    Rb: RingTrait<Point = P>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
{
    let edges_a = ring_edges(a);
    let edges_b = ring_edges(b);
    for (pa, qa) in &edges_a {
        for (pb, qb) in &edges_b {
            if segments_intersect(*pa, *qa, *pb, *qb) {
                return true;
            }
        }
    }
    false
}

/// Materialise every edge of `r` as a `(prev, curr)` pair of point
/// references. The closing edge is appended explicitly so callers do
/// not need to special-case open vs. closed rings.
fn ring_edges<R>(r: &R) -> alloc::vec::Vec<(&R::Point, &R::Point)>
where
    R: RingTrait,
    R::Point: PointTrait,
{
    let pts: alloc::vec::Vec<&R::Point> = r.points().collect();
    let mut out = alloc::vec::Vec::with_capacity(pts.len());
    if pts.len() < 2 {
        return out;
    }
    for w in pts.windows(2) {
        out.push((w[0], w[1]));
    }
    out.push((*pts.last().unwrap(), *pts.first().unwrap()));
    out
}

#[cfg(test)]
mod tests {
    //! Reference values from
    //! `geometry/test/algorithms/intersects/intersects.cpp:38-79`.
    //! Each test cites the C++ line it mirrors.

    use super::{CartesianIntersects, IntersectsStrategy, Reversed};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, Segment, linestring, polygon};

    type P = Point2D<f64, Cartesian>;

    fn pt(x: f64, y: f64) -> P {
        Point2D::new(x, y)
    }

    use geometry_model::Linestring;
    type LS = Linestring<P>;

    /// `intersects.cpp:38` — linestring crosses segment.
    #[test]
    fn ls_crosses_segment() {
        let ls: LS = linestring![(1.0, 1.0), (3.0, 3.0), (2.0, 5.0)];
        let s = Segment::new(pt(2.0, 0.0), pt(2.0, 6.0));
        assert!(CartesianIntersects.intersects(&ls, &s));
    }

    /// `intersects.cpp:39` — linestring touches segment endpoint.
    #[test]
    fn ls_touches_segment_endpoint() {
        let ls: LS = linestring![(1.0, 1.0), (3.0, 3.0)];
        let s = Segment::new(pt(1.0, 0.0), pt(1.0, 1.0));
        assert!(CartesianIntersects.intersects(&ls, &s));
    }

    /// `intersects.cpp:41` — linestring disjoint from segment.
    #[test]
    fn ls_disjoint_from_segment() {
        let ls: LS = linestring![(1.0, 1.0), (3.0, 3.0)];
        let s = Segment::new(pt(3.0, 0.0), pt(4.0, 1.0));
        assert!(!CartesianIntersects.intersects(&ls, &s));
    }

    /// `intersects.cpp:50` — linestring crosses linestring.
    #[test]
    fn ls_crosses_ls() {
        let a: LS = linestring![(0.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let b: LS = linestring![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)];
        assert!(CartesianIntersects.intersects(&a, &b));
    }

    /// `intersects.cpp:55` — collinear overlap.
    #[test]
    fn ls_overlap_collinear() {
        let a: LS = linestring![(0.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let b: LS = linestring![(1.0, 0.0), (4.0, 0.0), (5.0, 0.0)];
        assert!(CartesianIntersects.intersects(&a, &b));
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
        assert!(CartesianIntersects.intersects(&ls, &p));
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
        assert!(!CartesianIntersects.intersects(&ls, &p));
    }

    /// `Reversed<S>` swaps the arguments transparently.
    #[test]
    fn reversed_pair_compiles_and_agrees() {
        let ls: LS = linestring![(1.0, 1.0), (2.0, 2.0)];
        let p: Polygon<P> = polygon![[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ]];
        let forward = CartesianIntersects.intersects(&ls, &p);
        let reversed = Reversed(CartesianIntersects).intersects(&p, &ls);
        assert_eq!(forward, reversed);
    }

    // KC1.T2 witness: proves this strategy accepts read-only `Point`
    // operands (that need not implement `PointMut`). If it compiles,
    // the read-only bound is locked.
    fn _accepts_readonly_point<A, B, S>(s: &S, a: &A, b: &B) -> bool
    where
        A: geometry_trait::Point,
        B: geometry_trait::Point,
        S: IntersectsStrategy<A, B>,
    {
        s.intersects(a, b)
    }

    use super::IxPointPoint;

    /// The read-only-point witness compiles *and* returns the correct
    /// membership when actually invoked.
    #[test]
    #[allow(
        clippy::used_underscore_items,
        reason = "the test exists to run the compile-time witness's body"
    )]
    fn readonly_point_witness_computes_equality() {
        assert!(_accepts_readonly_point(
            &IxPointPoint,
            &pt(1.0, 1.0),
            &pt(1.0, 1.0)
        ));
        assert!(!_accepts_readonly_point(
            &IxPointPoint,
            &pt(1.0, 1.0),
            &pt(2.0, 2.0)
        ));
    }
}
