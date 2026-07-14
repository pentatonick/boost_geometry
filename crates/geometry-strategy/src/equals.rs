//! Per-CS strategy for the `equals` set-relation algorithm.
//!
//! Compares two geometries for equality up to **vertex sequence**: a
//! ring may start at a different vertex and run in either direction and
//! still compare equal, but the two rings must have the *same distinct
//! vertices*. This is a v1 simplification of `boost::geometry::equals`
//! (`boost/geometry/algorithms/equals.hpp`), whose areal arm is fully
//! topological (point-set + area via `collect_vectors`, so a ring with a
//! redundant collinear vertex still equals the same ring without it).
//!
//! Consequence: two polygons that describe the same region but differ in
//! how many collinear vertices they list compare **not equal** here. Run
//! `remove_spikes` / `unique` first to normalise vertices if full
//! topological equality is needed. The full topological comparison is
//! deferred; see `algorithms/detail/equals/implementation.hpp:36-160`.
//!
//! ## Symmetry
//!
//! `equals` is symmetric: `equals(a, b) == equals(b, a)`. Only the three
//! diagonal (same-kind) pairs are implemented, and the algorithm layer
//! does not need a reversed direction, so no `Reversed` wrapper is
//! required here.
//!
//! ## Tag dispatch (open to foreign types)
//!
//! Each diagonal pair is a distinct per-pair strategy struct
//! ([`EqPointPoint`], [`EqSegmentSegment`], [`EqPolygonPolygon`]) with a
//! single concept-pair-bounded [`EqualsStrategy`] impl; the tag-keyed
//! [`EqualsPairStrategy`] picker routes `(A::Kind, B::Kind)` to the right
//! struct. Because it keys on the tags, a concept-adapted foreign type
//! resolves through the same path as the equivalent `geometry-model`
//! value.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::{PointTag, PolygonTag, SameAs, SegmentTag};
use geometry_trait::{
    Point as PointTrait, PointMut, Polygon as PolygonTrait, Ring as RingTrait,
    Segment as SegmentTrait, segment_end, segment_start,
};

/// A strategy for "do these two geometries describe the same point
/// set?".
///
/// Mirrors `boost::geometry::equals(g1, g2)` from
/// `boost/geometry/algorithms/equals.hpp`.
pub trait EqualsStrategy<A, B> {
    /// `true` iff `a` and `b` describe the same point set.
    fn equals(&self, a: &A, b: &B) -> bool;
}

/// Cartesian equals for a pair of points. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct EqPointPoint;
/// Cartesian equals for a pair of segments. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct EqSegmentSegment;
/// Cartesian equals for a pair of polygons. See the [module docs](self).
#[derive(Debug, Default, Clone, Copy)]
pub struct EqPolygonPolygon;

// ---- Point × Point ---------------------------------------------------
//
// Coordinate-wise equality. Mirrors the pointlike/pointlike arm at
// `algorithms/detail/equals/implementation.hpp:36-71`.

impl<A, B> EqualsStrategy<A, B> for EqPointPoint
where
    A: PointTrait,
    B: PointTrait<Scalar = A::Scalar>,
    <A::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn equals(&self, a: &A, b: &B) -> bool {
        let mut i = 0;
        while i < A::DIM {
            let eq = match i {
                0 => a.get::<0>() == b.get::<0>(),
                1 => a.get::<1>() == b.get::<1>(),
                2 => a.get::<2>() == b.get::<2>(),
                _ => true,
            };
            if !eq {
                return false;
            }
            i += 1;
        }
        true
    }
}

// ---- Segment × Segment -----------------------------------------------
//
// Two segments are equal iff they describe the same point set —
// matching endpoints in either direction. Mirrors the segment/segment
// arm at `algorithms/detail/equals/implementation.hpp:73-120`.

impl<A, B, P> EqualsStrategy<A, B> for EqSegmentSegment
where
    A: SegmentTrait<Point = P>,
    B: SegmentTrait<Point = P>,
    P: PointTrait + PointMut + Default,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    #[inline]
    fn equals(&self, a: &A, b: &B) -> bool {
        let (a1, a2) = (segment_start(a), segment_end(a));
        let (b1, b2) = (segment_start(b), segment_end(b));
        (point_eq_2d(&a1, &b1) && point_eq_2d(&a2, &b2))
            || (point_eq_2d(&a1, &b2) && point_eq_2d(&a2, &b1))
    }
}

// ---- Polygon × Polygon -----------------------------------------------
//
// Two polygons are equal iff their exterior rings describe the same
// closed loop (modulo starting vertex and traversal direction) and
// their interior rings match pairwise under some permutation.
// Mirrors the polygon/polygon arm at
// `algorithms/detail/equals/implementation.hpp:120-160`.

impl<A, B, P> EqualsStrategy<A, B> for EqPolygonPolygon
where
    A: PolygonTrait<Point = P>,
    // Both operands share the same ring type — this keeps the pair a
    // true diagonal (a `ModelPolygon<P, CW, CL>` compares only against a
    // polygon with the same `Ring<P, CW, CL>`) so vertex order/closure
    // conventions line up and the two operands' const params unify.
    B: PolygonTrait<Point = P, Ring = A::Ring>,
    P: PointTrait,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn equals(&self, a: &A, b: &B) -> bool {
        if !rings_equal(a.exterior(), b.exterior()) {
            return false;
        }
        if a.interiors().count() != b.interiors().count() {
            return false;
        }
        // For each inner ring in `a`, find a matching inner ring in
        // `b` not yet consumed. v1: O(n^2) — interior counts are
        // tiny in practice.
        let bh: alloc::vec::Vec<&B::Ring> = b.interiors().collect();
        let mut matched = alloc::vec![false; bh.len()];
        for ha in a.interiors() {
            let mut found = false;
            for (j, hb) in bh.iter().enumerate() {
                if !matched[j] && rings_equal(ha, *hb) {
                    matched[j] = true;
                    found = true;
                    break;
                }
            }
            if !found {
                return false;
            }
        }
        true
    }
}

/// Type-level "which `EqualsStrategy` struct does this ordered pair of
/// geometry *kinds* use". A trait parameterised by the second tag `K2`,
/// keyed on the first tag `Self` — disjoint on the pair, so no overlap.
/// Only the three diagonal (same-kind) pairs are implemented. The
/// [`crate::equals`] free function routes `(A::Kind, B::Kind)` through
/// this trait.
#[doc(hidden)]
pub trait EqualsPairStrategy<K2> {
    /// The per-pair [`EqualsStrategy`] struct this tag pair is computed
    /// with.
    type S: Default;
}

impl EqualsPairStrategy<PointTag> for PointTag {
    type S = EqPointPoint;
}
impl EqualsPairStrategy<SegmentTag> for SegmentTag {
    type S = EqSegmentSegment;
}
impl EqualsPairStrategy<PolygonTag> for PolygonTag {
    type S = EqPolygonPolygon;
}

extern crate alloc;

// ---- Kernels ---------------------------------------------------------

#[inline]
fn point_eq_2d<Pa, Pb>(a: &Pa, b: &Pb) -> bool
where
    Pa: PointTrait,
    Pb: PointTrait<Scalar = Pa::Scalar>,
{
    a.get::<0>() == b.get::<0>() && a.get::<1>() == b.get::<1>()
}

/// Are two rings equal as closed loops? Equal distinct-vertex sequence
/// up to rotation and reversal (free starting vertex, free direction).
/// This is the v1 vertex-sequence simplification of Boost's topological
/// `equals::ring_or_polygon::apply`
/// (`algorithms/detail/equals/implementation.hpp:120-160`): a ring with
/// a redundant collinear vertex will *not* match one without it.
fn rings_equal<Ra, Rb>(a: &Ra, b: &Rb) -> bool
where
    Ra: RingTrait,
    Rb: RingTrait,
    Ra::Point: PointTrait,
    Rb::Point: PointTrait<Scalar = <Ra::Point as PointTrait>::Scalar>,
{
    let av = normalise_ring(a);
    let bv = normalise_ring(b);
    if av.len() != bv.len() {
        return false;
    }
    let n = av.len();
    if n == 0 {
        return true;
    }
    // Try every rotation of `b`, forward and reversed.
    for start in 0..n {
        if cyclic_match(&av, &bv, start, false) {
            return true;
        }
        if cyclic_match(&av, &bv, start, true) {
            return true;
        }
    }
    false
}

/// Strip the trailing closing vertex from a closed ring so the
/// rotation search compares only the distinct loop vertices.
fn normalise_ring<R>(r: &R) -> alloc::vec::Vec<&R::Point>
where
    R: RingTrait,
    R::Point: PointTrait,
{
    let mut pts: alloc::vec::Vec<&R::Point> = r.points().collect();
    if pts.len() >= 2 && point_eq_2d(pts[0], pts[pts.len() - 1]) {
        pts.pop();
    }
    pts
}

/// Does `a` match `b` when `b` is read starting at `start` and
/// optionally in reverse?
fn cyclic_match<Pa, Pb>(a: &[&Pa], b: &[&Pb], start: usize, reverse: bool) -> bool
where
    Pa: PointTrait,
    Pb: PointTrait<Scalar = Pa::Scalar>,
{
    let n = a.len();
    for (i, ai) in a.iter().enumerate() {
        let j = if reverse {
            (start + n - i) % n
        } else {
            (start + i) % n
        };
        if !point_eq_2d(*ai, b[j]) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::{EqPointPoint, EqPolygonPolygon, EqSegmentSegment, EqualsStrategy};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, Segment, polygon};

    type P = Point2D<f64, Cartesian>;

    fn pt(x: f64, y: f64) -> P {
        Point2D::new(x, y)
    }

    #[test]
    fn equals_same_point() {
        assert!(EqPointPoint.equals(&pt(1.0, 2.0), &pt(1.0, 2.0)));
        assert!(!EqPointPoint.equals(&pt(1.0, 2.0), &pt(1.0, 2.1)));
    }

    #[test]
    fn equals_segment_either_direction() {
        let a = Segment::new(pt(0.0, 0.0), pt(1.0, 1.0));
        let b = Segment::new(pt(1.0, 1.0), pt(0.0, 0.0));
        assert!(EqSegmentSegment.equals(&a, &b));
        let c = Segment::new(pt(0.0, 0.0), pt(1.0, 2.0));
        assert!(!EqSegmentSegment.equals(&a, &c));
    }

    #[test]
    fn equals_polygon_rotated_start() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
        // Same loop, different starting vertex.
        let b: Polygon<P> = polygon![[(4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0), (4.0, 0.0)]];
        assert!(EqPolygonPolygon.equals(&a, &b));
    }

    #[test]
    fn equals_polygon_reversed_direction() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)]];
        assert!(EqPolygonPolygon.equals(&a, &b));
    }

    #[test]
    fn polygon_not_equals_different_shape() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)]];
        assert!(!EqPolygonPolygon.equals(&a, &b));
    }

    // KC1.T2 witness: proves this strategy accepts read-only `Point`
    // operands (that need not implement `PointMut`). If it compiles,
    // the read-only bound is locked.
    fn _accepts_readonly_point<A, B, S>(s: &S, a: &A, b: &B) -> bool
    where
        A: geometry_trait::Point,
        B: geometry_trait::Point,
        S: EqualsStrategy<A, B>,
    {
        s.equals(a, b)
    }






    /// The read-only-point witness computes membership when invoked with
    /// a concrete strategy and points.
    #[test]
    #[allow(clippy::used_underscore_items, reason = "the test exists to run the compile-time witness's body")]
    fn readonly_witness_computes_equality() {
        assert!(_accepts_readonly_point(&EqPointPoint, &pt(1.0, 1.0), &pt(1.0, 1.0)));
        assert!(!_accepts_readonly_point(&EqPointPoint, &pt(1.0, 1.0), &pt(2.0, 2.0)));
    }
}
