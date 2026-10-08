//! Per-CS strategy for the `equals` set-relation algorithm.
//!
//! Mirrors `boost::geometry::equals` (`boost/geometry/algorithms/equals.hpp`)
//! for Cartesian points, segments and polygons.
//!
//! Points are equal as Boost's `equals_point_point` makes them: by
//! `math::equals` in every dimension ([`CoordinateScalar::tolerant_eq`]);
//! segments when their ends are, either way round. Two polygons are equal as
//! Boost's Cartesian `equals_by_collection<area_check>` decides
//! (`algorithms/detail/equals/implementation.hpp:135-237`): their areas agree
//! by `math::equals`, and so do their edges, each taken as its start and its
//! unit direction, with an edge running on in the direction of the one before
//! merged into it (`algorithms/detail/equals/collect_vectors.hpp`). A ring may
//! start at any vertex and carry vertices that do not turn it; it is read in
//! its declared orientation.
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

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::{PointTag, PolygonTag, SameAs, SegmentTag};
use geometry_trait::{
    Point as PointTrait, PointMut, Polygon as PolygonTrait, Ring as RingTrait,
    Segment as SegmentTrait, fold_dims, ordinate, segment_end, segment_start,
};

use crate::area::{AreaStrategy, ShoelacePolygonArea};
use crate::clockwise_view::closed_clockwise_points;

type Measure<P> = <<P as PointTrait>::Scalar as CoordinateScalar>::Measure;

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
// Coordinate-wise `math::equals`. Mirrors the pointlike/pointlike arm at
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
                0 => a.get::<0>().tolerant_eq(b.get::<0>()),
                1 => a.get::<1>().tolerant_eq(b.get::<1>()),
                2 => a.get::<2>().tolerant_eq(b.get::<2>()),
                3 => a.get::<3>().tolerant_eq(b.get::<3>()),
                _ => panic!("CartesianEquals: dimension exceeds MAX_DIM (4)"),
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
        (point_eq(&a1, &b1) && point_eq(&a2, &b2)) || (point_eq(&a1, &b2) && point_eq(&a2, &b1))
    }
}

// ---- Polygon × Polygon -----------------------------------------------
//
// Equal areas, then equal collected vectors once both collections are
// sorted. Mirrors `equals_by_collection<area_check>`, the Cartesian
// polygon/polygon arm at `algorithms/detail/equals/implementation.hpp:
// 135-237,334-336`.

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
    ShoelacePolygonArea: AreaStrategy<A, Out = Measure<P>> + AreaStrategy<B, Out = Measure<P>>,
{
    fn equals(&self, a: &A, b: &B) -> bool {
        let first_area = <ShoelacePolygonArea as AreaStrategy<A>>::area(&ShoelacePolygonArea, a);
        let second_area = <ShoelacePolygonArea as AreaStrategy<B>>::area(&ShoelacePolygonArea, b);
        if !first_area.tolerant_eq(second_area) {
            return false;
        }
        let mut first = polygon_vectors(a);
        let mut second = polygon_vectors(b);
        if first.len() != second.len() {
            return false;
        }
        sort_collected(&mut first);
        sort_collected(&mut second);
        first
            .iter()
            .zip(&second)
            .all(|(first, second)| first.matches(second))
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

/// Do two points coincide in every dimension of `a`, by `math::equals`?
#[inline]
fn point_eq<Pa, Pb>(a: &Pa, b: &Pb) -> bool
where
    Pa: PointTrait,
    Pb: PointTrait<Scalar = Pa::Scalar>,
{
    fold_dims(true, a, |equal, a, d| {
        equal && ordinate(a, d).tolerant_eq(ordinate(b, d))
    })
}

/// An edge as Boost's `collected_vector_cartesian` keeps it: its start and
/// its direction scaled to unit length
/// (`algorithms/detail/equals/collect_vectors.hpp:41-128`).
#[derive(Clone, Copy)]
struct CollectedVector<T> {
    x: T,
    y: T,
    dx: T,
    dy: T,
}

impl<T: CoordinateScalar> CollectedVector<T> {
    /// The edge from `start` to `end`; none when it has no length
    /// (`normalize`).
    fn new<P>(start: &P, end: &P) -> Option<Self>
    where
        P: PointTrait,
        P::Scalar: CoordinateScalar<Measure = T>,
    {
        let (x, y) = (start.get::<0>().to_measure(), start.get::<1>().to_measure());
        let dx = end.get::<0>().to_measure() - x;
        let dy = end.get::<1>().to_measure() - y;
        let magnitude = (dx * dx + dy * dy).sqrt();
        (magnitude > T::ZERO).then(|| Self {
            x,
            y,
            dx: dx / magnitude,
            dy: dy / magnitude,
        })
    }

    /// `same_direction`: the unit directions agree by `math::equals`.
    fn same_direction(&self, other: &Self) -> bool {
        self.dx.tolerant_eq(other.dx) && self.dy.tolerant_eq(other.dy)
    }

    /// `operator==`.
    fn matches(&self, other: &Self) -> bool {
        self.x.tolerant_eq(other.x) && self.y.tolerant_eq(other.y) && self.same_direction(other)
    }

    /// `operator<`.
    fn precedes(&self, other: &Self) -> bool {
        if !self.x.tolerant_eq(other.x) {
            self.x < other.x
        } else if !self.y.tolerant_eq(other.y) {
            self.y < other.y
        } else if !self.dx.tolerant_eq(other.dx) {
            self.dx < other.dx
        } else {
            self.dy < other.dy
        }
    }
}

/// The collected vectors of a polygon, exterior ring first
/// (`polygon_collect_vectors`).
fn polygon_vectors<G>(polygon: &G) -> Vec<CollectedVector<Measure<G::Point>>>
where
    G: PolygonTrait,
    G::Point: PointTrait,
{
    let mut vectors = Vec::new();
    collect_ring_vectors(polygon.exterior(), &mut vectors);
    for ring in polygon.interiors() {
        collect_ring_vectors(ring, &mut vectors);
    }
    vectors
}

/// Append a ring's edges, walked as `closed_clockwise_view` presents it:
/// an edge without length is skipped, one running on in the direction of
/// the last kept is merged into it, and a last edge running on into the
/// first takes the first's place (`range_collect_vectors`).
fn collect_ring_vectors<R>(ring: &R, vectors: &mut Vec<CollectedVector<Measure<R::Point>>>)
where
    R: RingTrait,
    R::Point: PointTrait,
{
    let points = closed_clockwise_points(ring);
    let start = vectors.len();
    let mut is_first = true;
    for edge in points.windows(2) {
        if let Some(vector) = CollectedVector::new(edge[0], edge[1]) {
            if is_first || !vectors[vectors.len() - 1].same_direction(&vector) {
                vectors.push(vector);
            }
            is_first = false;
        }
    }
    if vectors.len() > start + 1 && vectors[vectors.len() - 1].same_direction(&vectors[start]) {
        vectors[start] = vectors.pop().expect("more than one vector was collected");
    }
}

/// Order collected vectors by Boost's `operator<`, as `std::sort` orders
/// them. Its tolerance keeps it from being a total order, for which
/// `slice::sort_by` may panic; a merge sort settles on an order regardless.
fn sort_collected<T: CoordinateScalar>(vectors: &mut [CollectedVector<T>]) {
    if vectors.len() < 2 {
        return;
    }
    let middle = vectors.len() / 2;
    sort_collected(&mut vectors[..middle]);
    sort_collected(&mut vectors[middle..]);
    let mut merged = Vec::with_capacity(vectors.len());
    let (mut left, mut right) = (0, middle);
    while left < middle && right < vectors.len() {
        if vectors[right].precedes(&vectors[left]) {
            merged.push(vectors[right]);
            right += 1;
        } else {
            merged.push(vectors[left]);
            left += 1;
        }
    }
    merged.extend_from_slice(&vectors[left..middle]);
    merged.extend_from_slice(&vectors[right..]);
    vectors.copy_from_slice(&merged);
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

    /// A ring is read in its declared orientation: the square run the other
    /// way has the opposite signed area, so Boost's `area_check` already
    /// tells the two apart.
    #[test]
    fn a_ring_run_against_its_orientation_is_another_polygon() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)]];
        assert!(!EqPolygonPolygon.equals(&a, &b));
    }

    /// A vertex computed onto an edge turns it only by rounding, and Boost
    /// merges edges whose unit directions agree by `math::equals`. Boost
    /// (`aed7bc3`): the triangle equals itself rotated with an edge's
    /// midpoint inserted.
    #[test]
    fn a_vertex_rounded_onto_an_edge_does_not_turn_it() {
        let a: Polygon<P> = polygon![[
            (7.242_133_728_502_334, 6.467_601_271_821),
            (2.206_707_654_691_149_5, 6.086_749_332_056_838),
            (6.166_629_904_416_364, 7.389_201_017_986_58),
            (7.242_133_728_502_334, 6.467_601_271_821)
        ]];
        let b: Polygon<P> = polygon![[
            (6.166_629_904_416_364, 7.389_201_017_986_58),
            (7.242_133_728_502_334, 6.467_601_271_821),
            (4.724_420_691_596_742, 6.277_175_301_938_919),
            (2.206_707_654_691_149_5, 6.086_749_332_056_838),
            (6.166_629_904_416_364, 7.389_201_017_986_58)
        ]];
        assert!(EqPolygonPolygon.equals(&a, &b));
        assert!(EqPolygonPolygon.equals(&b, &a));
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
    #[allow(
        clippy::used_underscore_items,
        reason = "the test exists to run the compile-time witness's body"
    )]
    fn readonly_witness_computes_equality() {
        assert!(_accepts_readonly_point(
            &EqPointPoint,
            &pt(1.0, 1.0),
            &pt(1.0, 1.0)
        ));
        assert!(!_accepts_readonly_point(
            &EqPointPoint,
            &pt(1.0, 1.0),
            &pt(2.0, 2.0)
        ));
    }

    /// Point, segment, and ring equality compare every dimension, not
    /// just the first two.
    #[test]
    fn three_dimensional_geometries_differing_in_z_are_not_equal() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let a = Segment::new(P3::new(0.0, 0.0, 0.0), P3::new(1.0, 1.0, 0.0));
        let b = Segment::new(P3::new(0.0, 0.0, 5.0), P3::new(1.0, 1.0, 5.0));
        assert!(!EqPointPoint.equals(&P3::new(0.0, 0.0, 0.0), &P3::new(0.0, 0.0, 5.0)));
        assert!(!EqSegmentSegment.equals(&a, &b));
        assert!(EqSegmentSegment.equals(&a, &a));
    }

    /// Boost's areal `equals` is topological: a vertex lying inside a
    /// straight edge, or a repeated vertex, does not change the point set.
    #[test]
    fn rings_with_redundant_vertices_describe_the_same_region() {
        let a: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)]];
        let b: Polygon<P> = polygon![[
            (0.0, 0.0),
            (0.0, 4.0),
            (2.0, 4.0),
            (4.0, 4.0),
            (4.0, 0.0),
            (0.0, 0.0)
        ]];
        let c: Polygon<P> = polygon![[
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 4.0),
            (4.0, 4.0),
            (4.0, 4.0),
            (4.0, 0.0),
            (0.0, 0.0)
        ]];
        assert!(EqPolygonPolygon.equals(&a, &b));
        assert!(EqPolygonPolygon.equals(&b, &a));
        assert!(EqPolygonPolygon.equals(&a, &c));
        // A vertex that bends the boundary is not redundant.
        let notch: Polygon<P> = polygon![[
            (0.0, 0.0),
            (0.0, 4.0),
            (2.0, 3.0),
            (4.0, 4.0),
            (4.0, 0.0),
            (0.0, 0.0)
        ]];
        assert!(!EqPolygonPolygon.equals(&a, &notch));
    }
}
