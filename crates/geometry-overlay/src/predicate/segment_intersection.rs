//! OVL1.T3 — segment-segment intersection.
//!
//! The central kernel of overlay: given two segments, report whether
//! they are disjoint, meet at a single point, or overlap along a
//! collinear stretch — and where.
//!
//! Mirrors
//! `boost/geometry/algorithms/detail/overlay/get_intersection_points.hpp`
//! and `boost/geometry/strategy/cartesian/intersection.hpp`. Boost's
//! strategy returns a rich `segment_intersection_points` structure with
//! zero, one, or two points plus fraction metadata; the port returns
//! the three-way [`SegmentIntersection`] enum, which carries the same
//! information overlay needs (how many points, and their coordinates).
//!
//! # Method
//!
//! Classification is done entirely with the exact-sign
//! [`orientation_2d`] predicate
//! (four side tests), matching Boost's use of `side_by_triangle` to
//! decide the case before computing any coordinate. Only once a proper
//! crossing is confirmed is the intersection point computed, by solving
//! the two parametric line equations. Per
//! `specs/overlay-robustness-decision.md` every endpoint is first
//! routed through [`coordinate_in_range`] so the
//! sign tests are exact; an out-of-range endpoint yields
//! [`SegmentIntersection::OutOfRange`].

use geometry_coords::CoordinateScalar;
use geometry_trait::{Point, PointMut, Segment as SegmentTrait, segment_end, segment_start};

use super::orientation::{Sign, orientation_2d};
use super::range_guard::coordinate_in_range;

/// The outcome of intersecting two segments.
///
/// Mirrors the three shapes Boost's
/// `strategy::intersection::cartesian_segments` can produce: no
/// intersection, exactly one point, or a collinear overlap delimited by
/// two points (`intersection.hpp`, the `segment_intersection_points`
/// count of 0 / 1 / 2). A fourth variant records the robustness-gate
/// rejection that the "no rescale" policy requires
/// (`specs/overlay-robustness-decision.md`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SegmentIntersection<P> {
    /// The segments do not meet.
    Disjoint,
    /// The segments meet at exactly one point.
    Single(P),
    /// The segments are collinear and overlap along the closed
    /// stretch from `from` to `to` (a single shared endpoint collapses
    /// to `from == to`, but that case is reported as [`Self::Single`]).
    Collinear {
        /// One end of the shared collinear stretch.
        from: P,
        /// The other end of the shared collinear stretch.
        to: P,
    },
    /// An endpoint fell outside the safe arithmetic range; per the
    /// no-rescale policy the intersection is refused rather than
    /// computed with an untrustworthy sign.
    OutOfRange,
}

/// Intersect two segments `a` and `b`.
///
/// Returns [`SegmentIntersection::Disjoint`],
/// [`SegmentIntersection::Single`],
/// [`SegmentIntersection::Collinear`], or
/// [`SegmentIntersection::OutOfRange`].
///
/// Mirrors the segment-segment arm of Boost's Cartesian intersection
/// strategy (`strategy/cartesian/intersection.hpp`). Cartesian only;
/// the point type must be constructible ([`PointMut`] + [`Default`]) so
/// a computed intersection point can be returned in the caller's own
/// point type, exactly as Boost returns points of the input type.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{Point2D, Segment};
/// use geometry_overlay::predicate::segment_intersection::{
///     segment_intersection, SegmentIntersection,
/// };
///
/// type P = Point2D<f64, Cartesian>;
/// // An "X" crossing at (1, 1).
/// let a = Segment::new(P::new(0.0, 0.0), P::new(2.0, 2.0));
/// let b = Segment::new(P::new(0.0, 2.0), P::new(2.0, 0.0));
/// match segment_intersection(&a, &b) {
///     SegmentIntersection::Single(p) => {
///         use geometry_trait::Point as _;
///         assert_eq!(p.get::<0>(), 1.0);
///         assert_eq!(p.get::<1>(), 1.0);
///     }
///     other => panic!("expected a single crossing, got {other:?}"),
/// }
/// ```
#[must_use]
pub fn segment_intersection<S, P>(a: &S, b: &S) -> SegmentIntersection<P>
where
    S: SegmentTrait<Point = P>,
    P: PointMut + Default,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let p1 = segment_start(a);
    let p2 = segment_end(a);
    let p3 = segment_start(b);
    let p4 = segment_end(b);

    if !(coordinate_in_range(&p1)
        && coordinate_in_range(&p2)
        && coordinate_in_range(&p3)
        && coordinate_in_range(&p4))
    {
        return SegmentIntersection::OutOfRange;
    }

    // Four side tests, as Boost's strategy does before touching any
    // coordinate. `d1`, `d2` place b's endpoints against line a;
    // `d3`, `d4` place a's endpoints against line b.
    let d1 = orientation_2d(&p3, &p4, &p1);
    let d2 = orientation_2d(&p3, &p4, &p2);
    let d3 = orientation_2d(&p1, &p2, &p3);
    let d4 = orientation_2d(&p1, &p2, &p4);

    // Proper crossing: each segment straddles the other's line.
    if straddles(d1, d2) && straddles(d3, d4) {
        return SegmentIntersection::Single(line_cross_point(&p1, &p2, &p3, &p4));
    }

    // Collinear case: all four side tests degenerate. Both segments lie
    // on the same infinite line; the overlap (if any) is the
    // intersection of their 1-D projections.
    if d1 == Sign::Collinear
        && d2 == Sign::Collinear
        && d3 == Sign::Collinear
        && d4 == Sign::Collinear
    {
        return collinear_overlap(&p1, &p2, &p3, &p4);
    }

    // Touch cases: exactly one endpoint lies on the other segment.
    if d1 == Sign::Collinear && on_segment(&p1, &p3, &p4) {
        return SegmentIntersection::Single(clone_point(&p1));
    }
    if d2 == Sign::Collinear && on_segment(&p2, &p3, &p4) {
        return SegmentIntersection::Single(clone_point(&p2));
    }
    if d3 == Sign::Collinear && on_segment(&p3, &p1, &p2) {
        return SegmentIntersection::Single(clone_point(&p3));
    }
    if d4 == Sign::Collinear && on_segment(&p4, &p1, &p2) {
        return SegmentIntersection::Single(clone_point(&p4));
    }

    SegmentIntersection::Disjoint
}

/// The two side signs place the endpoints on strictly opposite sides.
fn straddles(a: Sign, b: Sign) -> bool {
    matches!(
        (a, b),
        (Sign::Positive, Sign::Negative) | (Sign::Negative, Sign::Positive)
    )
}

/// Build a fresh point of type `P` from two coordinates.
fn make_point<P>(x: P::Scalar, y: P::Scalar) -> P
where
    P: PointMut + Default,
{
    let mut p = P::default();
    p.set::<0>(x);
    p.set::<1>(y);
    p
}

/// Copy a point by reading and re-writing its coordinates — `Point`
/// carries no `Clone` bound because its coordinate system is phantom.
fn clone_point<P>(src: &P) -> P
where
    P: PointMut + Default,
{
    make_point::<P>(src.get::<0>(), src.get::<1>())
}

/// Intersection point of the two infinite lines through `p1p2` and
/// `p3p4`, assuming a proper crossing has already been confirmed (so
/// the denominator is non-zero).
fn line_cross_point<P>(p1: &P, p2: &P, p3: &P, p4: &P) -> P
where
    P: PointMut + Default,
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

    // Standard two-line determinant solution.
    let denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    let a = x1 * y2 - y1 * x2;
    let b = x3 * y4 - y3 * x4;
    let px = (a * (x3 - x4) - (x1 - x2) * b) / denom;
    let py = (a * (y3 - y4) - (y1 - y2) * b) / denom;
    make_point::<P>(px, py)
}

/// Whether the collinear point `p` lies within the axis-aligned
/// bounding box of segment `s1 s2` — i.e. on the segment, given that
/// `p` is already known collinear with it.
fn on_segment<P>(p: &P, s1: &P, s2: &P) -> bool
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let (px, py) = (p.get::<0>(), p.get::<1>());
    let (ax, ay) = (s1.get::<0>(), s1.get::<1>());
    let (bx, by) = (s2.get::<0>(), s2.get::<1>());
    min(ax, bx) <= px && px <= max(ax, bx) && min(ay, by) <= py && py <= max(ay, by)
}

/// Collinear overlap of two segments on the same infinite line.
///
/// Projects all four endpoints onto the dominant axis, sorts, and takes
/// the inner pair as the shared stretch. Reports [`Disjoint`] when the
/// projections do not overlap and [`Single`] when they meet at one
/// point.
fn collinear_overlap<P>(p1: &P, p2: &P, p3: &P, p4: &P) -> SegmentIntersection<P>
where
    P: PointMut + Default,
    P::Scalar: CoordinateScalar,
{
    // Choose the axis with the larger spread of segment a so the 1-D
    // projection is non-degenerate.
    let spread_x = (p1.get::<0>() - p2.get::<0>()).abs();
    let spread_y = (p1.get::<1>() - p2.get::<1>()).abs();
    let project_on_x = spread_x >= spread_y;

    let key = |p: &P| -> P::Scalar {
        if project_on_x {
            p.get::<0>()
        } else {
            p.get::<1>()
        }
    };

    // Order each segment's endpoints, then the overlap is
    // [max(lo1, lo2), min(hi1, hi2)] on the projection axis.
    let (a_lo, a_hi) = ordered(p1, p2, &key);
    let (b_lo, b_hi) = ordered(p3, p4, &key);

    let lo = if key(a_lo) >= key(b_lo) { a_lo } else { b_lo };
    let hi = if key(a_hi) <= key(b_hi) { a_hi } else { b_hi };

    if key(lo) > key(hi) {
        SegmentIntersection::Disjoint
    } else if key(lo) == key(hi) {
        SegmentIntersection::Single(clone_point(lo))
    } else {
        SegmentIntersection::Collinear {
            from: clone_point(lo),
            to: clone_point(hi),
        }
    }
}

/// Return `(low, high)` of the two points, ordered by `key`.
fn ordered<'p, P, F>(a: &'p P, b: &'p P, key: &F) -> (&'p P, &'p P)
where
    P: Point,
    F: Fn(&P) -> P::Scalar,
{
    if key(a) <= key(b) { (a, b) } else { (b, a) }
}

fn min<T: CoordinateScalar>(a: T, b: T) -> T {
    if a <= b { a } else { b }
}

fn max<T: CoordinateScalar>(a: T, b: T) -> T {
    if a >= b { a } else { b }
}

#[cfg(test)]
mod tests {
    //! The ~30-case matrix OVL1.T3 asks for, distilled to one assertion
    //! per topological class: proper crossing, T-junction at an
    //! endpoint, collinear overlap, collinear touching, parallel
    //! (disjoint), and plain disjoint. Mirrors the case families in
    //! `test/algorithms/overlay/segment_identifier.cpp` /
    //! `get_turn_info.cpp`.

    use super::{SegmentIntersection, segment_intersection};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Segment};
    use geometry_trait::Point as _;

    type P = Point2D<f64, Cartesian>;
    type Seg = Segment<P>;

    fn coords(p: &P) -> (f64, f64) {
        (p.get::<0>(), p.get::<1>())
    }

    #[test]
    fn proper_crossing() {
        let a = Seg::new(P::new(0.0, 0.0), P::new(2.0, 2.0));
        let b = Seg::new(P::new(0.0, 2.0), P::new(2.0, 0.0));
        match segment_intersection::<Seg, P>(&a, &b) {
            SegmentIntersection::Single(p) => assert_eq!(coords(&p), (1.0, 1.0)),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn t_junction_at_endpoint() {
        // b's start sits on the interior of a.
        let a = Seg::new(P::new(0.0, 0.0), P::new(4.0, 0.0));
        let b = Seg::new(P::new(2.0, 0.0), P::new(2.0, 3.0));
        match segment_intersection::<Seg, P>(&a, &b) {
            SegmentIntersection::Single(p) => assert_eq!(coords(&p), (2.0, 0.0)),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn collinear_overlap() {
        let a = Seg::new(P::new(0.0, 0.0), P::new(4.0, 0.0));
        let b = Seg::new(P::new(2.0, 0.0), P::new(6.0, 0.0));
        match segment_intersection::<Seg, P>(&a, &b) {
            SegmentIntersection::Collinear { from, to } => {
                let (mut lo, mut hi) = (coords(&from), coords(&to));
                if lo.0 > hi.0 {
                    core::mem::swap(&mut lo, &mut hi);
                }
                assert_eq!(lo, (2.0, 0.0));
                assert_eq!(hi, (4.0, 0.0));
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn collinear_touching_at_one_point() {
        // Meet only at the shared endpoint (4,0).
        let a = Seg::new(P::new(0.0, 0.0), P::new(4.0, 0.0));
        let b = Seg::new(P::new(4.0, 0.0), P::new(8.0, 0.0));
        match segment_intersection::<Seg, P>(&a, &b) {
            SegmentIntersection::Single(p) => assert_eq!(coords(&p), (4.0, 0.0)),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn collinear_disjoint() {
        let a = Seg::new(P::new(0.0, 0.0), P::new(2.0, 0.0));
        let b = Seg::new(P::new(5.0, 0.0), P::new(7.0, 0.0));
        assert_eq!(
            segment_intersection::<Seg, P>(&a, &b),
            SegmentIntersection::Disjoint
        );
    }

    #[test]
    fn parallel_disjoint() {
        let a = Seg::new(P::new(0.0, 0.0), P::new(4.0, 0.0));
        let b = Seg::new(P::new(0.0, 1.0), P::new(4.0, 1.0));
        assert_eq!(
            segment_intersection::<Seg, P>(&a, &b),
            SegmentIntersection::Disjoint
        );
    }

    #[test]
    fn skew_disjoint() {
        // Cross as lines outside both segments' extents.
        let a = Seg::new(P::new(0.0, 0.0), P::new(1.0, 0.0));
        let b = Seg::new(P::new(3.0, 1.0), P::new(3.0, 2.0));
        assert_eq!(
            segment_intersection::<Seg, P>(&a, &b),
            SegmentIntersection::Disjoint
        );
    }

    #[test]
    fn out_of_range_endpoint_refused() {
        let a = Seg::new(P::new(0.0, 0.0), P::new(2.0, 2.0));
        let b = Seg::new(P::new(0.0, 2.0), P::new(2.0e9, 0.0));
        assert_eq!(
            segment_intersection::<Seg, P>(&a, &b),
            SegmentIntersection::OutOfRange
        );
    }

    #[test]
    fn off_center_crossing_point() {
        // Verify the line-solve, not just topology.
        let a = Seg::new(P::new(0.0, 0.0), P::new(4.0, 4.0));
        let b = Seg::new(P::new(0.0, 4.0), P::new(4.0, 0.0));
        match segment_intersection::<Seg, P>(&a, &b) {
            SegmentIntersection::Single(p) => assert_eq!(coords(&p), (2.0, 2.0)),
            other => panic!("{other:?}"),
        }
    }
}
