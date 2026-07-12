//! M-OVL1 — predicate-parity milestone.
//!
//! Reproduces the predicate case matrix overlay depends on, matching
//! the case families Boost enumerates in
//! `test/algorithms/overlay/` (`get_turn_info.cpp`,
//! `segment_identifier.cpp`) and the side convention in
//! `test/strategies/spherical_side.cpp`. Each assertion is a
//! hand-verified reference value at Boost's exactness (these are
//! integer-coordinate cases, so equality is exact — no tolerance
//! band needed).

use geometry_cs::Cartesian;
use geometry_model::{Point2D, Segment};
use geometry_overlay::predicate::{
    SegmentIntersection, Sign, in_circle_2d, orientation_2d, segment_intersection,
};
use geometry_trait::Point as _;

type P = Point2D<f64, Cartesian>;
type Seg = Segment<P>;

fn xy(p: &P) -> (f64, f64) {
    (p.get::<0>(), p.get::<1>())
}

// ---- Orientation: the 'L' / 'R' / collinear matrix ------------------

#[test]
fn orientation_side_matrix() {
    let p = P::new(0.0, 0.0);
    let q = P::new(4.0, 0.0);
    // Points above / on / below the directed base line.
    assert_eq!(orientation_2d(&p, &q, &P::new(2.0, 3.0)), Sign::Positive); // L
    assert_eq!(orientation_2d(&p, &q, &P::new(2.0, -3.0)), Sign::Negative); // R
    assert_eq!(orientation_2d(&p, &q, &P::new(2.0, 0.0)), Sign::Collinear); // |
    // Endpoints are collinear with their own base line.
    assert_eq!(orientation_2d(&p, &q, &p), Sign::Collinear);
    assert_eq!(orientation_2d(&p, &q, &q), Sign::Collinear);
}

// ---- In-circle: inside / on / outside -------------------------------

#[test]
fn in_circle_matrix() {
    // Unit circle CCW.
    let a = P::new(1.0, 0.0);
    let b = P::new(0.0, 1.0);
    let c = P::new(-1.0, 0.0);
    assert_eq!(in_circle_2d(&a, &b, &c, &P::new(0.0, 0.0)), Sign::Positive);
    assert_eq!(
        in_circle_2d(&a, &b, &c, &P::new(0.0, -1.0)),
        Sign::Collinear
    );
    assert_eq!(in_circle_2d(&a, &b, &c, &P::new(2.0, 2.0)), Sign::Negative);
}

// ---- Segment intersection: the full case family --------------------

#[test]
fn proper_crossing_interior_point() {
    let a = Seg::new(P::new(0.0, 0.0), P::new(4.0, 4.0));
    let b = Seg::new(P::new(0.0, 4.0), P::new(4.0, 0.0));
    match segment_intersection::<Seg, P>(&a, &b) {
        SegmentIntersection::Single(p) => assert_eq!(xy(&p), (2.0, 2.0)),
        other => panic!("{other:?}"),
    }
}

#[test]
fn asymmetric_crossing_point() {
    // Lines y = x/2 and y = 3 - x cross at (2, 1).
    let a = Seg::new(P::new(0.0, 0.0), P::new(4.0, 2.0));
    let b = Seg::new(P::new(0.0, 3.0), P::new(3.0, 0.0));
    match segment_intersection::<Seg, P>(&a, &b) {
        SegmentIntersection::Single(p) => assert_eq!(xy(&p), (2.0, 1.0)),
        other => panic!("{other:?}"),
    }
}

#[test]
fn t_junction_endpoint_on_interior() {
    let a = Seg::new(P::new(0.0, 0.0), P::new(6.0, 0.0));
    let b = Seg::new(P::new(3.0, 0.0), P::new(3.0, 5.0));
    match segment_intersection::<Seg, P>(&a, &b) {
        SegmentIntersection::Single(p) => assert_eq!(xy(&p), (3.0, 0.0)),
        other => panic!("{other:?}"),
    }
}

#[test]
fn shared_endpoint_only() {
    let a = Seg::new(P::new(0.0, 0.0), P::new(2.0, 2.0));
    let b = Seg::new(P::new(2.0, 2.0), P::new(4.0, 0.0));
    match segment_intersection::<Seg, P>(&a, &b) {
        SegmentIntersection::Single(p) => assert_eq!(xy(&p), (2.0, 2.0)),
        other => panic!("{other:?}"),
    }
}

#[test]
fn collinear_partial_overlap() {
    let a = Seg::new(P::new(0.0, 0.0), P::new(5.0, 5.0));
    let b = Seg::new(P::new(2.0, 2.0), P::new(8.0, 8.0));
    match segment_intersection::<Seg, P>(&a, &b) {
        SegmentIntersection::Collinear { from, to } => {
            let (mut lo, mut hi) = (xy(&from), xy(&to));
            if lo.0 > hi.0 {
                core::mem::swap(&mut lo, &mut hi);
            }
            assert_eq!(lo, (2.0, 2.0));
            assert_eq!(hi, (5.0, 5.0));
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn collinear_one_inside_other() {
    // b fully contained in a → overlap is all of b.
    let a = Seg::new(P::new(0.0, 0.0), P::new(10.0, 0.0));
    let b = Seg::new(P::new(3.0, 0.0), P::new(7.0, 0.0));
    match segment_intersection::<Seg, P>(&a, &b) {
        SegmentIntersection::Collinear { from, to } => {
            let (mut lo, mut hi) = (xy(&from), xy(&to));
            if lo.0 > hi.0 {
                core::mem::swap(&mut lo, &mut hi);
            }
            assert_eq!(lo, (3.0, 0.0));
            assert_eq!(hi, (7.0, 0.0));
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn collinear_meeting_at_a_point() {
    let a = Seg::new(P::new(0.0, 0.0), P::new(3.0, 0.0));
    let b = Seg::new(P::new(3.0, 0.0), P::new(6.0, 0.0));
    match segment_intersection::<Seg, P>(&a, &b) {
        SegmentIntersection::Single(p) => assert_eq!(xy(&p), (3.0, 0.0)),
        other => panic!("{other:?}"),
    }
}

#[test]
fn collinear_gap_is_disjoint() {
    let a = Seg::new(P::new(0.0, 0.0), P::new(2.0, 0.0));
    let b = Seg::new(P::new(4.0, 0.0), P::new(6.0, 0.0));
    assert_eq!(
        segment_intersection::<Seg, P>(&a, &b),
        SegmentIntersection::Disjoint
    );
}

#[test]
fn parallel_offset_is_disjoint() {
    let a = Seg::new(P::new(0.0, 0.0), P::new(5.0, 0.0));
    let b = Seg::new(P::new(0.0, 2.0), P::new(5.0, 2.0));
    assert_eq!(
        segment_intersection::<Seg, P>(&a, &b),
        SegmentIntersection::Disjoint
    );
}

#[test]
fn lines_cross_outside_extents_is_disjoint() {
    let a = Seg::new(P::new(0.0, 0.0), P::new(1.0, 1.0));
    let b = Seg::new(P::new(5.0, 0.0), P::new(6.0, -1.0));
    assert_eq!(
        segment_intersection::<Seg, P>(&a, &b),
        SegmentIntersection::Disjoint
    );
}
