//! OVL2.T4 — turn classification.
//!
//! Assigns each turn its [`Method`] and sets the two [`OperationType`]s
//! that steer traversal. Mirrors the classification in
//! `boost/geometry/algorithms/detail/overlay/get_turn_info.hpp` and the
//! `turn_info` method/operation assignment.
//!
//! The v1 classifier covers the areal-areal cases the boolean overlay
//! needs: a proper crossing, an endpoint touch, and a collinear
//! overlap. The richer linear/areal (`get_turn_info_la.hpp`) and
//! self-turn cases are deferred with the rest of the linear overlay.
//!
//! # Operation assignment
//!
//! At a proper crossing the two inputs swap interior/exterior sides, so
//! one operation is [`OperationType::Union`] and the other
//! [`OperationType::Intersection`] — which is which is decided at
//! traversal time from the requested overlay op, so the classifier sets
//! the *pair* consistently and leaves the union/intersection **choice**
//! to [`operations_for_crossing`]. This matches Boost computing the
//! operation pair up front and letting `traverse` pick the arm.

use super::info::{Method, OperationType, Turn};
use crate::predicate::segment_intersection::SegmentIntersection;
use geometry_trait::Point;

/// The [`Method`] implied by a raw segment-intersection outcome.
///
/// * [`SegmentIntersection::Single`] → [`Method::Crosses`] (a proper
///   crossing) — endpoint-touch refinement is applied separately by
///   [`refine_touch`] once the segment endpoints are known.
/// * [`SegmentIntersection::Collinear`] → [`Method::Collinear`].
/// * [`SegmentIntersection::Disjoint`] / [`SegmentIntersection::OutOfRange`]
///   → [`Method::Disjoint`] (no turn should be emitted).
///
/// Mirrors the top-level case split in Boost's `get_turn_info`
/// (`get_turn_info.hpp`).
#[must_use]
pub fn method_of<P>(outcome: &SegmentIntersection<P>) -> Method {
    match outcome {
        SegmentIntersection::Single(_) => Method::Crosses,
        SegmentIntersection::Collinear { .. } => Method::Collinear,
        SegmentIntersection::Disjoint | SegmentIntersection::OutOfRange => Method::Disjoint,
    }
}

/// Refine a [`Method::Crosses`] to [`Method::Touch`] when the
/// intersection point coincides with an endpoint of either segment —
/// the lines meet but do not transversally cross.
///
/// `a0`,`a1` are the endpoints of the first segment, `b0`,`b1` the
/// second; `point` is the intersection. Mirrors the endpoint check in
/// `get_turn_info_for_endpoint.hpp`.
#[must_use]
pub fn refine_touch<P>(point: &P, a0: &P, a1: &P, b0: &P, b1: &P) -> Method
where
    P: Point,
    P::Scalar: PartialEq,
{
    let at_endpoint = eq(point, a0) || eq(point, a1) || eq(point, b0) || eq(point, b1);
    if at_endpoint {
        Method::Touch
    } else {
        Method::Crosses
    }
}

/// The operation pair for a proper crossing: the two inputs take
/// opposite roles. Returns `[union_side, intersection_side]` so the
/// caller stores one per source. Mirrors the operation assignment for a
/// crossing turn in `get_turn_info.hpp`.
#[must_use]
pub const fn operations_for_crossing() -> [OperationType; 2] {
    [OperationType::Union, OperationType::Intersection]
}

/// Set a turn's method and operations from a segment-intersection
/// outcome and the four segment endpoints.
///
/// This is the entry the turn collector calls per emitted turn.
pub fn set_from_outcome<P>(
    turn: &mut Turn<P>,
    outcome: &SegmentIntersection<P>,
    a0: &P,
    a1: &P,
    b0: &P,
    b1: &P,
) where
    P: Point,
    P::Scalar: PartialEq,
{
    let base = method_of(outcome);
    turn.method = match base {
        Method::Crosses => refine_touch(&turn.point, a0, a1, b0, b1),
        other => other,
    };
    match turn.method {
        Method::Crosses => {
            let ops = operations_for_crossing();
            turn.operations[0].operation = ops[0];
            turn.operations[1].operation = ops[1];
            turn.touch_only = false;
        }
        Method::Touch => {
            // A touch that does not cross: both sides continue on their
            // own ring, blocked from swapping.
            turn.operations[0].operation = OperationType::Union;
            turn.operations[1].operation = OperationType::Union;
            turn.touch_only = true;
        }
        Method::Collinear => {
            turn.operations[0].operation = OperationType::Continue;
            turn.operations[1].operation = OperationType::Continue;
            turn.touch_only = false;
        }
        _ => {}
    }
}

fn eq<P: Point>(a: &P, b: &P) -> bool
where
    P::Scalar: PartialEq,
{
    a.get::<0>() == b.get::<0>() && a.get::<1>() == b.get::<1>()
}

#[cfg(test)]
mod tests {
    //! OVL2.T4 done-when: method + operation assignment for each case.
    //! Mirrors the classification checks in
    //! `test/algorithms/overlay/get_turn_info.cpp`.

    use super::{method_of, operations_for_crossing, refine_touch, set_from_outcome};
    use crate::predicate::segment_intersection::SegmentIntersection;
    use crate::turn::info::{Method, Operation, OperationType, RingKind, SegmentId, Turn};
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type P = Point2D<f64, Cartesian>;

    fn sid(src: usize) -> SegmentId {
        SegmentId {
            source_index: src,
            ring: RingKind::Exterior,
            segment_index: 0,
        }
    }

    fn turn_at(x: f64, y: f64) -> Turn<P> {
        Turn {
            point: P::new(x, y),
            method: Method::None,
            operations: [Operation::new(sid(0)), Operation::new(sid(1))],
            touch_only: false,
        }
    }

    #[test]
    fn method_from_outcome() {
        assert_eq!(
            method_of(&SegmentIntersection::Single(P::new(0.0, 0.0))),
            Method::Crosses
        );
        assert_eq!(
            method_of(&SegmentIntersection::Collinear {
                from: P::new(0.0, 0.0),
                to: P::new(1.0, 0.0)
            }),
            Method::Collinear
        );
        assert_eq!(
            method_of::<P>(&SegmentIntersection::Disjoint),
            Method::Disjoint
        );
    }

    #[test]
    fn crossing_at_interior_is_cross() {
        // (1,1) is interior to both diagonals.
        let m = refine_touch(
            &P::new(1.0, 1.0),
            &P::new(0.0, 0.0),
            &P::new(2.0, 2.0),
            &P::new(0.0, 2.0),
            &P::new(2.0, 0.0),
        );
        assert_eq!(m, Method::Crosses);
    }

    #[test]
    fn crossing_at_endpoint_is_touch() {
        // (2,0) is an endpoint of the second segment.
        let m = refine_touch(
            &P::new(2.0, 0.0),
            &P::new(0.0, 0.0),
            &P::new(4.0, 0.0),
            &P::new(2.0, 0.0),
            &P::new(2.0, 3.0),
        );
        assert_eq!(m, Method::Touch);
    }

    #[test]
    fn crossing_sets_opposite_operations() {
        let mut t = turn_at(1.0, 1.0);
        set_from_outcome(
            &mut t,
            &SegmentIntersection::Single(P::new(1.0, 1.0)),
            &P::new(0.0, 0.0),
            &P::new(2.0, 2.0),
            &P::new(0.0, 2.0),
            &P::new(2.0, 0.0),
        );
        assert_eq!(t.method, Method::Crosses);
        let ops = operations_for_crossing();
        assert_eq!(t.operations[0].operation, ops[0]);
        assert_eq!(t.operations[1].operation, ops[1]);
        assert_eq!(t.operations[0].operation, OperationType::Union);
        assert_eq!(t.operations[1].operation, OperationType::Intersection);
    }

    #[test]
    fn collinear_sets_continue() {
        let mut t = turn_at(2.0, 0.0);
        set_from_outcome(
            &mut t,
            &SegmentIntersection::Collinear {
                from: P::new(2.0, 0.0),
                to: P::new(4.0, 0.0),
            },
            &P::new(0.0, 0.0),
            &P::new(4.0, 0.0),
            &P::new(2.0, 0.0),
            &P::new(6.0, 0.0),
        );
        assert_eq!(t.method, Method::Collinear);
        assert_eq!(t.operations[0].operation, OperationType::Continue);
    }
}
