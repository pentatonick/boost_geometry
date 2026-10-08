//! A ring as Boost's area walk visits it.
//!
//! Mirrors `detail::area::ring_area` (`algorithms/area.hpp:82-118`), which
//! walks every area strategy — Cartesian, spherical and geographic — over
//! `detail::closed_clockwise_view`
//! (`views/detail/closed_clockwise_view.hpp`): the ring closed and, when
//! counter-clockwise, reversed, so that the strategy sums its edges in the
//! order Boost does.

use alloc::vec::Vec;

use geometry_trait::{Closure, PointOrder, Ring};

/// The ring's points in the order Boost's area walk visits them, or none
/// for a ring too short to enclose an area.
///
/// A ring below its closure's minimum size — four points closed, three
/// open — has no area, and the rest are walked through
/// `closed_clockwise_view` ([`closed_clockwise_points`]).
pub(crate) fn clockwise_points<R: Ring>(ring: &R) -> Vec<&R::Point> {
    let minimum = match ring.closure() {
        Closure::Closed => 4,
        Closure::Open => 3,
    };
    if ring.points().len() < minimum {
        return Vec::new();
    }
    closed_clockwise_points(ring)
}

/// The ring's points as `closed_clockwise_view` presents them: closed and,
/// when counter-clockwise, reversed.
pub(crate) fn closed_clockwise_points<R: Ring>(ring: &R) -> Vec<&R::Point> {
    let mut points: Vec<&R::Point> = ring.points().collect();
    if matches!(ring.closure(), Closure::Open) && !points.is_empty() {
        points.push(points[0]);
    }
    if matches!(ring.point_order(), PointOrder::CounterClockwise) {
        points.reverse();
    }
    points
}
