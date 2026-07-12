//! OVL1 — the robust predicate layer.
//!
//! Every overlay operation eventually calls this module. It is the
//! boundary between "raw coordinates" and "topological decisions":
//! given points, it answers *which side*, *inside which circle*, and
//! *do these two segments meet and where*.
//!
//! Mirrors the Cartesian side / intersection strategies in
//! `boost/geometry/strategy/cartesian/` (`side_by_triangle.hpp`,
//! `intersection.hpp`) and the robustness policies in
//! `boost/geometry/policies/robustness/`.
//!
//! Module layout (the OVL1 tasks, in dependency order):
//!
//! * [`mod@orientation`] — OVL1.T1: the signed-area side predicate.
//! * [`mod@in_circle`] — OVL1.T2: the in-circle predicate used by
//!   `is_valid` and the turn graph.
//! * [`mod@segment_intersection`] — OVL1.T3: segment-segment
//!   intersection returning the meeting point(s).
//! * [`mod@range_guard`] — OVL1.T4: the coordinate-range robustness
//!   gate that enforces the "exact arithmetic, no rescale" policy.

pub mod in_circle;
pub mod orientation;
pub mod range_guard;
pub mod segment_intersection;

pub use in_circle::in_circle_2d;
pub use orientation::{Sign, orientation_2d};
pub use range_guard::{RangeError, SAFE_ABS_MAX, coordinate_in_range, polygon_in_range};
pub use segment_intersection::{SegmentIntersection, segment_intersection};
