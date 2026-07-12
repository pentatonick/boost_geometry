//! OVL2 — the turn graph.
//!
//! A *turn* is an intersection point between the two input geometries,
//! carried together with the metadata the traversal stage needs to walk
//! the turns in the right order. Mirrors
//! `boost/geometry/algorithms/detail/overlay/turn_info.hpp` and
//! `get_turns.hpp`.
//!
//! Module layout (the OVL2 tasks, in dependency order):
//!
//! * [`info`] — OVL2.T1: the turn data model ([`info::Turn`],
//!   [`info::Operation`], and the [`info::Method`] /
//!   [`info::OperationType`] enums).
//! * [`get_turns`] — OVL2.T2 / T3: collect the turns between two rings,
//!   and between two polygons.
//! * [`classify`] — OVL2.T4: classify each turn's method and set its
//!   two operations.

pub mod classify;
pub mod get_turns;
pub mod info;

pub use get_turns::{get_turns_polygon_polygon, get_turns_ring_ring};
pub use info::{Method, Operation, OperationType, RingKind, SegmentId, Turn};
