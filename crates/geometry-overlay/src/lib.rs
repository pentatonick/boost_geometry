//! Boolean-overlay engine — the segment-intersection kernel and the
//! machinery built on top of it.
//!
//! Mirrors `boost/geometry/algorithms/detail/overlay/`. Overlay is the
//! engine behind `intersection`, `union`, `difference`,
//! `sym_difference`, and (indirectly) `buffer`, `is_valid`, `relate`,
//! `crosses`, `overlaps`, `touches`, `point_on_surface`, and
//! `merge_elements`. Boost concentrates all of it under one `detail`
//! directory; the port gives it its own crate because the algorithmic
//! surface is too dense to share a crate with anything else.
//!
//! The build order is strict (see
//! `specs/phase_03-rust-port-implementation-plan-overlay.md`):
//!
//! * [`predicate`] — OVL1: the robust predicate layer every overlay
//!   operation eventually calls (orientation, in-circle,
//!   segment-segment intersection, coordinate-range gate).
//! * turn graph → traversal → output assembly → the four overlay free
//!   functions land in later phases, each behind the same strict
//!   ordering.
//!
//! # Robustness
//!
//! v1 uses **exact input arithmetic with no rescale** — the predicates
//! compute directly on the `f64` inputs and the
//! [`predicate::range_guard`] refuses inputs outside the safe
//! arithmetic range rather than silently returning a wrong sign. See
//! `specs/overlay-robustness-decision.md` for the rationale and the
//! slot left for a future rescale policy.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

pub mod assemble;
pub mod buffer;
pub mod merge;
pub mod operation;
pub mod predicate;
pub mod relate;
pub mod surface_point;
pub mod traverse;
pub mod turn;
pub mod validity;

pub use buffer::{JoinStrategy, PointStrategy, buffer_convex_polygon, buffer_point};
pub use merge::{merge_multipolygon, merge_polygons};
pub use operation::{OverlayError, difference, intersection, sym_difference, union_poly};
pub use relate::{De9im, Dimension, crosses, overlaps, relate as relate_matrix, touches};
pub use surface_point::point_on_surface;
pub use validity::{ValidityFailure, is_valid_polygon, is_valid_ring};
