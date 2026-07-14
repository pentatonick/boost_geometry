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
//! The build order is strict:
//!
//! * [`predicate`] — OVL1: the robust predicate layer every overlay
//!   operation eventually calls (orientation, in-circle,
//!   segment-segment intersection, coordinate-range gate).
//! * [`operation`] — OVL5: a split-edge arrangement handles crossings,
//!   colocations, shared edges, traversal, and output assembly for the four
//!   Cartesian polygon Boolean operations.
//! * [`mod@relate`] / [`validity`] / [`mod@buffer`] — the public topology consumers
//!   layered on those predicates and operations.
//!
//! # Robustness
//!
//! The Cartesian kernel uses **adaptive expansion predicates with no
//! rescale**. [`predicate::range_guard`] refuses inputs outside the supported
//! arithmetic range rather than silently returning a wrong sign.

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

pub use buffer::{
    JoinStrategy, PointStrategy, buffer, buffer_convex_polygon, buffer_point, buffer_with,
};
pub use merge::{merge_elements, merge_multipolygon, merge_polygons};
pub use operation::{OverlayError, difference, intersection, sym_difference, r#union, union_poly};
pub use relate::{
    De9im, Dimension, RelateError, crosses, overlaps, relate as relate_matrix, relate as relation,
    relate_mask as relate, touches,
};
pub use surface_point::point_on_surface;
pub use validity::{ValidityFailure, is_valid, is_valid_polygon, is_valid_ring};
