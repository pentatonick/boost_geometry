//! OVL5 — Cartesian polygon Boolean operations.
//!
//! Mirrors the public drivers and areal machinery behind
//! `boost/geometry/algorithms/{intersection,union,difference,sym_difference}.hpp`.
//! The `boolean` part owns the public entry contract, `areal` owns the
//! split-edge arrangement kernel and `section_partition` the order `get_turns`
//! visits section pairs in; this root exposes only the aggregate surface.

mod areal;
mod boolean;
mod section_partition;

pub use boolean::{
    OverlayError, difference, difference_multi, intersection, intersection_multi, sym_difference,
    sym_difference_multi, r#union, union_multi, union_poly,
};
