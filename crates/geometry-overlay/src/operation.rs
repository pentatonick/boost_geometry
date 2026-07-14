//! OVL5 — Cartesian polygon Boolean operations.
//!
//! Mirrors the public drivers and areal machinery behind
//! `boost/geometry/algorithms/{intersection,union,difference,sym_difference}.hpp`.
//! The `boolean` part owns the public entry contract and `areal` owns the
//! split-edge arrangement kernel; this root exposes only the aggregate surface.

mod areal;
mod boolean;

pub use boolean::{OverlayError, difference, intersection, sym_difference, r#union, union_poly};
