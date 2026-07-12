//! An R-tree spatial index over the geometry kernel.
//!
//! Mirrors `boost/geometry/index/rtree.hpp` and the support headers
//! under `boost/geometry/index/detail/`. Stores any [`Indexable`] value
//! (a value with an axis-aligned bounding box) and answers spatial
//! queries — intersects / within / contains — and k-nearest-neighbour
//! search, pruning the tree with each node's bounding box.
//!
//! The split strategy is a type parameter of [`Rtree`]:
//! [`Quadratic`] (the default) or [`Linear`]. Bulk loading via
//! [`FromIterator`] uses Sort-Tile-Recursive packing for a balanced
//! tree in one pass.
//!
//! Cartesian, 2D, `f64` for v1 — see `specs/rtree-split-decision.md`.
//!
//! Module layout:
//!
//! * [`bounds`] — the axis-aligned box arithmetic (area, enlargement,
//!   union, distance) the tree keys on.
//! * [`indexable`] — the [`Indexable`] trait.
//! * [`node`] — the leaf / branch [`Node`](node::Node) enum.
//! * [`split`] — the [`SplitParameters`] strategies.
//! * [`predicate`] — the query [`Predicate`]s.
//! * [`rtree`](mod@rtree) — the [`Rtree`] and its insert / query /
//!   nearest / bulk load.
//!
//! [`Indexable`]: indexable::Indexable

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

pub mod bounds;
pub mod indexable;
pub mod node;
pub mod predicate;
pub mod rtree;
pub mod split;

pub use bounds::Bounds;
pub use indexable::Indexable;
pub use predicate::Predicate;
pub use rtree::Rtree;
pub use split::{Linear, Quadratic, SplitParameters};
