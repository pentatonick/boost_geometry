//! An R-tree spatial index over the geometry kernel.
//!
//! Mirrors `boost/geometry/index/rtree.hpp` and the support headers
//! under `boost/geometry/index/detail/`. Stores any [`Indexable`] value
//! (a value with an axis-aligned bounding box) and answers spatial
//! queries — intersects / within / contains — and k-nearest-neighbour
//! search, pruning the tree with each node's bounding box.
//!
//! The split strategy is a type parameter of [`Rtree`]. The default is
//! [`AsymmetricRStarSplit`] with six-child branches and 12-value
//! leaves for insertion, and four-child branches/four-value leaves for
//! bulk packing; symmetric [`RStarSplit`], [`Quadratic`], and [`Linear`]
//! configurations remain available. Bulk loading via [`FromIterator`] uses
//! Sort-Tile-Recursive packing for a balanced tree in one pass.
//! See [`split`] for parameter semantics, validity constraints, tuning
//! guidance, and the benchmark evidence behind the default.
//!
//! Cartesian, 2D, `f64` for v1.
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
//! * [`query_iter`] — [`QueryIter`], the lazy
//!   spatial-query walk.
//! * [`nearest_iter`] — [`NearestIter`], the
//!   unbounded nearest-first stream.
//! * `search_frontier` / `nearest_bound` (crate-internal) — the nearest
//!   search's stack-first frontier and k-th-best rank buffer.
//!
//! [`Indexable`]: indexable::Indexable

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod nearest_bound;
mod search_frontier;

pub mod bounds;
pub mod indexable;
pub mod nearest_iter;
pub mod node;
pub mod predicate;
pub mod query_iter;
pub mod rtree;
pub mod split;

pub use bounds::Bounds;
pub use indexable::Indexable;
pub use nearest_iter::NearestIter;
pub use predicate::Predicate;
pub use query_iter::QueryIter;
pub use rtree::Rtree;
pub use split::{
    AsymmetricQuadratic, AsymmetricRStarSplit, Linear, Quadratic, RStarSplit, SplitParameters,
};
