//! An R-tree spatial index over the geometry kernel.
//!
//! Mirrors `boost/geometry/index/rtree.hpp` and the support headers
//! under `boost/geometry/index/detail/`. Stores any [`Indexable`] value
//! (a value with an axis-aligned bounding box) and answers Boost's box
//! predicates plus logical/satisfies queries and k-nearest-neighbour
//! search. It also provides insertion, condensing removal, count,
//! iteration, clear, bulk packing, and opt-in serde persistence.
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
//! ## Index polygons and query a point
//!
//! Implement [`Indexable`] for an application type by returning its
//! axis-aligned bounds, then build an [`Rtree`] from an iterator. This example
//! uses rectangular polygons, so a bounds intersection with a point is also an
//! exact polygon intersection. For other polygon shapes, treat the result as a
//! candidate set and apply an exact point-in-polygon test afterward.
//!
//! ```
//! use geometry_rtree::{Bounds, Indexable, Predicate, Rtree};
//!
//! #[derive(Debug)]
//! struct Parcel {
//!     name: &'static str,
//!     boundary: [[f64; 2]; 5],
//!     bounds: Bounds,
//! }
//!
//! impl Parcel {
//!     fn rectangle(name: &'static str, min: [f64; 2], max: [f64; 2]) -> Self {
//!         Self {
//!             name,
//!             boundary: [
//!                 min,
//!                 [min[0], max[1]],
//!                 max,
//!                 [max[0], min[1]],
//!                 min,
//!             ],
//!             bounds: Bounds::new(min, max),
//!         }
//!     }
//! }
//!
//! impl Indexable for Parcel {
//!     fn bounds(&self) -> Bounds {
//!         self.bounds
//!     }
//! }
//!
//! let parcels = [
//!     Parcel::rectangle("park", [0.0, 0.0], [4.0, 3.0]),
//!     Parcel::rectangle("school", [5.0, 0.0], [8.0, 2.0]),
//!     Parcel::rectangle("lake", [1.0, 5.0], [3.0, 7.0]),
//! ];
//! let tree: Rtree<Parcel> = parcels.into_iter().collect();
//!
//! let point = Bounds::point([2.0, 1.0]);
//! let hits = tree.query(Predicate::Intersects(point));
//!
//! assert_eq!(hits.len(), 1);
//! assert_eq!(hits[0].name, "park");
//! assert_eq!(hits[0].boundary[0], [0.0, 0.0]);
//! ```
//!
//! Module layout:
//!
//! * [`bounds`] — the axis-aligned box arithmetic (area, enlargement,
//!   union, distance) the tree keys on.
//! * [`indexable`] — the [`Indexable`] trait.
//! * [`node`] — the leaf / branch [`Node`](node::Node) enum.
//! * [`split`] — the [`SplitParameters`] strategies.
//! * [`predicate`] — built-in and composable query predicates.
//! * [`rtree`](mod@rtree) — the [`Rtree`] and its mutation / query /
//!   nearest / bulk-load operations.
//! * [`query_iter`] — [`QueryIter`] and [`QueryWithIter`], the lazy
//!   spatial-query walks.
//! * [`nearest_iter`] — [`NearestIter`], the
//!   unbounded nearest-first stream.
//! * [`values`] — iteration over every stored value.
//! * `serialization` (feature-gated) — serde value-sequence persistence.
//! * `search_frontier` / `nearest_bound` (crate-internal) — the nearest
//!   search's stack-first frontier and k-th-best rank buffer.
//!
//! [`Indexable`]: indexable::Indexable

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod nearest_bound;
mod search_frontier;
#[cfg(feature = "serde")]
mod serialization;

pub mod bounds;
pub mod indexable;
pub mod nearest_iter;
pub mod node;
pub mod predicate;
pub mod query_iter;
pub mod rtree;
pub mod split;
pub mod values;

pub use bounds::Bounds;
pub use indexable::Indexable;
pub use nearest_iter::NearestIter;
// feature-group: Spatial index
pub use predicate::{
    AndPredicate, NotPredicate, Predicate, QueryPredicate, Satisfies, and, not, satisfies,
};
pub use query_iter::{QueryIter, QueryWithIter};
// feature-group: Spatial index
// feature-desc: Bulk-loadable R-tree with nearest-neighbour and predicate queries
// feature-keep: Rtree
pub use rtree::Rtree;
pub use split::{
    AsymmetricQuadratic, AsymmetricRStarSplit, Linear, Quadratic, RStarSplit, SplitParameters,
};
pub use values::Values;
