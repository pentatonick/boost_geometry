# `geometry-rtree`

**Layer 7 — spatial index.** Depends on the kernel (model/trait/coords). `#![no_std]` + `alloc`.

Mirrors `boost/geometry/index/rtree.hpp` and `boost/geometry/index/detail/`.

## Purpose

An R-tree spatial index over any `Indexable` value (a value with an
axis-aligned bounding box). Answers the full Boost box-predicate set,
composable logical/satisfies queries, and k-nearest-neighbour search while
pruning by node bounds. It also supports condensing removal, count, clear,
iteration, bulk loading, and optional serde persistence.

**v1 scope:** Cartesian, 2D, `f64`.

## Files

| File | Contents |
|---|---|
| `src/bounds.rs` | `Bounds` — axis-aligned box arithmetic: area, enlargement, union, distance |
| `src/indexable.rs` | `Indexable` trait |
| `src/node.rs` | `Node` — the leaf/branch enum |
| `src/split.rs` | `SplitParameters` — asymmetric/symmetric R\*, quadratic, and linear split strategies |
| `src/predicate.rs` | Built-in, logical, and `satisfies` query predicates |
| `src/query_iter.rs` | Lazy built-in and extensible query walks |
| `src/values.rs` | Depth-first value iteration |
| `src/serialization.rs` | Feature-gated serde value persistence |
| `src/rtree.rs` | `Rtree<T, Params>` — mutation / query / nearest / bulk load |

## Public surface

* **`Rtree<T, Params>`** — `Params` selects the split strategy. The measured
  default is `AsymmetricRStarSplit<6, 2, 12, 4, 4, 4>`; symmetric R\*,
  quadratic, and linear policies remain public.
* **`Indexable`** — implement this to make a type storable in the tree.
* **`Predicate` / `QueryPredicate`** — built-in box relations through
  `query`, and logical/user-defined predicates through `query_with`.
* **Container operations** — insert/extend, count, condensing single/range
  removal, clear, bounds, clone, and iteration.
* Bulk loading via `FromIterator` uses **Sort-Tile-Recursive** packing for a
  balanced tree in one pass, rather than repeated single inserts.

## Who depends on this

Re-exported by the `boost_geometry` facade as `boost_geometry::rtree`.
