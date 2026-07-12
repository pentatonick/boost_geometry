# `geometry-rtree`

**Layer 7 — spatial index.** Depends on the kernel (model/trait/coords). `#![no_std]` + `alloc`.

Mirrors `boost/geometry/index/rtree.hpp` and `boost/geometry/index/detail/`.

## Purpose

An R-tree spatial index over any `Indexable` value (a value with an
axis-aligned bounding box). Answers spatial queries — intersects / within /
contains — and k-nearest-neighbour search, pruning the tree with each
node's bounding box.

**v1 scope:** Cartesian, 2D, `f64`.

## Files

| File | Contents |
|---|---|
| `src/bounds.rs` | `Bounds` — axis-aligned box arithmetic: area, enlargement, union, distance |
| `src/indexable.rs` | `Indexable` trait |
| `src/node.rs` | `Node` — the leaf/branch enum |
| `src/split.rs` | `SplitParameters` — `Linear` and `Quadratic` (default) split strategies |
| `src/predicate.rs` | `Predicate` — query predicates |
| `src/rtree.rs` | `Rtree<T, Params>` — insert / query / nearest / bulk load |

## Public surface

* **`Rtree<T, Params>`** — `Params` is a type parameter selecting the split
  strategy (`Quadratic` default, `Linear` opt-in) — same shape as the
  distance-strategy pattern elsewhere in the workspace.
* **`Indexable`** — implement this to make a type storable in the tree.
* **`Predicate`** — the query surface (intersects / within / contains).
* Bulk loading via `FromIterator` uses **Sort-Tile-Recursive** packing for a
  balanced tree in one pass, rather than repeated single inserts.

## Who depends on this

Re-exported by the `boost_geometry` facade as `boost_geometry::rtree`.
