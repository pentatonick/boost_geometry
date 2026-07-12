# geometry-rtree

An R-tree spatial index over the geometry kernel, mirroring
`boost/geometry/index/rtree.hpp`.

Stores any `Indexable` value (a value with an axis-aligned bounding
box) and answers spatial queries — intersects / within / contains —
and k-nearest-neighbour search, by pruning the tree with each node's
bounding box.

The split strategy is a type parameter (`Rtree<T, Params>`):

* **`Quadratic`** (default here) — Boost's textbook split, good
  all-round trees.
* **`Linear`** — cheaper inserts, looser trees (opt-in).

Bulk loading via Sort-Tile-Recursive (`FromIterator`) builds a
balanced tree in one pass.

Cartesian, 2D, `f64` for v1 (see `specs/rtree-split-decision.md`).
