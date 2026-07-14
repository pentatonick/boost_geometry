# geometry-overlay

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Boolean-overlay engine — the segment-intersection kernel and the
machinery built on top of it.

Mirrors `boost/geometry/algorithms/detail/overlay/`. Overlay is the
engine behind `intersection`, `union`, `difference`,
`sym_difference`, and (indirectly) `buffer`, `is_valid`, `relate`,
`crosses`, `overlaps`, `touches`, `point_on_surface`, and
`merge_elements`. Boost concentrates all of it under one `detail`
directory; the port gives it its own crate because the algorithmic
surface is too dense to share a crate with anything else.

The build order is strict:

* [`predicate`] — OVL1: the robust predicate layer every overlay
  operation eventually calls (orientation, in-circle,
  segment-segment intersection, coordinate-range gate).
* [`operation`] — OVL5: a split-edge arrangement handles crossings,
  colocations, shared edges, traversal, and output assembly for the four
  Cartesian polygon Boolean operations.
* [`mod@relate`] / [`validity`] / [`mod@buffer`] — the public topology consumers
  layered on those predicates and operations.

## Robustness

The Cartesian kernel uses **adaptive expansion predicates with no
rescale**. [`predicate::range_guard`] refuses inputs outside the supported
arithmetic range rather than silently returning a wrong sign.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
