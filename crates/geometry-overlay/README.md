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
* turn graph → traversal → output assembly → the four overlay free
  functions land in later phases, each behind the same strict
  ordering.

## Robustness

v1 uses **exact input arithmetic with no rescale** — the predicates
compute directly on the `f64` inputs and the
[`predicate::range_guard`] refuses inputs outside the safe
arithmetic range rather than silently returning a wrong sign,
leaving a slot for a future rescale policy.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
