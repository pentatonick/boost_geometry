# `geometry-overlay`

**Layer 6 — overlay engine.** Depends on `geometry-algorithm`, `geometry-strategy`, `geometry-model`. `#![no_std]` + `alloc`.

Mirrors `boost/geometry/algorithms/detail/overlay/`. The largest single
subsystem in the port. **Full deep-dive: [03-overlay-engine.md](../03-overlay-engine.md).**

## Purpose

The segment-intersection kernel and everything built on top of it: the turn
graph, traversal, ring assembly, and the four boolean-overlay free
functions (`intersection`, `union`, `difference`, `sym_difference`), plus
`buffer`, `relate`/`crosses`/`overlaps`/`touches`, `is_valid`, and
`point_on_surface`.

## Module map

| Module | Phase | Role |
|---|---|---|
| `predicate` (+ `orientation`, `in_circle`, `segment_intersection`, `range_guard`) | OVL1 | Robust primitive layer — side/in-circle/segment-intersection predicates, the `SAFE_ABS_MAX` robustness gate |
| `turn` (+ `info`, `get_turns`, `classify`) | OVL2 | The turn graph — where two boundaries meet, and what to do there |
| `traverse` (+ `enrich`, `state`) | OVL3 | Weiler–Atherton-style ring traversal — walk the turn graph, assemble output rings |
| `assemble` | OVL4 | Nest traversed rings into `Polygon`/`MultiPolygon` by containment |
| `operation` | OVL5 | `intersection`, `r#union` (`union_poly` compatibility name), `difference`, `sym_difference`, `OverlayError` |
| `relate` | OVL6 | `relation` matrix, `relate` mask, `De9im`, `crosses`/`overlaps`/`touches`, `Dimension` |
| `validity` | OVL6 | generic `is_valid`, `is_valid_ring`/`is_valid_polygon`, `ValidityFailure` |
| `surface_point` | — | `point_on_surface` — a representative interior point, used by `assemble` and `relate` |
| `buffer` | OVL7 | generic `buffer`, `buffer_point`, `buffer_convex_polygon`, `JoinStrategy`, `PointStrategy` |
| `merge` | — | `merge_elements`, `merge_polygons`, `merge_multipolygon` |

## Robustness policy

Exact `f64` arithmetic, no coordinate rescaling. `range_guard::SAFE_ABS_MAX`
bounds the safe magnitude; out-of-range coordinates are **refused**, not
silently miscomputed.

## Scope (v1)

Polygon × polygon, exterior ring only (no holes on the input side — holes
in the *output* are fine, since assembly nests them). Clean transversal
crossings only; clustered turns, self-intersections, and long collinear
overlaps return `Unsupported`. See the [deep-dive](../03-overlay-engine.md#the-recurring-design-principle-refuse-dont-guess)
for the "refuse, don't guess" design principle that runs through this
entire crate.

## Why this is a separate crate, not part of `geometry-algorithm`

`geometry-overlay` depends on `geometry-algorithm` (for `within`/`ring_area`).
The original plan placed the four overlay functions inside
`geometry-algorithm`, which would have created a cycle. They live here
instead and `geometry-algorithm` never depends back.

## Who depends on this

`geometry-rtree` sits above it in the facade layering; re-exported wholesale
by the `boost_geometry` facade as `boost_geometry::overlay`.
