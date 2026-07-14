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
| `operation` (+ `boolean`, `areal`) | OVL5 | split-edge arrangement; `intersection`, `r#union` (`union_poly` compatibility name), `difference`, `sym_difference`, `OverlayError` |
| `relate` | OVL6 | Cartesian static/multi/runtime/collection `relation` matrix, `relate` mask, `De9im`, `crosses`/`overlaps`/`touches`, `Dimension` |
| `validity` | OVL6 | ring/polygon/multi-polygon `is_valid`, inter-ring/member topology, `ValidityFailure` |
| `surface_point` | — | `point_on_surface` — a representative interior point, used by `assemble` and `relate` |
| `buffer` | OVL7 | Cartesian/spherical/geographic single/multi dispatch, `buffer`/`buffer_with`/`buffer_with_strategy`, signed areal offsets, linear ends, strategy bundles |
| `merge` | — | `merge_elements`, `merge_polygons`, `merge_multipolygon` |

## Robustness policy

Adaptive expansion predicates over the input `f64` coordinates, with no
coordinate rescaling. `range_guard::SAFE_ABS_MAX` bounds the safe magnitude;
out-of-range coordinates are **refused**, not silently miscomputed.

## Scope

Boolean operations cover Cartesian polygon × polygon inputs with holes,
containment, colocated vertices, shared edges, and collinear overlaps. Relate
covers Cartesian static singles, homogeneous multis, runtime geometries, and
heterogeneous geometry collections. Buffer covers point, segment, linestring,
ring, polygon, box, and homogeneous multi kinds in all three coordinate
families. Angular buffer dispatch uses the recorded local-tangent
approximation. Invalid self-intersecting overlay inputs and linear/pointlike
set-operation output remain outside this scope.

## Why this is a separate crate, not part of `geometry-algorithm`

`geometry-overlay` depends on `geometry-algorithm` (for `within`/`ring_area`).
The original plan placed the four overlay functions inside
`geometry-algorithm`, which would have created a cycle. They live here
instead and `geometry-algorithm` never depends back.

## Who depends on this

`geometry-rtree` sits above it in the facade layering; re-exported wholesale
by the `boost_geometry` facade as `boost_geometry::overlay`.
