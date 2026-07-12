# `geometry-trait`

**Layer 1 — concepts.** Depends on `geometry-tag`, `geometry-coords`, `geometry-cs`.

Mirrors `boost/geometry/doc/concept/*.qbk` and
`boost/geometry/core/access.hpp` + the `geometries/concepts/*_concept.hpp`
headers.

## Purpose

One Rust trait per Boost.Geometry *concept*. This is the crate that defines
what it *means* to be a Point, a Ring, a Polygon — independent of any
concrete representation. Any type, owned or foreign (via `geometry-adapt`),
that implements these traits works with every algorithm in the workspace.

## Files

| File | Concept |
|---|---|
| `src/geometry.rs` | `Geometry` — the super-trait every concept extends; carries `Kind` (a tag) and `Point` (associated point type) |
| `src/point.rs` | `Point` / `PointMut` — `get::<D>()` / `set::<D>()` indexed coordinate access, `fold_dims` |
| `src/segment.rs` | `Segment` — 2-point convenience geometry; `segment_start`/`segment_end` |
| `src/boxg.rs` | `Box` — axis-aligned bounding box; `box_min`/`box_max` |
| `src/linestring.rs` | `Linestring` |
| `src/ring.rs` | `Ring` |
| `src/polygon.rs` | `Polygon` |
| `src/multi.rs` | `MultiPoint`, `MultiLinestring`, `MultiPolygon` |
| `src/collection.rs` | `GeometryCollection` |
| `src/polyhedral.rs` | `PolyhedralSurface` |
| `src/indexed_access.rs` | `IndexedAccess` trait + `corner` helper |
| `src/closure.rs` | `Closure` — open vs. closed ring representation |
| `src/point_order.rs` | `PointOrder` — CW/CCW winding |
| `src/check.rs` | `check_*` — concept-check functions (compile-time assertions a type satisfies a concept) |

## Public surface

Re-exports every concept trait plus `geometry_tag`'s tags and marker traits
(`Areal`, `Linear`, `Polygonal`, …), so a downstream `impl Point for MyType`
needs only one `use geometry_trait::*;` line to reach both the concept and
the tag machinery.

The **`PointMut: Point` split** is a deliberate design point
(mirroring
`geometries/concepts/point_concept.hpp`'s `ro_point`/`rw_point`): algorithms
that only *read* coordinates (`distance`, `area`, `length`, `within`,
`intersects`, `equals`) bound on `Point`; only materialising algorithms
(`envelope`, `Default + set::<D>`) require `PointMut`. This is what lets
immutable structs, `&[T; N]` borrows, and serde-deserialised points feed
read-only algorithms.

## Who depends on this

`geometry-model` (implements these traits for the shipped concrete types),
`geometry-adapt`, `geometry-derive`, every ecosystem adapter, `geometry-strategy`,
`geometry-algorithm`, `geometry-overlay`.
