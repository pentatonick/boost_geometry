# `geometry-tag`

**Layer 0 — foundation.** No domain dependencies. `#![no_std]`.

Mirrors `boost/geometry/core/{tags,tag,tag_cast}.hpp`.

## Purpose

Eleven zero-sized tag types identify each OGC geometry kind, plus a set of
marker traits reproducing the C++ struct-inheritance hierarchy at the
Rust trait-bound level. See [the tag-dispatch pattern](../02-tag-dispatch-pattern.md)
for the full diagram and how this crate is used downstream.

## Files

| File | Contents |
|---|---|
| `src/tag.rs` | The 11 tag structs |
| `src/hierarchy.rs` | The 8 marker traits + their impls |
| `src/same_as.rs` | `SameAs` — compile-time `std::is_same` equivalent |
| `src/lib.rs` | Re-exports only (manifest) |

## Public surface

**Tags** (`src/tag.rs`) — each `#[derive(Debug, Default, Clone, Copy)] pub struct`:

`PointTag`, `SegmentTag`, `LinestringTag`, `RingTag`, `PolygonTag`, `BoxTag`,
`MultiPointTag`, `MultiLinestringTag`, `MultiPolygonTag`,
`GeometryCollectionTag`, `PolyhedralSurfaceTag`, `DynamicGeometryTag`
(runtime-tagged/variant geometry, e.g. `DynGeometry`).

**Marker traits** (`src/hierarchy.rs`):

* `Single`, `Multi` — cardinality
* `Pointlike`, `Linear`, `Areal`, `Volumetric` — base categories
* `Polylinear: Linear` — linestrings/multi-linestrings
* `Polygonal: Areal` — rings/polygons/multi-polygons

**`SameAs`** (`src/same_as.rs`) — used throughout `geometry-strategy` to
fence a strategy impl to one coordinate-system family (`<P::Cs as
CoordinateSystem>::Family: SameAs<CartesianFamily>`), carrying a
`#[diagnostic::on_unimplemented]` message that redirects compile errors to
the correct fix.

## Who depends on this

Everything. `geometry-trait` re-exports it wholesale so downstream crates
importing `geometry_trait::*` get the tags for free without a second
dependency line.
