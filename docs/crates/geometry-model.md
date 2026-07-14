# `geometry-model`

**Layer 2 — concrete types.** Depends on `geometry-trait` (transitively tag/coords/cs). `#![no_std]` + `alloc`.

Mirrors
`boost/geometry/geometries/{point,segment,pointing_segment,box,linestring,ring,polygon,multi_*,infinite_line,helper_geometry,polyhedral_surface}.hpp`
— Boost's `model::` namespace of stock geometries.

## Purpose

Concrete, ready-to-use geometry types for callers who don't want to adapt
their own. Every type here implements the matching `geometry-trait` concept.

## Files

| File | Contents |
|---|---|
| `src/point.rs` | `Point<T, D, Cs>`, `Point2D`, `Point3D` |
| `src/segment.rs` | `Segment` |
| `src/pointing_segment.rs` | `PointingSegment` — segment borrowing two endpoints |
| `src/boxg.rs` | `Box` |
| `src/infinite_line.rs` | `InfiniteLine` and general-form line arithmetic |
| `src/geometry_rebind.rs` | `Rebound` type-level stock-model selection |
| `src/linestring.rs` | `Linestring` |
| `src/ring.rs` | `Ring` |
| `src/polygon.rs` | `Polygon` (exterior + interior rings) |
| `src/multi.rs` | `MultiPoint`, `MultiLinestring`, `MultiPolygon` |
| `src/polyhedral_surface.rs` | `PolyhedralSurface` face collection |
| `src/dyn_geometry.rs` | `DynGeometry`, `DynKind` — runtime-tagged enum, one variant per OGC kind |
| `src/dyn_collection.rs` | `DynGeometryCollection` |
| `src/macros.rs` | `point!`, `linestring!`, `polygon!` — `#[macro_export]`ed literal constructors |

## Public surface

`Box`, `DynGeometryCollection`, `DynGeometry`, `DynKind`, `Linestring`,
`InfiniteLine`, `MultiLinestring`, `MultiPoint`, `MultiPolygon`,
`Point`, `Point2D`, `Point3D`, `PointingSegment`, `Polygon`,
`PolyhedralSurface`, `RebindGeometry`, `Rebound`, `Ring`, `Segment`.

### `DynGeometry` — dynamic-kind geometry

An enum with one variant per OGC kind (mirrors `core/tags.hpp`'s
`dynamic_geometry_tag`). This is where the WKT/WKB/GeoJSON parsers land
their output, and where a heterogeneous collection (e.g. an rtree of mixed
kinds) has somewhere to go. Every algorithm that needs to work on
heterogeneous input grows a thin `_dyn` wrapper elsewhere (see
`geometry-algorithm`'s `dyn_area`/`dyn_distance`/etc.) that match-and-forwards
to the static per-kind impl — dispatch stays monomorphic inside each arm.

### Literal macros

`polygon![[outer], [hole1], [hole2], …]`, `linestring![...]`, `point!(x, y)`
— readable test/example construction. Exported at the `boost_geometry` facade's
crate root (not under `boost_geometry::model`) because `#[macro_export]` always
places macros at the crate root.

## Who depends on this

`geometry-adapt`, `geometry-strategy`, `geometry-algorithm`, `geometry-overlay`,
`geometry-rtree`, all four I/O crates, both ecosystem adapters, `geometry-proj`.
This is the most widely depended-on crate above the concept layer.
