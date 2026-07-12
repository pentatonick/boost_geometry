# `geometry-adapt-geo-types`

**Ecosystem adapter, peer of `geometry-adapt`.** Depends on `geometry-model`, `geometry-trait`.

Adapts [`geo-types`](https://docs.rs/geo-types) — the de-facto Rust
geometry ecosystem crate — to the `geometry-trait` concept surface.

## Purpose

Same orphan-rule problem as `geometry-adapt` (Rust forbids implementing a
foreign trait for a foreign type), solved the same way: wrap each
`geo-types` value in a local newtype and hang the geometry concepts off the
wrapper. Rust's answer to Boost's `BOOST_GEOMETRY_REGISTER_*` macros in
`geometries/adapted/`.

## Wrapper inventory

| Wrapper | Wraps | Concept |
|---|---|---|
| `GeoCoord` | `geo_types::Coord` | `Point` + `PointMut` |
| `GeoPoint` | `geo_types::Point` | `Point` + `PointMut` |
| `GeoLineString` | `geo_types::LineString` | `Linestring` |
| `GeoRing` | `geo_types::LineString` | `Ring` (caller asserts closure/orientation — `geo-types` has no separate ring type) |
| `GeoPolygon` | `geo_types::Polygon` | `Polygon` |
| `GeoMultiPoint` / `GeoMultiLineString` / `GeoMultiPolygon` | matching `geo_types::Multi*` | `Multi*` |
| `GeoLine` | `geo_types::Line` | `Segment` |
| `GeoRect` | `geo_types::Rect` | `Box` |
| `GeoCollection` | `geo_types::GeometryCollection` | `GeometryCollection` |

All pin `Cs = Cartesian`, `DIM = 2`.

## Storage note

Container concepts yield points **by reference**
(`points() -> impl Iterator<Item = &Self::Point>`), and this crate is
`#![forbid(unsafe_code)]`, so the container wrappers store their vertices as
the wrapped point type (`GeoCoord`/`GeoPoint`) rather than re-casting raw
`geo-types` storage. Construction and `into_inner` copy ordinates
element-by-element — lossless, since both representations are `Copy`.

## `DynGeometry` interop

`src/dyn_conversion.rs` provides `From<geo_types::Geometry<T>>` ↔
`DynGeometry<T, Cartesian>`, so a whole geometry tree can move between the
two ecosystems in one call.
