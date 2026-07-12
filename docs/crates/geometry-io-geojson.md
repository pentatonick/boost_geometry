# `geometry-io-geojson`

**I/O peer, consumes `geometry-model`.** `#![no_std]` + `alloc`.

Follows RFC 7946. **Not part of Boost.Geometry.**

## Purpose

GeoJSON reader and writer. Geometry members only — `Feature`/
`FeatureCollection` property bags are explicitly out of scope.

## Files

| File | Contents |
|---|---|
| `src/json.rs` | `GeoJsonError` |
| `src/parse.rs` | `from_geojson` |
| `src/write.rs` | `to_geojson` |

## Public surface

`from_geojson` parses into a `DynGeometry` (a GeoJSON `GeometryCollection`
is heterogeneous). `to_geojson` serialises any concrete model geometry to a
GeoJSON string.
