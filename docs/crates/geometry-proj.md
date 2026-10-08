# `geometry-proj`

**Standalone — not in Boost.Geometry.** Depends on `geometry-model`, `geometry-trait`.

Boost.Geometry keeps its CRS projections in `boost/geometry/srs/`
(`srs::projection`, `srs::transformation`), outside its documented reference.
This crate does not port them; it uses the pure-Rust
[`proj4rs`](https://crates.io/crates/proj4rs) engine — no C dependency (in
contrast to the usual [PROJ](https://proj.org/) C library most GIS stacks
bind against).

## Purpose

Reproject a geometry from one coordinate-reference system to another, in
place.

## Files

| File | Contents |
|---|---|
| `src/crs.rs` | `Crs`, `CrsError` |
| `src/reproject.rs` | `reproject`, `ReprojectPoints` hook |

## Public surface

```rust
let wgs84 = Crs::from_epsg(4326)?;      // lon/lat, radians
let mercator = Crs::from_epsg(3857)?;   // metres
reproject(&mut point, &wgs84, &mercator)?;
```

`Crs` can be built from a proj4 string, an EPSG code, or a WKT definition.

**Units gotcha:** `proj4rs` carries geographic coordinates in **radians**;
convert with `f64::to_radians`/`to_degrees` at the boundary — this is called
out prominently in the crate's own docs because it's the easy mistake to
make when wiring this crate to `geometry-cs::Geographic<Degree>` points.
