# geometry-proj

Coordinate-reference-system reprojection for the geometry kernel,
backed by the pure-Rust [`proj4rs`](https://crates.io/crates/proj4rs)
engine — no C dependency.

Boost.Geometry leaves projections to its unsupported
`extensions/gis/projections/`; this crate provides them:

```rust
use geometry_proj::{reproject, Crs};

let wgs84 = Crs::from_epsg(4326)?;      // lon/lat in radians
let mercator = Crs::from_epsg(3857)?;   // metres
reproject(&mut geometry, &wgs84, &mercator)?;
```

Build a `Crs` from a proj4 string, an EPSG code, or a WKT definition
(parsed by `proj4wkt`). `reproject` transforms any point / linestring /
ring / polygon / multipolygon in place.

Geographic coordinates are carried in **radians**, matching `proj4rs`.
