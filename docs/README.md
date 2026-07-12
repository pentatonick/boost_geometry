# boost_geometry — documentation

A Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/):
dimension-agnostic, coordinate-system-agnostic, bring-your-own-type,
strategy-pluggable. 18 crates, ~66k lines of Rust, 52 test files. Every
public item carries a `///` doc comment referencing the exact Boost C++
header it mirrors — this documentation set is a *map* on top of that
existing rustdoc, not a replacement for it. When in doubt, the crate's own
`src/lib.rs` doc comment is the primary source; these pages tell you which
file to open.

## Start here

New to this codebase? Read in this order:

1. **[Architecture](01-architecture.md)** — the 18-crate dependency spine,
   what each layer owns, why the shape is acyclic by construction. The map
   of the whole workspace.
2. **[The tag-dispatch pattern](02-tag-dispatch-pattern.md)** — the one
   recurring idiom (`XxxStrategy` trait + one ZST struct per geometry kind +
   a tag→struct picker) that explains most of the shape you'll see inside
   `geometry-strategy` and `geometry-algorithm`. Once this clicks, the rest
   of the code stops looking repetitive and starts looking systematic.
3. **[Overlay engine deep-dive](03-overlay-engine.md)** — `geometry-overlay`
   is the largest, densest subsystem (turn graph → traversal → assembly →
   `intersection`/`union`/`difference`/`buffer`). Its own page because it
   doesn't fit the per-crate template below.

## Per-crate reference

One page per crate — purpose, file-by-file contents, public surface, who
depends on it. Ordered by dependency layer (foundation first):

| Layer | Crate | One-line role |
|---|---|---|
| 0 | [`geometry-tag`](crates/geometry-tag.md) | Kind tags + tag-hierarchy marker traits |
| 0 | [`geometry-coords`](crates/geometry-coords.md) | Coordinate scalar trait, type promotion, comparable-distance |
| 0 | [`geometry-cs`](crates/geometry-cs.md) | Cartesian / Spherical / Geographic / Polar coordinate systems |
| 1 | [`geometry-trait`](crates/geometry-trait.md) | The concepts: `Geometry`, `Point`, `Linestring`, `Ring`, `Polygon`, … |
| 2 | [`geometry-model`](crates/geometry-model.md) | Concrete types: `Point2D`, `Polygon`, `DynGeometry`, … |
| 2 | [`geometry-derive`](crates/geometry-derive.md) | `#[derive(Point)]` |
| 3 | [`geometry-adapt`](crates/geometry-adapt.md) | `Adapt<T>`, `WithCs<T, Cs>`, `register_*!` macros |
| 4 | [`geometry-strategy`](crates/geometry-strategy.md) | Pluggable per-algorithm, per-CS-family strategies |
| 5 | [`geometry-algorithm`](crates/geometry-algorithm.md) | Free functions: `distance`, `area`, `within`, … |
| 6 | [`geometry-overlay`](crates/geometry-overlay.md) | Boolean overlay: `intersection`, `union`, `difference`, `buffer` |
| 7 | [`geometry-rtree`](crates/geometry-rtree.md) | R-tree spatial index |
| 7 | [`boost_geometry`](crates/geometry.md) | Umbrella facade — one dependency, everything re-exported |
| adapter | [`geometry-adapt-geo-types`](crates/geometry-adapt-geo-types.md) | Adapts the [`geo-types`](https://docs.rs/geo-types) ecosystem crate |
| adapter | [`geometry-adapt-nalgebra`](crates/geometry-adapt-nalgebra.md) | Adapts [`nalgebra`](https://nalgebra.org) points/vectors |
| I/O | [`geometry-io-wkt`](crates/geometry-io-wkt.md) | Well-Known Text |
| I/O | [`geometry-io-wkb`](crates/geometry-io-wkb.md) | Well-Known Binary |
| I/O | [`geometry-io-geojson`](crates/geometry-io-geojson.md) | GeoJSON (RFC 7946) |
| I/O | [`geometry-io-svg`](crates/geometry-io-svg.md) | SVG output (debugging) |
| standalone | [`geometry-proj`](crates/geometry-proj.md) | CRS reprojection |

## Answering "where do I...?"

| I want to... | Go to |
|---|---|
| Use the library as an end user | [`boost_geometry`](crates/geometry.md) facade + its runnable doctest tour in `crates/geometry/src/lib.rs` |
| Adapt my own point type | [`geometry-adapt`](crates/geometry-adapt.md) — three paths, pick the first that fits |
| Adapt a `geo-types` or `nalgebra` type | [`geometry-adapt-geo-types`](crates/geometry-adapt-geo-types.md) / [`geometry-adapt-nalgebra`](crates/geometry-adapt-nalgebra.md) |
| Add a new algorithm | Read [tag-dispatch pattern](02-tag-dispatch-pattern.md) first, then [`geometry-strategy`](crates/geometry-strategy.md)'s own "how to write a strategy" tutorial in its `lib.rs` |
| Understand `intersection`/`union`/`difference` | [Overlay deep-dive](03-overlay-engine.md) |
| Parse/write WKT, WKB, or GeoJSON | [`geometry-io-wkt`](crates/geometry-io-wkt.md) / [`geometry-io-wkb`](crates/geometry-io-wkb.md) / [`geometry-io-geojson`](crates/geometry-io-geojson.md) |
| Spatially index a set of geometries | [`geometry-rtree`](crates/geometry-rtree.md) |
| Reproject between coordinate systems | [`geometry-proj`](crates/geometry-proj.md) |
| See the full crate dependency graph | [Architecture](01-architecture.md) |

## What this documentation set is *not*

* **Not API documentation.** Run `cargo doc --workspace --open` for that —
  every public item already has a `///` comment with a Boost header
  reference and, for non-trivial functions, a runnable example.
* **Not the porting history.** This documentation set is a snapshot of the
  code as it exists now, oriented at someone exploring the codebase rather
  than someone tracking what's left to build; the phased porting history
  lives in the git log.

## Conventions used across these pages

* Every crate page states its **dependency layer** and **`no_std`/`alloc`
  status** up top — both are structural facts enforced by the workspace's
  Cargo lints (`unsafe_code = "forbid"` workspace-wide; no crate here uses
  `unsafe`).
* "Mirrors `boost/geometry/...`" points at the exact C++ header a crate or
  module is porting — the same reference each crate's own rustdoc carries.
* Diagrams are [Mermaid](https://mermaid.js.org/), which renders natively
  in GitHub's Markdown viewer and most editor Markdown previews — no build
  step required to view them.

## Original git hash

Ported from `aed7bc3bb55f0fbf13d1762e9e65bee3452adc1b`
