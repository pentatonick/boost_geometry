# `boost_geometry`

**Layer 7 — facade.** Depends on nearly every other crate in the workspace.

Mirrors `boost/geometry/geometry.hpp` — one entry point that fans out into
every public sub-namespace.

## Purpose

The umbrella crate. Add `boost_geometry` alone to your `Cargo.toml` and every
other crate in the workspace is reachable through one namespace. Each
`pub mod` in its `lib.rs` is a *re-export* of one underlying crate — this
file is itself a good example of [Rule 1a](../01-architecture.md) (a root
file as pure manifest) applied at the facade level.

## Module map

| Facade module | Re-exports |
|---|---|
| `boost_geometry::tag` | `geometry-tag` |
| `boost_geometry::coords` | `geometry-coords` |
| `boost_geometry::cs` | `geometry-cs` |
| `boost_geometry::trait_` (trailing underscore — `trait` is a keyword) | `geometry-trait` |
| `boost_geometry::model` | `geometry-model` |
| `boost_geometry::strategy` | `geometry-strategy` (including its `cartesian`/`spherical`/`geographic` submodules) |
| `boost_geometry::algorithm` | `geometry-algorithm` |
| `boost_geometry::adapt` | `geometry-adapt` |
| `boost_geometry::overlay` | `geometry-overlay` |
| `boost_geometry::rtree` | `geometry-rtree` |
| `boost_geometry::Point` (crate root) | `geometry-derive`'s `#[derive(Point)]` |
| `boost_geometry::prelude` | curated star-import module — the recommended entry point |

Two macro families land at the **crate root** rather than under their
logical submodule, because `#[macro_export]` always places declarative
macros at the crate root: `polygon!`/`linestring!`/`point!` (from
`geometry-model`) and `register_linestring!`/`register_ring!`/`register_polygon!`
(from `geometry-adapt`).

## `__private`

A `#[doc(hidden)] pub mod __private` re-exports the exact paths
`#[derive(Point)]`'s generated code needs (`geometry_cs::{Cartesian,
Geographic, Polar, Spherical}`, `geometry_tag::PointTag`,
`geometry_trait::{Geometry, Point}`). This exists so a future revision of
the derive macro can migrate to `::geometry::__private::…` paths without
breaking downstream crates that depend only on `boost_geometry` — never reference
this module directly.

## Doctest walkthrough

The crate-level doc comment on `lib.rs` is a runnable end-user tour — every
fenced code block is a doctest, so the examples can't drift from the real
API. It covers: Cartesian distance (Pythagoras default), spherical distance
(Haversine, an Amsterdam→Paris example ported from Boost's
`quick_start.cpp`), geographic distance (Andoyer default vs. explicit
Vincenty on WGS84), `#[derive(Point)]`, `Adapt<T>` for foreign layouts, and
a polygon/area/point-in-polygon example. Read `crates/geometry/src/lib.rs`
directly for the full runnable tour.
