# `geometry-derive`

**Layer 2 — concrete types (derive path).** proc-macro crate; depends on `geometry-trait`/`geometry-tag`/`geometry-cs` only at the generated-code level.

Mirrors `boost/geometry/geometries/register/point.hpp`'s
`BOOST_GEOMETRY_REGISTER_POINT_2D` macro family.

## Purpose

`#[derive(Point)]` for structs whose named fields are dimensions in
declaration order — the ergonomic path for types you *own* (companion to
`geometry-adapt`'s declarative macros, which handle types you don't own).

## Files

| File | Contents |
|---|---|
| `src/derive_point.rs` | The macro expansion logic |
| `src/lib.rs` | `#[proc_macro_derive(Point, attributes(geometry))]` entry point |

## Usage

```rust
#[derive(Default, Point)]
#[geometry(cs = "Cartesian", scalar = "f64")]
struct MyPoint { x: f64, y: f64 }
```

Both `cs` and `scalar` are optional (`Cartesian` / `f64` by default). Field
order becomes dimension order in the generated `Point::get::<D>()` /
`Point::set::<D>()` match arms. Generates `impl Geometry` + `impl Point` (and
`PointMut`) using **absolute paths** into the kernel crates
(`::geometry_trait::Point`, `::geometry_tag::PointTag`, `::geometry_cs::Cartesian`)
so downstream callers need `geometry-trait`/`geometry-tag`/`geometry-cs` in
scope — directly, or transitively via the `boost_geometry` facade.

## Who depends on this

Re-exported by the `boost_geometry` facade crate as `boost_geometry::Point` so end users
only need one dependency line.
