# geometry-derive

Procedural derive macros (e.g. `#[derive(Point)]`).

Companion to `geometry-adapt`'s declarative `register_*!` macros — provides
the derive-based path for types you own.

## `#[derive(Point)]`

Generates an `impl Geometry` + `impl Point` block for a struct whose named
fields are the dimensions of a point, in declaration order. Rust counterpart
of `BOOST_GEOMETRY_REGISTER_POINT_2D` (and the 3D variant) declared at
`geometry/include/boost/geometry/geometries/register/point.hpp:81-87`.

```rust
use geometry_derive::Point;

#[derive(Point)]
#[geometry(cs = "Cartesian", scalar = "f64")]
struct MyPoint { x: f64, y: f64 }
```

The `#[geometry(...)]` attribute is optional. Defaults: `cs = "Cartesian"`,
`scalar = "f64"`.

### Required dependencies on the calling crate

The generated code refers to the kernel via absolute paths
(`::geometry_trait::Point`, `::geometry_tag::PointTag`,
`::geometry_cs::Cartesian`, …). Until the `geometry` facade crate (T47)
lands, downstream callers must depend on those crates directly:

```toml
[dependencies]
geometry-derive = { path = "…/geometry-derive" }
geometry-trait  = { path = "…/geometry-trait" }
geometry-tag    = { path = "…/geometry-tag" }
geometry-cs     = { path = "…/geometry-cs" }
```
