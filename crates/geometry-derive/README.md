# geometry-derive

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

`#[derive(Point)]` for structs whose named fields are dimensions in
declaration order.

Rust analogue of the `BOOST_GEOMETRY_REGISTER_POINT_2D` family of
macros declared at `boost/geometry/geometries/register/point.hpp:81-87`.
Where the C++ macro injects template specialisations in the
`boost::geometry::traits` namespace, this proc-macro emits an
`impl Geometry` + `impl Point` block on the annotated struct.

The derive accepts an optional `#[geometry(...)]` attribute:

```text
#[derive(Point)]
#[geometry(cs = "Cartesian", scalar = "f64")]
struct MyPoint { x: f64, y: f64 }
```

Both keys are optional. `cs` defaults to `Cartesian` and `scalar`
to `f64`. Field order in the struct becomes dimension order in the
emitted `Point::get::<D>` / `Point::set::<D>` match arms.

## Crate dependencies (pragmatic approach)

The generated code uses absolute paths into the kernel crates —
`::geometry_trait::Point`, `::geometry_tag::PointTag`,
`::geometry_cs::Cartesian`, etc. Downstream callers must therefore
depend on `geometry-trait`, `geometry-tag`, and `geometry-cs`
(directly, or transitively via the `geometry` facade crate landing
in T47).

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
