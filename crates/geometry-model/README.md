# geometry-model

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Default concrete geometry types.

Each type here implements the matching trait from `geometry-trait`.
Mirrors `boost/geometry/geometries/*.hpp` — the Boost.Geometry
"model" namespace of stock geometries (`model::point`,
`model::segment`, `model::box`, …) that the library ships for
callers who do not want to adapt their own types.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
