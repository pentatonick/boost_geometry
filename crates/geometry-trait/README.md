# geometry-trait

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Geometry concepts as Rust traits. One trait per `doc/concept/*.qbk`.

Mirrors `boost/geometry/core/{access,tag,coordinate_type,
coordinate_dimension,coordinate_system}.hpp` and the
`boost/geometry/geometries/concepts/*_concept.hpp` headers.

v1 surface — only the [`Geometry`] super-trait and the [`Point`]
concept land here. Subsequent tasks add `Box`, `Segment`,
`Linestring`, `Ring`, `Polygon`, and the multi / collection
concepts as siblings of [`Point`].

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
