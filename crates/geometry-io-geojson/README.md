# geometry-io-geojson

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

RFC 7946 `GeoJSON` reader and writer.

Not part of Boost.Geometry; follows RFC 7946. The parser emits a
[`geometry_model::DynGeometry`] (a `GeoJSON` `GeometryCollection` is
heterogeneous); the writer serialises any concrete model geometry
to a `GeoJSON` string. Feature objects and property bags are out of
scope — only the `geometry` member's OGC-equivalent kinds.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
