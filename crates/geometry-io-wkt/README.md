# geometry-io-wkt

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

OGC Well-Known Text (WKT) reader and writer.

Mirrors `boost/geometry/io/wkt/{read,write,wkt}.hpp`. The parser
emits a [`geometry_model::DynGeometry`] because WKT is heterogeneous
by construction (a `GEOMETRYCOLLECTION` mixes kinds); the writer
accepts any concrete geometry that implements the model traits.

Reference: OGC Simple Feature Access Part 1 (SFA-1) §7 for the WKT
grammar.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
