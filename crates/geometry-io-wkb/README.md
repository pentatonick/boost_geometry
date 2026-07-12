# geometry-io-wkb

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

OGC Well-Known Binary (WKB) reader and writer.

Follows OGC Simple Feature Access 06-103r4 §8 (Boost.Geometry ships
no WKB). The parser emits a [`geometry_model::DynGeometry`]; the
writer serialises any concrete model geometry to a byte vector in a
caller-chosen [`ByteOrder`].

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
