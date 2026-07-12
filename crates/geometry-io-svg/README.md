# geometry-io-svg

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

SVG output for Cartesian geometries — a debugging convenience.

Mirrors `boost/geometry/io/svg/svg_mapper.hpp`. Cartesian only: an
[`SvgMapper`] accumulates geometries, tracks their combined bounding
box, and emits a self-contained `<svg>` document mapping world
coordinates to a fixed pixel canvas (y flipped, since SVG y grows
downward).

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
