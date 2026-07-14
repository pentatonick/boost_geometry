# geometry-algorithm

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Free-function algorithm entry points users actually call.

Each function mirrors the matching free function in
`boost/geometry/algorithms/`. Strategy-driven algorithms expose a
strategy-less default plus a `_with` explicit-strategy companion. Algorithms
that require the overlay engine live in `geometry-overlay` to preserve the
workspace's one-way dependency graph.

## References

* `boost/geometry/algorithms/`
* `boost/geometry/algorithms/detail/`

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
