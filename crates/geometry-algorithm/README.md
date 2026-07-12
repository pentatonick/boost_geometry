# geometry-algorithm

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Free-function algorithm entry points users actually call.

Each function mirrors the matching free function in
`boost/geometry/algorithms/`. T23 lands the strategy-dispatch layer
for `distance` / `comparable_distance`; subsequent tasks add
`length`, `area`, `envelope`, `within`, `intersects`, … on the same
pattern (a strategy-less default plus a `_with` explicit-strategy
companion).

## References

* `boost/geometry/algorithms/distance.hpp`
* `boost/geometry/algorithms/comparable_distance.hpp`
* `boost/geometry/algorithms/detail/distance/interface.hpp`

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
