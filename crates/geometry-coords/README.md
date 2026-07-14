# geometry-coords

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Coordinate scalars, type promotion, and the comparable-distance newtype.

Mirrors the trio of Boost.Geometry headers that together encode
"what numeric types are allowed as coordinates, how do we widen them
when we have to do arithmetic across two geometries, and how do we
defer the `sqrt` in a distance comparison":

- `boost/geometry/util/select_most_precise.hpp` — the type-priority
  metafunction picking the wider of two scalars.
- `boost/geometry/util/calculation_type.hpp` — the policy that
  chooses the working type for binary/ternary algorithms.
- `boost/geometry/util/math.hpp` — fundamental scalar primitives
  (`abs`, `sqrt`, …) abstracted over the numeric type.
- `boost/geometry/util/precise_math.hpp` — adaptive expansion arithmetic
  for robust orientation and in-circle signs.
- `boost/geometry/util/series_expansion.hpp` — eighth-order Karney
  coefficient tables and Clenshaw evaluation.
- `boost/geometry/strategies/cartesian/distance_pythagoras.hpp`
  (lines 71-117, `namespace comparable`) — the squared-distance
  wrapper that callers can compare without paying for a `sqrt`.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
