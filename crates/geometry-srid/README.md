# geometry-srid

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

The `PostGIS` spatial-reference identifier (SRID).

A spatial-reference id names the coordinate reference system a
geometry's ordinates are expressed in. It is a foundation noun: both
`PostGIS` dialect crates carry one, `geometry-io-ewkt` as the decimal
integer after `SRID=` and `geometry-io-ewkb` as a 32-bit field in the
record header. Neither owns it, so it lives here, below both.

This crate interprets nothing. It does not resolve an id to a
coordinate system, does not validate one against a registry, and does
not rewrite values the way `PostGIS` does on ingest — see [`Srid`]'s
own documentation for the range `PostGIS` stores.

```rust
use geometry_srid::Srid;

let srid = Srid::new(4326);
assert_eq!(srid.get(), 4326);
assert_eq!(srid.to_string(), "4326");
assert_eq!(Srid::UNKNOWN.get(), 0);
```

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
