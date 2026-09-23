# geometry-io-ewkt

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Checked XY Extended Well-Known Text (EWKT) reader and writer.

The geometry body uses [`geometry_io_wkt`]'s `PostGIS` 3.4.3-compatible
XY profile. Z, M, ZM, and extra ordinates are rejected, including on
empty geometries. No coordinate is silently discarded.

[`from_ewkt_2d`] retains empty points and empty multipoint members using
[`geometry_model::GeometryValue`]. [`from_ewkt`] returns the legacy
[`geometry_model::DynGeometry`] and rejects unrepresentable empty points.

The optional `SRID=` prefix accepts checked signed 32-bit integers.
Nonpositive values normalize to zero; values above 999999 normalize to
`999000 + (value % 999)`, matching the pinned `PostGIS` parser. Only space,
tab, CR and LF are accepted as whitespace. Whitespace is allowed before
the prefix and before its semicolon, but not inside `SRID=` or before digits.
Output SRIDs must be at most 999999; writers return an error otherwise.
`None` omits the prefix, while `Some(Srid::UNKNOWN)` writes `SRID=0;`.

## Migration

Owned and streaming writers return typed errors. Use [`to_ewkt`] instead
of formatting [`Ewkt`] with `Display`; formatting cannot convey geometry
validation errors. Negative input SRIDs now normalize to unknown, and
positive input SRIDs outside the `PostGIS` range normalize as described above.
The binary codecs and [`Srid`] itself retain their existing behavior.

```rust
use geometry_io_ewkt::{Srid, from_ewkt_2d, to_ewkt};

let e = from_ewkt_2d("SRID=4326;POINT EMPTY").unwrap();
assert_eq!(e.srid, Some(Srid::new(4326)));
assert_eq!(to_ewkt(&e.geometry, e.srid).unwrap(), "SRID=4326;POINT EMPTY");
```

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
