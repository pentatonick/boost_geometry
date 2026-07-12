# geometry-adapt-geo-types

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Adapt `geo-types` geometries to the `geometry-trait` concept
surface.

Mirrors the role of the Boost `BOOST_GEOMETRY_REGISTER_*` macros in
`boost/geometry/geometries/adapted/` — those specialise the C++
`traits::{tag, dimension, coordinate_type, coordinate_system,
access}` metafunctions (and, for containers, `ring_const_type`,
`exterior_ring`, `interior_rings`, …) for a foreign storage layout.
Rust has no specialisation and the orphan rule forbids implementing
a foreign concept trait for a foreign `geo-types` type, so — as with
[`geometry_adapt::Adapt`](https://docs.rs/geometry-adapt) and the
[`geometry_adapt_nalgebra`](https://docs.rs/geometry-adapt-nalgebra)
sibling — each foreign value is wrapped in a local newtype and the
geometry concepts hang off the wrapper.

## Point wrappers

* [`GeoCoord`] — `geo_types::Coord`, the `(x, y)` ordinate pair.
* [`GeoPoint`] — `geo_types::Point`, the newtype around a `Coord`.

Both pin `Cs = Cartesian`, `DIM = 2`, and are read-write
([`geometry_trait::PointMut`]).

## Container wrappers

* [`GeoLineString`] — `geo_types::LineString` as a `Linestring`.
* [`GeoRing`] — `geo_types::LineString` as a `Ring` (`geo-types` has
  no separate ring type; the caller asserts closure and
  orientation).
* [`GeoPolygon`] — `geo_types::Polygon`.
* [`GeoMultiPoint`] / [`GeoMultiLineString`] / [`GeoMultiPolygon`].
* [`GeoLine`] — `geo_types::Line` as a `Segment`.
* [`GeoRect`] — `geo_types::Rect` as a `Box`.
* [`GeoCollection`] — `geo_types::GeometryCollection`.

Because the container concepts yield their points *by reference*
(`points() -> impl Iterator<Item = &Self::Point>`) and this crate is
`#![forbid(unsafe_code)]`, the container wrappers store their
vertices as the wrapped point type ([`GeoCoord`] / [`GeoPoint`])
rather than re-casting the raw `geo-types` storage. Construction and
[`into_inner`](GeoLineString::into_inner) copy ordinates
element-by-element; both coordinate representations are `Copy`, so
the round trip is lossless.

## `DynGeometry` interop

[`crate::dyn_conversion`] provides
`From<geo_types::Geometry<T>>` ↔
[`DynGeometry<T, Cartesian>`](geometry_model::DynGeometry) so a whole
geometry tree can move between the two ecosystems.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
