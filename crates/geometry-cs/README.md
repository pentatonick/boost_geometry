# geometry-cs

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Coordinate systems, angle units, and reference spheroids.

Mirrors `boost/geometry/core/cs.hpp` (the `cs::cartesian`,
`cs::spherical`, `cs::geographic`, `cs::polar` tag templates and their
`cs_tag` family classifier) together with
`boost/geometry/core/coordinate_system.hpp` (the
`traits::coordinate_system<Point>` typedef that picks one of those
tags out per point type) and `boost/geometry/srs/spheroid.hpp` (the
reference ellipsoid carried alongside any geographic strategy).

Crate split per proposal §3.3: a coordinate system is a *type* with
an associated [`CoordinateSystem::Family`], and algorithm strategies
bind on the family — never on the concrete CS — so that degree and
radian variants share one impl.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
