# boost_geometry

[![CI](https://github.com/pentatonick/boost_geometry/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/pentatonick/boost_geometry/actions/workflows/ci.yml)
[![Publish](https://github.com/pentatonick/boost_geometry/actions/workflows/publish.yml/badge.svg)](https://github.com/pentatonick/boost_geometry/actions/workflows/publish.yml)
[![crates.io](https://img.shields.io/crates/v/boost_geometry.svg)](https://crates.io/crates/boost_geometry)
[![docs.rs](https://img.shields.io/docsrs/boost_geometry)](https://docs.rs/boost_geometry)
[![license](https://img.shields.io/crates/l/boost_geometry.svg)](https://github.com/pentatonick/boost_geometry/blob/main/README.md#license)
[![MSRV](https://img.shields.io/crates/msrv/boost_geometry)](https://github.com/pentatonick/boost_geometry/blob/main/rust-toolchain.toml)

A Rust port of [Boost.Geometry][boost-geometry] following its
philosophy: dimension-agnostic, coordinate-system-agnostic,
bring-your-own-type, strategy-pluggable.

The library's algorithms are written against *concept traits*
(`Point`, `Ring`, `Polygon`, …), not concrete structs — so your own
domain types participate directly, exactly like
`BOOST_GEOMETRY_REGISTER_*` in C++.

- **Edition:** Rust 2024, MSRV 1.85
- **Safety:** `unsafe_code = "forbid"` across the whole workspace
- **Docs:** see [`docs/`](https://github.com/pentatonick/boost_geometry/tree/main/docs) for the
  architecture, the tag-dispatch pattern, and the overlay engine

## Quick start — your own polygon type, buffered and validated

Add the dependency:

```sh
cargo add boost_geometry
```

Register your own ring and polygon types with one macro declaration
each, run `is_valid_polygon` on them directly, and buffer:

```rust
use boost_geometry::adapt::{register_polygon, register_ring};
use boost_geometry::model::{Polygon, Ring};
use boost_geometry::overlay::{JoinStrategy, buffer_convex_polygon, is_valid_polygon};
use boost_geometry::prelude::*;

type P = Point2D<f64, Cartesian>;

// Your own geometry types — no wrapper, no conversion trait.
struct Boundary {
    points: Vec<P>,
}

struct Parcel {
    outer: Boundary,
    holes: Vec<Boundary>,
}

// One declaration each and the library's algorithms accept them.
register_ring!(Boundary, P, |s| s.points.iter());
register_polygon!(
    Parcel,
    P,
    ring = Boundary,
    |s| outer = &s.outer, inners = s.holes.iter()
);

fn main() {
    // A 2×2 square parcel (clockwise, closed — the default ring convention).
    let parcel = Parcel {
        outer: Boundary {
            points: vec![
                P::new(0.0, 0.0),
                P::new(0.0, 2.0),
                P::new(2.0, 2.0),
                P::new(2.0, 0.0),
                P::new(0.0, 0.0),
            ],
        },
        holes: vec![],
    };

    // Validate the user-owned type directly.
    println!("parcel valid: {:?}", is_valid_polygon(&parcel));

    // Buffer it outward by 1.0 with round joins. The buffer kernel
    // works on the library's model polygon, so lift the points over.
    let model: Polygon<P> = Polygon::new(Ring::from_vec(
        parcel.outer.points.clone(),
    ));
    let mut grown = buffer_convex_polygon(
        &model,
        1.0,
        JoinStrategy::Round {
            points_per_circle: 360,
        },
    );

    // Normalise ring orientation, then validate the result.
    correct(&mut grown);
    println!("buffered valid: {:?}", is_valid_polygon(&grown));
    println!("area {:.3} -> {:.3}", area(&parcel), area(&grown));
}
```

Output:

```text
parcel valid: Ok(())
buffered valid: Ok(())
area 4.000 -> 15.141
```

The buffered area matches the closed form for a square grown by
distance *d* with round corners: *s*² + 4·*s*·*d* + π·*d*² =
4 + 8 + π ≈ 15.14.

What the example shows:

- **`register_ring!` / `register_polygon!`** implement the concept
  traits for your structs (Rust's orphan rule forbids a blanket impl,
  so the macros mint it per type — the same coherence workaround the
  `BOOST_GEOMETRY_REGISTER_*` macros perform in C++). Defaults are
  closed, clockwise rings; both are overridable per type.
- **`is_valid_polygon`** checks the OGC simple-feature rules — point
  count, closure, finite coordinates, spikes, self-intersections,
  ring orientation, hole containment — and reports the first failure
  as a `ValidityFailure` variant (`Err(SelfIntersection)`,
  `Err(WrongOrientation)`, …) rather than a bare `false`.
- **`buffer_convex_polygon`** grows the polygon outward, rounding
  each corner with a circular arc (`JoinStrategy::Miter` gives sharp
  corners instead). v1 buffers points and convex polygons with
  positive distances.
- **`correct`** fixes ring closure and orientation in place — the
  Boost `bg::correct` counterpart.

## Workspace layout

Nineteen crates form a dependency spine from foundational tag/coords
crates up through traits, models, strategies, and algorithms to the
`boost_geometry` facade — plus adapters (nalgebra, geo-types), IO
(WKT, WKB, GeoJSON, SVG), an R-tree, overlay operations, and
projections. `boost_geometry` re-exports everything; depend on it alone
unless you need a slimmer build.

See [`docs/01-architecture.md`](https://github.com/pentatonick/boost_geometry/blob/main/docs/01-architecture.md)
for the full map.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).

[boost-geometry]: https://www.boost.org/doc/libs/release/libs/geometry/
