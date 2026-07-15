# boost_geometry

[![CI](https://github.com/pentatonick/boost_geometry/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/pentatonick/boost_geometry/actions/workflows/ci.yml)
[![Publish](https://github.com/pentatonick/boost_geometry/actions/workflows/publish.yml/badge.svg)](https://github.com/pentatonick/boost_geometry/actions/workflows/publish.yml)
[![docs.rs](https://img.shields.io/docsrs/boost_geometry)](https://docs.rs/boost_geometry)
[![codecov](https://codecov.io/gh/pentatonick/boost_geometry/branch/main/graph/badge.svg)](https://codecov.io/gh/pentatonick/boost_geometry)
[![crates.io](https://img.shields.io/crates/v/boost_geometry.svg)](https://crates.io/crates/boost_geometry)
[![MSRV](https://img.shields.io/crates/msrv/boost_geometry)](https://github.com/pentatonick/boost_geometry/blob/main/rust-toolchain.toml)
[![license](https://img.shields.io/crates/l/boost_geometry.svg)](https://github.com/pentatonick/boost_geometry/blob/main/README.md#license)

A Rust port of [Boost.Geometry][boost-geometry], carrying over its design
philosophy: dimension-agnostic, coordinate-system-agnostic,
bring-your-own-type, strategy-pluggable.

The library's algorithms are written against *concept traits*
(`Point`, `Ring`, `Polygon`, …), not concrete structs — so your own
domain types participate directly, exactly like
`BOOST_GEOMETRY_REGISTER_*` in C++. There is no mandatory point or
polygon type to convert into: you register the types you already have
and call the algorithms on them. That makes `boost_geometry` a
**complement** to whatever geometry types your stack already uses,
rather than a container you must migrate onto.

- **Edition:** Rust 2024, MSRV 1.85
- **Safety:** `unsafe_code = "forbid"` across the whole workspace
- **Docs:** see [`docs/`](https://github.com/pentatonick/boost_geometry/tree/main/docs) for the
  architecture, the tag-dispatch pattern, and the overlay engine

## Quick start — your own polygon type, buffered and validated

Add the dependency:

```sh
cargo add boost_geometry
```

<details open>
<summary><b>Coupled — with <code>#[derive(Point)]</code></b></summary>

Derive `Point` on your own coordinate struct, register your own ring
and polygon types with one macro declaration each, run
`is_valid_polygon` on them directly, and buffer
(runnable as `cargo run --example parcel_buffer`):

<!-- example:parcel_buffer:start -->
```rust
use boost_geometry::Point;
use boost_geometry::adapt::{register_polygon, register_ring};
use boost_geometry::overlay::{JoinStrategy, buffer_convex_polygon, is_valid_polygon};
use boost_geometry::prelude::*;

// Your own geometry types — no wrapper, no conversion trait, and no
// library point type: the derive turns your struct into a Point.
#[derive(Clone, Copy, Default, Point)]
struct Coord {
    x: f64,
    y: f64,
}

struct Boundary {
    points: Vec<Coord>,
}

struct Parcel {
    outer: Boundary,
    holes: Vec<Boundary>,
}

// One declaration each and the library's algorithms accept them.
register_ring!(Boundary, Coord, |s| s.points.iter());
register_polygon!(
    Parcel,
    Coord,
    ring = Boundary,
    |s| outer = &s.outer,
    inners = s.holes.iter()
);

fn main() {
    let c = |x, y| Coord { x, y };

    // A 2×2 square parcel (clockwise, closed — the default ring convention).
    let parcel = Parcel {
        outer: Boundary {
            points: vec![
                c(0.0, 0.0),
                c(0.0, 2.0),
                c(2.0, 2.0),
                c(2.0, 0.0),
                c(0.0, 0.0),
            ],
        },
        holes: vec![],
    };

    // Validate the user-owned type directly.
    println!("parcel valid: {:?}", is_valid_polygon(&parcel));

    // Buffer it outward by 1.0 with round joins — directly on the
    // user-owned type.
    let mut grown = buffer_convex_polygon(
        &parcel,
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
<!-- example:parcel_buffer:end -->

Output:

```text
parcel valid: Ok(())
buffered valid: Ok(())
area 4.000 -> 15.141
```

</details>

<details>
<summary><b>De-coupled — without the derive</b></summary>

The derive is pure sugar: it emits the `Geometry` + `Point`
(+ `PointMut`) impls below. Writing them by hand is the escape hatch
for computed coordinates, packed storage, or FFI structs — same flow,
same output (runnable as `cargo run --example parcel_buffer_manual`):

<!-- example:parcel_buffer_manual:start -->
```rust
use boost_geometry::adapt::{register_polygon, register_ring};
use boost_geometry::cs::Cartesian;
use boost_geometry::overlay::{JoinStrategy, buffer_convex_polygon, is_valid_polygon};
use boost_geometry::prelude::*;
use boost_geometry::tag::PointTag;
use boost_geometry::trait_::{Geometry, Point, PointMut};

// Your own point type, registered by hand — the escape hatch for
// computed coordinates, packed storage, or FFI structs the derive
// cannot express.
#[derive(Clone, Copy, Default)]
struct Coord {
    x: f64,
    y: f64,
}

impl Geometry for Coord {
    type Kind = PointTag;
    type Point = Coord;
}

impl Point for Coord {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;

    fn get<const D: usize>(&self) -> f64 {
        match D {
            0 => self.x,
            1 => self.y,
            _ => unreachable!(),
        }
    }
}

// Only needed by algorithms that construct points (buffer, correct).
impl PointMut for Coord {
    fn set<const D: usize>(&mut self, v: f64) {
        match D {
            0 => self.x = v,
            1 => self.y = v,
            _ => unreachable!(),
        }
    }
}

struct Boundary {
    points: Vec<Coord>,
}

struct Parcel {
    outer: Boundary,
    holes: Vec<Boundary>,
}

register_ring!(Boundary, Coord, |s| s.points.iter());
register_polygon!(
    Parcel,
    Coord,
    ring = Boundary,
    |s| outer = &s.outer,
    inners = s.holes.iter()
);

fn main() {
    let c = |x, y| Coord { x, y };

    // A 2×2 square parcel (clockwise, closed — the default ring convention).
    let parcel = Parcel {
        outer: Boundary {
            points: vec![
                c(0.0, 0.0),
                c(0.0, 2.0),
                c(2.0, 2.0),
                c(2.0, 0.0),
                c(0.0, 0.0),
            ],
        },
        holes: vec![],
    };

    println!("parcel valid: {:?}", is_valid_polygon(&parcel));

    let mut grown = buffer_convex_polygon(
        &parcel,
        1.0,
        JoinStrategy::Round {
            points_per_circle: 360,
        },
    );

    correct(&mut grown);
    println!("buffered valid: {:?}", is_valid_polygon(&grown));
    println!("area {:.3} -> {:.3}", area(&parcel), area(&grown));
}
```
<!-- example:parcel_buffer_manual:end -->

The buffered area matches the closed form for a square grown by
distance *d* with round corners: *s*² + 4·*s*·*d* + π·*d*² =
4 + 8 + π ≈ 15.14.

</details>

What the example shows (identical for both paths above):

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

## How it stays type-agnostic

The library never sees your `Coord`, `Boundary`, or `Parcel` as a concrete
type. It only ever sees *"some `G` that satisfies the `Point` (or `Ring`, or
`Polygon`) concept trait"*, and reads it through that trait's methods. Your
struct keeps its own fields, layout, and ownership; the `register_*!` macros
just teach the trait how to read it. That is the whole trick — the same one
`BOOST_GEOMETRY_REGISTER_*` performs in C++, rebuilt from ordinary Rust
generics instead of template specialisation.

The techniques below are worth stealing for any *bring-your-own-type* library.

<details>
<summary><b>The Rust features that make it work</b></summary>

**1. A concept is a trait; the data stays yours.** A geometry is anything
implementing the concept trait — the library owns *behaviour*, you own the
*bytes*. The read surface is tiny:

```rust
pub trait Point: Geometry<Kind = PointTag, Point = Self> {
    type Scalar: CoordinateScalar;   // associated type — your f64, i32, fixed-point…
    type Cs: CoordinateSystem;       // associated type — Cartesian / Spherical / Geographic
    const DIM: usize;                // dimensions, known at compile time
    fn get<const D: usize>(&self) -> Self::Scalar;   // read axis D
}
```

Because `Scalar` and `Cs` are **associated types** (not generic parameters),
a function written `fn distance<P: Point>(a: &P, b: &P)` carries the scalar and
coordinate system along for free — no `<P, S, Cs>` soup at every call site.

**2. Const generics move dimension checks to compile time.** `get::<const D>`
takes the axis as a const generic, so `p.get::<2>()` on a 2-D point is a
*compile* error, not a runtime panic — the bound check happens once, in the
type system.

**3. Zero-sized marker tags + supertraits give free category dispatch.** Each
kind has a ZST tag (`PointTag`, `RingTag`, …) named by `Geometry::Kind`, and the
tags form a hierarchy with plain supertraits:

```rust
pub trait Polylinear: Linear {}   // reproduces C++'s `polylinear_tag : linear_tag`
```

So `fn f<T: Linear>()` accepts segments, linestrings, *and* multi-linestrings
in one signature — category dispatch with no macro, no enum, no `dyn`.

**4. The orphan rule is sidestepped with per-type macros.** Rust forbids a
blanket `impl<T> Point for T`, and you can't `impl Point for Vec<YourCoord>`
from this crate either (neither is yours). So `register_ring!` / `#[derive(Point)]`
**mint one concrete impl per type** at *your* call site, where the coherence
rules allow it — exactly the role the `BOOST_GEOMETRY_REGISTER_*` macros play.

**5. Dispatch resolves statically, then vanishes.** "One function, many kinds"
(`within` on a ring vs. a polygon) is solved by a `Kind → zero-sized-strategy`
type-level picker; every layer is a ZST and a static trait resolution, so at
`-O` the whole chain collapses to a single direct call — no vtable, no branch.
This is the codebase's one recurring idiom; the full mechanism (and why the
obvious `impl<G: Ring>` + `impl<G: Polygon>` approach hits `E0119`) is written
up in
**[docs/02-tag-dispatch-pattern.md](https://github.com/pentatonick/boost_geometry/blob/main/docs/02-tag-dispatch-pattern.md)**.

The payoff: your types participate with **no wrapper, no conversion, and no
runtime cost** — the generic code monomorphises straight onto your struct's own
accessors.

</details>

## Features

Every capability below is a free function you call on your own registered
types — no conversion step. Adding the single `boost_geometry` crate brings
in all of the ✅ rows; the I/O and reprojection formats are separate crates
so a default build stays lean. The **Docs** link opens the rustdoc for that
item; the `no_std` column is the status of the crate the function lives in
(see the [full matrix](#no_std-support) below).

Naming mirrors Boost.Geometry: a strategy-driven algorithm exposes a
strategy-less default (picks the right strategy for the coordinate system)
plus a `_with` companion that takes an explicit strategy.

<!-- The table below is generated by .github/scripts/feature_table.py from the
     `// feature-group:` tags on each crate's `pub use` lines; do not edit it by
     hand. CI fails if it drifts. Adding an algorithm? Tag its export. -->

<sub>Auto-generated from the source; run `python3 .github/scripts/feature_table.py` after adding an export.</sub>

<!-- feature-table:start -->
| Function | `no_std` | Docs |
|---|:---:|---|
| **Measures** — Scalar quantities of a geometry |||
| `area` / `area_with` / `box_area` / `multi_polygon_area` / `ring_area` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.area.html) |
| `area_dyn` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.area_dyn.html) |
| `azimuth` / `azimuth_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.azimuth.html) |
| `centroid` / `centroid_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.centroid.html) |
| `closest_points` / `closest_points_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.closest_points.html) |
| `comparable_distance` / `comparable_distance_with` / `distance` / `distance_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.comparable_distance.html) |
| `discrete_frechet_distance` / `discrete_frechet_distance_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.discrete_frechet_distance.html) |
| `discrete_hausdorff_distance` / `discrete_hausdorff_distance_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.discrete_hausdorff_distance.html) |
| `distance_dyn` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.distance_dyn.html) |
| `length` / `length_with` / `perimeter` / `perimeter_with` / `ring_perimeter` / `ring_perimeter_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.length.html) |
| `length_dyn` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.length_dyn.html) |
| **Spatial predicates** — Boolean relationships between geometries |||
| `contains_properly` / `crosses` / `overlaps` / `relate_matrix` / `relation` / `relate` / `touches` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/overlay/fn.contains_properly.html) |
| `coordinate_position` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.coordinate_position.html) |
| `covered_by` / `within` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.covered_by.html) |
| `disjoint` / `disjoint_box_box` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.disjoint.html) |
| `equals` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.equals.html) |
| `intersects` / `intersects_reversed` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.intersects.html) |
| `within_dyn` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.within_dyn.html) |
| **Boolean operations** — Overlay and offset of areal geometries |||
| `buffer` / `buffer_convex_polygon` / `buffer_point` / `buffer_with` / `buffer_with_strategy` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/overlay/fn.buffer.html) |
| `difference` / `intersection` / `sym_difference` / `union` / `union_poly` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/overlay/fn.difference.html) |
| `line_intersection` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/overlay/fn.line_intersection.html) |
| `point_on_surface` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/overlay/fn.point_on_surface.html) |
| **Construction & transformation** — Derive a new geometry from an existing one |||
| `chaikin_smoothing` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.chaikin_smoothing.html) |
| `concave_hull` / `concave_hull_with` / `k_nearest_concave_hull` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.concave_hull.html) |
| `convex_hull` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.convex_hull.html) |
| `densify` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.densify.html) |
| `destination` / `destination_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.destination.html) |
| `envelope` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.envelope.html) |
| `envelope_dyn` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.envelope_dyn.html) |
| `expand` / `expand_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.expand.html) |
| `line_interpolate` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.line_interpolate.html) |
| `line_locate_point` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.line_locate_point.html) |
| `linestring_segmentize` / `linestring_segmentize_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.linestring_segmentize.html) |
| `map_coords` / `map_coords_in_place` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.map_coords.html) |
| `minimum_rotated_rect` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.minimum_rotated_rect.html) |
| `monotone_subdivision` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.monotone_subdivision.html) |
| `rhumb_azimuth` / `rhumb_azimuth_with` / `rhumb_destination` / `rhumb_destination_with` / `rhumb_distance` / `rhumb_distance_with` / `rhumb_length` / `rhumb_length_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.rhumb_azimuth.html) |
| `simplify` / `simplify_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.simplify.html) |
| `transform` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.transform.html) |
| `triangulate_earcut` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.triangulate_earcut.html) |
| **Inspection** — Query a geometry's shape or membership |||
| `for_each_point` / `for_each_segment` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.for_each_point.html) |
| `is_convex` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.is_convex.html) |
| `is_empty` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.is_empty.html) |
| `is_simple` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.is_simple.html) |
| `is_valid` / `is_valid_polygon` / `is_valid_polygon_with` / `is_valid_ring` / `is_valid_ring_with` / `is_valid_with` / `validity_reason` / `validity_reason_with` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/overlay/fn.is_valid.html) |
| `num_geometries` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.num_geometries.html) |
| `num_interior_rings` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.num_interior_rings.html) |
| `num_points` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.num_points.html) |
| `num_segments` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.num_segments.html) |
| **Mutation & assembly** — Build up or normalise a geometry in place |||
| `append` / `append_to_ring` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.append.html) |
| `assign_values` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.assign_values.html) |
| `clear` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.clear.html) |
| `convert` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.convert.html) |
| `correct` / `correct_closure` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.correct.html) |
| `make_box` / `make_point` / `make_segment` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.make_box.html) |
| `merge_elements` / `merge_multipolygon` / `merge_polygons` / `stitch_triangles` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/overlay/fn.merge_elements.html) |
| `remove_spikes` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.remove_spikes.html) |
| `reverse` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.reverse.html) |
| `unique` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/algorithm/fn.unique.html) |
| **Spatial index** — Bulk-loadable R-tree with nearest-neighbour and predicate queries |||
| `and` / `not` / `satisfies` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/rtree/fn.and.html) |
| `Rtree` | ✅ | [→](https://docs.rs/boost_geometry/latest/boost_geometry/rtree/struct.Rtree.html) |
| **I/O — Well-Known Text** — Parse and write the OGC WKT format |||
| `from_wkt` / `parse_linestring` / `parse_multi_linestring` / `parse_multi_point` / `parse_multi_polygon` / `parse_point` / `parse_polygon` | ❌ | [→](https://docs.rs/geometry-io-wkt) |
| `to_wkt` / `to_wkt_polygon` / `write_wkt` | ❌ | [→](https://docs.rs/geometry-io-wkt) |
| **I/O — Well-Known Binary** — Parse and write the OGC WKB format |||
| `from_wkb` | ❌ | [→](https://docs.rs/geometry-io-wkb) |
| `to_wkb` / `to_wkb_polygon` | ❌ | [→](https://docs.rs/geometry-io-wkb) |
| **I/O — GeoJSON** — Parse and write GeoJSON (RFC 7946) |||
| `from_geojson` | ❌ | [→](https://docs.rs/geometry-io-geojson) |
| `to_geojson` / `to_geojson_polygon` | ❌ | [→](https://docs.rs/geometry-io-geojson) |
| **I/O — SVG** — Render geometries to SVG (debugging) |||
| `SvgMapper` | ❌ | [→](https://docs.rs/geometry-io-svg) |
| **Reprojection** — CRS-to-CRS point reprojection (standalone crate) |||
| `reproject` | ✅ | [→](https://docs.rs/geometry-proj) |
<!-- feature-table:end -->

**Ecosystem adapters** register the types of other crates so the algorithms
above accept them directly:
[`geo-types`](https://docs.rs/geometry-adapt-geo-types) (`GeoPoint`,
`GeoPolygon`, …) and [`nalgebra`](https://docs.rs/geometry-adapt-nalgebra)
(`NaPoint2`, `NaVector3`, …). Both are `no_std`.

Everything the `boost_geometry` facade re-exports is browsable from one
place: **[docs.rs/boost_geometry](https://docs.rs/boost_geometry)**. The I/O,
projection, and adapter crates are separate dependencies — their columns link
to their own docs.rs pages.

## Workspace layout

Nineteen crates form a dependency spine from foundational tag/coords
crates up through traits, models, strategies, and algorithms to the
`boost_geometry` facade — plus adapters (nalgebra, geo-types), IO
(WKT, WKB, GeoJSON, SVG), an R-tree, overlay operations, and
projections. `boost_geometry` re-exports everything; depend on it alone
unless you need a slimmer build.

See [`docs/01-architecture.md`](https://github.com/pentatonick/boost_geometry/blob/main/docs/01-architecture.md)
for the full map, and the per-crate `no_std` status just below.

## `no_std` support

Generated by `.github/scripts/no_std_support.py`, which builds every crate
with `--no-default-features` (falling back to `--features libm` for crates
that need a libm-backed `Float` impl) and is checked in CI:

<!-- no-std-table:start -->
| Crate | `no_std` |
|---|---|
| `boost_geometry` | ✅ |
| `geometry-adapt` | ✅ |
| `geometry-adapt-geo-types` | ✅ |
| `geometry-adapt-nalgebra` | ✅ |
| `geometry-algorithm` | ✅ |
| `geometry-coords` | ✅ |
| `geometry-cs` | ✅ |
| `geometry-derive` | ✅ |
| `geometry-io-geojson` | ❌ |
| `geometry-io-svg` | ❌ |
| `geometry-io-wkb` | ❌ |
| `geometry-io-wkt` | ❌ |
| `geometry-model` | ✅ |
| `geometry-overlay` | ✅ |
| `geometry-proj` | ✅ |
| `geometry-rtree` | ✅ |
| `geometry-strategy` | ✅ |
| `geometry-tag` | ✅ |
| `geometry-trait` | ✅ |
<!-- no-std-table:end -->

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).

[boost-geometry]: https://www.boost.org/doc/libs/release/libs/geometry/
