# geometry-adapt

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

## Adapting your own types

Three paths get a user type into the geometry kernel, ordered from
least to most indirect. Pick the first one that applies.

### Path 1 — Direct `impl Point for MyPoint`

If you own the type, implement [`geometry_trait::Geometry`] and
[`geometry_trait::Point`] directly (add [`geometry_trait::PointMut`]
too if you need to *write* coordinates). This is the most explicit
form and the one with the smallest compile-time footprint — no
wrapper types, no macros.

```rust
use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut};

#[derive(Default)]
struct MyPoint { x: f64, y: f64 }

impl Geometry for MyPoint {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for MyPoint {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;
    fn get<const D: usize>(&self) -> f64 {
        if D == 0 { self.x } else { self.y }
    }
}

impl PointMut for MyPoint {
    fn set<const D: usize>(&mut self, v: f64) {
        if D == 0 { self.x = v } else { self.y = v }
    }
}
```

### Path 2 — `#[derive(Point)]`

For the common "struct with named coordinate fields" case the
`Point` proc-macro generates the impl. Mirrors the C++
`BOOST_GEOMETRY_REGISTER_POINT_2D` macro from
`boost/geometry/geometries/register/point.hpp`. The derive lives
in `geometry-derive` and is re-exported by the `geometry` facade
so downstream users only need a single dependency:

```rust
use geometry::Point;             // the derive macro
use geometry::prelude::*;        // brings in the `Point` trait + algorithms

#[derive(Default, Point)]
#[geometry(cs = "Cartesian", scalar = "f64")]
struct MyPoint { x: f64, y: f64 }

let d = distance(&MyPoint { x: 0.0, y: 0.0 }, &MyPoint { x: 3.0, y: 4.0 });
assert_eq!(d, 5.0);
```

### Path 3 — `Adapt<T>` + optional `WithCs<T, Cs>`

When you do *not* own the type (a `[T; N]`, a `(T, T)`, a foreign
crate's point type), wrap it in [`Adapt<T>`]. `Adapt` is a
`#[repr(transparent)]` newtype that forwards coordinate access
into the foreign storage layout. Coherence forbids a blanket impl
on the foreign type directly; the wrapper sidesteps the orphan
rule with zero runtime cost.

```rust
use geometry_adapt::Adapt;
use geometry_trait::Point;

let p = Adapt([3.0_f64, 4.0]);
assert_eq!(p.get::<0>(), 3.0);
assert_eq!(p.get::<1>(), 4.0);
```

[`Adapt<T>`] is **shape-only** and defaults the coordinate system
to [`Cartesian`](geometry_cs::Cartesian). For any other CS (latitude /
longitude in degrees, radians, …) layer [`WithCs<T, Cs>`] on top.
This is the orthogonality from proposal §3.7: `Adapt` answers
*"how do I read coordinates out of this foreign data layout?"*,
while `WithCs` answers *"what does that coordinate pair mean?"*.

```rust
use geometry_adapt::{Adapt, WithCs};
use geometry_algorithm::distance;
use geometry_cs::{Degree, Geographic};

// [lon, lat] degrees on WGS84.
let ams = WithCs::<_, Geographic<Degree>>::new(Adapt([4.90_f64, 52.37]));
let par = WithCs::<_, Geographic<Degree>>::new(Adapt([2.35_f64, 48.86]));
let _ = distance(&ams, &par); // picks Andoyer for the geographic family
```

Either wrapper can also re-tag a user type that already
implements [`Point`](geometry_trait::Point), so a `MyPoint` from
Path 1 can be reused as a geographic point by writing
`WithCs::<MyPoint, Geographic<Degree>>::new(p)` instead of
defining a second adapter.

### Container adaptation — the `register_*!` macros

Coherence also forbids a blanket impl on foreign sequence types
(`impl<P: Point, C: AsRef<[P]>> Linestring for C`). For that case,
`geometry-adapt` ships declarative macros `register_linestring!`,
`register_ring!`, `register_polygon!`, and the three `register_multi_*!`
forms, mirroring
`BOOST_GEOMETRY_REGISTER_LINESTRING` and siblings
(`boost/geometry/geometries/register/linestring.hpp` and co.).

---

Module layout:

 * [`Adapt<T>`] — shape-only wrapper. Forwards coordinate access
   into a foreign storage layout (array, tuple, third-party point).
   Defaults to [`Cartesian`](geometry_cs::Cartesian); layer
   [`WithCs`] on top for other systems.
 * [`WithCs<T, Cs>`] — coordinate-system re-tagging wrapper.

Mirrors the role of `boost/geometry/geometries/adapted/*.hpp`
(`c_array.hpp`, `std_array.hpp`, `boost_tuple.hpp`,
`boost_polygon.hpp`, …).

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
