# geometry-tag

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

Geometry kind tags and the tag-hierarchy marker traits.

Eleven empty tag types identify each OGC geometry kind, and a set of
marker traits (`Single`, `Multi`, `Pointlike`, `Linear`, `Polylinear`,
`Areal`, `Polygonal`, `Volumetric`) reproduce the C++ struct-inheritance
hierarchy at the Rust trait-bound level. Together they let downstream
crates dispatch on tag *identity* (one impl per tag) and tag *category*
(one impl that covers every linear tag, every areal tag, etc.) — the
Rust analogue of `tag_cast<Tag, Stops...>`.

References:
- `boost/geometry/core/tags.hpp` — tag hierarchy declarations.
- `boost/geometry/core/tag.hpp` — the `traits::tag<G>::type` metafunction.
- `boost/geometry/core/tag_cast.hpp` — base-tag walking, replaced here
  by Rust trait super-bounds.

## Examples

Category dispatch — one impl covers every linear tag:

```rust
use geometry_tag::{Linear, LinestringTag, MultiLinestringTag, SegmentTag};

fn accepts_linear<T: Linear>() {}
accepts_linear::<SegmentTag>();
accepts_linear::<LinestringTag>();
accepts_linear::<MultiLinestringTag>();
```

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
