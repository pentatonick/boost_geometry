# `geometry-adapt`

**Layer 3 — adaptation.** Depends on `geometry-model`, `geometry-trait` (transitively tag/coords/cs). `#![no_std]`.

Mirrors `boost/geometry/geometries/{adapted,register}/*.hpp`.

## Purpose

Three paths to get a foreign or owned type into the geometry kernel,
documented in the crate's `lib.rs` in order from least to most indirect:

1. **Direct `impl Point for MyPoint`** — you own the type; implement
   `geometry_trait::{Geometry, Point, PointMut}` yourself. Smallest
   compile-time footprint, no wrapper.
2. **`#[derive(Point)]`** — the common case; see `geometry-derive`.
3. **`Adapt<T>` (+ optional `WithCs<T, Cs>`)** — you do *not* own the type
   (`[T; N]`, `(T, T)`, a foreign crate's point type). Coherence forbids a
   blanket impl on a foreign type directly (the orphan rule); the wrapper
   sidesteps it at zero runtime cost.

## Files

| File | Contents |
|---|---|
| `src/adapt.rs` | `Adapt<T>` — `#[repr(transparent)]` shape-only wrapper |
| `src/adapt_array.rs`, `src/adapt_borrowed_array.rs` | `Adapt` impls for `[T; N]` / `&[T; N]` |
| `src/adapt_tuple.rs` | `Adapt` impl for `(T, T)` |
| `src/with_cs.rs` | `WithCs<T, Cs>` — coordinate-system re-tagging wrapper |
| `src/macros.rs` | `__macros` — expansion target for `register_*!`, not part of the public API |

## Public surface

* **`Adapt<T>`** — answers *"how do I read coordinates out of this foreign
  data layout?"*. Defaults CS to `Cartesian`.
* **`WithCs<T, Cs>`** — answers *"what does that coordinate pair mean?"*
  Layers on top of `Adapt` (or any `Point`-implementing type) to change its
  coordinate system — e.g. `WithCs<Adapt<[f64; 2]>, Geographic<Degree>>`.
  Either wrapper can also re-tag a type that already implements `Point`, so
  a `MyPoint` from path 1 can be reused as a geographic point without a
  second adapter type.
* **`register_linestring!` / `register_ring!` / `register_polygon!` /
  `register_multi_point!` / `register_multi_linestring!` /
  `register_multi_polygon!`**
  (declarative, `#[macro_export]`ed) — container-level adaptation, since
  coherence also forbids a blanket `impl<P: Point, C: AsRef<[P]>> Linestring for C`.
  Mirrors `BOOST_GEOMETRY_REGISTER_LINESTRING` and siblings, including the
  three multi-geometry registration headers.

## Who depends on this

`geometry-strategy` (its worked "how to write a strategy" example uses
`WithCs`), `geometry-algorithm`, `geometry-overlay`, re-exported by the
`boost_geometry` facade as `boost_geometry::adapt`.
