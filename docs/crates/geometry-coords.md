# `geometry-coords`

**Layer 0 — foundation.** No domain dependencies. `#![no_std]`.

Mirrors `boost/geometry/util/{select_most_precise,calculation_type,math}.hpp`
and `strategies/cartesian/distance_pythagoras.hpp`'s `namespace comparable`.

## Purpose

Answers three questions: what numeric types are allowed as coordinates, how
do we widen types when doing arithmetic across two geometries with
different scalar types, and how do we defer a `sqrt` in a distance
comparison.

## Files

| File | Contents |
|---|---|
| `src/scalar.rs` | `CoordinateScalar` trait |
| `src/promote.rs` | `Promote` — type-widening metafunction |
| `src/comparable.rs` | `Comparable<T>` — skip-sqrt distance wrapper |
| `src/math.rs` | `abs`, `sqrt`, etc., abstracted over the numeric type |
| `src/lib.rs` | Re-exports only |

## Public surface

* **`CoordinateScalar`** — the trait bounding what can be a coordinate.
* **`Promote`** — given two scalar types, picks the wider for cross-geometry
  arithmetic (mirrors `select_most_precise.hpp`).
* **`Comparable<T>`** — a newtype whose ordering matches a real distance but
  whose computation skips the trailing `sqrt`; see `comparable_distance` in
  `geometry-algorithm`.
* **`math`** module — numeric primitives (`abs`, `sqrt`, …) that work the
  same over `f32`/`f64` without pulling in `std`.

## Who depends on this

`geometry-trait`, `geometry-strategy`, `geometry-algorithm`, `geometry-overlay`
— any crate doing scalar arithmetic on coordinates.
