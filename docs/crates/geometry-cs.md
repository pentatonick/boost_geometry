# `geometry-cs`

**Layer 0 — foundation.** No domain dependencies. `#![no_std]`.

Mirrors `boost/geometry/core/cs.hpp` and `boost/geometry/core/coordinate_system.hpp`,
plus `boost/geometry/srs/spheroid.hpp`.

## Purpose

A coordinate system is a **type** with an associated `CoordinateSystem::Family`.
Algorithm strategies bind on the *family*, never the concrete CS, so a
degree variant and a radian variant of the same family share one impl.

## Files

| File | Contents |
|---|---|
| `src/system.rs` | `Cartesian`, `Spherical<U>`, `Geographic<U>`, `Polar<U>`, `CoordinateSystem` trait |
| `src/family.rs` | `CartesianFamily`, `SphericalFamily`, `GeographicFamily`, `PolarFamily` |
| `src/unit.rs` | `Degree`, `Radian` angle-unit tags, `AngleUnit`, `FromF64` |
| `src/spheroid.rs` | `Spheroid` — reference ellipsoid (e.g. WGS84) for geographic strategies |
| `src/lib.rs` | Re-exports only |

## Public surface

* **`CoordinateSystem`** trait — every concrete CS type implements this;
  its associated `Family` type is what strategies actually bind on.
* **`Cartesian`** — the default, no angle unit.
* **`Spherical<U: AngleUnit>`** — unit sphere, **equatorial convention**
  (latitude measured from the equator — documented prominently on the type
  itself).
* **`Geographic<U: AngleUnit>`** — ellipsoidal earth model, carries a
  `Spheroid`.
* **`Polar<U: AngleUnit>`** — polar coordinates.
* **`Degree` / `Radian`** — zero-sized angle-unit tags implementing `AngleUnit`.
* **`Spheroid`** — semi-major/semi-minor axis pair; WGS84 constant lives
  wherever the geographic strategies need it (`geometry-strategy::geographic`).

## Who depends on this

`geometry-trait` (a `Point`'s `Cs` associated type), `geometry-strategy`
(every strategy's family fence), `geometry-model`, `geometry-adapt`'s `WithCs<T, Cs>`.
