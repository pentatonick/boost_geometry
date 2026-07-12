# `geometry-strategy`

**Layer 4 — strategies.** Depends on `geometry-model`, `geometry-adapt`, `geometry-cs`, `geometry-coords` (transitively tag/trait). `#![no_std]` + `alloc`.

Mirrors `boost/geometry/strategies/{cartesian,spherical,geographic}/*.hpp`.

## Purpose

Every algorithm has a **strategy trait** here; concrete strategies live in
submodules keyed by coordinate-system family. This crate's `lib.rs` doc
comment is the canonical "how to write a new strategy" tutorial in the
codebase — read it directly if you're adding one. Summary of its 3 steps:

1. **Bind on `CoordinateSystem::Family`**, never the concrete CS, using
   `SameAs<CartesianFamily>` as the fence — one impl then covers both
   `Spherical<Degree>` and `Spherical<Radian>`.
2. **Decide on a `Comparable` sibling** — a strategy that returns the same
   *ordering* while skipping work the ordering doesn't need (e.g. skip the
   final `sqrt` for distance comparisons). If there's no shortcut, set
   `type Comparable = Self;`.
3. **Wire the default** via a `DefaultXxx<Family>` trait so the no-strategy
   call (`distance(a, b)`) resolves to the right concrete strategy by
   walking `A → A::Point → Cs → Family → DefaultXxx<Family>::Strategy`.

See also [the tag-dispatch pattern](../02-tag-dispatch-pattern.md) for the
orthogonal per-**kind** dispatch axis (this crate is where both axes —
kind and coordinate-system family — combine).

## Module layout

| Module | Strategy trait | Concrete strategies |
|---|---|---|
| `distance` | `DistanceStrategy`, `DefaultDistance`, `Reversed` | `Pythagoras`/`ComparablePythagoras` (cartesian), `Haversine`/`ComparableHaversine` (spherical), `Andoyer`/`Vincenty`/`Thomas` (geographic) |
| `area` | `AreaStrategy`, `DefaultArea` | `ShoelaceArea`, `ShoelacePolygonArea`, `ShoelaceBoxArea`, `ShoelaceMultiPolygonArea` (cartesian); `SphericalArea`/`SphericalPolygonArea`; `GeographicArea`/`GeographicPolygonArea` |
| `length` | `LengthStrategy`, `DefaultLength` | `CartesianLength`/`CartesianPerimeter`; `SphericalLength`/`SphericalPerimeter`; `GeographicLength`/`GeographicPerimeter` |
| `envelope` | `EnvelopeStrategy`, `EnvelopeStrategyForKind` | `Envelope{Point,Segment,Linestring,Ring,Polygon,Box,MultiPoint,MultiLinestring,MultiPolygon}` — see [tag-dispatch pattern](../02-tag-dispatch-pattern.md) |
| `within` | `WithinStrategy`, `WithinStrategyForKind` | `WithinRing`, `WithinPoly`, `WithinBox` |
| `intersects` | `IntersectsStrategy`, `IntersectsPairStrategy` | `CartesianIntersects` |
| `disjoint` | `DisjointStrategy` | `CartesianDisjoint` |
| `equals` | `EqualsStrategy`, `EqualsPairStrategy` | `EqPointPoint`, `EqSegmentSegment`, `EqPolygonPolygon` |
| `centroid` | `CentroidStrategy`, `CentroidStrategyForKind` | `Cartesian{Polygon,Ring,Linestring,Segment,Box,MultiPoint}Centroid` |
| `azimuth` | `AzimuthStrategy`, `DefaultAzimuth` | `CartesianAzimuth`; `SphericalAzimuth`; `GeographicAzimuth` |
| `convex_hull` | `ConvexHullStrategy` | `MonotoneChain` (Andrew's monotone chain) |
| `simplify` | `SimplifyStrategy` | `DouglasPeucker` |
| `densify` | `DensifyStrategy` | `CartesianDensify` |
| `line_interpolate` | `LineInterpolateStrategy` | `CartesianLineInterpolate` |
| `transform` | `TransformStrategy` | `Affine2`, `Affine3` |
| `closest_points` | `ClosestPointsStrategy` | `CartesianClosestPoints` |

`cartesian`, `spherical`, `geographic` are the per-family submodules that
hold the concrete impls above; `normalise` is `pub(crate)` (angular
normalisation helper, not part of the public surface).

## Reverse dispatch

For algorithms whose two arguments are symmetric, write one impl per tag
pair `(A, B)` and the `Reversed<S>` blanket impl (in `distance::Reversed`)
picks up `(B, A)` automatically — the Rust analogue of Boost's
`core/reverse_dispatch.hpp` partial specialisation, done once at the
strategy-trait layer instead of per-algorithm.

## Who depends on this

`geometry-algorithm` (every free function dispatches through a default
strategy from here), `geometry-overlay` (`validity.rs` uses `AreaStrategy`/`WithinStrategy` directly).
