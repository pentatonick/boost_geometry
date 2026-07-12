# `geometry-algorithm`

**Layer 5 — algorithms.** Depends on `geometry-strategy`, `geometry-model` (transitively everything below). `#![no_std]` + `alloc`.

Mirrors `boost/geometry/algorithms/*.hpp` and the dispatch structs in
`boost/geometry/algorithms/dispatch/`.

## Purpose

The free functions users actually call. Each function is a strategy-less
default (resolves the right `geometry-strategy` impl for you) plus, where
relevant, a `_with` companion that takes an explicit strategy.

## Module inventory (34 modules)

| Category | Modules |
|---|---|
| Construction | `assign`, `convert`, `make`, `clear`, `append` |
| Measurement | `distance`, `length`, `area`, `closest_points`, `discrete_frechet`, `discrete_hausdorff` |
| Predicates | `within`, `intersects`, `disjoint`, `equals`, `is_empty`, `is_simple`, `is_convex` |
| Shape queries | `envelope`, `centroid`, `convex_hull`, `azimuth`, `num_geometries`, `num_interior_rings`, `num_points`, `num_segments` |
| Mutation / transform | `correct`, `reverse`, `unique`, `remove_spikes`, `simplify`, `densify`, `transform`, `line_interpolate` |
| Traversal | `for_each` |
| Dynamic-kind wrappers | `dyn_area`, `dyn_distance`, `dyn_envelope`, `dyn_length`, `dyn_within`, `dyn_error` (private modules; their public functions are re-exported at crate root) |

## Public surface (selected)

`distance`/`distance_with`/`comparable_distance`, `length`/`length_with`/`perimeter`/`ring_perimeter`,
`area`/`area_with`/`ring_area`/`box_area`/`multi_polygon_area`, `within`/`covered_by`,
`intersects`/`intersects_reversed`, `disjoint`/`disjoint_box_box`, `equals`,
`envelope`, `centroid`/`centroid_with`, `convex_hull`, `azimuth`/`azimuth_with`,
`correct`, `reverse`, `unique`, `remove_spikes`, `simplify`, `densify`,
`transform`, `line_interpolate`, `for_each_point`/`for_each_segment`,
`is_empty`, `is_simple`, `is_convex`,
`num_geometries`/`num_interior_rings`/`num_points`/`num_segments`,
`make_box`/`make_point`/`make_segment`, `convert`, `assign_values`, `clear`,
`discrete_frechet_distance`/`_with`, `discrete_hausdorff_distance`/`_with`.

**Dynamic-kind wrappers** (`area_dyn`, `distance_dyn`, `envelope_dyn`,
`length_dyn`, `within_dyn`) match-and-forward on a `DynGeometry`'s runtime
kind to the static implementation — dispatch stays monomorphic inside each
match arm. `DynKindMismatch` is the shared error type for a dyn-wrapper call
whose two operands carry incompatible kinds.

## What's *not* here

Anything that needs the overlay engine — `intersection`, `union`,
`difference`, `sym_difference`, `buffer`, `is_valid`, `relate`, `crosses`,
`overlaps`, `touches`, `point_on_surface` all live in `geometry-overlay`
instead, specifically to avoid a dependency cycle (`geometry-overlay`
already depends on this crate for `within`/`ring_area`). See
[architecture](../01-architecture.md).

## Who depends on this

`geometry-overlay` (uses `within`, `ring_area`), re-exported wholesale by
the `boost_geometry` facade as `boost_geometry::algorithm`.
