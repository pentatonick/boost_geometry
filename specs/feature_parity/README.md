# Feature Parity Map — Boost.Geometry (C++) → Rust port

A breadcrumb map of **every** feature area in the C++ Boost.Geometry library and
its status in this Rust crate workspace. For each area: what is **ported**, what
is **missing**, and a **file reference** into the original C++ tree so the code
can be found again.

## Coordinates of the two trees

- **C++ original** root: `/Users/discipline/Development/geometry/geometry`
  - headers: `include/boost/geometry/`
  - tests (parity oracles): `test/`
- **Rust port** root: `/Users/discipline/Development/Rust/boost_geometry`
  - crates: `crates/`
- **Migration specs** (authoritative order & rationale):
  `/Users/discipline/Development/geometry/specs/`
  - `phase_00-overview.md` … `phase_08-*.md` (plan-of-plans)
  - `FUTURE_ITERATIONS.md` (catalogued deferrals)
  - `cross-cutting-status.md` (what landed / what remains)

## Status legend

| Mark | Meaning |
| --- | --- |
| ✅ | Ported (has a Rust home; may be partial — see notes) |
| ◑ | Partial — some of the area ported, some missing |
| ❌ | Missing — should be ported, not done yet |
| 🚫 | **Won't port** — deliberately out of scope (Rust idiom replaces it, or C++-ecosystem-only). Never becomes ❌. |

## How to execute the port

**[`PORTING_INSTRUCTIONS.md`](./PORTING_INSTRUCTIONS.md)** — the step-by-step
method for porting a feature so it matches the existing code's conventions
(C++-origin breadcrumbs, default+`_with` strategy shape, oracle tests, no_std
gating, divergence docs). Read it before writing any port.

## Current progress — 2026-07-14

**The algorithms and R-tree milestones are complete.** Every public entry header mapped in
[`algorithms.md`](./algorithms.md) now has a Rust home and is reachable through
the `boost_geometry` public facade. Public-facade integration tests cover the
entries and the workspace passes its full test suite, Clippy, strict rustdoc,
formatting, and `libm` no-default-features checks.

The completed algorithms scope includes:

- standalone entries such as `perimeter`, `comparable_distance`, `expand`,
  `covered_by`, and `merge_elements`;
- all four Cartesian polygon Boolean operations, including holes,
  containment, colocated vertices, and shared edges;
- Cartesian buffering for single and homogeneous multi geometry kinds, with
  signed areal offsets and all five strategy roles;
- Cartesian DE-9IM consumers for point, linestring, and polygon pairs, plus
  inter-ring and inter-member areal validity checks; and
- Thomas, Vincenty, and Karney direct/inverse formula support, differential
  quantities, meridian/vertex formulas, and geodesic segment intersection.

The ✅ algorithms status means the complete mapped **public entry surface** is
present. Broader specialization work—linear/pointlike set-operation output,
box/multi/collection relate dispatch, non-areal validity kinds, and
spherical/geographic buffers—remains explicitly tracked in
[`overlay.md`](./overlay.md) and [`strategies.md`](./strategies.md).

### Coverage closure

The algorithms port finished with a public-API-first coverage audit against the
local Boost.Geometry C++ tests. `cargo llvm-cov --workspace --all-features`
improved from the pre-audit baseline to:

| Metric | Baseline | Final | Change |
| --- | ---: | ---: | ---: |
| Lines | 18,700 / 19,623 (95.30%) | 19,438 / 19,767 (98.34%) | +3.04 pp |
| Functions | 2,345 / 2,436 (96.26%) | 2,442 / 2,448 (99.75%) | +3.49 pp |
| Regions | 31,928 / 33,590 (95.05%) | 33,024 / 33,868 (97.51%) | +2.46 pp |

The default-feature audit is effectively the same: 98.33% lines, 99.75%
functions, and 97.51% regions. Tests use the `boost_geometry` facade or another
documented public crate API wherever that path exists. Private unit tests were
used only for helpers with no public route, such as formatter expansion and
test-only metric machinery.

The audit also found and fixed three behavior/API defects rather than merely
raising counters: rational parsing now accepts the valid `"1."` spelling,
point equality/intersection inspect the fourth ordinate instead of stopping at
three, and the GeoJSON/WKB writer traits are publicly re-exported so downstream
types can implement the bounds advertised by the public writer functions.

A post-port no-std audit also restored every previously supported crate in the
generated support table. Robust arithmetic and Cartesian overlay now dispatch
their fused and transcendental floating-point operations through
`geometry_coords::math`, selecting the standard library or `libm` without
disabling the algorithms.

### R-tree closure

The [`index/` R-tree map](./index_rtree.md) is now closed through the public
`boost_geometry::rtree` facade. Public-API-first tests cover all seven Boost
box predicates, logical `and`/`not` and `satisfies`, repeated condensing
removal/root collapse under the default, linear, quadratic, and R\* split
policies, count/range removal, bounds, clone, iteration/extend/clear, and serde
round trips. Existing scan-oracle, exact-distance, lazy-iterator, and allocation
budget suites remain green.

The crate's `no_std` claim now covers its dependency graph rather than relying
on dependencies' default `std` features. Both
`cargo build -p geometry-rtree --no-default-features` and the same build with
`--features serde` pass with `alloc` and the libm-backed coordinate kernel.

The final `RTREE_PROFILING.md` default remains
`AsymmetricRStarSplit<6, 2, 12, 4, 4, 4>`. The compact 18-row Callgrind matrix
was rerun twice after the API closure: insertion build and all query-only kNN
rows were instruction-identical, bulk build changed by +0.0003%, and both range
totals stayed within 0.05% of the pre-port baseline—well inside the retained 1%
gate. R\* forced reinsertion and the experimental k-means splitter are recorded
as deliberate, revisitable non-ports rather than left as ambiguous missing
work.

Exactly six compiled functions remain uncalled in the all-features report:

- two GeoJSON UTF-8 error closures whose byte slices are derived from an
  already-valid Rust `&str` (the invalid state cannot be constructed through
  safe input);
- one Karney fallback after two non-empty seed loops, which always establish a
  best candidate; and
- three R-tree test-instrumentation closures behind packed-leaf group branches,
  while the current test `LeafProbe<Vec<T>>` representation always reports no
  packed groups.

The remaining uncovered lines/regions are predominantly those impossible
branches, defensive invariant failures, assertion failure arms, and
const-generic monomorphizations where LLVM counts unselected match arms once
per instantiation. Reaching 100% would therefore require deleting safeguards,
constructing invalid states, or reshaping code solely for the metric.

### Revisitable assumptions

These assumptions define the recorded ✅ and the coverage ceiling. Revisit them
if the scope or implementation changes:

1. **“Algorithms complete” means the mapped public entry surface.** It does not
   silently include the broader pair/coordinate-system specializations still
   listed in `overlay.md` and `strategies.md`.
2. **The local C++ checkout is the parity oracle.** Tests under
   `/Users/discipline/Development/geometry/geometry/test` are treated as the
   authoritative cases when C++ coverage exists; Rust-specific error and
   invariant cases fill genuine gaps.
3. **Coverage is workspace-wide.** The percentages above include supporting
   crates and tests, not only files named `algorithm` or `algorithms`.
4. **“Maxed out” means maximum meaningful executable coverage.** No production
   control flow is rewritten solely to appease LLVM's generic-instantiation
   accounting, and passing tests do not deliberately enter panic-only failure
   callbacks.
5. **The six uncalled functions are unreachable under current invariants.** In
   particular, revisit the three R-tree branches if leaves gain stored packed
   groups, and revisit the parser/fallback classifications if their input or
   seed construction changes.
6. **The no-std table records build support, using `libm` when required.** It
   does not claim that geographic strategies historically gated behind `std`
   expose those transcendental methods in a core-only build. Revisit that API
   boundary if geographic strategy execution becomes a no-std requirement.
7. **R-tree built-in relations are bounding-box relations.** They are exact for
   points/boxes and candidate filters for non-rectangular application values;
   revisit if `Indexable` exposes exact geometry.
8. **R\* forced reinsertion is deliberately omitted.** Profiling isolated and
   retained R\* split selection without reinsertion; revisit on a correctness
   gap or a representative workload that fails the 1% performance gate.
9. **R-tree removal condenses by reinserting values.** It preserves tree
   invariants but not Boost's original-level subtree reinsertion cost model;
   revisit if deletion throughput becomes a measured bottleneck.
10. **R-tree serde stores values, not node topology.** Loading reconstructs an
    STR-packed tree under the selected parameters so private layout is not a
    wire-format commitment.
11. **C++ inserter/adaptor and copy/destroy visitors are replaced by Rust
    language facilities.** `Extend`/`FromIterator`, lazy iterators, `Clone`,
    ownership, and `Drop` provide the corresponding public behavior.

## Index of maps (one file per C++ subsystem)

| Map file | C++ subsystem | Overall |
| --- | --- | --- |
| [`core.md`](./core.md) | `core/`, `util/`, `arithmetic/` | ◑ |
| [`geometries.md`](./geometries.md) | `geometries/` (models + adapters + register) | ◑ |
| [`algorithms.md`](./algorithms.md) | `algorithms/` public entry points | ✅ |
| [`strategies.md`](./strategies.md) | `strategies/`, `strategy/`, `formulas/` | ◑ |
| [`overlay.md`](./overlay.md) | `algorithms/detail/overlay,relate,buffer,...` | ◑ |
| [`io.md`](./io.md) | `io/` (WKT, DSV, SVG) + geo I/O | ◑ |
| [`index_rtree.md`](./index_rtree.md) | `index/` (R-tree) | ✅ |
| [`srs_projections.md`](./srs_projections.md) | `srs/` (CRS, projections, transforms) | ◑ |
| [`iterators_views.md`](./iterators_views.md) | `iterators/`, `views/` | 🚫 |
| [`policies.md`](./policies.md) | `policies/` (robustness, interrupt, compare) | ◑ |
| [`beyond_boost.md`](./beyond_boost.md) | Rust-only additions (no C++ origin) | — |

## Subsystem parity matrix

| Subsystem | ✅ ported | ◑ partial | ❌ missing | 🚫 won't port |
| --- | :---: | :---: | :---: | :---: |
| core / util / arithmetic | ● | ● | | ● |
| geometries (models/adapters/register) | ● | ● | ● | ● |
| algorithms (public entry points) | ● | | | |
| strategies / formulas | ● | ● | ● | |
| overlay / relate / buffer | ● | ● | ● | |
| I/O | ● | | ● | |
| index (R-tree) | ● | | | ● |
| srs / projections | ● | ● | ● | ● |
| iterators / views | | | | ● |
| policies | ● | ● | ● | ● |

(A ● in multiple columns means the subsystem spans states — see its map file for
the per-feature breakdown.)

## Port checklist — **SHOULD** be ported (progress)

Ranked highest-value first. Tick when done. Detail + C++ breadcrumbs in the
linked map.

- [x] **Set-operation public entries** — `intersection`, `r#union`,
  `difference`, `sym_difference` ([`algorithms.md`](./algorithms.md); pair
  coverage in [`overlay.md`](./overlay.md))
- [x] **`buffer` public entry** — Cartesian tag dispatch for single and
  homogeneous multi geometry kinds
  ([`algorithms.md`](./algorithms.md))
- [x] **Cartesian buffer completion** — non-convex/negative-distance engine,
  holes, linear ends, and the five Boost strategy roles
  ([`overlay.md`](./overlay.md), [`strategies.md`](./strategies.md))
- [ ] **Spherical/geographic buffer strategies**
  ([`overlay.md`](./overlay.md),
  [`strategies.md`](./strategies.md))
- [x] **Cartesian areal overlay engine** — interior rings, containment,
  colocations/shared edges, nested assembly, and all four Boolean operations
  ([`overlay.md`](./overlay.md))
- [ ] **Broader overlay pairs** — linear/pointlike operations and invalid
  self-intersecting input handling
  ([`overlay.md`](./overlay.md))
- [x] **relate public consumers** — `crosses`, `overlaps`, `touches`, `relate`,
  `relation` (Cartesian point/linestring/polygon pairs; broader pair coverage
  remains in [`overlay.md`](./overlay.md))
- [x] **`is_valid` public entry + failure categories** — ring, polygon, and
  per-member multi-polygon dispatch ([`algorithms.md`](./algorithms.md))
- [x] **Areal `is_valid` completion** — inter-ring and inter-member topology
  for rings, polygons, and multi-polygons
  ([`algorithms.md`](./algorithms.md), [`policies.md`](./policies.md))
- [ ] **Remaining `is_valid` geometry kinds** — pointlike/linear/box/collection
  dispatch ([`algorithms.md`](./algorithms.md))
- [x] **Geodesic direct formulas** (Thomas/Vincenty direct) + **Karney**
  ([`strategies.md`](./strategies.md))
- [x] **Geodesic segment intersection** (`gnomonic`, `sjoberg`) — unblocks
  geographic overlay ([`strategies.md`](./strategies.md))
- [x] **`perimeter`** — coordinate-system-family default + explicit strategy
  companion ([`algorithms.md`](./algorithms.md))
- [x] **`merge_elements`** — areal collection entry ([`algorithms.md`](./algorithms.md))
- [x] **`comparable_distance` / `expand` / `covered_by`** standalone entries
  ([`algorithms.md`](./algorithms.md))
- [ ] **DSV write** (`io/dsv/write.hpp`) — low priority ([`io.md`](./io.md))
- [x] **R-tree** — quadratic/R\*-split confirmation, remove visitor, and serialization
  ([`index_rtree.md`](./index_rtree.md))
- [ ] **Boost projections** — native ports of the 99 `srs/projections/proj/*`
  headers *if* `proj4rs` proves insufficient ([`srs_projections.md`](./srs_projections.md))
- [x] **register multi-\* macros** completion ([`geometries.md`](./geometries.md))

## Won't-port checklist — 🚫 **deliberately out of scope**

Do **not** convert these to ❌. They are settled decisions; a Rust idiom or
ecosystem choice replaces them.

- [x] 🚫 **`iterators/` — ALL of it, permanently.** `point_iterator`,
  `segment_iterator`, `closing_iterator`, `ever_circling_iterator`,
  `concatenate_iterator`, `flatten_iterator`, reverse. Rust std iterator
  adapters (`.chain()`, `.flatten()`, `.rev()`, `.cycle()`, `.enumerate()`)
  cover every case inline. **Never ported.** ([`iterators_views.md`](./iterators_views.md))
- [x] 🚫 **`views/` — ALL of it, permanently.** `box_view`, `segment_view`,
  `closeable_view`, `reversible_view`, `identity_view`, `enumerate_view`. Same
  reason. **Never ported.** ([`iterators_views.md`](./iterators_views.md))
- [x] 🚫 **C++ dispatch scaffolding** — `algorithms/dispatch/`,
  `not_implemented.hpp`, `core/tag_cast` mechanics. Rust trait dispatch replaces
  it. ([`algorithms.md`](./algorithms.md))
- [x] 🚫 **TMP plumbing** — `util/{tuples,sequence,type_traits,combine_if,
  compress_variant,transform_variant,bare_type,bounds}.hpp`. Subsumed by Rust
  generics. ([`core.md`](./core.md))
- [x] 🚫 **Boost-ecosystem adapters** — `geometries/adapted/{boost_polygon,
  boost_range,boost_fusion,boost_array}`. No Rust analogue; `geo`-types +
  `nalgebra` adapters serve the Rust ecosystem instead.
  ([`geometries.md`](./geometries.md), [`beyond_boost.md`](./beyond_boost.md))
- [x] 🚫 **Boost projection code as native Rust** (default stance) — delegated to
  `proj4rs`. Only revisit if `proj4rs` is insufficient (see open-work list
  above). ([`srs_projections.md`](./srs_projections.md))
- [x] 🚫 **`extensions/`** — Boost's experimental/unsupported tree
  (`extensions/gis`, `nsphere`, etc.). Out of scope. ([`beyond_boost.md`](./beyond_boost.md))
- [x] 🚫 **`segment_ratio` rescaling robustness** — replaced by adaptive-precision
  predicates by design ([`policies.md`](./policies.md),
  `overlay-robustness-decision.md`).

## How to read a breadcrumb

Each row cites the C++ header **relative to** `include/boost/geometry/` unless
prefixed with `test/` (relative to the C++ repo root). The Rust column cites the
crate + file **relative to** `crates/`. A missing feature has an empty/❌ Rust
cell but always keeps its C++ path so the source is findable.
