# Rust 2024 Edition Migration Report

Date: 2026-07-12
Scope: entire workspace (19 crates, 242 `.rs` files, ~42k lines)
Verification: the full migration was rehearsed on a pristine copy of the
workspace with rustc/cargo 1.96.0 stable. Every finding below is measured,
not estimated.

## Verdict

The migration is small, mechanical, and low-risk: **6 source files need code
changes (23 macro fragment-specifier rewrites, all produced automatically by
`cargo fix --edition`), plus two manifest lines and one rustfmt line.** After
the flip, the whole workspace compiles with zero warnings and all 76 test
suites pass. The largest diff is formatting churn (150 files) from the 2024
style edition, which should be isolated in its own commit. Estimated effort:
about an hour.

## Required changes

### 1. Manifests (single-point change)

All 19 crates inherit `edition.workspace = true` and
`rust-version.workspace = true`, so only the root `Cargo.toml` changes:

```toml
[workspace.package]
edition      = "2024"   # was "2021"
rust-version = "1.85"   # was "1.78" — 1.85 is the minimum that supports edition 2024
```

`rust-toolchain.toml` pins `channel = "stable"` with no version, so no change
is needed there (installed stable is 1.96.0).

### 2. Macro fragment specifiers — the only source-code change

In edition 2024 the `:expr` macro fragment additionally matches `const { ... }`
blocks and `_` expressions. `cargo fix --edition` conservatively rewrites every
`:expr` to `:expr_2021` to preserve byte-identical matching. It produced 23
rewrites across 6 files:

| File | Rewrites |
|---|---|
| `crates/geometry-adapt/src/macros.rs` | 7 |
| `crates/geometry-model/src/macros.rs` | 8 |
| `crates/geometry-strategy/src/cartesian/distance_projected_point.rs` | 2 |
| `crates/geometry-strategy/src/cartesian/distance_pythagoras.rs` | 2 |
| `crates/geometry-trait/src/point.rs` | 2 |
| `crates/geometry-trait/src/segment.rs` | 2 |

**Recommendation: revert the `_2021` suffixes and keep plain `:expr`.**
Rationale:

- The 2024 matching is strictly *more* permissive, so for the exported macros
  (`point!`, `linestring!`, `polygon!`, `register_linestring!`, `register_ring!`,
  `register_polygon!`) this is non-breaking for downstream users — inputs that
  matched before still match, and `const { ... }` coordinate expressions or
  iterator closures additionally start working.
- The internal unroll macros (`impl_walk!`, `impl_sum_squares!`, `impl_recurse!`,
  `impl_write_dims!`) are only ever invoked with integer literals; the
  distinction cannot matter.
- `expr_2021` is deprecation-path syntax and reads as noise.

Either choice compiles and passes all tests; this is a style/API-policy call.

### 3. rustfmt

`rustfmt.toml` currently pins `edition = "2021"`; change to `"2024"`. That also
switches the *style edition* to 2024, which reformats **150 of 242 files** —
almost entirely import-sort order (2024 style version-sorts, placing uppercase
names like `DynKindMismatch` before lowercase ones). Two options:

- **Recommended:** accept it as a one-shot `cargo fmt` commit, separate from
  the semantic changes, so `git blame` damage is contained.
- Alternative: add `style_edition = "2021"` to keep the old formatting while
  parsing 2024 code, deferring the churn.

Observation (pre-existing, unrelated to the migration):
`imports_granularity = "Module"` in `rustfmt.toml` is a nightly-only option and
is silently ignored on stable (rustfmt prints a warning).

### 4. Clippy fallout from the MSRV bump (not from the edition)

Bumping `rust-version` to 1.85 unlocks MSRV-gated clippy suggestions. Measured:
0 warnings at `rust-version = "1.78"`, 4 at `"1.85"`, on identical code:

| Location | Lint | Fix |
|---|---|---|
| `crates/geometry-strategy/src/closest_points.rs:150` | `unnecessary_map_or` | `is_none_or` (stable 1.82) |
| `crates/geometry-strategy/src/geographic/distance_thomas.rs:186` | `manual_midpoint` | `f64::midpoint` (stable 1.85) |
| `crates/geometry-rtree/src/rtree.rs:393` | `manual_midpoint` | `usize::midpoint` (stable 1.85) |
| `crates/geometry-algorithm/tests/quickstart_spherical.rs:71` | `unnecessary_trailing_comma` | remove comma |

All four are auto-applicable with `cargo clippy --fix`.

## Verified non-issues

Every other Rust 2024 breaking change was checked and does not apply:

- **Everything unsafe-related** — the workspace sets `unsafe_code = "forbid"`
  and every crate additionally has `#![forbid(unsafe_code)]`. So `unsafe extern`
  blocks, `#[unsafe(no_mangle)]`-style attributes, `static mut` references,
  `unsafe_op_in_unsafe_fn`, and the newly-unsafe `std::env::set_var` are all
  moot: zero occurrences.
- **RPIT lifetime capture** (the headline 2024 change): 53 `-> impl Trait`
  sites across the trait/model/adapter crates compile clean under 2024 with no
  `use<...>` precise-capture annotations needed — `cargo fix` emitted none and
  there are no over-capture warnings.
- **`if let` / tail-expression temporary-scope changes**: no rewrites emitted;
  all tests pass, so no drop-order dependence exists.
- **Never-type fallback, `gen` keyword, `IntoIterator` for `Box<[T]>`, prelude
  `Future`/`IntoFuture` collisions, reserved string-literal prefixes**: zero
  occurrences.
- **Dependencies**: geo-types 0.7, nalgebra 0.32, syn 2, proc-macro2 1, quote 1,
  proj4rs 0.1, proj4wkt 0.1, libm 0.2, serde 1, serde_json 1, criterion 0.5,
  trybuild 1 — all build fine; none constrains against a 1.85 floor.
- **trybuild UI tests** (`crates/geometry-trait/tests/ui.rs`): pass unchanged
  under 2024 — the compile-error snapshots are unaffected.
- **CI / docs**: no `.github/` workflows exist, and no README or
  `docs/` file states the edition or MSRV, so nothing else to update.

## Optional, recommended alongside

- `resolver = "2"` → `"3"` in the workspace root. Edition 2024's default;
  enables MSRV-aware dependency resolution (cargo picks dependency versions
  compatible with `rust-version = "1.85"`). Independent, low risk.

## Migration plan

1. Root `Cargo.toml`: `edition = "2024"`, `rust-version = "1.85"`
   (optionally `resolver = "3"`); `rustfmt.toml`: `edition = "2024"`.
2. `cargo fix --edition --workspace --all-targets --all-features`, then decide
   the `:expr_2021` vs `:expr` policy (recommended: plain `:expr`).
3. `cargo clippy --fix` for the 4 MSRV-unlocked lints.
4. Gate: `cargo check --workspace --all-targets --all-features`,
   `cargo test --workspace --all-features`, `cargo clippy`, `cargo fmt --check`.
5. Separate commit: `cargo fmt` (the 150-file style-2024 reformat).

## Rehearsal evidence (pristine copy, rustc 1.96.0)

1. Baseline 2021 `cargo check --workspace --all-targets --all-features`: clean.
2. `cargo fix --edition`: 23 fixes in 6 files, nothing else.
3. Edition + MSRV flipped, `cargo check`: clean, zero warnings.
4. `cargo test --workspace --all-features`: 76/76 suites pass, 0 failures
   (includes doctests and trybuild UI tests).
5. `cargo clippy --all-targets --all-features`: 4 warnings, all MSRV-unlock.
6. `cargo fmt` (style 2024): 150 files reformatted; `--check` clean afterwards.
