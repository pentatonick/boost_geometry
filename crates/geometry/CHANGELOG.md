# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.9](https://github.com/pentatonick/boost_geometry/compare/v0.0.8...v0.0.9) - 2026-09-05

### Added

- *(buffer)* a zero-width buffer of a polygon, which is not a no-op

### Fixed

- *(overlay)* a hole sharing an edge with the exterior is a self-intersection
- *(overlay)* distinguish the two ways multi-polygon members can be wrong

### Other

- *(readme)* regenerate the feature table for the multi-polygon operations

## [0.0.8](https://github.com/pentatonick/boost_geometry/compare/v0.0.7...v0.0.8) - 2026-07-15

### Added

- complete parity tier two algorithms
- complete parity tier one algorithms
- complete parity tier zero algorithms
- complete policy parity
- complete relate and buffer strategy parity
- expose expand, buffer, union, relate, and _with strategy variants

### Fixed

- cover algorithm parity edge cases
- preserve exact cardinal Karney results
- harden overlay edge cases

### Other

- generate the feature table from pub-use tags
- apply clippy lints and tighten float assertions
- close remaining executable coverage gaps
- expand parity edge coverage
- extend overlay edge coverage
- extend public algorithm edge coverage
- expand algorithm edge case coverage
- optimize polygon intersection predicates
- Complete R-tree feature parity
- Complete algorithms parity port

## [0.0.7](https://github.com/pentatonick/boost_geometry/compare/v0.0.6...v0.0.7) - 2026-07-14

### Fixed

- drop redundant license-file key from crate manifests

### Other

- reorder README badges (CI, Publish, Docs, Coverage, Crates, MSRV, License)

## [0.0.6](https://github.com/pentatonick/boost_geometry/compare/v0.0.5...v0.0.6) - 2026-07-13

### Other

- decouple crate versions from workspace.package for release-plz
- revert to per-crate intra-workspace version requirements
- add a generated no_std support table to the README
- inherit intra-workspace crate versions from workspace.dependencies
- upload coverage to Codecov and add badge
- anchor crate descriptions to Boost.Geometry port

## [0.0.5](https://github.com/pentatonick/boost_geometry/compare/v0.0.4...v0.0.5) - 2026-07-12

### Other

- add a derive-free variant of the quick-start
- make the quick-start fully bring-your-own, including the point type

## [0.0.4](https://github.com/pentatonick/boost_geometry/compare/v0.0.3...v0.0.4) - 2026-07-12

### Other

- remove unnecessary clones from overlay, WKT parser, and equals

## [0.0.3](https://github.com/pentatonick/boost_geometry/compare/boost_geometry-v0.0.2...boost_geometry-v0.0.3) - 2026-07-12

### Other

- make the crate publicly consumable end to end
- add crates.io keywords and categories to every crate
- use cargo add in the quick start so the version never goes stale
- serve the workspace README on the boost_geometry crates.io page

## [0.0.2](https://github.com/pentatonick/boost_geometry/compare/boost_geometry-v0.0.1...boost_geometry-v0.0.2) - 2026-07-12

### Other

- post-publish README cleanup; ship BSL-1.0 license with crates
