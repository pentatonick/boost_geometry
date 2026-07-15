# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.8](https://github.com/pentatonick/boost_geometry/compare/geometry-overlay-v0.0.7...geometry-overlay-v0.0.8) - 2026-07-15

### Added

- complete parity tier one algorithms
- complete parity tier zero algorithms
- complete policy parity
- complete relate and buffer strategy parity
- expose expand, buffer, union, relate, and _with strategy variants

### Fixed

- cover algorithm parity edge cases
- harden overlay edge cases

### Other

- generate the feature table from pub-use tags
- apply clippy lints and tighten float assertions
- close remaining executable coverage gaps
- expand parity edge coverage
- Restore no_std support after algorithms port
- Complete algorithms parity port

## [0.0.7](https://github.com/pentatonick/boost_geometry/compare/geometry-overlay-v0.0.6...geometry-overlay-v0.0.7) - 2026-07-14

### Fixed

- drop redundant license-file key from crate manifests

### Other

- close category-D coverage gaps via public API

## [0.0.6](https://github.com/pentatonick/boost_geometry/compare/geometry-overlay-v0.0.5...geometry-overlay-v0.0.6) - 2026-07-13

### Other

- decouple crate versions from workspace.package for release-plz
- revert to per-crate intra-workspace version requirements
- inherit intra-workspace crate versions from workspace.dependencies
- anchor crate descriptions to Boost.Geometry port
- release v0.0.5

## [0.0.5](https://github.com/pentatonick/boost_geometry/compare/geometry-overlay-v0.0.4...geometry-overlay-v0.0.5) - 2026-07-12

### Other

- release v0.0.5

## [0.0.4](https://github.com/pentatonick/boost_geometry/compare/geometry-overlay-v0.0.3...geometry-overlay-v0.0.4) - 2026-07-12

### Other

- remove unnecessary clones from overlay, WKT parser, and equals

## [0.0.3](https://github.com/pentatonick/boost_geometry/compare/geometry-overlay-v0.0.2...geometry-overlay-v0.0.3) - 2026-07-12

### Other

- add crates.io keywords and categories to every crate
- regenerate sub-crate READMEs from their lib.rs doc comments

## [0.0.2](https://github.com/pentatonick/boost_geometry/compare/geometry-overlay-v0.0.1...geometry-overlay-v0.0.2) - 2026-07-12

### Other

- post-publish README cleanup; ship BSL-1.0 license with crates
