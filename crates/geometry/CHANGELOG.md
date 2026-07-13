# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
