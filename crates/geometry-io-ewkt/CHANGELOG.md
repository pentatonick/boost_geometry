# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.10](https://github.com/pentatonick/boost_geometry/compare/geometry-io-ewkt-v0.0.9...geometry-io-ewkt-v0.0.10) - 2026-09-20

### Added

- *(coords)* add a round primitive and give every I/O crate a libm feature
- *(io-ewkt)* add EWKT reader and writer crate

### Fixed

- *(io-wkt)* emit re-parseable WKT for empty members and reject non-finite input

### Other

- *(io-wkb,io-wkt,io-ewkt)* sweep every prefix and pin the untested contracts
- *(coords,io-wkt,io-ewkt)* close the eight lines the patch left uncovered
- *(io-ewkt)* pin the new crate to the workspace's 0.0.9 release
- *(io-ewkt)* cover the typed parsers outside their doc examples
