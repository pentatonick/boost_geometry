# Contributing to boost_geometry

Thanks for your interest! Issues and pull requests are welcome.

## Building and testing

The workspace pins its toolchain in `rust-toolchain.toml` (MSRV 1.85,
edition 2024); `rustup` picks it up automatically.

```sh
cargo build --workspace --all-targets
cargo test --workspace
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
```

CI runs exactly those four commands (plus a release-mode build), so a
clean local run means a green PR.

## READMEs are generated

Every sub-crate `README.md` is generated from the `//!` doc comment in
its `src/lib.rs`, and the quick-start code block in the root `README.md`
is spliced from `crates/geometry/examples/parcel_buffer.rs`. Never edit
those blocks by hand — edit the rustdoc or the example, then run:

```sh
python3 crate_readme.py
```

CI fails if the generated files are out of sync.

## Design ground rules

- **Mirror Boost.Geometry.** Every public item cites the Boost C++
  header it ports. Read `docs/` first — the architecture, the
  tag-dispatch pattern, and the overlay engine are documented there.
- **`unsafe_code = "forbid"`** across the whole workspace.
- **Dependency spine.** Crates depend strictly downward (tags/coords →
  traits → models → strategies → algorithms → overlay → facade). No
  cycles, no reaching into another crate's internals.

## Commits and releases

- Commit messages follow Conventional Commits (`feat:`, `fix:`,
  `docs:`, `ci:`, `build:`).
- Releases are automated: [release-plz](https://release-plz.dev)
  opens a version-bump PR; merging it and pushing a `v*` tag publishes
  every crate to crates.io via GitHub Actions. Contributors never
  publish manually.

## License

By contributing, you agree that your contributions are licensed under
the [Boost Software License 1.0](LICENSE).
