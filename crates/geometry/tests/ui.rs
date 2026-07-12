//! Trybuild harness for the facade crate's compile-fail fixtures.
//!
//! Mirrors the per-crate `tests/ui.rs` harnesses (see
//! `geometry-trait/tests/ui.rs`); this one pins the facade-layer
//! diagnostics that prove the KC1 `Point` / `PointMut` split holds
//! through the public `boost_geometry::*` surface.

#[test]
fn ui() {
    let t = trybuild::TestCases::new();

    // M-KC1 — a read-only point handed to `envelope` (which must
    // construct a `Box<P>`) fails with the `PointMut` plate.
    t.compile_fail("tests/ui/readonly_envelope.rs");
}
