//! Trybuild harness for compile-fail fixtures under `tests/ui/`.
//!
//! Pins the user-facing diagnostic produced by the silent-Cartesian
//! mitigation plate on `geometry_tag::SameAs` (T31). Counterpart to
//! the C++ concept-checker tests in
//! `boost/geometry/test/strategies/`, but for compiler diagnostics
//! rather than runtime asserts.

#[test]
fn ui() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/cartesian_only.rs");
}
