//! Trybuild harness for compile-fail fixtures under `tests/ui/`.
//!
//! Mirrors the spirit of
//! `boost/geometry/test/concepts/point_with_incorrect_dimension.cpp`:
//! constructing a point with the wrong number of coordinates must fail
//! at compile time, not produce silently-truncated data.

#[test]
fn ui() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/wrong_arity.rs");
}
