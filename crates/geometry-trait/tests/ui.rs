//! Trybuild harness for compile-fail fixtures under `tests/ui/`.
//!
//! Counterpart to the C++ "concept checker" tests in
//! `boost/geometry/test/concepts/*_checker.cpp` — except where the
//! C++ checker asserts at compile time via
//! `BOOST_CONCEPT_ASSERT`, we assert via `trybuild` snapshots of
//! the actual compiler diagnostic.

#[test]
fn ui() {
    let t = trybuild::TestCases::new();
    // Registering a `pass` fixture alongside the `compile_fail` one
    // forces trybuild to invoke `cargo build` rather than `cargo
    // check` (see `trybuild::cargo::build_test`: it picks `build`
    // iff `has_pass`). We need full codegen so the
    // `const { assert!(D < DIM) }` guard inside `Xy::get` is
    // evaluated and the out-of-range fixture actually errors.
    t.pass("tests/ui/get_in_range.rs");
    t.compile_fail("tests/ui/get_out_of_range.rs");
    t.compile_fail("tests/ui/indexed_out_of_range.rs");

    // A read-only point (getter only, no `set`) is valid after the
    // KC1.T1 `Point` / `PointMut` split — it satisfies `check_point`.
    t.pass("tests/ui/point_no_set.rs");

    // Concept-check fail fixtures — mirror Boost's
    // `test/concepts/point_without_*.cpp` and
    // `test/concepts/point_with_incorrect_dimension.cpp`.
    t.compile_fail("tests/ui/point_no_scalar.rs");
    t.compile_fail("tests/ui/point_no_dim.rs");
    t.compile_fail("tests/ui/point_wrong_dim.rs");

    // KC1.T3 — a read-only `Point` (no `PointMut`) handed to a
    // materialiser (`segment_start`) fails with the `PointMut` plate.
    t.compile_fail("tests/ui/ro_point_no_setter.rs");

    // KC5.T2 — a wrong angle-unit CS (`Spherical<f64>` / `Geographic<f64>`
    // instead of `Degree` / `Radian`) is rejected by the `AngleUnit`
    // bound. Mirrors Boost's `point_{spherical,geographic}_with_wrong_units.cpp`.
    t.compile_fail("tests/ui/spherical_wrong_unit.rs");
    t.compile_fail("tests/ui/geographic_wrong_unit.rs");
}
