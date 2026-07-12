//! End-to-end tests for `#[derive(Point)]`.
//!
//! Each case derives `Point` on a struct, then runs the canonical
//! Cartesian 3-4-5 distance check through `geometry_algorithm::distance`
//! to prove the generated `impl` is actually wired into the kernel.
//!
//! The 3-4-5 triangle is the same reference value used by the Boost
//! Pythagoras tests at `geometry/test/strategies/pythagoras.cpp:50-66`
//! and by T23's own `distance` smoke test.

use geometry_algorithm::distance;
use geometry_derive::Point;

// ---------------------------------------------------------------------
// Case 1 — explicit cs + scalar (the canonical 3-4-5 case).
// ---------------------------------------------------------------------

#[derive(Default, Point)]
#[geometry(cs = "Cartesian", scalar = "f64")]
struct MyPoint {
    x: f64,
    y: f64,
}

#[allow(clippy::float_cmp, reason = "3-4-5 is exact in IEEE-754 f64.")]
#[test]
fn derive_produces_working_point() {
    let a = MyPoint { x: 0.0, y: 0.0 };
    let b = MyPoint { x: 3.0, y: 4.0 };
    assert_eq!(distance(&a, &b), 5.0);
}

// ---------------------------------------------------------------------
// Case 2 — defaults (no `#[geometry(...)]` attribute).
// ---------------------------------------------------------------------

#[derive(Default, Point)]
struct DefaultsPoint {
    x: f64,
    y: f64,
}

#[allow(clippy::float_cmp, reason = "1-1 case is exact in IEEE-754 f64.")]
#[test]
fn defaults_to_cartesian_f64() {
    let a = DefaultsPoint { x: 0.0, y: 0.0 };
    let b = DefaultsPoint { x: 3.0, y: 4.0 };
    assert_eq!(distance(&a, &b), 5.0);
}

// ---------------------------------------------------------------------
// Case 3 — 3D struct, three named fields → DIM = 3.
// ---------------------------------------------------------------------

#[derive(Default, Point)]
#[geometry(cs = "Cartesian", scalar = "f64")]
struct MyPoint3D {
    x: f64,
    y: f64,
    z: f64,
}

#[test]
fn three_dimensions_works() {
    // 1² + 2² + 2² = 9, √9 = 3 exactly in IEEE-754.
    let a = MyPoint3D {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let b = MyPoint3D {
        x: 1.0,
        y: 2.0,
        z: 2.0,
    };
    let d = distance(&a, &b);
    assert!((d - 3.0).abs() < 1e-12);
}

// ---------------------------------------------------------------------
// Case 4 — f32 scalar via the `scalar = "f32"` key.
// ---------------------------------------------------------------------

#[derive(Default, Point)]
#[geometry(cs = "Cartesian", scalar = "f32")]
struct F32Point {
    x: f32,
    y: f32,
}

#[allow(clippy::float_cmp, reason = "3-4-5 is exact in IEEE-754 f32.")]
#[test]
fn f32_scalar_works() {
    let a = F32Point { x: 0.0, y: 0.0 };
    let b = F32Point { x: 3.0, y: 4.0 };
    let d = distance(&a, &b);
    assert_eq!(d, 5.0_f32);
}

// ---------------------------------------------------------------------
// trybuild — compile-fail fixture for an unknown coordinate-system name.
// ---------------------------------------------------------------------

#[test]
fn ui() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/unknown_cs.rs");
}
