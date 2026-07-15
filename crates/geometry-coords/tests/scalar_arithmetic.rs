//! The callable arithmetic surface exposed by `geometry_coords`.
//! `CoordinateScalar::abs` is defined for every scalar (integers included —
//! only `sqrt` is float-only, guarded by `Promote`), while `math` provides
//! the public `std`/`libm` dispatch boundary.

use geometry_coords::CoordinateScalar;
use geometry_coords::math::{
    abs, atan2, ceil, cos, hypot, ln, mul_add, rem_euclid, sin, sqrt, tan,
};

#[test]
fn integer_abs_is_callable() {
    assert_eq!(CoordinateScalar::abs(-3_i32), 3);
    assert_eq!(CoordinateScalar::abs(-3_i64), 3);
    assert_eq!(CoordinateScalar::abs(3_i32), 3);
}

#[test]
fn float_abs_matches_std() {
    assert!((CoordinateScalar::abs(-2.5_f64) - 2.5).abs() < 1e-15);
    assert!((CoordinateScalar::abs(-2.5_f32) - 2.5).abs() < 1e-6);
}

#[test]
fn integer_square_root_rejects_unpromoted_arithmetic() {
    let panic = std::panic::catch_unwind(|| CoordinateScalar::sqrt(4_i32));
    assert!(panic.is_err());
}

#[test]
fn public_math_dispatch_covers_robust_and_overlay_primitives() {
    assert!((sqrt(25.0_f64) - 5.0).abs() < f64::EPSILON);
    assert!((abs(-2.5_f64) - 2.5).abs() < f64::EPSILON);
    assert!((mul_add(2.0_f64, 3.0, 4.0) - 10.0).abs() < f64::EPSILON);
    assert!((hypot(3.0_f64, 4.0) - 5.0).abs() < f64::EPSILON);
    assert!((ceil(2.25_f64) - 3.0).abs() < f64::EPSILON);
    assert!((sin(core::f64::consts::FRAC_PI_2) - 1.0).abs() < 1e-15);
    assert!((cos(core::f64::consts::PI) + 1.0).abs() < 1e-15);
    assert!((atan2(1.0_f64, 0.0) - core::f64::consts::FRAC_PI_2).abs() < 1e-15);
    assert!((tan(core::f64::consts::FRAC_PI_4) - 1.0).abs() < 1e-15);
    assert!(ln(1.0_f64).abs() < f64::EPSILON);
    assert!((rem_euclid(-0.5_f64, 2.0) - 1.5).abs() < f64::EPSILON);
    assert!((rem_euclid(0.5_f64, 2.0) - 0.5).abs() < f64::EPSILON);
}

/// Boost's `test/util/math_abs.cpp` and `math_sqrt.cpp` exercise both native
/// floating widths. The additional primitives have no equivalent upstream
/// facade test, so exercise their `f32` dispatch directly through this crate's
/// public API, including both branches of Euclidean remainder normalization.
#[test]
fn public_math_dispatch_supports_f32() {
    let epsilon = 1e-6_f32;

    assert!((sqrt(25.0_f32) - 5.0).abs() < epsilon);
    assert!((abs(-2.5_f32) - 2.5).abs() < epsilon);
    assert!((mul_add(2.0_f32, 3.0, 4.0) - 10.0).abs() < epsilon);
    assert!((hypot(3.0_f32, 4.0) - 5.0).abs() < epsilon);
    assert!((ceil(2.25_f32) - 3.0).abs() < epsilon);
    assert!((sin(core::f32::consts::FRAC_PI_2) - 1.0).abs() < epsilon);
    assert!((cos(core::f32::consts::PI) + 1.0).abs() < epsilon);
    assert!((atan2(1.0_f32, 0.0) - core::f32::consts::FRAC_PI_2).abs() < epsilon);
    assert!((tan(core::f32::consts::FRAC_PI_4) - 1.0).abs() < epsilon);
    assert!(ln(1.0_f32).abs() < epsilon);
    assert!((rem_euclid(-0.5_f32, 2.0) - 1.5).abs() < epsilon);
    assert!((rem_euclid(0.5_f32, 2.0) - 0.5).abs() < epsilon);
}
