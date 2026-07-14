//! The callable arithmetic surface of `CoordinateScalar`: `abs` is
//! defined for every scalar (integers included — only `sqrt` is
//! float-only, guarded by `Promote`).

use geometry_coords::CoordinateScalar;

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
