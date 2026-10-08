//! The callable arithmetic surface exposed by `geometry_coords`.
//! `CoordinateScalar::abs` is defined for every scalar (integers included —
//! only `sqrt` is float-only, reached through `Measure`), while `math`
//! provides the public `std`/`libm` dispatch boundary.

use core::cmp::Ordering;

use geometry_coords::math::{
    abs, acos, asin, atan2, ceil, cos, hypot, ln, mul_add, rem_euclid, round, sin, sqrt, tan,
};
use geometry_coords::{CoordinateScalar, Rational};

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

/// An integer is measured in `f64` and converted back the way Boost's
/// `numeric_cast` does: truncated toward zero, saturated at the range.
#[test]
fn integer_measures_are_f64_and_convert_back_truncated() {
    let measure: f64 = 7_i32.to_measure();
    assert_eq!(measure, 7.0);
    assert_eq!(i64::MAX.to_measure(), 9.223_372_036_854_776e18);
    assert_eq!(i32::from_measure(2.9), 2);
    assert_eq!(i32::from_measure(-2.9), -2);
    assert_eq!(i32::from_measure(1e20), i32::MAX);
    assert_eq!(f64::from_measure(-2.9), -2.9);
}

/// The integer side and turn tests are exact across the whole range of the
/// type, where the products need more than twice its width.
#[test]
fn integer_cross_and_dot_signs_are_exact_at_the_extremes() {
    let (low, high) = (i64::MIN, i64::MAX);
    // u = v − (0, 1) at the full width: u × v = −(2^64 − 1).
    assert_eq!(
        i64::cross_sign((low, low), (high, high), (low, low), (high, high - 1)),
        Some(Ordering::Less)
    );
    assert_eq!(
        i64::cross_sign((low, low), (high, high), (high, high), (low, low)),
        Some(Ordering::Equal)
    );
    assert_eq!(
        i64::dot_sign((low, low), (high, high), (high, high), (low, low)),
        Some(Ordering::Less)
    );
    assert_eq!(
        i64::dot_sign((0, 0), (high, 0), (0, 0), (0, high)),
        Some(Ordering::Equal)
    );
    // One unit of area at products near 2^60, which `f64` rounds away.
    let n = 1_073_741_822_i32;
    assert_eq!(
        i32::cross_sign((0, 0), (n + 1, n), (0, 0), (n, n - 1)),
        Some(Ordering::Less)
    );
    assert_eq!(
        i32::cross_sign(
            (i32::MIN, i32::MIN),
            (i32::MAX, i32::MAX),
            (i32::MIN, i32::MIN),
            (i32::MIN, i32::MAX)
        ),
        Some(Ordering::Greater)
    );
}

/// A float evaluates the same products in itself, and a NaN has no sign.
#[test]
fn float_cross_and_dot_signs_evaluate_in_the_float() {
    assert_eq!(
        f64::cross_sign((0.0, 0.0), (1.0, 0.0), (0.0, 0.0), (0.0, 1.0)),
        Some(Ordering::Greater)
    );
    assert_eq!(
        f64::dot_sign((0.0, 0.0), (1.0, 0.0), (0.0, 0.0), (-1.0, 0.0)),
        Some(Ordering::Less)
    );
    assert_eq!(
        f64::cross_sign((0.0, 0.0), (f64::NAN, 0.0), (0.0, 0.0), (0.0, 1.0)),
        None
    );
}

/// `direction_code` is Boost's: beyond `b`, short of it, level with it, or
/// no direction at all. A float rounds the perpendicular's equation as Boost
/// does, so a point `4e-12` from `b` at `6e5` is level with it, as in Boost
/// (`aed7bc3`), though its exact dot product is positive; an integer is
/// exact.
#[test]
fn direction_code_follows_boosts_perpendicular_line() {
    assert_eq!(
        f64::direction_code((0.0, 0.0), (1.0, 0.0), (2.0, 5.0)),
        Ordering::Greater
    );
    assert_eq!(
        f64::direction_code((0.0, 0.0), (1.0, 0.0), (0.5, 5.0)),
        Ordering::Less
    );
    assert_eq!(
        f64::direction_code((0.0, 0.0), (1.0, 0.0), (1.0, 5.0)),
        Ordering::Equal
    );
    assert_eq!(
        f64::direction_code((1.0, 1.0), (1.0, 1.0), (7.0, 5.0)),
        Ordering::Equal
    );

    let a = (15_682.546_124_542_228, 174_769.657_699_793_93);
    let b = (-630_679.312_290_246_7, 23_817.278_083_611_003);
    let c = (-630_679.312_290_246_7, 23_817.278_083_611);
    assert_eq!(f64::dot_sign(a, b, b, c), Some(Ordering::Greater));
    assert_eq!(f64::direction_code(a, b, c), Ordering::Equal);

    assert_eq!(
        i64::direction_code((0, 0), (i64::MAX, 0), (i64::MIN, 1)),
        Ordering::Less
    );
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
    assert!((asin(1.0_f64) - core::f64::consts::FRAC_PI_2).abs() < 1e-15);
    assert!((acos(-1.0_f64) - core::f64::consts::PI).abs() < 1e-15);
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
    assert!((asin(1.0_f32) - core::f32::consts::FRAC_PI_2).abs() < epsilon);
    assert!((acos(-1.0_f32) - core::f32::consts::PI).abs() < epsilon);
    assert!(ln(1.0_f32).abs() < epsilon);
    assert!((rem_euclid(-0.5_f32, 2.0) - 1.5).abs() < epsilon);
    assert!((rem_euclid(0.5_f32, 2.0) - 0.5).abs() < epsilon);
}

/// `round` promises "halfway cases away from zero", which is the rule that
/// separates it from the even-biased rounding a reader may assume: under
/// banker's rounding `2.5` would land on `2`, not `3`. Pin the tie on both
/// sides of zero for both native widths — every tie here is exact in binary,
/// so the comparisons are exact too.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "every result here is an exact integer-valued float"
)]
fn round_breaks_ties_away_from_zero() {
    assert_eq!(round(2.5_f64), 3.0);
    assert_eq!(round(-2.5_f64), -3.0);
    assert_eq!(round(3.5_f64), 4.0);
    assert_eq!(round(0.5_f64), 1.0);
    assert_eq!(round(-0.5_f64), -1.0);
    assert_eq!(round(2.25_f64), 2.0);

    assert_eq!(round(2.5_f32), 3.0);
    assert_eq!(round(-2.5_f32), -3.0);
    assert_eq!(round(3.5_f32), 4.0);
    assert_eq!(round(0.5_f32), 1.0);
    assert_eq!(round(-0.5_f32), -1.0);
    assert_eq!(round(2.25_f32), 2.0);
}

/// A scalar without infinities is always finite, and an exact one has no
/// rounding to hide a parallel pair behind: the trait's defaults, which
/// integers and rationals keep.
#[test]
fn exact_scalars_are_finite_and_never_nearly_parallel() {
    assert!(CoordinateScalar::is_finite(i32::MAX));
    assert!(CoordinateScalar::is_finite(Rational::new(1_i64, 3)));
    assert!(!i32::nearly_parallel((0, 0), (4, 0), (0, 1), (4, 1)));
    let r = |n: i64| Rational::from_integer(n);
    assert!(!Rational::<i64>::nearly_parallel(
        (r(0), r(0)),
        (r(4), r(0)),
        (r(0), r(1)),
        (r(4), r(1)),
    ));
}

/// A rational measures as itself, and is bounded by its integer's range
/// over one, as `util/rational.hpp:141-151` specializes `util::bounds`.
#[test]
fn rational_measures_as_itself_within_its_integers_bounds() {
    let third = Rational::new(1_i64, 3);
    assert_eq!(Rational::from_measure(third.to_measure()), third);
    assert_eq!(Rational::<i64>::highest(), Rational::from_integer(i64::MAX));
    assert_eq!(Rational::<i64>::lowest(), Rational::from_integer(i64::MIN));
}

/// A scalar type `numeric::bounds` does not know is bounded by `T()`
/// both ways: the trait's default `highest` and `lowest`.
#[test]
fn an_unknown_scalar_is_bounded_by_zero() {
    #[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
    struct Units(i32);
    impl core::ops::Add for Units {
        type Output = Self;
        fn add(self, other: Self) -> Self {
            Self(self.0 + other.0)
        }
    }
    impl core::ops::Sub for Units {
        type Output = Self;
        fn sub(self, other: Self) -> Self {
            Self(self.0 - other.0)
        }
    }
    impl core::ops::Mul for Units {
        type Output = Self;
        fn mul(self, other: Self) -> Self {
            Self(self.0 * other.0)
        }
    }
    impl core::ops::Div for Units {
        type Output = Self;
        fn div(self, other: Self) -> Self {
            Self(self.0 / other.0)
        }
    }
    impl core::ops::Neg for Units {
        type Output = Self;
        fn neg(self) -> Self {
            Self(-self.0)
        }
    }
    impl CoordinateScalar for Units {
        const ZERO: Self = Self(0);
        const ONE: Self = Self(1);
        type Measure = f64;
        fn to_measure(self) -> f64 {
            f64::from(self.0)
        }
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the test scalar truncates as an integer coordinate does"
        )]
        fn from_measure(measure: f64) -> Self {
            Self(measure as i32)
        }
        fn sqrt(self) -> Self {
            unreachable!("not exercised")
        }
        fn abs(self) -> Self {
            Self(self.0.abs())
        }
        fn tolerant_eq(self, other: Self) -> bool {
            self == other
        }
    }
    assert_eq!(Units::highest(), Units(0));
    assert_eq!(Units::lowest(), Units(0));
}
