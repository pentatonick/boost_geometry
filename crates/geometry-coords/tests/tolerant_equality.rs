//! `CoordinateScalar::tolerant_eq` — equality the way the kernel means it.
//!
//! Counterpart to `boost::geometry::math::equals`
//! (`boost/geometry/util/math.hpp`) under `equals_default_policy`. Boost
//! selects on the type: a floating-point type compares within one epsilon of
//! `greatest(abs(a), abs(b), T(1))`, and every other type compares exactly.
//! This is not a convenience — the side predicate calls three points collinear
//! when any two of them are equal by this rule, so the width of the tolerance
//! is the width of the whole kernel's notion of a coincident point.

use geometry_coords::{CoordinateScalar, Rational};

/// The scale factor is `greatest(|a|, |b|, 1)`, so at a large coordinate the
/// tolerance grows with the coordinate and near zero it does not shrink below
/// an absolute epsilon.
///
/// The two directions are what make this a real test. At 1e6 the epsilon
/// admits about 2.22e-10 while one ulp is about 1.16e-10, so the tolerance is
/// worth a single ulp and no more — one step apart is one coordinate, two
/// steps apart is two. An absolute `f64::EPSILON` comparison would reject
/// both. Near zero the `T(1)` floor admits 2.22e-16, which a purely relative
/// comparison would reject. So the boundary is pinned from both sides at both
/// magnitudes.
#[test]
fn the_float_tolerance_scales_with_the_larger_magnitude() {
    let large = 1.0e6_f64;
    let one_step = f64::from_bits(large.to_bits() + 1);
    let two_steps = f64::from_bits(large.to_bits() + 2);
    assert!(
        CoordinateScalar::tolerant_eq(large, one_step),
        "at 1e6 the epsilon is worth an ulp, so one step is the same coordinate"
    );
    assert!(
        !CoordinateScalar::tolerant_eq(large, two_steps),
        "two steps is 2.33e-10, past the 2.22e-10 the epsilon allows"
    );
    assert!(
        !CoordinateScalar::tolerant_eq(1.0_f64, large),
        "and the tolerance never grows enough to merge unrelated coordinates"
    );

    // Near zero the factor floors at 1, so the tolerance is absolute.
    let small = 1.0e-12_f64;
    assert!(CoordinateScalar::tolerant_eq(small, small + f64::EPSILON));
    assert!(!CoordinateScalar::tolerant_eq(small, small * 2.0));

    // Exactly zero against a value the absolute floor does not reach.
    assert!(CoordinateScalar::tolerant_eq(0.0_f64, f64::EPSILON));
    assert!(!CoordinateScalar::tolerant_eq(0.0_f64, 1.0e-15));
}

/// A value that is not finite is equal only to itself bit for bit.
///
/// `infinity == infinity` is true before the tolerance is ever consulted, so
/// it stays equal; every other pairing involving a non-finite value would
/// otherwise reach a subtraction that yields `NaN` or `inf`, and a comparison
/// against `NaN` is false in a way that says nothing. Boost's `equals` has the
/// same shape — the exact test comes first — and `NaN` is equal to nothing,
/// itself included.
#[test]
fn a_non_finite_value_equals_only_itself() {
    assert!(CoordinateScalar::tolerant_eq(f64::INFINITY, f64::INFINITY));
    assert!(!CoordinateScalar::tolerant_eq(
        f64::INFINITY,
        f64::NEG_INFINITY
    ));
    assert!(!CoordinateScalar::tolerant_eq(f64::INFINITY, f64::MAX));
    assert!(!CoordinateScalar::tolerant_eq(f64::MAX, f64::INFINITY));
    assert!(!CoordinateScalar::tolerant_eq(f64::NAN, f64::NAN));
    assert!(!CoordinateScalar::tolerant_eq(f64::NAN, 0.0));
}

/// `f32` carries its own epsilon, which is a great deal wider than `f64`'s.
///
/// The same absolute gap is one thing at `f32` and another at `f64`; using one
/// type's epsilon for the other is exactly the mistake the per-type
/// implementation exists to prevent.
#[test]
fn the_float_tolerance_is_the_types_own_epsilon() {
    let gap = 1.0e-8_f32;
    assert!(
        CoordinateScalar::tolerant_eq(1.0_f32, 1.0_f32 + gap),
        "1e-8 is well inside f32's epsilon of about 1.19e-7"
    );
    assert!(
        !CoordinateScalar::tolerant_eq(1.0_f64, 1.0_f64 + f64::from(gap)),
        "the same gap is far outside f64's epsilon of about 2.2e-16"
    );
}

/// An integer coordinate carries no rounding error, so equality is exact.
///
/// Boost reaches `equals` through a type dispatch, not a numeric one: the
/// non-floating-point specialisation is `a == b` with no epsilon anywhere. Two
/// adjacent integers are two different coordinates however large they are.
#[test]
fn an_integer_coordinate_compares_exactly() {
    assert!(CoordinateScalar::tolerant_eq(7_i32, 7_i32));
    assert!(!CoordinateScalar::tolerant_eq(7_i32, 8_i32));
    assert!(!CoordinateScalar::tolerant_eq(i64::MAX, i64::MAX - 1));
    assert!(CoordinateScalar::tolerant_eq(-5_i64, -5_i64));
}

/// A rational coordinate is exact for the same reason, and its equality is the
/// *reduced* one — `1/2` and `2/4` are one coordinate, while `1/3` and the
/// nearest neighbouring third are two.
///
/// This is the case a tolerance would get wrong in the other direction: two
/// rationals a hair apart are genuinely distinct, however small the hair, so
/// there is no rounding error for an epsilon to absorb and admitting one would
/// merge points the exact kernel is there to keep apart.
#[test]
fn a_rational_coordinate_compares_exactly_and_in_lowest_terms() {
    let half = Rational::new(1_i64, 2);
    assert!(CoordinateScalar::tolerant_eq(half, Rational::new(2_i64, 4)));
    assert!(CoordinateScalar::tolerant_eq(
        half,
        Rational::new(-50_i64, -100)
    ));
    assert!(!CoordinateScalar::tolerant_eq(
        half,
        Rational::new(1_i64, 3)
    ));

    // A difference of one part in a billion is still a difference.
    let a = Rational::new(1_i64, 1_000_000_000);
    assert!(!CoordinateScalar::tolerant_eq(a, Rational::ZERO));
    assert!(CoordinateScalar::tolerant_eq(
        Rational::<i64>::ZERO,
        Rational::new(0_i64, 5)
    ));
}
