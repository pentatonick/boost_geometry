//! Public-facade parity tests for exact rational coordinates.

use core::any::TypeId;
use core::str::FromStr;

use boost_geometry::coords::{CoordinateScalar, ParseRationalError, Promote, Rational};
use boost_geometry::model::{Box as ModelBox, Point2D};
use boost_geometry::prelude::{Cartesian, box_area, within};

type Q = Rational<i64>;
type Q32 = Rational<i32>;
type P = Point2D<Q, Cartesian>;

/// `test/util/rational.cpp:32-47,130-145` — decimal/fraction parsing and
/// numeric conversion preserve the exact numerator and denominator.
#[test]
fn rational_parsing_and_conversion_are_exact() {
    assert_eq!(Q::from_str("0").unwrap(), Q::from_integer(0));
    assert_eq!(Q::from_str("1.").unwrap(), Q::from_integer(1));
    assert_eq!(Q::from_str("-1.5").unwrap(), Q::new(-3, 2));
    assert_eq!(Q::from_str("2.12345").unwrap(), Q::new(42_469, 20_000));
    assert_eq!(Q::from_str("3/2").unwrap(), Q::new(3, 2));
    assert!((Q::new(3, 4).to_f64() - 0.75).abs() < 1e-12);
}

/// `test/util/rational.cpp:41-47,132-145` — reduction, signs, arithmetic,
/// ordering, and presentation preserve exact rational values.
#[test]
fn rational_value_operations_are_exact() {
    let half = Q::new(2, 4);
    let third = Q::new(-1, -3);
    let negative_half = Q::new(1, -2);

    assert_eq!((half.numerator(), half.denominator()), (1, 2));
    assert_eq!(negative_half, -half);
    assert_eq!(half + third, Q::new(5, 6));
    assert_eq!(half - third, Q::new(1, 6));
    assert_eq!(half * third, Q::new(1, 6));
    assert_eq!(half / third, Q::new(3, 2));
    assert!(half > third);
    assert_eq!(half.partial_cmp(&third), Some(core::cmp::Ordering::Greater));
    assert_eq!(Q::default(), Q::from(0));
    assert_eq!(negative_half.abs(), half);
    assert_eq!(half.abs(), half);
    assert_eq!(Q::from_integer(7).to_string(), "7");
    assert_eq!(half.to_string(), "1/2");
}

/// The C++ rational wrapper delegates malformed-input handling to its stream
/// parser; these cases exercise the Rust parser's documented error contract.
#[test]
fn rational_parse_errors_are_specific() {
    assert_eq!(Q::from_str(""), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str(" 1"), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str("1/2/3"), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str("x/2"), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str("1/x"), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str("1/0"), Err(ParseRationalError::ZeroDenominator));
    assert_eq!(Q::from_str("1.2.3"), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str(".5"), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str("1.x"), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str("x.5"), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str("1e2"), Err(ParseRationalError::Invalid));
    assert_eq!(Q::from_str("+1.5").unwrap(), Q::new(3, 2));

    assert_eq!(
        Q32::from_str("2147483648"),
        Err(ParseRationalError::Overflow)
    );
    assert_eq!(
        Q32::from_str("2147483648.0"),
        Err(ParseRationalError::Overflow)
    );

    assert_eq!(
        ParseRationalError::Invalid.to_string(),
        "invalid rational number"
    );
    assert_eq!(
        ParseRationalError::ZeroDenominator.to_string(),
        "rational denominator is zero"
    );
    assert_eq!(
        ParseRationalError::Overflow.to_string(),
        "rational value exceeds integer storage"
    );
}

/// A zero denominator violates the public construction invariant.
#[test]
#[should_panic(expected = "rational denominator must be non-zero")]
fn rational_constructor_rejects_zero_denominator() {
    let _ = Q::new(1, 0);
}

/// Negating the minimum storage value cannot be represented after denominator
/// normalization.
#[test]
#[should_panic(expected = "normalized value must fit storage")]
fn rational_constructor_rejects_normalization_overflow() {
    let _ = Rational::<i32>::new(i32::MIN, -1);
}

/// Exact rational coordinates deliberately have no closed square-root
/// operation; the public scalar contract reports that unsupported operation.
#[test]
#[should_panic(expected = "exact rational square root")]
fn rational_square_root_is_not_closed() {
    let _ = <Q as CoordinateScalar>::sqrt(Q::new(1, 2));
}

/// `test/util/rational.cpp:61-104` — rationals work as public model
/// coordinates in box algorithms rather than only as standalone numbers.
#[test]
fn rational_box_supports_area_and_within() {
    let bounds = ModelBox::from_corners(
        P::new(Q::from_integer(4), Q::from_integer(4)),
        P::new(Q::from_integer(8), Q::from_integer(8)),
    );
    assert_eq!(box_area(&bounds), Q::from_integer(16));
    assert!(within(
        &P::new(Q::from_integer(6), Q::from_integer(6)),
        &bounds
    ));
    assert!(!within(
        &P::new(Q::from_integer(2), Q::from_integer(2)),
        &bounds
    ));
}

/// `test/util/rational.cpp:106-118` — promotion keeps the rational wrapper
/// and promotes its integer storage when two rational widths differ.
#[test]
fn rational_promotion_keeps_exact_storage() {
    type Q32 = Rational<i32>;
    type Q64 = Rational<i64>;
    assert_eq!(
        TypeId::of::<<Q32 as Promote<Q64>>::Out>(),
        TypeId::of::<Q64>()
    );
    assert_eq!(
        TypeId::of::<<f64 as Promote<Q32>>::Out>(),
        TypeId::of::<Q32>()
    );
}
