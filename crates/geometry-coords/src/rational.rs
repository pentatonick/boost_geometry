//! Exact rational coordinate values.
//!
//! This is the Rust coordinate-scalar counterpart to Boost.Geometry's
//! `util/rational.hpp:21-152`, which integrates `boost::rational` with the
//! geometry type-selection and numeric-conversion machinery.

use core::fmt;
use core::ops::{Add, Div, Mul, Neg, Sub};
use core::str::FromStr;

use crate::CoordinateScalar;

/// Integer storage supported by [`Rational`].
///
/// Boost's wrapper accepts any integer type supported by `boost::rational`
/// (`util/rational.hpp:35-70`). The Rust port deliberately supports the two
/// signed coordinate widths in the crate's promotion lattice; keeping the
/// set closed makes overflow behavior explicit and preserves `no_std`.
pub trait RationalInteger: Copy + Eq + Ord + fmt::Debug {
    /// Additive identity for const coordinate initialization.
    #[doc(hidden)]
    const ZERO: Self;

    /// Multiplicative identity for const coordinate initialization.
    #[doc(hidden)]
    const ONE: Self;

    /// Convert into the wider intermediate used for checked arithmetic.
    #[doc(hidden)]
    fn to_i128(self) -> i128;

    /// Convert a normalized intermediate back into storage.
    #[doc(hidden)]
    fn from_i128(value: i128) -> Option<Self>;
}

macro_rules! impl_rational_integer {
    ($($integer:ty),* $(,)?) => {
        $(
            impl RationalInteger for $integer {
                const ZERO: Self = 0;
                const ONE: Self = 1;

                #[inline]
                fn to_i128(self) -> i128 {
                    i128::from(self)
                }

                #[inline]
                fn from_i128(value: i128) -> Option<Self> {
                    Self::try_from(value).ok()
                }
            }
        )*
    };
}

impl_rational_integer!(i32, i64);

/// An exact, reduced fraction usable anywhere a geometry coordinate is read
/// with arithmetic and ordering operations.
///
/// The denominator is always positive and the numerator and denominator are
/// reduced by their greatest common divisor. Construction panics for a zero
/// denominator or when the normalized value cannot fit in `I`, matching the
/// invariant-enforcing construction of `boost::rational` used by
/// `util/rational.hpp:35-70`.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct Rational<I: RationalInteger> {
    numerator: I,
    denominator: I,
}

impl<I: RationalInteger> Rational<I> {
    /// Construct and reduce `numerator / denominator`.
    ///
    /// # Panics
    ///
    /// Panics if `denominator` is zero or if normalization overflows `I`.
    #[inline]
    #[must_use]
    pub fn new(numerator: I, denominator: I) -> Self {
        Self::from_wide(numerator.to_i128(), denominator.to_i128())
            .expect("rational denominator must be non-zero and normalized value must fit storage")
    }

    /// Construct an integral rational value.
    #[inline]
    #[must_use]
    pub fn from_integer(value: I) -> Self {
        Self {
            numerator: value,
            denominator: I::ONE,
        }
    }

    /// Return the reduced numerator.
    #[inline]
    #[must_use]
    pub const fn numerator(self) -> I {
        self.numerator
    }

    /// Return the positive reduced denominator.
    #[inline]
    #[must_use]
    pub const fn denominator(self) -> I {
        self.denominator
    }

    /// Convert to `f64` for presentation or interaction with an inexact API.
    ///
    /// Mirrors the rational-to-floating conversion registered by
    /// `util/rational.hpp:102-120` and exercised by
    /// `test/util/rational.cpp:32-47`.
    #[inline]
    #[must_use]
    #[allow(
        clippy::cast_precision_loss,
        reason = "this method is the caller-requested inexact conversion"
    )]
    pub fn to_f64(self) -> f64 {
        self.numerator.to_i128() as f64 / self.denominator.to_i128() as f64
    }

    fn from_wide(numerator: i128, denominator: i128) -> Option<Self> {
        if denominator == 0 {
            return None;
        }

        let divisor = gcd(numerator.unsigned_abs(), denominator.unsigned_abs());
        let divisor = i128::try_from(divisor).ok()?;
        let mut numerator = numerator / divisor;
        let mut denominator = denominator / divisor;
        if denominator < 0 {
            numerator = numerator.checked_neg()?;
            denominator = denominator.checked_neg()?;
        }

        Some(Self {
            numerator: I::from_i128(numerator)?,
            denominator: I::from_i128(denominator)?,
        })
    }

    fn from_wide_or_panic(numerator: i128, denominator: i128) -> Self {
        Self::from_wide(numerator, denominator)
            .expect("rational arithmetic overflow or division by zero")
    }
}

const fn gcd(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

impl<I: RationalInteger> Default for Rational<I> {
    #[inline]
    fn default() -> Self {
        Self::from_integer(I::from_i128(0).expect("rational storage must represent zero"))
    }
}

impl<I: RationalInteger> From<I> for Rational<I> {
    #[inline]
    fn from(value: I) -> Self {
        Self::from_integer(value)
    }
}

impl<I: RationalInteger> Add for Rational<I> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        let left = self
            .numerator
            .to_i128()
            .checked_mul(rhs.denominator.to_i128())
            .expect("rational addition intermediate overflow");
        let right = rhs
            .numerator
            .to_i128()
            .checked_mul(self.denominator.to_i128())
            .expect("rational addition intermediate overflow");
        let numerator = left
            .checked_add(right)
            .expect("rational addition intermediate overflow");
        let denominator = self
            .denominator
            .to_i128()
            .checked_mul(rhs.denominator.to_i128())
            .expect("rational addition intermediate overflow");
        Self::from_wide_or_panic(numerator, denominator)
    }
}

impl<I: RationalInteger> Sub for Rational<I> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        self + (-rhs)
    }
}

impl<I: RationalInteger> Mul for Rational<I> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        let numerator = self
            .numerator
            .to_i128()
            .checked_mul(rhs.numerator.to_i128())
            .expect("rational multiplication intermediate overflow");
        let denominator = self
            .denominator
            .to_i128()
            .checked_mul(rhs.denominator.to_i128())
            .expect("rational multiplication intermediate overflow");
        Self::from_wide_or_panic(numerator, denominator)
    }
}

impl<I: RationalInteger> Div for Rational<I> {
    type Output = Self;

    #[inline]
    fn div(self, rhs: Self) -> Self::Output {
        let numerator = self
            .numerator
            .to_i128()
            .checked_mul(rhs.denominator.to_i128())
            .expect("rational division intermediate overflow");
        let denominator = self
            .denominator
            .to_i128()
            .checked_mul(rhs.numerator.to_i128())
            .expect("rational division intermediate overflow");
        Self::from_wide_or_panic(numerator, denominator)
    }
}

impl<I: RationalInteger> Neg for Rational<I> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        let numerator = self
            .numerator
            .to_i128()
            .checked_neg()
            .expect("rational negation overflow");
        Self::from_wide_or_panic(numerator, self.denominator.to_i128())
    }
}

impl<I: RationalInteger> PartialOrd for Rational<I> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<I: RationalInteger> Ord for Rational<I> {
    #[inline]
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        let left = self
            .numerator
            .to_i128()
            .checked_mul(other.denominator.to_i128())
            .expect("rational comparison intermediate overflow");
        let right = other
            .numerator
            .to_i128()
            .checked_mul(self.denominator.to_i128())
            .expect("rational comparison intermediate overflow");
        left.cmp(&right)
    }
}

impl<I: RationalInteger + fmt::Display> fmt::Display for Rational<I> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.denominator.to_i128() == 1 {
            write!(formatter, "{}", self.numerator)
        } else {
            write!(formatter, "{}/{}", self.numerator, self.denominator)
        }
    }
}

/// Error returned when text cannot be parsed as a [`Rational`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParseRationalError {
    /// The input was neither an integer, a decimal, nor `numerator/denominator`.
    Invalid,
    /// The parsed denominator was zero.
    ZeroDenominator,
    /// The exact reduced value cannot fit in the chosen integer storage.
    Overflow,
}

impl fmt::Display for ParseRationalError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid => formatter.write_str("invalid rational number"),
            Self::ZeroDenominator => formatter.write_str("rational denominator is zero"),
            Self::Overflow => formatter.write_str("rational value exceeds integer storage"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ParseRationalError {}

impl<I: RationalInteger> FromStr for Rational<I> {
    type Err = ParseRationalError;

    /// Parse an integer, decimal, or `numerator/denominator` exactly.
    ///
    /// This mirrors the exact input cases in
    /// `test/util/rational.cpp:130-158`; scientific notation is deliberately
    /// rejected because it is not one of the reference grammar forms.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        if input.is_empty() || input.trim() != input {
            return Err(ParseRationalError::Invalid);
        }

        if let Some((numerator, denominator)) = input.split_once('/') {
            if denominator.contains('/') {
                return Err(ParseRationalError::Invalid);
            }
            let numerator = numerator
                .parse::<i128>()
                .map_err(|_| ParseRationalError::Invalid)?;
            let denominator = denominator
                .parse::<i128>()
                .map_err(|_| ParseRationalError::Invalid)?;
            if denominator == 0 {
                return Err(ParseRationalError::ZeroDenominator);
            }
            return Self::from_wide(numerator, denominator).ok_or(ParseRationalError::Overflow);
        }

        if let Some((whole, fraction)) = input.split_once('.') {
            if fraction.contains('.') || !fraction.bytes().all(|byte| byte.is_ascii_digit()) {
                return Err(ParseRationalError::Invalid);
            }

            let (negative, digits) = match whole.strip_prefix('-') {
                Some(digits) => (true, digits),
                None => (false, whole.strip_prefix('+').unwrap_or(whole)),
            };
            if digits.is_empty() || !digits.bytes().all(|byte| byte.is_ascii_digit()) {
                return Err(ParseRationalError::Invalid);
            }

            let whole = digits
                .parse::<i128>()
                .map_err(|_| ParseRationalError::Overflow)?;
            let fraction = if fraction.is_empty() {
                0
            } else {
                fraction
                    .parse::<i128>()
                    .map_err(|_| ParseRationalError::Overflow)?
            };
            let exponent =
                u32::try_from(fraction_digits(input)).map_err(|_| ParseRationalError::Overflow)?;
            let denominator = 10_i128
                .checked_pow(exponent)
                .ok_or(ParseRationalError::Overflow)?;
            let numerator = whole
                .checked_mul(denominator)
                .and_then(|value| value.checked_add(fraction))
                .ok_or(ParseRationalError::Overflow)?;
            let numerator = if negative {
                numerator
                    .checked_neg()
                    .ok_or(ParseRationalError::Overflow)?
            } else {
                numerator
            };
            return Self::from_wide(numerator, denominator).ok_or(ParseRationalError::Overflow);
        }

        let value = input
            .parse::<i128>()
            .map_err(|_| ParseRationalError::Invalid)?;
        Self::from_wide(value, 1).ok_or(ParseRationalError::Overflow)
    }
}

fn fraction_digits(input: &str) -> usize {
    input
        .split_once('.')
        .map_or(0, |(_, fraction)| fraction.len())
}

impl<I: RationalInteger> CoordinateScalar for Rational<I> {
    const ZERO: Self = Self {
        numerator: I::ZERO,
        denominator: I::ONE,
    };
    const ONE: Self = Self {
        numerator: I::ONE,
        denominator: I::ONE,
    };

    #[inline]
    fn sqrt(self) -> Self {
        unreachable!("exact rational square root is not closed over rational coordinates")
    }

    #[inline]
    fn abs(self) -> Self {
        if self.numerator.to_i128() < 0 {
            -self
        } else {
            self
        }
    }
}
