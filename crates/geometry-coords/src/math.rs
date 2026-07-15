//! Per-scalar math primitives dispatched across `std` and `libm` according
//! to the active cargo feature.
//!
//! Mirrors the role of `boost/geometry/util/math.hpp`: a thin layer of
//! type-generic primitives that the rest of the geometry kernel calls
//! instead of going to `std::sqrt` / `std::fabs` directly. Keeping the
//! `#[cfg]` here means the public `CoordinateScalar` impls stay free of
//! conditional compilation noise.
//!
//! Exactly one of the `std` and `libm` cargo features must be enabled
//! (the default-features path enables `std`).

#[cfg(not(any(feature = "std", feature = "libm")))]
compile_error!(
    "geometry-coords requires either the `std` (default) or `libm` cargo feature \
     to be enabled so that floating-point math primitives have an implementation."
);

/// Square root of a floating-point coordinate.
///
/// Counterpart to `boost::geometry::math::sqrt` in
/// `boost/geometry/util/math.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_coords::math::sqrt;
/// assert_eq!(sqrt(25.0_f64), 5.0);
/// ```
pub fn sqrt<T: Float>(value: T) -> T {
    value.sqrt()
}

/// Absolute value of a floating-point coordinate.
///
/// Counterpart to `boost::geometry::math::abs` in
/// `boost/geometry/util/math.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_coords::math::abs;
/// assert_eq!(abs(-3.0_f64), 3.0);
/// ```
pub fn abs<T: Float>(value: T) -> T {
    value.abs()
}

/// Fused multiply-add of floating-point coordinates.
///
/// Computes `(value * multiplier) + addend` with a single rounding step,
/// dispatching to the standard library or `libm` according to the active
/// feature. Counterpart to `std::fma` used by Boost's precise-math kernel.
///
/// # Examples
///
/// ```
/// use geometry_coords::math::mul_add;
/// assert_eq!(mul_add(2.0_f64, 3.0, 4.0), 10.0);
/// ```
pub fn mul_add<T: Float>(value: T, multiplier: T, addend: T) -> T {
    value.mul_add(multiplier, addend)
}

/// Sine of a floating-point coordinate in radians.
pub fn sin<T: Float>(value: T) -> T {
    value.sin()
}

/// Cosine of a floating-point coordinate in radians.
pub fn cos<T: Float>(value: T) -> T {
    value.cos()
}

/// Four-quadrant arctangent of `y` and `x`.
pub fn atan2<T: Float>(y: T, x: T) -> T {
    y.atan2(x)
}

/// Length of the hypotenuse formed by `x` and `y`.
pub fn hypot<T: Float>(x: T, y: T) -> T {
    x.hypot(y)
}

/// Smallest integer-valued coordinate greater than or equal to `value`.
pub fn ceil<T: Float>(value: T) -> T {
    value.ceil()
}

/// Tangent of a floating-point coordinate in radians.
pub fn tan<T: Float>(value: T) -> T {
    value.tan()
}

/// Natural logarithm of a positive floating-point coordinate.
pub fn ln<T: Float>(value: T) -> T {
    value.ln()
}

/// Least non-negative remainder of `value` divided by `modulus`.
pub fn rem_euclid<T: Float>(value: T, modulus: T) -> T {
    value.rem_euclid(modulus)
}

/// Sealed marker for the floating-point types this crate dispatches
/// math primitives over (`f32`, `f64`).
///
/// The trait is sealed (the sub-bound on `private::Sealed` cannot be
/// implemented downstream) so that adding a new floating type stays a
/// deliberate change inside this crate. Mirrors the way
/// `boost/geometry/util/math.hpp` keeps its primitive set closed.
///
/// # Examples
///
/// ```
/// use geometry_coords::math::Float;
/// fn root<T: Float>(x: T) -> T { x.sqrt() }
/// assert_eq!(root(9.0_f32), 3.0);
/// assert_eq!(root(9.0_f64), 3.0);
/// ```
pub trait Float: private::Sealed + Copy {
    /// `value.sqrt()` dispatched onto `std` or `libm`.
    #[must_use]
    fn sqrt(self) -> Self;
    /// `value.abs()` dispatched onto `std` or `libm`.
    #[must_use]
    fn abs(self) -> Self;
    /// `value.mul_add(multiplier, addend)` dispatched onto `std` or `libm`.
    #[must_use]
    fn mul_add(self, multiplier: Self, addend: Self) -> Self;
    /// `value.sin()` dispatched onto `std` or `libm`.
    #[must_use]
    fn sin(self) -> Self;
    /// `value.cos()` dispatched onto `std` or `libm`.
    #[must_use]
    fn cos(self) -> Self;
    /// `value.atan2(other)` dispatched onto `std` or `libm`.
    #[must_use]
    fn atan2(self, other: Self) -> Self;
    /// `value.hypot(other)` dispatched onto `std` or `libm`.
    #[must_use]
    fn hypot(self, other: Self) -> Self;
    /// `value.ceil()` dispatched onto `std` or `libm`.
    #[must_use]
    fn ceil(self) -> Self;
    /// `value.tan()` dispatched onto `std` or `libm`.
    #[must_use]
    fn tan(self) -> Self;
    /// `value.ln()` dispatched onto `std` or `libm`.
    #[must_use]
    fn ln(self) -> Self;
    /// `value.rem_euclid(modulus)` dispatched onto `std` or core arithmetic.
    #[must_use]
    fn rem_euclid(self, modulus: Self) -> Self;
}

impl Float for f32 {
    #[cfg(feature = "std")]
    #[inline]
    fn sqrt(self) -> Self {
        f32::sqrt(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn sqrt(self) -> Self {
        libm::sqrtf(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn abs(self) -> Self {
        f32::abs(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn abs(self) -> Self {
        libm::fabsf(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn mul_add(self, multiplier: Self, addend: Self) -> Self {
        f32::mul_add(self, multiplier, addend)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn mul_add(self, multiplier: Self, addend: Self) -> Self {
        libm::fmaf(self, multiplier, addend)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn sin(self) -> Self {
        f32::sin(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn sin(self) -> Self {
        libm::sinf(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn cos(self) -> Self {
        f32::cos(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn cos(self) -> Self {
        libm::cosf(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn atan2(self, other: Self) -> Self {
        f32::atan2(self, other)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn atan2(self, other: Self) -> Self {
        libm::atan2f(self, other)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn hypot(self, other: Self) -> Self {
        f32::hypot(self, other)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn hypot(self, other: Self) -> Self {
        libm::hypotf(self, other)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn ceil(self) -> Self {
        f32::ceil(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn ceil(self) -> Self {
        libm::ceilf(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn tan(self) -> Self {
        f32::tan(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn tan(self) -> Self {
        libm::tanf(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn ln(self) -> Self {
        f32::ln(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn ln(self) -> Self {
        libm::logf(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn rem_euclid(self, modulus: Self) -> Self {
        f32::rem_euclid(self, modulus)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn rem_euclid(self, modulus: Self) -> Self {
        let remainder = self % modulus;
        if remainder < 0.0 {
            remainder + libm::fabsf(modulus)
        } else {
            remainder
        }
    }
}

impl Float for f64 {
    #[cfg(feature = "std")]
    #[inline]
    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn sqrt(self) -> Self {
        libm::sqrt(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn abs(self) -> Self {
        f64::abs(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn abs(self) -> Self {
        libm::fabs(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn mul_add(self, multiplier: Self, addend: Self) -> Self {
        f64::mul_add(self, multiplier, addend)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn mul_add(self, multiplier: Self, addend: Self) -> Self {
        libm::fma(self, multiplier, addend)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn sin(self) -> Self {
        f64::sin(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn sin(self) -> Self {
        libm::sin(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn cos(self) -> Self {
        f64::cos(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn cos(self) -> Self {
        libm::cos(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn atan2(self, other: Self) -> Self {
        f64::atan2(self, other)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn atan2(self, other: Self) -> Self {
        libm::atan2(self, other)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn hypot(self, other: Self) -> Self {
        f64::hypot(self, other)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn hypot(self, other: Self) -> Self {
        libm::hypot(self, other)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn ceil(self) -> Self {
        f64::ceil(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn ceil(self) -> Self {
        libm::ceil(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn tan(self) -> Self {
        f64::tan(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn tan(self) -> Self {
        libm::tan(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn ln(self) -> Self {
        f64::ln(self)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn ln(self) -> Self {
        libm::log(self)
    }

    #[cfg(feature = "std")]
    #[inline]
    fn rem_euclid(self, modulus: Self) -> Self {
        f64::rem_euclid(self, modulus)
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    #[inline]
    fn rem_euclid(self, modulus: Self) -> Self {
        let remainder = self % modulus;
        if remainder < 0.0 {
            remainder + libm::fabs(modulus)
        } else {
            remainder
        }
    }
}

mod private {
    pub trait Sealed {}
    impl Sealed for f32 {}
    impl Sealed for f64 {}
}
