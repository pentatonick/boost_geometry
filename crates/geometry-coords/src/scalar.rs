//! The `CoordinateScalar` trait: the bound every algorithm in the
//! kernel places on a coordinate value.
//!
//! Distilled from the implicit set of operations Boost.Geometry's
//! strategies assume of their `CalculationType` — see
//! `boost/geometry/util/calculation_type.hpp` for how the C++ side
//! picks that working type, and `boost/geometry/util/math.hpp` for
//! the primitive operations (`abs`, `sqrt`) it then invokes on it.

use core::ops::{Add, Div, Mul, Neg, Sub};

/// Numeric type usable as a geometry coordinate.
///
/// The operator and `Copy + PartialOrd` bounds capture what Boost's
/// strategies require of their `coordinate_type<P>` / `CalculationType`
/// (see `boost/geometry/util/calculation_type.hpp`). The `ZERO`/`ONE`
/// constants and `sqrt`/`abs` methods mirror the primitives in
/// `boost/geometry/util/math.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_coords::CoordinateScalar;
/// fn norm<T: CoordinateScalar>(x: T, y: T) -> T {
///     (x * x + y * y).sqrt()
/// }
/// assert_eq!(norm(3.0_f64, 4.0), 5.0);
/// ```
pub trait CoordinateScalar:
    Copy
    + PartialOrd
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
{
    /// The additive identity. Counterpart to literal `T(0)` in
    /// `boost/geometry/util/math.hpp`.
    const ZERO: Self;
    /// The multiplicative identity. Counterpart to literal `T(1)` in
    /// `boost/geometry/util/math.hpp`.
    const ONE: Self;

    /// Square root.
    ///
    /// For integers this is a placeholder: the [`crate::Promote`]
    /// lattice widens any expression that needs a `sqrt` to a floating
    /// type first, so calling this on `i32`/`i64` indicates a kernel
    /// bug.
    ///
    /// Counterpart to `boost::geometry::math::sqrt`
    /// (`boost/geometry/util/math.hpp`).
    #[must_use]
    fn sqrt(self) -> Self;

    /// Absolute value. Counterpart to `boost::geometry::math::abs`
    /// (`boost/geometry/util/math.hpp`).
    #[must_use]
    fn abs(self) -> Self;

    /// Equality the way the kernel means it.
    ///
    /// Counterpart to `boost::geometry::math::equals`
    /// (`boost/geometry/util/math.hpp`) under `equals_default_policy`:
    /// exact for an integer, and for a float, equal when the difference is
    /// within one epsilon of the larger magnitude — or of `1`, so that two
    /// values near zero still have to agree to an absolute epsilon.
    ///
    /// This is not a convenience. Boost's side predicate calls three points
    /// collinear when any *two* of them are equal by this rule, so a pair a
    /// few last bits apart at a large coordinate is coincident to the whole
    /// kernel, and every predicate built on the side test follows.
    #[must_use]
    fn tolerant_eq(self, other: Self) -> bool;
}

macro_rules! impl_scalar_float {
    ($($t:ty),*) => { $(
        impl CoordinateScalar for $t {
            const ZERO: Self = 0.0;
            const ONE:  Self = 1.0;
            #[inline]
            fn sqrt(self) -> Self { crate::math::sqrt(self) }
            #[inline]
            fn abs(self)  -> Self { crate::math::abs(self) }
            #[inline]
            fn tolerant_eq(self, other: Self) -> bool {
                if self == other {
                    return true;
                }
                if !self.is_finite() || !other.is_finite() {
                    return false;
                }
                // C++: `greatest(abs(a), abs(b), T(1))`, the factor
                // `equals_default_policy` supplies.
                let factor = crate::math::abs(self)
                    .max(crate::math::abs(other))
                    .max(1.0);
                crate::math::abs(self - other) <= <$t>::EPSILON * factor
            }
        }
    )* };
}
impl_scalar_float!(f32, f64);

// Integer support: lets callers feed integer-coordinate geometries
// (think `model::Point<i32, 2, Cartesian>`) into the kernel. The
// `Promote` lattice widens to `f64` for any algorithm whose arithmetic
// needs a non-integer answer (`sqrt`, trig). Calling `sqrt` here
// therefore signals that `Promote` was bypassed, and the panic that
// `unreachable!` produces is the diagnostic we want.
macro_rules! impl_scalar_int {
    ($($t:ty),*) => { $(
        impl CoordinateScalar for $t {
            const ZERO: Self = 0;
            const ONE:  Self = 1;
            #[inline]
            fn sqrt(self) -> Self {
                unreachable!(
                    "integer sqrt called on `{}`: `Promote` must widen to a float first",
                    core::stringify!($t),
                )
            }
            #[inline]
            fn abs(self) -> Self { <$t>::abs(self) }
            #[inline]
            fn tolerant_eq(self, other: Self) -> bool { self == other }
        }
    )* };
}
impl_scalar_int!(i32, i64);
