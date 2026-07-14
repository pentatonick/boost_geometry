//! The `Promote<Other>` lattice: pick the wider of two coordinate
//! scalar types when an algorithm operates on a pair of geometries.
//!
//! Direct counterpart to `boost::geometry::select_most_precise`
//! (`boost/geometry/util/select_most_precise.hpp`) and the
//! `calculation_type::geometric::binary` helper
//! (`boost/geometry/util/calculation_type.hpp`). The unary
//! [`PromoteIntegral`] counterpart mirrors
//! `boost/geometry/util/promote_integral.hpp`.

use crate::rational::{Rational, RationalInteger};
use crate::scalar::CoordinateScalar;

/// "Pick the wider of two scalar types."
///
/// `T: Promote<U>` reads as "the binary calculation type for a `T` and
/// a `U` is `<T as Promote<U>>::Out`".
///
/// The implementation is a closed lattice (one impl per ordered pair
/// drawn from `{f32, f64, i32, i64}`) so that the choice is a `const`
/// fact about the input types and produces a hard compile error for
/// any pair we have not deliberately considered. The three regions of
/// the lattice mirror Boost's `type_priority`
/// (`select_most_precise.hpp:34-37`):
///
/// * float × float — the larger float, preserving precision;
/// * int × int — the larger int, preserving integer arithmetic;
/// * int × float — always widen to a float (`f64`) so that the
///   result can survive a `sqrt` / trig call.
///
/// # Examples
///
/// ```
/// use geometry_coords::Promote;
/// fn promoted<A: Promote<B>, B>(_a: A, _b: B) -> <A as Promote<B>>::Out {
///     unimplemented!()
/// }
/// // The lattice picks `f64` for an `(f32, f64)` pair.
/// fn _check() {
///     let _: f64 = promoted(1.0_f32, 2.0_f64);
/// }
/// ```
pub trait Promote<Other> {
    /// The chosen working type. Constrained to `CoordinateScalar` so
    /// that downstream algorithms can keep their bounds in terms of
    /// `Out` alone, the same way Boost's strategies use the result of
    /// `calculation_type::geometric::binary<..>::type` directly.
    type Out: CoordinateScalar;
}

/// Promote a calculation type to an integer with roughly twice its bit width.
///
/// Mirrors `boost::geometry::promote_integral<T,
/// PromoteUnsignedToUnsigned>` from `util/promote_integral.hpp:113-159`.
/// Signed inputs promote to the first wider signed primitive. Unsigned inputs
/// promote to a signed primitive with room for a sign bit by default; setting
/// `PRESERVE_UNSIGNED` selects an unsigned result instead. Floating-point
/// inputs pass through unchanged, matching Boost's non-integral arm.
///
/// # Fixed-width policy
///
/// Rust exposes `i128` and `u128` on every supported target, so they are always
/// candidates here; Boost only considers its 128-bit extension when configured
/// with `BOOST_GEOMETRY_ENABLE_INT128`. Like Boost when multiprecision is
/// disabled, an input for which no wider fixed-width primitive exists is
/// returned unchanged (`util/promote_integral.hpp:80-96,138-146`).
pub trait PromoteIntegral<const PRESERVE_UNSIGNED: bool = false> {
    /// The promoted calculation type.
    ///
    /// Mirrors `promote_integral::type` from
    /// `util/promote_integral.hpp:272-297` and `344-367`.
    type Out;
}

macro_rules! impl_signed_integral_promotion {
    ($($input:ty => $output:ty),* $(,)?) => {
        $(
            impl PromoteIntegral<false> for $input {
                type Out = $output;
            }
            impl PromoteIntegral<true> for $input {
                type Out = $output;
            }
        )*
    };
}

impl_signed_integral_promotion!(
    i8 => i16,
    i16 => i32,
    i32 => i64,
    i64 => i128,
    i128 => i128,
);

#[cfg(target_pointer_width = "32")]
impl_signed_integral_promotion!(isize => i64);
#[cfg(target_pointer_width = "64")]
impl_signed_integral_promotion!(isize => i128);

macro_rules! impl_unsigned_integral_promotion {
    ($($input:ty => $signed:ty, $unsigned:ty),* $(,)?) => {
        $(
            impl PromoteIntegral<false> for $input {
                type Out = $signed;
            }
            impl PromoteIntegral<true> for $input {
                type Out = $unsigned;
            }
        )*
    };
}

impl_unsigned_integral_promotion!(
    u8 => i32, u16,
    u16 => i64, u32,
    u32 => i128, u64,
    u64 => u64, u128,
    u128 => u128, u128,
);

#[cfg(target_pointer_width = "32")]
impl_unsigned_integral_promotion!(usize => i128, u64);
#[cfg(target_pointer_width = "64")]
impl_unsigned_integral_promotion!(usize => usize, u128);

macro_rules! impl_non_integral_promotion {
    ($($input:ty),* $(,)?) => {
        $(
            impl PromoteIntegral<false> for $input {
                type Out = $input;
            }
            impl PromoteIntegral<true> for $input {
                type Out = $input;
            }
        )*
    };
}

impl_non_integral_promotion!(f32, f64);

// ---- float × float -------------------------------------------------
impl Promote<f32> for f32 {
    type Out = f32;
}
impl Promote<f64> for f32 {
    type Out = f64;
}
impl Promote<f32> for f64 {
    type Out = f64;
}
impl Promote<f64> for f64 {
    type Out = f64;
}

// ---- int × int -----------------------------------------------------
impl Promote<i32> for i32 {
    type Out = i32;
}
impl Promote<i64> for i32 {
    type Out = i64;
}
impl Promote<i32> for i64 {
    type Out = i64;
}
impl Promote<i64> for i64 {
    type Out = i64;
}

// ---- int × float (always widen to f64) -----------------------------
//
// Boost makes the same choice: a mixed integer/float calculation goes
// through `double` because the algorithm will reach for a `sqrt` or a
// trig function whose answer is not representable in an integer, and
// `f32` is rarely the right answer for a coordinate that started life
// as an integer (`select_most_precise.hpp:34-110`, the
// floating-vs-integral arm).
impl Promote<f64> for i32 {
    type Out = f64;
}
impl Promote<i32> for f64 {
    type Out = f64;
}
impl Promote<f64> for i64 {
    type Out = f64;
}
impl Promote<i64> for f64 {
    type Out = f64;
}
impl Promote<f32> for i32 {
    type Out = f64;
}
impl Promote<i32> for f32 {
    type Out = f64;
}
impl Promote<f32> for i64 {
    type Out = f64;
}
impl Promote<i64> for f32 {
    type Out = f64;
}

// ---- exact rational coordinates ------------------------------------
//
// Mirrors the `select_most_precise` specializations registered for
// `boost::rational` by `util/rational.hpp:73-100`. Rational × rational
// promotes the integer storage; rational × floating-point retains the exact
// rational type, as exercised by `test/util/rational.cpp:106-118`.
impl<I, J> Promote<Rational<J>> for Rational<I>
where
    I: RationalInteger + Promote<J>,
    J: RationalInteger,
    <I as Promote<J>>::Out: RationalInteger,
{
    type Out = Rational<<I as Promote<J>>::Out>;
}

macro_rules! impl_rational_float_promotion {
    ($($float:ty),* $(,)?) => {
        $(
            impl<I: RationalInteger> Promote<$float> for Rational<I> {
                type Out = Rational<I>;
            }

            impl<I: RationalInteger> Promote<Rational<I>> for $float {
                type Out = Rational<I>;
            }
        )*
    };
}

impl_rational_float_promotion!(f32, f64);
