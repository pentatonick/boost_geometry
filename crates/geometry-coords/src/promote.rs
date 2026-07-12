//! The `Promote<Other>` lattice: pick the wider of two coordinate
//! scalar types when an algorithm operates on a pair of geometries.
//!
//! Direct counterpart to `boost::geometry::select_most_precise`
//! (`boost/geometry/util/select_most_precise.hpp`) and the
//! `calculation_type::geometric::binary` helper
//! (`boost/geometry/util/calculation_type.hpp`).

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
