//! Per-scalar math primitives (`sqrt`, `abs`) dispatched across
//! `std` and `libm` according to the active cargo feature.
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
     to be enabled so that `sqrt` and `abs` have an implementation."
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

/// Sealed marker for the floating-point types this crate dispatches
/// `sqrt` / `abs` over (`f32`, `f64`).
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
}

mod private {
    pub trait Sealed {}
    impl Sealed for f32 {}
    impl Sealed for f64 {}
}
