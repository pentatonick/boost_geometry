//! Plane-angle units: [`Degree`] and [`Radian`].
//!
//! Mirrors `boost::geometry::degree` and `boost::geometry::radian` —
//! the empty tag types declared at `boost/geometry/core/cs.hpp:41-51`,
//! together with their use as the `units` typedef on the
//! `cs::{spherical,geographic,polar}<DegreeOrRadian>` templates
//! (`cs.hpp:54-79`).
//!
//! Boost.Geometry never expresses degree↔radian conversion as a method
//! on the tag; it scatters `* 0.017453292…` constants and ad-hoc helpers
//! across `util/math.hpp` and the spherical/geographic strategies. We
//! collect both directions into the [`AngleUnit`] trait so a strategy
//! can write `U::to_radians(angle)` once and have it monomorphise to
//! either `angle` (for [`Radian`]) or `angle * π/180` (for [`Degree`]).

use geometry_coords::CoordinateScalar;

/// Unit of plane angle.
///
/// Mirrors `boost::geometry::degree` / `boost::geometry::radian` from
/// `boost/geometry/core/cs.hpp:41-51` and their role as the
/// `cs_angular_units<Cs>::type` in `cs.hpp:251-281`. The conversion
/// methods are the Rust analogue of the unnamed scaling factors Boost
/// inlines inside each strategy.
///
/// # Examples
///
/// ```
/// use geometry_cs::{AngleUnit, Degree, Radian};
///
/// // 180° == π rad
/// let rad = Degree::to_radians(180.0_f64);
/// assert!((rad - core::f64::consts::PI).abs() < 1e-12);
///
/// // Round-trip through Radian is the identity.
/// assert_eq!(Radian::to_radians(1.5_f64), 1.5);
/// ```
pub trait AngleUnit {
    /// Convert a value in this unit to radians.
    ///
    /// For [`Radian`] this is the identity; for [`Degree`] it multiplies
    /// by `π / 180`.
    fn to_radians<T: CoordinateScalar + FromF64>(value: T) -> T;

    /// Convert a value in radians to this unit.
    ///
    /// For [`Radian`] this is the identity; for [`Degree`] it multiplies
    /// by `180 / π`.
    fn from_radians<T: CoordinateScalar + FromF64>(value: T) -> T;
}

/// Degrees (−180 … 180). Mirrors `boost::geometry::degree`
/// (`boost/geometry/core/cs.hpp:41`).
///
/// # Examples
///
/// ```
/// use geometry_cs::{AngleUnit, Degree};
/// assert!((Degree::to_radians(90.0_f64) - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct Degree;

/// Radians (−π … π). Mirrors `boost::geometry::radian`
/// (`boost/geometry/core/cs.hpp:51`).
///
/// # Examples
///
/// ```
/// use geometry_cs::{AngleUnit, Radian};
/// assert_eq!(Radian::to_radians(1.5_f64), 1.5);
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct Radian;

impl AngleUnit for Degree {
    #[inline]
    fn to_radians<T: CoordinateScalar + FromF64>(value: T) -> T {
        value * T::from_f64(core::f64::consts::PI / 180.0)
    }

    #[inline]
    fn from_radians<T: CoordinateScalar + FromF64>(value: T) -> T {
        value * T::from_f64(180.0 / core::f64::consts::PI)
    }
}

impl AngleUnit for Radian {
    #[inline]
    fn to_radians<T: CoordinateScalar + FromF64>(value: T) -> T {
        value
    }

    #[inline]
    fn from_radians<T: CoordinateScalar + FromF64>(value: T) -> T {
        value
    }
}

/// Construct a scalar from an `f64` literal.
///
/// Lives in `geometry-cs` rather than `geometry-coords` because it is
/// only ever needed for the degree↔radian conversion factor — the rest
/// of the kernel either uses `T::ZERO` / `T::ONE` from
/// [`CoordinateScalar`] or stays in the scalar type the caller picked.
/// Keeping the bound local means strategies that never touch angle
/// units do not pay for it.
///
/// # Examples
///
/// ```
/// use geometry_cs::FromF64;
/// assert_eq!(f64::from_f64(1.5), 1.5);
/// assert_eq!(f32::from_f64(1.5), 1.5_f32);
/// ```
pub trait FromF64 {
    /// Cast an `f64` constant down to `Self`. Truncating for `f32` is
    /// fine — these are conversion-factor constants, not user data.
    fn from_f64(value: f64) -> Self;
}

impl FromF64 for f32 {
    #[inline]
    #[allow(clippy::cast_possible_truncation)]
    fn from_f64(value: f64) -> Self {
        value as f32
    }
}

impl FromF64 for f64 {
    #[inline]
    fn from_f64(value: f64) -> Self {
        value
    }
}
