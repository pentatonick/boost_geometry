//! Canonical longitude/latitude intervals on spherical coordinate systems.
//!
//! Mirrors `boost::geometry::math::normalize_spheroidal_box_coordinates`
//! from `util/normalize_spheroidal_box_coordinates.hpp`. The Rust coordinate
//! systems expose only Boost's equatorial convention, so the polar-latitude
//! conversion template parameter has no analogue here.

use core::ops::{Add, Neg, Rem, Sub};

use crate::{AngleUnit, Degree, Radian};

/// Scalar operations needed to canonicalize spheroidal angles.
///
/// Mirrors the arithmetic requirements on `CoordinateType` in
/// `util/normalize_spheroidal_coordinates.hpp:205-250`. Unlike trigonometric
/// strategies, normalization also supports signed integer degree values.
pub trait SpheroidalScalar:
    Copy
    + PartialOrd
    + Add<Output = Self>
    + Sub<Output = Self>
    + Neg<Output = Self>
    + Rem<Output = Self>
{
    /// Additive identity used for canonical pole longitudes.
    const ZERO: Self;

    /// Absolute value used to detect full longitude bands and poles.
    #[must_use]
    fn abs(self) -> Self;

    /// Whether two angles are equal as Boost's `math::equals` decides
    /// (`util/math.hpp:165-202`): a floating-point scalar within one
    /// epsilon of the larger magnitude, or of 1; any other exactly, which
    /// is the default.
    #[must_use]
    #[inline]
    fn equals(self, other: Self) -> bool {
        self == other
    }
}

macro_rules! impl_spheroidal_float {
    ($($scalar:ty),* $(,)?) => {
        $(
            impl SpheroidalScalar for $scalar {
                const ZERO: Self = 0.0;

                #[inline]
                fn abs(self) -> Self {
                    self.abs()
                }

                #[inline]
                fn equals(self, other: Self) -> bool {
                    geometry_coords::CoordinateScalar::tolerant_eq(self, other)
                }
            }
        )*
    };
}

impl_spheroidal_float!(f32, f64);

macro_rules! impl_spheroidal_integer {
    ($($scalar:ty),* $(,)?) => {
        $(
            impl SpheroidalScalar for $scalar {
                const ZERO: Self = 0;

                #[inline]
                fn abs(self) -> Self {
                    self.abs()
                }
            }
        )*
    };
}

impl_spheroidal_integer!(i16, i32, i64, i128, isize);

/// Canonical angular constants for a spheroidal unit and scalar pair.
///
/// Mirrors `constants_on_spheroid<CoordinateType, Units>` from
/// `util/normalize_spheroidal_coordinates.hpp:34-145`.
pub trait SpheroidalUnits<T: SpheroidalScalar>: AngleUnit {
    /// One complete longitude turn.
    const PERIOD: T;
    /// The positive antimeridian.
    const HALF_PERIOD: T;
    /// The positive pole latitude in the equatorial convention.
    const QUARTER_PERIOD: T;
}

macro_rules! impl_degree_float_constants {
    ($($scalar:ty),* $(,)?) => {
        $(
            impl SpheroidalUnits<$scalar> for Degree {
                const PERIOD: $scalar = 360.0;
                const HALF_PERIOD: $scalar = 180.0;
                const QUARTER_PERIOD: $scalar = 90.0;
            }
        )*
    };
}

impl_degree_float_constants!(f32, f64);

macro_rules! impl_degree_integer_constants {
    ($($scalar:ty),* $(,)?) => {
        $(
            impl SpheroidalUnits<$scalar> for Degree {
                const PERIOD: $scalar = 360;
                const HALF_PERIOD: $scalar = 180;
                const QUARTER_PERIOD: $scalar = 90;
            }
        )*
    };
}

impl_degree_integer_constants!(i16, i32, i64, i128, isize);

impl SpheroidalUnits<f32> for Radian {
    const PERIOD: f32 = core::f32::consts::TAU;
    const HALF_PERIOD: f32 = core::f32::consts::PI;
    const QUARTER_PERIOD: f32 = core::f32::consts::FRAC_PI_2;
}

impl SpheroidalUnits<f64> for Radian {
    const PERIOD: f64 = core::f64::consts::TAU;
    const HALF_PERIOD: f64 = core::f64::consts::PI;
    const QUARTER_PERIOD: f64 = core::f64::consts::FRAC_PI_2;
}

/// Normalize the two corners of an equatorial spheroidal box in place.
///
/// Mirrors `boost::geometry::math::normalize_spheroidal_box_coordinates`
/// from `util/normalize_spheroidal_box_coordinates.hpp:113-145`.
/// Longitudes are canonicalized to one increasing interval, an input spanning
/// a complete turn becomes `[-half_period, half_period]`, and a box collapsed
/// at either pole receives canonical zero longitudes. The antimeridian, the
/// poles and the full turn are recognized up to rounding, by
/// [`SpheroidalScalar::equals`], as Boost's `math::equals` and
/// `math::larger_or_equals` recognize them.
#[inline]
pub fn normalize_spheroidal_box_coordinates<U, T>(
    longitude1: &mut T,
    latitude1: &mut T,
    longitude2: &mut T,
    latitude2: &mut T,
) where
    U: SpheroidalUnits<T>,
    T: SpheroidalScalar,
{
    let span = (*longitude1 - *longitude2).abs();
    let band = span >= U::PERIOD || span.equals(U::PERIOD);

    normalize_longitude::<U, T>(longitude1);
    normalize_longitude::<U, T>(longitude2);

    let south = -U::QUARTER_PERIOD;
    if (latitude1.equals(south) && latitude2.equals(south))
        || (latitude1.equals(U::QUARTER_PERIOD) && latitude2.equals(U::QUARTER_PERIOD))
    {
        *longitude1 = T::ZERO;
        *longitude2 = T::ZERO;
    } else if band {
        *longitude1 = -U::HALF_PERIOD;
        *longitude2 = U::HALF_PERIOD;
    } else if *longitude1 > *longitude2 {
        *longitude2 = *longitude2 + U::PERIOD;
    }
}

/// Normalize a longitude, or a difference of longitudes, into
/// `(-half_period, half_period]` in place.
///
/// Mirrors `boost::geometry::math::normalize_longitude` from
/// `util/normalize_spheroidal_coordinates.hpp:226-250,378-385`: a value
/// that is the antimeridian by [`SpheroidalScalar::equals`], of either
/// sign, becomes `+half_period`, one beyond it is folded back by whole
/// turns, and any other is left as it is.
///
/// # Examples
///
/// ```
/// use geometry_cs::{Degree, normalize_longitude};
///
/// let mut longitude = -180.0_f64;
/// normalize_longitude::<Degree, _>(&mut longitude);
/// assert_eq!(longitude, 180.0);
///
/// let mut longitude = 370_i32;
/// normalize_longitude::<Degree, _>(&mut longitude);
/// assert_eq!(longitude, 10);
/// ```
#[inline]
pub fn normalize_longitude<U, T>(longitude: &mut T)
where
    U: SpheroidalUnits<T>,
    T: SpheroidalScalar,
{
    if longitude.abs().equals(U::HALF_PERIOD) {
        *longitude = U::HALF_PERIOD;
    } else if *longitude > U::HALF_PERIOD {
        *longitude = (*longitude + U::HALF_PERIOD) % U::PERIOD - U::HALF_PERIOD;
        if longitude.equals(-U::HALF_PERIOD) {
            *longitude = U::HALF_PERIOD;
        }
    } else if *longitude < -U::HALF_PERIOD {
        *longitude = (*longitude - U::HALF_PERIOD) % U::PERIOD + U::HALF_PERIOD;
    }
}
