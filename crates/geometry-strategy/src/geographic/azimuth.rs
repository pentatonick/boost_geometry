//! Geodesic bearing (azimuth) on a spheroid via the Andoyer inverse.
//!
//! Mirrors `boost::geometry::strategy::azimuth::geographic` from
//! `boost/geometry/strategies/geographic/azimuth.hpp`, whose default
//! `FormulaPolicy` is `strategy::andoyer`
//! (`strategies/geographic/azimuth.hpp:33`, `:137-145`). The forward
//! azimuth is the `alpha_1` output of
//! `formula::andoyer_inverse` — the same iteration
//! [`Andoyer`](crate::geographic::Andoyer) runs for distance — computed
//! here from the closed-form Andoyer expansion in
//! `boost/geometry/formulas/andoyer_inverse.hpp:165-219`:
//!
//! ```text
//! A  = atan2(sin Δλ, cos φ₁·tan φ₂ − sin φ₁·cos Δλ)
//! U  = (f/2)·cos²φ₁·sin 2A
//! B  = atan2(sin Δλ, cos φ₂·tan φ₁ − sin φ₂·cos Δλ)
//! V  = (f/2)·cos²φ₂·sin 2B
//! T  = d / sin d
//! azimuth = A − (V·T − U)
//! ```
//!
//! # Convention
//!
//! `0` = north, increasing **clockwise** — the geodetic bearing
//! convention, matching [`SphericalAzimuth`](crate::spherical::SphericalAzimuth)
//! and *differing* from
//! [`CartesianAzimuth`](crate::azimuth::CartesianAzimuth). `azimuth(p, p)`
//! and coincident / antipodal short-circuits return `0`, mirroring
//! `andoyer_inverse.hpp:127-163`.

use geometry_cs::{CoordinateSystem, GeographicFamily, Spheroid};
use geometry_tag::SameAs;
use geometry_trait::Point;

use crate::azimuth::AzimuthStrategy;

#[cfg(feature = "std")]
use crate::geographic::spheroid_calc::SpheroidCalc;
#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Geodesic initial bearing on a spheroid, in radians (`0` = north,
/// clockwise), via the Andoyer inverse.
///
/// Carries the reference [`Spheroid`]; `Default::default()` produces
/// [`GeographicAzimuth::WGS84`].
///
/// Mirrors `boost::geometry::strategy::azimuth::geographic<andoyer>`
/// (`strategies/geographic/azimuth.hpp`).
#[derive(Debug, Clone, Copy)]
pub struct GeographicAzimuth {
    /// Reference ellipsoid the bearing is measured on.
    pub spheroid: Spheroid,
}

impl GeographicAzimuth {
    /// Bearing on the WGS84 reference ellipsoid — the default for
    /// nearly every real geographic dataset (matches `Andoyer::WGS84`).
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };
}

impl Default for GeographicAzimuth {
    #[inline]
    fn default() -> Self {
        Self::WGS84
    }
}

#[cfg(feature = "std")]
impl<P1, P2> AzimuthStrategy<P1, P2> for GeographicAzimuth
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    P1::Cs: HasAngularUnits,
    P2::Cs: HasAngularUnits,
    <P1::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
{
    type Out = f64;

    // The single-letter names `A, B, U, V, T, M, N, d` mirror
    // `formula::andoyer_inverse::apply` in
    // `formulas/andoyer_inverse.hpp:165-219` letter-for-letter; the
    // exact `== 0.0` short-circuits are the intentional analogue of
    // Boost's `math::equals` guards on the same lines.
    #[allow(clippy::many_single_char_names, clippy::float_cmp)]
    #[inline]
    fn azimuth(&self, p1: &P1, p2: &P2) -> f64 {
        let calc = SpheroidCalc::from(self.spheroid);
        let f = calc.f;
        let (lon1, lat1) = lonlat_radians(p1);
        let (lon2, lat2) = lonlat_radians(p2);

        let dlon = lon2 - lon1;
        let sin_dlon = dlon.sin();
        let cos_dlon = dlon.cos();
        let sin_lat1 = lat1.sin();
        let cos_lat1 = lat1.cos();
        let sin_lat2 = lat2.sin();
        let cos_lat2 = lat2.cos();

        // Spherical central angle, clamped to the acos domain — mirrors
        // `andoyer_inverse.hpp:90-97`.
        let cos_d = (sin_lat1 * sin_lat2 + cos_lat1 * cos_lat2 * cos_dlon).clamp(-1.0, 1.0);
        let d = cos_d.acos();
        let sin_d = d.sin();

        // Coincident / antipodal short-circuit — mirrors
        // `andoyer_inverse.hpp:127-163`. Boost returns 0 for the
        // aligned case (which is all this port needs to reproduce the
        // reference table).
        if sin_d == 0.0 {
            return 0.0;
        }

        let pi = core::f64::consts::PI;

        // Forward-azimuth term A + first-order flattening correction U.
        let (a, u) = if cos_lat2 == 0.0 {
            (if sin_lat2 < 0.0 { pi } else { 0.0 }, 0.0)
        } else {
            let tan_lat2 = sin_lat2 / cos_lat2;
            let m = cos_lat1 * tan_lat2 - sin_lat1 * cos_dlon;
            let a = sin_dlon.atan2(m);
            let u = (f / 2.0) * (cos_lat1 * cos_lat1) * (2.0 * a).sin();
            (a, u)
        };

        // Correction term V (from the reverse-azimuth term B), needed
        // for the forward `dA = V·T − U`. B itself is not used forward.
        let v = if cos_lat1 == 0.0 {
            0.0
        } else {
            let tan_lat1 = sin_lat1 / cos_lat1;
            let n = cos_lat2 * tan_lat1 - sin_lat2 * cos_dlon;
            let b = sin_dlon.atan2(n);
            (f / 2.0) * (cos_lat2 * cos_lat2) * (2.0 * b).sin()
        };

        let t = d / sin_d;
        let da = v * t - u;
        let mut azimuth = a - da;
        normalize_azimuth(&mut azimuth, a, da);
        azimuth
    }
}

/// Clamp the corrected forward azimuth so the flattening correction
/// cannot push it past the pole it started on. Mirrors
/// `formula::andoyer_inverse::normalize_azimuth` at
/// `formulas/andoyer_inverse.hpp:246-286`.
#[cfg(feature = "std")]
#[inline]
fn normalize_azimuth(azimuth: &mut f64, a: f64, da: f64) {
    let pi = core::f64::consts::PI;
    if a >= 0.0 {
        // A in the Eastern hemisphere.
        if da >= 0.0 {
            if *azimuth < 0.0 {
                *azimuth = 0.0;
            }
        } else if *azimuth > pi {
            *azimuth = pi;
        }
    } else {
        // A in the Western hemisphere.
        if da <= 0.0 {
            if *azimuth > 0.0 {
                *azimuth = 0.0;
            }
        } else if *azimuth < -pi {
            *azimuth = -pi;
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/azimuth.cpp:59-66` (`test_geo`,
    //! default = Andoyer), given in degrees; asserted here in radians.
    #![allow(
        clippy::float_cmp,
        reason = "azimuths are compared with an explicit absolute tolerance, not `==`"
    )]
    #![allow(
        clippy::excessive_precision,
        reason = "reference constants are copied verbatim from Boost's azimuth.cpp; the trailing digits document the source value even past f64 precision."
    )]

    use super::GeographicAzimuth;
    use crate::azimuth::AzimuthStrategy;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Geographic};

    type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;

    #[inline]
    fn gg(lon: f64, lat: f64) -> Gg {
        WithCs::new(Adapt([lon, lat]))
    }

    /// `azimuth.cpp:61` — `(0,0) → (0,0)` returns `0`.
    #[test]
    fn coincident_is_zero() {
        let got = GeographicAzimuth::WGS84.azimuth(&gg(0., 0.), &gg(0., 0.));
        assert!(got.abs() < 1e-12);
    }

    /// `azimuth.cpp:62` — `(0,0) → (1,1) = 45.187718848°`.
    #[test]
    fn diagonal() {
        let got = GeographicAzimuth::WGS84.azimuth(&gg(0., 0.), &gg(1., 1.));
        let expected = 45.187_718_848_049_521_f64.to_radians();
        assert!((got - expected).abs() < 1e-9, "got {got}");
    }

    /// `azimuth.cpp:63` — `(0,0) → (100,1) = 88.986933066°`.
    #[test]
    fn shallow_east() {
        let got = GeographicAzimuth::WGS84.azimuth(&gg(0., 0.), &gg(100., 1.));
        let expected = 88.986_933_066_023_497_f64.to_radians();
        assert!((got - expected).abs() < 1e-9, "got {got}");
    }

    /// `azimuth.cpp:64` — `(0,0) → (-1,1) = -45.187718848°`.
    #[test]
    fn diagonal_west() {
        let got = GeographicAzimuth::WGS84.azimuth(&gg(0., 0.), &gg(-1., 1.));
        let expected = (-45.187_718_848_049_521_f64).to_radians();
        assert!((got - expected).abs() < 1e-9, "got {got}");
    }

    /// The clamp branches of `normalize_azimuth`
    /// (`andoyer_inverse.hpp:246-286`): the flattening correction must
    /// not push an azimuth past 0 / ±π on the side it started.
    #[test]
    fn normalize_azimuth_clamps_all_four_quadrants() {
        use super::normalize_azimuth;
        let pi = core::f64::consts::PI;

        // A ≥ 0, dA ≥ 0: an azimuth pushed below 0 clamps to 0.
        let mut az = -0.1;
        normalize_azimuth(&mut az, 0.05, 0.15);
        assert_eq!(az, 0.0);

        // A ≥ 0, dA < 0: an azimuth pushed above π clamps to π.
        let mut az = pi + 0.1;
        normalize_azimuth(&mut az, pi - 0.05, -0.15);
        assert_eq!(az, pi);

        // A < 0, dA ≤ 0: an azimuth pushed above 0 clamps to 0.
        let mut az = 0.1;
        normalize_azimuth(&mut az, -0.05, -0.15);
        assert_eq!(az, 0.0);

        // A < 0, dA > 0: an azimuth pushed below −π clamps to −π.
        let mut az = -pi - 0.1;
        normalize_azimuth(&mut az, -pi + 0.05, 0.15);
        assert_eq!(az, -pi);

        // In-range azimuths pass through untouched.
        let mut az = 0.5;
        normalize_azimuth(&mut az, 0.4, -0.1);
        assert_eq!(az, 0.5);
    }
}
