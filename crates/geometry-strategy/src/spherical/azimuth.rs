//! Great-circle bearing (azimuth) on a sphere.
//!
//! Mirrors `boost::geometry::strategy::azimuth::spherical` from
//! `boost/geometry/strategies/spherical/azimuth.hpp`, whose kernel is
//! `formula::spherical_azimuth` in
//! `boost/geometry/formulas/spherical.hpp:127-168`:
//!
//! ```text
//! θ = atan2( sin(Δλ)·cos φ₂,
//!            cos φ₁·sin φ₂ − sin φ₁·cos φ₂·cos(Δλ) )
//! ```
//!
//! where φ₁, λ₁, φ₂, λ₂ are the input lat/lon in radians.
//!
//! # Convention
//!
//! `0` = north, increasing **clockwise** — the geodetic bearing
//! convention. This is *different* from
//! [`CartesianAzimuth`](crate::azimuth::CartesianAzimuth) (`0` = east,
//! counter-clockwise). `azimuth(p, p)` returns `0`, mirroring the
//! `math::equals(dlon, 0)`-style short-circuits Boost applies for
//! coincident points.

use geometry_cs::{CoordinateSystem, SphericalFamily};
use geometry_tag::SameAs;
use geometry_trait::Point;

use crate::azimuth::AzimuthStrategy;

#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Great-circle initial bearing on a sphere, in radians (`0` = north,
/// clockwise).
///
/// Mirrors `boost::geometry::strategy::azimuth::spherical`
/// (`strategies/spherical/azimuth.hpp`). Radius-independent — a bearing
/// is an angle, so no sphere radius is carried.
#[derive(Debug, Default, Clone, Copy)]
pub struct SphericalAzimuth;

#[cfg(feature = "std")]
impl<P1, P2> AzimuthStrategy<P1, P2> for SphericalAzimuth
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    P1::Cs: HasAngularUnits,
    P2::Cs: HasAngularUnits,
    <P1::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
{
    type Out = f64;

    #[inline]
    fn azimuth(&self, p1: &P1, p2: &P2) -> f64 {
        let (lon1, lat1) = lonlat_radians(p1);
        let (lon2, lat2) = lonlat_radians(p2);
        let dlon = lon2 - lon1;
        // Mirrors `formula::spherical_azimuth` at
        // `formulas/spherical.hpp:156-158`.
        let y = dlon.sin() * lat2.cos();
        let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
        y.atan2(x)
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/azimuth.cpp:49-56` (`test_sph`),
    //! given in degrees; asserted here in radians.
    #![allow(
        clippy::float_cmp,
        reason = "azimuths are compared with an explicit absolute tolerance, not `==`"
    )]
    #![allow(
        clippy::excessive_precision,
        reason = "reference constants are copied verbatim from Boost's azimuth.cpp; the trailing digits document the source value even past f64 precision."
    )]

    use super::SphericalAzimuth;
    use crate::azimuth::AzimuthStrategy;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Spherical};

    type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;

    #[inline]
    fn sp(lon: f64, lat: f64) -> Sp {
        WithCs::new(Adapt([lon, lat]))
    }

    /// `azimuth.cpp:51` — `(0,0) → (0,0)` returns `0`.
    #[test]
    fn coincident_is_zero() {
        let got = SphericalAzimuth.azimuth(&sp(0., 0.), &sp(0., 0.));
        assert!(got.abs() < 1e-12);
    }

    /// `azimuth.cpp:52` — `(0,0) → (1,1) = 44.995636455°`.
    #[test]
    fn diagonal() {
        let got = SphericalAzimuth.azimuth(&sp(0., 0.), &sp(1., 1.));
        let expected = 44.995_636_455_344_851_f64.to_radians();
        assert!((got - expected).abs() < 1e-9, "got {got}");
    }

    /// `azimuth.cpp:53` — `(0,0) → (100,1) = 88.984576593°`.
    #[test]
    fn shallow_east() {
        let got = SphericalAzimuth.azimuth(&sp(0., 0.), &sp(100., 1.));
        let expected = 88.984_576_593_576_293_f64.to_radians();
        assert!((got - expected).abs() < 1e-9, "got {got}");
    }

    /// `azimuth.cpp:54` — `(0,0) → (-1,1) = -44.995636455°`.
    #[test]
    fn diagonal_west() {
        let got = SphericalAzimuth.azimuth(&sp(0., 0.), &sp(-1., 1.));
        let expected = (-44.995_636_455_344_851_f64).to_radians();
        assert!((got - expected).abs() < 1e-9, "got {got}");
    }
}
