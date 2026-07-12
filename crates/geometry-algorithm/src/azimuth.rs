//! `azimuth(&p1, &p2)` — the bearing from `p1` to `p2`.
//!
//! Mirrors `boost::geometry::azimuth` from
//! `boost/geometry/algorithms/azimuth.hpp`. Dispatches to the per-CS-
//! family default strategy via [`geometry_strategy::DefaultAzimuth`] —
//! Cartesian points resolve to [`geometry_strategy::CartesianAzimuth`],
//! spherical to [`geometry_strategy::SphericalAzimuth`] and geographic
//! to [`geometry_strategy::GeographicAzimuth`]. All three share one
//! bearing convention (`0` = north, increasing clockwise — the geodetic
//! convention Boost uses uniformly, `azimuth.hpp:76`).

use geometry_cs::CoordinateSystem;
use geometry_strategy::{AzimuthStrategy, DefaultAzimuth, DefaultAzimuthStrategy};
use geometry_trait::Point;

/// Shorthand for the CS family of a point type. Keeps the `where`
/// clauses readable; matches the projection in
/// [`geometry_strategy::DefaultAzimuthStrategy`].
type Family<P> = <<P as Point>::Cs as CoordinateSystem>::Family;

/// The azimuth (bearing) from `p1` to `p2`, in radians, using the
/// default strategy for the points' coordinate-system family.
///
/// Mirrors `boost::geometry::azimuth(p1, p2)` from
/// `boost/geometry/algorithms/azimuth.hpp`. The strategy is resolved
/// via [`geometry_strategy::DefaultAzimuthStrategy`]; for an explicit
/// strategy use [`azimuth_with`].
#[must_use]
pub fn azimuth<P1, P2>(
    p1: &P1,
    p2: &P2,
) -> <DefaultAzimuthStrategy<P1> as AzimuthStrategy<P1, P2>>::Out
where
    P1: Point,
    P2: Point,
    Family<P1>: DefaultAzimuth<Family<P1>>,
    DefaultAzimuthStrategy<P1>: AzimuthStrategy<P1, P2> + Default,
{
    DefaultAzimuthStrategy::<P1>::default().azimuth(p1, p2)
}

/// Azimuth from `p1` to `p2` using an explicitly supplied strategy.
///
/// Mirrors the `azimuth(p1, p2, strategy)` overload at
/// `boost/geometry/algorithms/azimuth.hpp`. Taking the strategy by
/// value matches the by-value call shape of [`crate::distance_with`];
/// concrete strategies are zero-sized or small `Copy` configuration
/// objects, so this monomorphises into nothing.
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "Strategies are ZST/small Copy configuration objects; by-value matches the user-facing call shape."
)]
pub fn azimuth_with<P1, P2, S>(p1: &P1, p2: &P2, s: S) -> S::Out
where
    P1: Point,
    P2: Point,
    S: AzimuthStrategy<P1, P2>,
{
    s.azimuth(p1, p2)
}

#[cfg(test)]
mod tests {
    //! Reference from `boost/geometry/test/algorithms/azimuth.cpp`.
    //! Cartesian is `atan2(dx, dy)` (0 = north, clockwise); spherical /
    //! geographic are the geodetic bearings from `test_sph` / `test_geo`
    //! (`:49-66`).
    #![allow(
        clippy::float_cmp,
        reason = "azimuths are compared with an explicit absolute tolerance, not `==`"
    )]
    #![allow(
        clippy::excessive_precision,
        reason = "reference constants are copied verbatim from Boost's azimuth.cpp; the trailing digits document the source value even past f64 precision."
    )]

    use super::azimuth;
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn due_north_is_zero() {
        assert!((azimuth(&P::new(0.0, 0.0), &P::new(0.0, 1.0)) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn due_east_is_half_pi() {
        let a = azimuth(&P::new(0.0, 0.0), &P::new(1.0, 0.0));
        assert!((a - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    /// `azimuth.cpp:52` (`test_sph`) — spherical `(0,0) → (1,1)` ≈
    /// 44.9956° dispatches to `SphericalAzimuth`.
    #[cfg(feature = "std")]
    #[test]
    fn spherical_diagonal_close_to_45() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Spherical};

        type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;
        let sp = |lon: f64, lat: f64| -> Sp { WithCs::new(Adapt([lon, lat])) };

        let got = azimuth(&sp(0., 0.), &sp(1., 1.));
        let expected = 44.995_636_455_344_851_f64.to_radians();
        assert!((got - expected).abs() < 1e-9, "got {got}");
    }

    /// `azimuth.cpp:62` (`test_geo`) — geographic `(0,0) → (1,1)` ≈
    /// 45.1877° dispatches to `GeographicAzimuth` (Andoyer).
    #[cfg(feature = "std")]
    #[test]
    fn geographic_diagonal_close_to_45() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Geographic};

        type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;
        let gg = |lon: f64, lat: f64| -> Gg { WithCs::new(Adapt([lon, lat])) };

        let got = azimuth(&gg(0., 0.), &gg(1., 1.));
        let expected = 45.187_718_848_049_521_f64.to_radians();
        assert!((got - expected).abs() < 1e-9, "got {got}");
    }
}
