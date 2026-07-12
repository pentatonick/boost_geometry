//! `AzimuthStrategy<P1, P2>` — the angle from `p1` to `p2`.
//!
//! Mirrors `boost::geometry::strategy::azimuth::*` from
//! `boost/geometry/strategies/cartesian/azimuth.hpp`. Cartesian only in
//! LA4.T4; spherical / geographic azimuth land in LA8.T4.
//!
//! The scalar is fixed to `f64` — the same convention the Haversine
//! kernel uses (`distance_haversine.rs`) to reach `f64`'s inherent
//! `atan2`, since `CoordinateScalar` does not yet carry a `Trig` bound.

use geometry_cs::{CartesianFamily, CoordinateSystem, GeographicFamily, SphericalFamily};
// `SameAs` is used only by the `std`-gated `CartesianAzimuth` impl below.
#[cfg(feature = "std")]
use geometry_tag::SameAs;
use geometry_trait::Point;

/// Strategy computing the azimuth (bearing) from one point to another.
pub trait AzimuthStrategy<P1: Point, P2: Point> {
    /// The angle type (radians for the Cartesian implementation).
    type Out;

    /// The signed angle from `p1` to `p2`.
    fn azimuth(&self, p1: &P1, p2: &P2) -> Self::Out;
}

/// "Which azimuth strategy do we pick by default for this CS family?"
///
/// Mirrors v1's [`DefaultDistance`](crate::distance::DefaultDistance)
/// for the azimuth algorithm — the Rust analogue of Boost's
/// `strategy::azimuth::services::default_strategy<cs_tag>` in
/// `strategies/azimuth.hpp` / `strategies/{cartesian,spherical,
/// geographic}/azimuth.hpp`. Each family reports its default bearing
/// formula:
///
/// ```ignore
/// impl DefaultAzimuth<CartesianFamily>  for CartesianFamily  { type Strategy = CartesianAzimuth;  }
/// impl DefaultAzimuth<SphericalFamily>  for SphericalFamily  { type Strategy = SphericalAzimuth;  }
/// impl DefaultAzimuth<GeographicFamily> for GeographicFamily { type Strategy = GeographicAzimuth; }
/// ```
pub trait DefaultAzimuth<Family> {
    /// The azimuth strategy chosen for this family. Must implement
    /// [`Default`] because the free-function `azimuth(p1, p2)` builds
    /// it without arguments.
    type Strategy: Default;
}

/// Cartesian family defaults to [`CartesianAzimuth`].
impl DefaultAzimuth<CartesianFamily> for CartesianFamily {
    type Strategy = CartesianAzimuth;
}

/// Spherical family defaults to [`SphericalAzimuth`](crate::spherical::SphericalAzimuth).
impl DefaultAzimuth<SphericalFamily> for SphericalFamily {
    type Strategy = crate::spherical::SphericalAzimuth;
}

/// Geographic family defaults to [`GeographicAzimuth`](crate::geographic::GeographicAzimuth)
/// (Andoyer inverse — Boost's default geographic azimuth policy).
impl DefaultAzimuth<GeographicFamily> for GeographicFamily {
    type Strategy = crate::geographic::GeographicAzimuth;
}

/// Type alias resolving the default azimuth strategy for a point `P` by
/// walking `P -> Cs -> Family -> DefaultAzimuth::Strategy`.
///
/// Mirrors [`DefaultDistanceStrategy`](crate::distance::DefaultDistanceStrategy)
/// for the azimuth algorithm; the free-function `azimuth(p1, p2)`
/// monomorphises against this at the call site.
pub type DefaultAzimuthStrategy<P> =
    <<<P as Point>::Cs as CoordinateSystem>::Family as DefaultAzimuth<
        <<P as Point>::Cs as CoordinateSystem>::Family,
    >>::Strategy;

/// Cartesian azimuth: `atan2(dx, dy)`, in radians, measured **clockwise
/// from the positive y-axis** (i.e. `0` points along +y ("North"), `π/2`
/// along +x ("East")).
///
/// Mirrors `boost::geometry::strategy::azimuth::cartesian` from
/// `boost/geometry/strategies/cartesian/azimuth.hpp:74-76`, whose
/// comment is explicit: *"azimuth 0 is at Y axis, increasing right — as
/// in spherical/geographic where 0 is at North axis"*. This keeps the
/// Cartesian bearing convention consistent with the crate's spherical
/// and geographic azimuths, which also measure clockwise from North.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianAzimuth;

// `std`-gated: the kernel uses `f64::atan2`, which `geometry-coords`
// does not shim under `libm` (the scalar trait exposes only `sqrt`/`abs`;
// see the module doc). Matches the gating on the spherical/geographic
// azimuth impls, which are `std`-only for the same reason.
#[cfg(feature = "std")]
impl<P1, P2> AzimuthStrategy<P1, P2> for CartesianAzimuth
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    <P1::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = f64;

    #[inline]
    fn azimuth(&self, p1: &P1, p2: &P2) -> f64 {
        let dx = p2.get::<0>() - p1.get::<0>();
        let dy = p2.get::<1>() - p1.get::<1>();
        // `atan2(dx, dy)`, NOT `atan2(dy, dx)`: Boost measures the
        // bearing clockwise from +y ("North"), so 0 points North and
        // π/2 points East (azimuth.hpp:76).
        dx.atan2(dy)
    }
}

#[cfg(test)]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/azimuth.cpp` (`test_car`) — a
    //! Cartesian azimuth is `atan2(dx, dy)`, measured clockwise from +y
    //! ("North"): 0 = North, π/2 = East, ±π = South, −π/2 = West.

    use super::{AzimuthStrategy, CartesianAzimuth};
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn north_is_zero() {
        // Boost: azimuth 0 is at the +y ("North") axis.
        let a = P::new(0.0, 0.0);
        let b = P::new(0.0, 1.0);
        assert!((CartesianAzimuth.azimuth(&a, &b) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn east_is_half_pi() {
        // Boost: bearing increases clockwise (to the right), so +x is π/2.
        let a = P::new(0.0, 0.0);
        let b = P::new(1.0, 0.0);
        assert!((CartesianAzimuth.azimuth(&a, &b) - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn west_is_minus_half_pi() {
        // `azimuth.cpp` `test_car`: due West → −45°'s sibling, −π/2.
        let a = P::new(0.0, 0.0);
        let b = P::new(-1.0, 0.0);
        assert!((CartesianAzimuth.azimuth(&a, &b) + core::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn diagonal_is_quarter_pi() {
        // NE diagonal is π/4 under either convention — the case that
        // hid the earlier transposed-argument bug.
        let a = P::new(0.0, 0.0);
        let b = P::new(1.0, 1.0);
        assert!((CartesianAzimuth.azimuth(&a, &b) - core::f64::consts::FRAC_PI_4).abs() < 1e-12);
    }

    #[test]
    fn northwest_diagonal_is_minus_quarter_pi() {
        // `azimuth.cpp` `test_car`: (-1, 1) → −45° = −π/4. This case
        // *diverges* between `atan2(dx,dy)` (correct) and the old
        // `atan2(dy,dx)` (which gave +3π/4), so it locks in the fix.
        let a = P::new(0.0, 0.0);
        let b = P::new(-1.0, 1.0);
        assert!((CartesianAzimuth.azimuth(&a, &b) + core::f64::consts::FRAC_PI_4).abs() < 1e-12);
    }
}
