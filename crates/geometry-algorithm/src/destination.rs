//! Point at a bearing and distance from an angular-coordinate point.
//!
//! The public entry wraps the direct geodesic formulas ported from Boost's
//! `formulas/*_direct.hpp` and selects a default by coordinate-system family.

use geometry_cs::CoordinateSystem;
use geometry_strategy::{DefaultDestination, DefaultDestinationStrategy, DestinationStrategy};
use geometry_trait::Point;

type Family<P> = <<P as Point>::Cs as CoordinateSystem>::Family;

/// Compute the destination using the point's coordinate-system default.
///
/// `bearing` is in radians, clockwise from north. `distance` uses the default
/// strategy's radius or spheroid units. The result uses the origin point's
/// coordinate system and angular unit.
#[inline]
#[must_use]
pub fn destination<P>(
    origin: &P,
    bearing: f64,
    distance: f64,
) -> <DefaultDestinationStrategy<P> as DestinationStrategy<P>>::Output
where
    P: Point,
    Family<P>: DefaultDestination<Family<P>>,
    DefaultDestinationStrategy<P>: DestinationStrategy<P> + Default,
{
    DefaultDestinationStrategy::<P>::default().destination(origin, bearing, distance)
}

/// Compute the destination using an explicitly supplied direct strategy.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "direct strategies are small Copy configuration values, matching other _with entries"
)]
pub fn destination_with<P, S>(origin: &P, bearing: f64, distance: f64, strategy: S) -> S::Output
where
    P: Point,
    S: DestinationStrategy<P>,
{
    strategy.destination(origin, bearing, distance)
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::{destination, destination_with};
    use geometry_cs::{Degree, Geographic, Spherical};
    use geometry_model::Point2D;
    use geometry_strategy::{Haversine, VincentyDirect};
    use geometry_trait::Point as _;

    #[test]
    fn spherical_and_geographic_defaults_return_input_units() {
        type SphericalPoint = Point2D<f64, Spherical<Degree>>;
        type GeographicPoint = Point2D<f64, Geographic<Degree>>;

        let spherical = destination(
            &SphericalPoint::new(0.0, 0.0),
            core::f64::consts::FRAC_PI_2,
            Haversine::EARTH.radius,
        );
        assert!((spherical.get::<0>().to_radians() - 1.0).abs() < 1e-12);

        let geographic = destination_with(
            &GeographicPoint::new(0.0, 0.0),
            core::f64::consts::FRAC_PI_2,
            100_000.0,
            VincentyDirect::WGS84,
        );
        assert!((geographic.get::<0>() - 0.898_315_284_1).abs() < 1e-6);
    }
}
