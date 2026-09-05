//! Constant-bearing rhumb-line measurements on an angular coordinate system.
//!
//! Boost.Geometry has no rhumb-line strategy. The closed-form spherical
//! formulas follow Ed Williams' *Aviation Formulary* and the isometric-latitude
//! treatment described by Bowring. The strategy is shared by spherical and
//! geographic coordinate families; geographic use is a spherical mean-radius
//! approximation rather than an ellipsoidal geodesic.

#[cfg(not(feature = "std"))]
use geometry_coords::math::Float;
use geometry_cs::{AngleUnit, CoordinateSystem, GeographicFamily, SphericalFamily};
use geometry_model::Point2D;
use geometry_trait::{Linestring, Point};

use crate::azimuth::AzimuthStrategy;
use crate::destination::DestinationStrategy;
use crate::distance::DistanceStrategy;
use crate::length::LengthStrategy;
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Coordinate-system families for which a loxodrome is defined.
#[doc(hidden)]
pub trait RhumbFamily {}

impl RhumbFamily for SphericalFamily {}
impl RhumbFamily for GeographicFamily {}

/// Spherical rhumb-line metric with a configurable radius.
#[derive(Debug, Clone, Copy)]
pub struct Rhumb {
    /// Sphere radius in distance units.
    pub radius: f64,
}

impl Rhumb {
    /// IUGG mean Earth radius in metres.
    pub const EARTH: Self = Self {
        radius: 6_371_008.8,
    };

    /// Unit sphere, returning angular distance in radians.
    pub const UNIT: Self = Self { radius: 1.0 };

    /// Construct a rhumb metric with an application-defined radius.
    #[must_use]
    pub const fn with_radius(radius: f64) -> Self {
        Self { radius }
    }
}

impl Default for Rhumb {
    fn default() -> Self {
        Self::EARTH
    }
}

impl<P1, P2> DistanceStrategy<P1, P2> for Rhumb
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64, Cs = P1::Cs>,
    P1::Cs: HasAngularUnits + CoordinateSystem,
    <P1::Cs as CoordinateSystem>::Family: RhumbFamily,
{
    type Out = f64;
    type Comparable = Self;

    fn distance(&self, first: &P1, second: &P2) -> Self::Out {
        rhumb_inverse(first, second).0 * self.radius
    }

    fn comparable(&self) -> Self::Comparable {
        *self
    }
}

impl<P1, P2> AzimuthStrategy<P1, P2> for Rhumb
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64, Cs = P1::Cs>,
    P1::Cs: HasAngularUnits + CoordinateSystem,
    <P1::Cs as CoordinateSystem>::Family: RhumbFamily,
{
    type Out = f64;

    fn azimuth(&self, first: &P1, second: &P2) -> Self::Out {
        rhumb_inverse(first, second).1
    }
}

impl<P> DestinationStrategy<P> for Rhumb
where
    P: Point<Scalar = f64>,
    P::Cs: HasAngularUnits + CoordinateSystem,
    <P::Cs as CoordinateSystem>::Family: RhumbFamily,
{
    type Output = Point2D<f64, P::Cs>;

    fn destination(&self, origin: &P, bearing: f64, distance: f64) -> Self::Output {
        type Units<P> = <<P as Point>::Cs as HasAngularUnits>::Units;
        let (longitude1, latitude1) = lonlat_radians(origin);
        let angular_distance = distance / self.radius;
        let delta_latitude = angular_distance * bearing.cos();
        let latitude2 = reflect_latitude(latitude1 + delta_latitude);
        let delta_psi = isometric_latitude(latitude2) - isometric_latitude(latitude1);
        let q = meridional_scale(delta_latitude, delta_psi, latitude1);
        let delta_longitude = if q.abs() <= f64::EPSILON {
            0.0
        } else {
            angular_distance * bearing.sin() / q
        };
        let longitude2 = normalize_longitude(longitude1 + delta_longitude);
        Point2D::new(
            Units::<P>::from_radians(longitude2),
            Units::<P>::from_radians(latitude2),
        )
    }
}

impl<L> LengthStrategy<L> for Rhumb
where
    L: Linestring,
    L::Point: Point<Scalar = f64>,
    <L::Point as Point>::Cs: HasAngularUnits + CoordinateSystem,
    <<L::Point as Point>::Cs as CoordinateSystem>::Family: RhumbFamily,
{
    type Out = f64;

    fn length(&self, line: &L) -> Self::Out {
        let points = line.points();
        points
            .clone()
            .zip(points.skip(1))
            .map(|(first, second)| {
                <Self as DistanceStrategy<L::Point, L::Point>>::distance(self, first, second)
            })
            .sum()
    }
}

fn rhumb_inverse<P1, P2>(first: &P1, second: &P2) -> (f64, f64)
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64, Cs = P1::Cs>,
    P1::Cs: HasAngularUnits,
{
    let (longitude1, latitude1) = lonlat_radians(first);
    let (longitude2, latitude2) = lonlat_radians(second);
    let delta_latitude = latitude2 - latitude1;
    let delta_longitude = normalize_delta(longitude2 - longitude1);
    let delta_psi = isometric_latitude(latitude2) - isometric_latitude(latitude1);
    let q = meridional_scale(delta_latitude, delta_psi, latitude1);
    let angular_distance = delta_latitude.hypot(q * delta_longitude);
    let azimuth = delta_longitude
        .atan2(delta_psi)
        .rem_euclid(core::f64::consts::TAU);
    (angular_distance, azimuth)
}

fn isometric_latitude(latitude: f64) -> f64 {
    (core::f64::consts::FRAC_PI_4 + latitude / 2.0).tan().ln()
}

fn meridional_scale(delta_latitude: f64, delta_psi: f64, latitude: f64) -> f64 {
    if delta_psi.abs() > 1e-12 {
        delta_latitude / delta_psi
    } else {
        latitude.cos()
    }
}

fn normalize_delta(delta: f64) -> f64 {
    (delta + core::f64::consts::PI).rem_euclid(core::f64::consts::TAU) - core::f64::consts::PI
}

fn normalize_longitude(longitude: f64) -> f64 {
    normalize_delta(longitude)
}

fn reflect_latitude(latitude: f64) -> f64 {
    let latitude = (latitude + core::f64::consts::PI).rem_euclid(core::f64::consts::TAU)
        - core::f64::consts::PI;
    if latitude > core::f64::consts::FRAC_PI_2 {
        core::f64::consts::PI - latitude
    } else if latitude < -core::f64::consts::FRAC_PI_2 {
        -core::f64::consts::PI - latitude
    } else {
        latitude
    }
}

#[cfg(test)]
mod tests {
    use geometry_cs::{Degree, Spherical};
    use geometry_model::{Linestring, Point2D};

    use super::*;

    #[test]
    fn equatorial_degree_has_expected_measurements() {
        type P = Point2D<f64, Spherical<Degree>>;
        let start = P::new(0.0, 0.0);
        let east = P::new(1.0, 0.0);
        let distance = Rhumb::EARTH.distance(&start, &east);
        assert!((distance - 111_195.080_233_532_9).abs() < 1e-6);
        assert!((Rhumb::EARTH.azimuth(&start, &east) - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
        let destination = Rhumb::EARTH.destination(&start, core::f64::consts::FRAC_PI_2, distance);
        assert!((destination.get::<0>() - 1.0).abs() < 1e-10);

        let line = Linestring::from_vec(alloc::vec![start, east, P::new(2.0, 0.0)]);
        assert!((Rhumb::EARTH.length(&line) - 2.0 * distance).abs() < 1e-6);
    }
}
