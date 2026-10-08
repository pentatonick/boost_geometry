//! Point-at-bearing-and-distance strategies.
//!
//! This is the strategy-facing wrapper around Boost's direct geodesic formulas.
//! Bearings and direct-formula inputs are radians; output coordinates are
//! converted back to the angular unit carried by the input point.

use geometry_cs::{CoordinateSystem, GeographicFamily, SphericalFamily};
use geometry_trait::Point;

#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};
#[cfg(feature = "std")]
use crate::spherical::direct::spherical_direct;
#[cfg(feature = "std")]
use geometry_cs::AngleUnit;
#[cfg(feature = "std")]
use geometry_model::Point2D;

/// Strategy computing the endpoint reached from a point, bearing, and distance.
pub trait DestinationStrategy<P: Point> {
    /// Destination point type.
    type Output: Point;

    /// Compute the destination. `bearing` is measured clockwise from north in
    /// radians; `distance` uses the strategy's radius or spheroid units.
    fn destination(&self, origin: &P, bearing: f64, distance: f64) -> Self::Output;
}

/// Select the default destination strategy for a coordinate-system family.
pub trait DefaultDestination<Family> {
    /// Default strategy type.
    type Strategy: Default;
}

impl DefaultDestination<SphericalFamily> for SphericalFamily {
    type Strategy = crate::spherical::Haversine;
}

impl DefaultDestination<GeographicFamily> for GeographicFamily {
    type Strategy = crate::geographic::KarneyDirect;
}

/// Default destination strategy for a point type.
pub type DefaultDestinationStrategy<P> =
    <<<P as Point>::Cs as CoordinateSystem>::Family as DefaultDestination<
        <<P as Point>::Cs as CoordinateSystem>::Family,
    >>::Strategy;

#[cfg(feature = "std")]
impl<P> DestinationStrategy<P> for crate::spherical::Haversine
where
    P: Point<Scalar = f64>,
    P::Cs: HasAngularUnits,
    <P::Cs as CoordinateSystem>::Family: geometry_tag::SameAs<SphericalFamily>,
{
    type Output = Point2D<f64, P::Cs>;

    fn destination(&self, origin: &P, bearing: f64, distance: f64) -> Self::Output {
        let (longitude, latitude) = lonlat_radians(origin);
        let (longitude2, latitude2) =
            spherical_direct(longitude, latitude, distance / self.radius, bearing);
        point_from_radians::<P>(longitude2, latitude2)
    }
}

macro_rules! impl_geographic_destination {
    ($strategy:ty) => {
        #[cfg(feature = "std")]
        impl<P> DestinationStrategy<P> for $strategy
        where
            P: Point<Scalar = f64>,
            P::Cs: HasAngularUnits,
            <P::Cs as CoordinateSystem>::Family: geometry_tag::SameAs<GeographicFamily>,
        {
            type Output = Point2D<f64, P::Cs>;

            fn destination(&self, origin: &P, bearing: f64, distance: f64) -> Self::Output {
                let (longitude, latitude) = lonlat_radians(origin);
                let result = self.apply(longitude, latitude, distance, bearing);
                point_from_radians::<P>(result.lon2, result.lat2)
            }
        }
    };
}

impl_geographic_destination!(crate::geographic::KarneyDirect);
impl_geographic_destination!(crate::geographic::ThomasDirect);
impl_geographic_destination!(crate::geographic::VincentyDirect);

#[cfg(feature = "std")]
fn point_from_radians<P>(longitude: f64, latitude: f64) -> Point2D<f64, P::Cs>
where
    P: Point<Scalar = f64>,
    P::Cs: HasAngularUnits,
{
    type Units<P> = <<P as Point>::Cs as HasAngularUnits>::Units;
    Point2D::new(
        Units::<P>::from_radians(longitude),
        Units::<P>::from_radians(latitude),
    )
}
