//! Spherical point-to-segment cross-track distance.
//!
//! Ports the strategy from
//! `boost/geometry/strategies/spherical/distance_cross_track.hpp`.

use geometry_cs::{CoordinateSystem, SphericalFamily};
use geometry_model::Segment;
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut};

use crate::distance::DistanceStrategy;
use crate::normalise::HasAngularUnits;

use super::great_circle;

/// Great-circle distance from a point to the nearest location on a segment.
#[derive(Debug, Clone, Copy)]
pub struct CrossTrack {
    /// Sphere radius in output distance units.
    pub radius: f64,
}

impl CrossTrack {
    /// Mean Earth radius used by the spherical Haversine strategy.
    pub const EARTH: Self = Self {
        radius: 6_372_795.0,
    };
    /// Unit sphere, returning angular distance in radians.
    pub const UNIT: Self = Self { radius: 1.0 };
}

impl Default for CrossTrack {
    fn default() -> Self {
        Self::EARTH
    }
}

impl<P> DistanceStrategy<P, Segment<P>> for CrossTrack
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    P::Cs: HasAngularUnits,
    <P::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
{
    type Out = f64;
    type Comparable = Self;

    fn distance(&self, point: &P, segment: &Segment<P>) -> Self::Out {
        great_circle::project(point, segment.start(), segment.end()).angular_distance * self.radius
    }

    fn comparable(&self) -> Self::Comparable {
        *self
    }
}

impl<P> DistanceStrategy<Segment<P>, P> for CrossTrack
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    P::Cs: HasAngularUnits,
    <P::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
{
    type Out = f64;
    type Comparable = Self;

    fn distance(&self, segment: &Segment<P>, point: &P) -> Self::Out {
        <Self as DistanceStrategy<P, Segment<P>>>::distance(self, point, segment)
    }

    fn comparable(&self) -> Self::Comparable {
        *self
    }
}
