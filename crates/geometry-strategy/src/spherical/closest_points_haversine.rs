//! Spherical point-to-segment closest points.
//!
//! Ports the great-circle projection shape from
//! `boost/geometry/strategies/spherical/closest_points_pt_seg.hpp`.

use geometry_cs::{CoordinateSystem, SphericalFamily};
use geometry_model::Segment;
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut};

use crate::closest_points::ClosestPointsStrategy;
use crate::normalise::HasAngularUnits;

use super::great_circle;

/// Haversine-compatible closest-point projection onto a spherical segment.
#[derive(Debug, Clone, Copy)]
pub struct HaversineClosestPoints {
    /// Sphere radius, retained for parity with the distance strategy bundle.
    pub radius: f64,
}

impl HaversineClosestPoints {
    /// Mean Earth radius.
    pub const EARTH: Self = Self {
        radius: 6_372_795.0,
    };
    /// Unit sphere.
    pub const UNIT: Self = Self { radius: 1.0 };
}

impl Default for HaversineClosestPoints {
    fn default() -> Self {
        Self::EARTH
    }
}

impl<P> ClosestPointsStrategy<P, Segment<P>> for HaversineClosestPoints
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    P::Cs: HasAngularUnits,
    <P::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
{
    type Out = P;

    fn closest_points(&self, point: &P, segment: &Segment<P>) -> (Self::Out, Self::Out) {
        let projected = great_circle::project(point, segment.start(), segment.end()).point;
        (*point, projected)
    }
}

impl<P> ClosestPointsStrategy<Segment<P>, P> for HaversineClosestPoints
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    P::Cs: HasAngularUnits,
    <P::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
{
    type Out = P;

    fn closest_points(&self, segment: &Segment<P>, point: &P) -> (Self::Out, Self::Out) {
        let projected = great_circle::project(point, segment.start(), segment.end()).point;
        (projected, *point)
    }
}
