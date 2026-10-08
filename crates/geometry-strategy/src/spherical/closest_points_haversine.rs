//! Spherical point-to-segment closest points.
//!
//! Ports the cross-track strategy from
//! `boost/geometry/strategies/spherical/closest_points_pt_seg.hpp`.

#[cfg(not(feature = "std"))]
use geometry_coords::math::Float;
use geometry_cs::{AngleUnit, CoordinateSystem, SphericalFamily};
use geometry_model::Segment;
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut};

use crate::closest_points::ClosestPointsStrategy;
use crate::normalise::{HasAngularUnits, lonlat_radians};

use super::Haversine;
use super::azimuth::spherical_azimuth;
use super::direct::spherical_direct;
use super::distance_haversine::comparable_haversine_h;
use super::great_circle::{self, Foot};

/// Haversine-compatible closest-point projection onto a spherical segment.
#[derive(Debug, Clone, Copy)]
pub struct HaversineClosestPoints {
    /// Sphere radius, retained for parity with the distance strategy bundle.
    pub radius: f64,
}

impl HaversineClosestPoints {
    /// Mean Earth radius.
    pub const EARTH: Self = Self {
        radius: Haversine::EARTH.radius,
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
        (*point, self.nearest(point, segment))
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
        (self.nearest(point, segment), *point)
    }
}

impl HaversineClosestPoints {
    /// The point of `segment` nearest `point`.
    ///
    /// Mirrors `strategy::closest_points::cross_track::apply`
    /// (`closest_points_pt_seg.hpp:86-187`): a foot inside the segment is
    /// reached from its start along the segment's course, the arc from
    /// the start to the foot taken from the right spherical triangle the
    /// point, the start and the foot form.
    fn nearest<P>(self, point: &P, segment: &Segment<P>) -> P
    where
        P: Point<Scalar = f64> + PointMut + Default + Copy,
        P::Cs: HasAngularUnits,
    {
        match great_circle::foot(point, segment.start(), segment.end()) {
            Foot::Start(_) => *segment.start(),
            Foot::End(_) => *segment.end(),
            Foot::Inside { h, d1 } => {
                let radius = self.radius;
                let (lon1, lat1) = lonlat_radians(segment.start());
                let (lon2, lat2) = lonlat_radians(segment.end());
                let dist = 2.0 * h.sqrt().asin() * radius;
                let dist_d1 = 2.0 * d1.sqrt().asin() * radius;
                let cos_frac = (dist_d1 / radius).cos() / (dist / radius).cos();
                let s14 = if cos_frac >= 1.0 {
                    0.0
                } else if cos_frac <= -1.0 {
                    core::f64::consts::PI * radius
                } else {
                    cos_frac.acos() * radius
                };
                // Boost leaves the arc unbounded. From a pole of the
                // segment's great circle, level with all of it, `cos_frac`
                // is `0/0` and rounds past ±1, so its foot can land half a
                // turn away, off the segment — `(170 0)` from the north pole
                // to `(−10 0)–(10 0)`. The arc stops at the segment's end.
                let length = 2.0
                    * comparable_haversine_h(segment.start(), segment.end())
                        .sqrt()
                        .asin()
                    * radius;
                let s14 = s14.min(length);
                let a12 = spherical_azimuth::<false>(lon1, lat1, lon2, lat2).0;
                let (lon, lat) = spherical_direct(lon1, lat1, s14 / radius, a12);
                let mut nearest = P::default();
                nearest.set::<0>(<P::Cs as HasAngularUnits>::Units::from_radians(lon));
                nearest.set::<1>(<P::Cs as HasAngularUnits>::Units::from_radians(lat));
                nearest
            }
        }
    }
}
