//! Chamberlain–Duquette spherical polygon area.
//!
//! This is an opt-in, lower-cost alternative to Boost.Geometry's default
//! spherical-excess area strategy. Boost has no corresponding strategy; the
//! formula comes from Chamberlain & Duquette, *Some Algorithms for Polygons on
//! a Sphere* (JPL Publication 07-03, 2007).

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use geometry_coords::math::Float;
use geometry_cs::{CoordinateSystem, SphericalFamily};
use geometry_tag::SameAs;
use geometry_trait::{Point, PointOrder, Polygon, Ring};

use crate::area::AreaStrategy;
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Chamberlain–Duquette area for a spherical polygon.
///
/// For every ring vertex `i`, the strategy accumulates
/// `(lon[i+1] - lon[i-1]) * sin(lat[i])`, then multiplies by `R² / 2`.
/// The result follows this crate's signed-area convention: a ring matching its
/// declared order is positive, while an oppositely wound ring is negative.
#[derive(Debug, Clone, Copy)]
pub struct ChamberlainDuquetteArea {
    /// Sphere radius in output units.
    pub radius: f64,
}

impl ChamberlainDuquetteArea {
    /// Mean Earth radius in metres.
    pub const EARTH: Self = Self {
        radius: 6_371_008.8,
    };

    /// Unit sphere; output is a solid angle in steradians.
    pub const UNIT: Self = Self { radius: 1.0 };
}

impl Default for ChamberlainDuquetteArea {
    fn default() -> Self {
        Self::EARTH
    }
}

impl<Pg> AreaStrategy<Pg> for ChamberlainDuquetteArea
where
    Pg: Polygon,
    Pg::Point: Point<Scalar = f64>,
    <Pg::Point as Point>::Cs: HasAngularUnits + CoordinateSystem,
    <<Pg::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
{
    type Out = f64;

    fn area(&self, polygon: &Pg) -> Self::Out {
        let mut total = ring_area(polygon.exterior(), self.radius);
        for interior in polygon.interiors() {
            total += ring_area(interior, self.radius);
        }
        total
    }
}

fn ring_area<R>(ring: &R, radius: f64) -> f64
where
    R: Ring,
    R::Point: Point<Scalar = f64>,
    <R::Point as Point>::Cs: HasAngularUnits,
{
    let mut coordinates: Vec<(f64, f64)> = ring.points().map(lonlat_radians).collect();
    if coordinates.len() > 1 && coordinates.first() == coordinates.last() {
        coordinates.pop();
    }
    if coordinates.len() < 3 {
        return 0.0;
    }

    let mut sum = 0.0;
    for index in 0..coordinates.len() {
        let previous = coordinates[(index + coordinates.len() - 1) % coordinates.len()];
        let current = coordinates[index];
        let next = coordinates[(index + 1) % coordinates.len()];
        sum += longitude_delta(next.0 - previous.0) * current.1.sin();
    }
    let signed = sum * radius * radius / 2.0;
    match ring.point_order() {
        PointOrder::Clockwise => signed,
        PointOrder::CounterClockwise => -signed,
    }
}

fn longitude_delta(delta: f64) -> f64 {
    (delta + core::f64::consts::PI).rem_euclid(core::f64::consts::TAU) - core::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use geometry_cs::{Degree, Spherical};
    use geometry_model::{Point2D, Polygon, Ring};

    use super::*;

    #[test]
    fn one_degree_square_on_unit_sphere() {
        type P = Point2D<f64, Spherical<Degree>>;
        let polygon: Polygon<P> = Polygon::new(Ring::from_vec(alloc::vec![
            P::new(0.0, 0.0),
            P::new(0.0, 1.0),
            P::new(1.0, 1.0),
            P::new(1.0, 0.0),
            P::new(0.0, 0.0),
        ]));
        let area = ChamberlainDuquetteArea::UNIT.area(&polygon);
        assert!((area - 0.000_304_601_954_726_850_5).abs() < 1e-12);
    }
}
