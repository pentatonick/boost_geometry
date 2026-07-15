//! Named rhumb-line measurement entries.
//!
//! Boost.Geometry has no loxodrome entry points. These functions expose the
//! published constant-bearing formulas through the same default and `_with`
//! strategy shape as the Boost-origin measurement algorithms.

use geometry_strategy::{
    AzimuthStrategy, DestinationStrategy, DistanceStrategy, LengthStrategy, Rhumb,
};
use geometry_trait::{Geometry, Linestring, Point};

/// Rhumb-line distance using the mean-Earth-radius default.
#[inline]
#[must_use]
pub fn rhumb_distance<A, B>(first: &A, second: &B) -> f64
where
    A: Geometry,
    B: Geometry,
    Rhumb: DistanceStrategy<A, B, Out = f64>,
{
    rhumb_distance_with(first, second, Rhumb::default())
}

/// Rhumb-line distance using an explicit metric strategy.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "rhumb strategies are small Copy radius configurations"
)]
pub fn rhumb_distance_with<A, B, S>(first: &A, second: &B, strategy: S) -> S::Out
where
    A: Geometry,
    B: Geometry,
    S: DistanceStrategy<A, B>,
{
    strategy.distance(first, second)
}

/// Constant compass bearing in radians, clockwise from north.
#[inline]
#[must_use]
pub fn rhumb_azimuth<P1, P2>(first: &P1, second: &P2) -> f64
where
    P1: Point,
    P2: Point,
    Rhumb: AzimuthStrategy<P1, P2, Out = f64>,
{
    rhumb_azimuth_with(first, second, Rhumb::default())
}

/// Constant compass bearing using an explicit rhumb strategy.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "rhumb strategies are small Copy radius configurations"
)]
pub fn rhumb_azimuth_with<P1, P2, S>(first: &P1, second: &P2, strategy: S) -> S::Out
where
    P1: Point,
    P2: Point,
    S: AzimuthStrategy<P1, P2>,
{
    strategy.azimuth(first, second)
}

/// Destination reached along a constant-bearing rhumb line.
#[inline]
#[must_use]
pub fn rhumb_destination<P>(origin: &P, bearing: f64, distance: f64) -> PointOutput<P>
where
    P: Point,
    Rhumb: DestinationStrategy<P>,
{
    rhumb_destination_with(origin, bearing, distance, Rhumb::default())
}

/// Rhumb destination using an explicit radius strategy.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "rhumb strategies are small Copy radius configurations"
)]
pub fn rhumb_destination_with<P, S>(
    origin: &P,
    bearing: f64,
    distance: f64,
    strategy: S,
) -> S::Output
where
    P: Point,
    S: DestinationStrategy<P>,
{
    strategy.destination(origin, bearing, distance)
}

/// Sum rhumb-line distances along a linestring.
#[inline]
#[must_use]
pub fn rhumb_length<L>(line: &L) -> f64
where
    L: Linestring,
    Rhumb: LengthStrategy<L, Out = f64>,
{
    rhumb_length_with(line, Rhumb::default())
}

/// Sum rhumb-line distances using an explicit radius strategy.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "rhumb strategies are small Copy radius configurations"
)]
pub fn rhumb_length_with<L, S>(line: &L, strategy: S) -> S::Out
where
    L: Geometry,
    S: LengthStrategy<L>,
{
    strategy.length(line)
}

type PointOutput<P> = <Rhumb as DestinationStrategy<P>>::Output;

#[cfg(test)]
mod tests {
    use geometry_cs::{Degree, Spherical};
    use geometry_model::{Linestring, Point2D};
    use geometry_trait::Point as _;

    use super::*;

    #[test]
    fn named_entries_share_one_metric() {
        type P = Point2D<f64, Spherical<Degree>>;
        let start = P::new(0.0, 0.0);
        let east = P::new(1.0, 0.0);
        let distance = rhumb_distance(&start, &east);
        assert!((rhumb_azimuth(&start, &east) - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
        let endpoint = rhumb_destination(&start, core::f64::consts::FRAC_PI_2, distance);
        assert!((endpoint.get::<0>() - 1.0).abs() < 1e-10);
        let line = Linestring::from_vec(alloc::vec![start, east]);
        assert!((rhumb_length(&line) - distance).abs() < 1e-9);
    }
}
