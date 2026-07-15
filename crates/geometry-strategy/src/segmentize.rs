//! Equal-length linestring subdivision strategies.
//!
//! Cartesian subdivision interpolates each edge linearly. The spherical
//! implementation uses Haversine lengths and great-circle interpolation, so
//! explicit strategy selection changes both measurement and cut placement.

use alloc::{vec, vec::Vec};

use geometry_cs::{CartesianFamily, CoordinateSystem, SphericalFamily};
use geometry_model::{Linestring as ModelLinestring, MultiLinestring};
use geometry_tag::SameAs;
use geometry_trait::{Linestring, Point, PointMut};

use crate::{DistanceStrategy, Haversine, Pythagoras};

#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};
#[cfg(feature = "std")]
use geometry_cs::AngleUnit;

/// Strategy for splitting a linestring into equal-length pieces.
pub trait SegmentizeStrategy<L> {
    /// Segmented output geometry.
    type Output;

    /// Split `line` into `count` pieces.
    fn segmentize(&self, line: &L, count: usize) -> Self::Output;
}

/// Cartesian length and linear-interpolation segmentization.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianSegmentize;

impl<L, P> SegmentizeStrategy<L> for CartesianSegmentize
where
    L: Linestring<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    Pythagoras: DistanceStrategy<P, P, Out = f64>,
{
    type Output = MultiLinestring<ModelLinestring<P>>;

    fn segmentize(&self, line: &L, count: usize) -> Self::Output {
        segmentize(line, count, self)
    }
}

impl<P> SegmentMetric<P> for CartesianSegmentize
where
    P: Point<Scalar = f64> + PointMut + Default,
    Pythagoras: DistanceStrategy<P, P, Out = f64>,
{
    fn distance(&self, first: &P, second: &P) -> f64 {
        Pythagoras.distance(first, second)
    }

    fn interpolate(&self, first: &P, second: &P, fraction: f64) -> P {
        linear_interpolate(first, second, fraction)
    }
}

#[cfg(feature = "std")]
impl<L, P> SegmentizeStrategy<L> for Haversine
where
    L: Linestring<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    P::Cs: HasAngularUnits,
    <P::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
    Haversine: DistanceStrategy<P, P, Out = f64>,
{
    type Output = MultiLinestring<ModelLinestring<P>>;

    fn segmentize(&self, line: &L, count: usize) -> Self::Output {
        segmentize(line, count, self)
    }
}

#[cfg(feature = "std")]
impl<P> SegmentMetric<P> for Haversine
where
    P: Point<Scalar = f64> + PointMut + Default,
    P::Cs: HasAngularUnits,
    Haversine: DistanceStrategy<P, P, Out = f64>,
{
    fn distance(&self, first: &P, second: &P) -> f64 {
        DistanceStrategy::distance(self, first, second)
    }

    fn interpolate(&self, first: &P, second: &P, fraction: f64) -> P {
        great_circle_interpolate(first, second, fraction, self.radius)
    }
}

trait SegmentMetric<P> {
    fn distance(&self, first: &P, second: &P) -> f64;
    fn interpolate(&self, first: &P, second: &P, fraction: f64) -> P;
}

#[allow(
    clippy::cast_precision_loss,
    reason = "piece counts become normalized f64 fractions"
)]
fn segmentize<L, P, M>(line: &L, count: usize, metric: &M) -> MultiLinestring<ModelLinestring<P>>
where
    L: Linestring<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    M: SegmentMetric<P>,
{
    let points: Vec<P> = line.points().copied().collect();
    if count == 0 || points.len() < 2 {
        return MultiLinestring(Vec::new());
    }

    let mut cumulative = Vec::with_capacity(points.len());
    cumulative.push(0.0);
    for edge in points.windows(2) {
        let next = cumulative.last().copied().unwrap_or(0.0) + metric.distance(&edge[0], &edge[1]);
        cumulative.push(next);
    }
    // `cumulative` is initialized with the zero-distance origin above.
    let total = *cumulative
        .last()
        .expect("cumulative distances are non-empty");
    if total == 0.0 {
        return MultiLinestring(vec![ModelLinestring::from_vec(points)]);
    }

    let mut pieces = Vec::with_capacity(count);
    for piece_index in 0..count {
        let start_distance = total * piece_index as f64 / count as f64;
        let end_distance = total * (piece_index + 1) as f64 / count as f64;
        let mut piece = Vec::new();
        piece.push(point_at_distance(
            &points,
            &cumulative,
            start_distance,
            metric,
        ));
        for (index, distance) in cumulative.iter().copied().enumerate().skip(1) {
            if distance > start_distance && distance < end_distance {
                piece.push(points[index]);
            }
        }
        piece.push(point_at_distance(
            &points,
            &cumulative,
            end_distance,
            metric,
        ));
        pieces.push(ModelLinestring::from_vec(piece));
    }
    MultiLinestring(pieces)
}

fn point_at_distance<P, M>(points: &[P], cumulative: &[f64], distance: f64, metric: &M) -> P
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    M: SegmentMetric<P>,
{
    if distance <= 0.0 {
        return points[0];
    }
    let total = cumulative
        .last()
        .copied()
        .expect("segmentization builds one cumulative distance per input point");
    if distance >= total {
        return *points.last().unwrap_or(&points[0]);
    }
    let edge_index = cumulative
        .windows(2)
        .position(|range| distance <= range[1])
        .unwrap_or(cumulative.len().saturating_sub(2));
    let edge_length = cumulative[edge_index + 1] - cumulative[edge_index];
    // A positive in-range distance selects the first cumulative interval
    // ending at that distance; zero-length plateaus are therefore skipped.
    metric.interpolate(
        &points[edge_index],
        &points[edge_index + 1],
        (distance - cumulative[edge_index]) / edge_length,
    )
}

fn linear_interpolate<P>(first: &P, second: &P, fraction: f64) -> P
where
    P: Point<Scalar = f64> + PointMut + Default,
{
    let mut output = P::default();
    geometry_trait::fold_dims((), first, |(), _, dimension| {
        let first_value = get_dimension(first, dimension);
        let second_value = get_dimension(second, dimension);
        set_dimension(
            &mut output,
            dimension,
            first_value + fraction * (second_value - first_value),
        );
    });
    output
}

#[cfg(feature = "std")]
fn great_circle_interpolate<P>(first: &P, second: &P, fraction: f64, radius: f64) -> P
where
    P: Point<Scalar = f64> + PointMut + Default,
    P::Cs: HasAngularUnits,
    Haversine: DistanceStrategy<P, P, Out = f64>,
{
    type Units<P> = <<P as Point>::Cs as HasAngularUnits>::Units;
    let metric = Haversine { radius };
    let angle = DistanceStrategy::distance(&metric, first, second) / radius;
    let sine = angle.sin();
    if sine.abs() < f64::EPSILON {
        return linear_interpolate(first, second, fraction);
    }

    let (longitude1, latitude1) = lonlat_radians(first);
    let (longitude2, latitude2) = lonlat_radians(second);
    let first_weight = ((1.0 - fraction) * angle).sin() / sine;
    let second_weight = (fraction * angle).sin() / sine;
    let x = first_weight * latitude1.cos() * longitude1.cos()
        + second_weight * latitude2.cos() * longitude2.cos();
    let y = first_weight * latitude1.cos() * longitude1.sin()
        + second_weight * latitude2.cos() * longitude2.sin();
    let z = first_weight * latitude1.sin() + second_weight * latitude2.sin();
    let longitude = y.atan2(x);
    let latitude = z.atan2(x.hypot(y));

    let mut output = linear_interpolate(first, second, fraction);
    output.set::<0>(Units::<P>::from_radians(longitude));
    output.set::<1>(Units::<P>::from_radians(latitude));
    output
}

fn get_dimension<P: Point<Scalar = f64>>(point: &P, dimension: usize) -> f64 {
    match dimension {
        0 => point.get::<0>(),
        1 => point.get::<1>(),
        2 => point.get::<2>(),
        3 => point.get::<3>(),
        _ => unreachable!("point folds are limited to four dimensions"),
    }
}

fn set_dimension<P: PointMut<Scalar = f64>>(point: &mut P, dimension: usize, value: f64) {
    match dimension {
        0 => point.set::<0>(value),
        1 => point.set::<1>(value),
        2 => point.set::<2>(value),
        3 => point.set::<3>(value),
        _ => unreachable!("point folds are limited to four dimensions"),
    }
}
