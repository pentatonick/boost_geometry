//! Chaikin corner-cutting smoothing.
//!
//! Each iteration replaces an edge with points one quarter and three quarters
//! along it. Open lines retain their endpoints; rings remain rings and preserve
//! their declared closure and orientation.

use alloc::vec::Vec;

use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{Linestring, Polygon, Ring};
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut};

/// Apply `iterations` rounds of Chaikin corner cutting.
#[inline]
#[must_use]
pub fn chaikin_smoothing<G>(geometry: &G, iterations: usize) -> G::Output
where
    G: ChaikinSmoothing,
{
    geometry.chaikin_smoothing(iterations)
}

/// Per-model Chaikin dispatch.
#[doc(hidden)]
pub trait ChaikinSmoothing {
    /// Smoothed geometry type.
    type Output;

    /// Apply the requested number of iterations.
    fn chaikin_smoothing(&self, iterations: usize) -> Self::Output;
}

impl<P> ChaikinSmoothing for Linestring<P>
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = Linestring<P>;

    fn chaikin_smoothing(&self, iterations: usize) -> Self::Output {
        let mut points = self.0.clone();
        for _ in 0..iterations {
            points = smooth_open(&points);
        }
        Linestring::from_vec(points)
    }
}

impl<P, const CW: bool, const CL: bool> ChaikinSmoothing for Ring<P, CW, CL>
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = Ring<P, CW, CL>;

    fn chaikin_smoothing(&self, iterations: usize) -> Self::Output {
        let mut points = self.0.clone();
        for _ in 0..iterations {
            points = smooth_ring::<P, CL>(&points);
        }
        Ring::from_vec(points)
    }
}

impl<P, const CW: bool, const CL: bool> ChaikinSmoothing for Polygon<P, CW, CL>
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = Polygon<P, CW, CL>;

    fn chaikin_smoothing(&self, iterations: usize) -> Self::Output {
        Polygon::with_inners(
            self.outer.chaikin_smoothing(iterations),
            self.inners
                .iter()
                .map(|ring| ring.chaikin_smoothing(iterations))
                .collect(),
        )
    }
}

fn smooth_open<P>(points: &[P]) -> Vec<P>
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
{
    if points.len() < 2 {
        return points.to_vec();
    }
    let mut output = Vec::with_capacity(points.len() * 2);
    output.push(points[0]);
    for edge in points.windows(2) {
        output.push(blend(&edge[0], &edge[1], 0.25));
        output.push(blend(&edge[0], &edge[1], 0.75));
    }
    output.push(*points.last().expect("non-empty line checked above"));
    output
}

fn smooth_ring<P, const CLOSED: bool>(points: &[P]) -> Vec<P>
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
{
    let unique_len = if points.len() > 1 && same_xy(&points[0], &points[points.len() - 1]) {
        points.len() - 1
    } else {
        points.len()
    };
    if unique_len < 3 {
        return points.to_vec();
    }

    let mut output = Vec::with_capacity(unique_len * 2 + usize::from(CLOSED));
    for index in 0..unique_len {
        let next = (index + 1) % unique_len;
        output.push(blend(&points[index], &points[next], 0.25));
        output.push(blend(&points[index], &points[next], 0.75));
    }
    if CLOSED {
        output.push(output[0]);
    }
    output
}

fn blend<P>(first: &P, second: &P, fraction: f64) -> P
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

#[allow(
    clippy::float_cmp,
    reason = "ring closure is represented by exact endpoint identity, matching the model's closure convention"
)]
fn same_xy<P: Point<Scalar = f64>>(first: &P, second: &P) -> bool {
    first.get::<0>() == second.get::<0>() && first.get::<1>() == second.get::<1>()
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

#[cfg(test)]
mod tests {
    use super::chaikin_smoothing;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, Ring};

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn open_line_keeps_endpoints_and_closed_ring_stays_closed() {
        let line = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(2.0, 0.0)]);
        let smoothed = chaikin_smoothing(&line, 1);
        assert_eq!(smoothed.0.first(), Some(&P::new(0.0, 0.0)));
        assert_eq!(smoothed.0.last(), Some(&P::new(2.0, 0.0)));

        let ring: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 0.0),
        ]);
        let smoothed = chaikin_smoothing(&ring, 1);
        assert_eq!(smoothed.0.first(), smoothed.0.last());
        assert_eq!(smoothed.0.len(), 7);
    }
}
