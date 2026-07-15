//! Locate the closest point as a fraction of Cartesian linestring length.

use alloc::vec::Vec;

use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::Segment;
use geometry_strategy::{
    CartesianClosestPoints, ClosestPointsStrategy, DistanceStrategy, Pythagoras,
};
use geometry_tag::SameAs;
use geometry_trait::{Linestring, Point, PointMut};

/// Return the fractional arc-length position nearest to `point`.
///
/// Returns `None` for an empty linestring and `Some(0.0)` for a single-point or
/// zero-length linestring. Ties retain the earliest position along the line.
#[must_use]
pub fn line_locate_point<L, P>(line: &L, point: &P) -> Option<f64>
where
    L: Linestring<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    CartesianClosestPoints: ClosestPointsStrategy<P, Segment<P>, Out = P>,
    Pythagoras: DistanceStrategy<P, P, Out = f64>,
{
    let points: Vec<P> = line.points().copied().collect();
    if points.is_empty() {
        return None;
    }
    if points.len() == 1 {
        return Some(0.0);
    }

    let total: f64 = points
        .windows(2)
        .map(|edge| Pythagoras.distance(&edge[0], &edge[1]))
        .sum();
    if total == 0.0 {
        return Some(0.0);
    }

    let mut elapsed = 0.0;
    let mut best_distance = f64::INFINITY;
    let mut best_position = 0.0;
    for edge in points.windows(2) {
        let segment = Segment::new(edge[0], edge[1]);
        let (_, projected) = CartesianClosestPoints.closest_points(point, &segment);
        let candidate_distance = Pythagoras.distance(point, &projected);
        if candidate_distance < best_distance {
            best_distance = candidate_distance;
            best_position = elapsed + Pythagoras.distance(&edge[0], &projected);
        }
        elapsed += Pythagoras.distance(&edge[0], &edge[1]);
    }
    Some((best_position / total).clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::line_locate_point;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D};

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn empty_and_bent_lines_have_defined_results() {
        assert_eq!(
            line_locate_point(&Linestring::<P>::new(), &P::new(0.0, 0.0)),
            None
        );
        let line = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(2.0, 0.0), P::new(2.0, 2.0)]);
        assert_eq!(line_locate_point(&line, &P::new(2.5, 1.0)), Some(0.75));
    }
}
