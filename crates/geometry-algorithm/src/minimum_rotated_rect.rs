//! Minimum-area rotated rectangle around a planar geometry.
//!
//! Boost.Geometry has no rotated-envelope entry. This implementation follows
//! Toussaint's rotating-calipers construction (1983): compute the convex hull,
//! align an orthogonal frame with every hull edge, and retain the frame with
//! minimum projected area.

use alloc::vec;

#[cfg(not(feature = "std"))]
use geometry_coords::math::Float;
use geometry_model::{Polygon, Ring};
use geometry_strategy::{ConvexHullStrategy, MonotoneChain};
use geometry_trait::{Point, PointMut};

use crate::convex_hull::convex_hull;

/// Compute the minimum-area rotated rectangle enclosing `geometry`.
///
/// This is Cartesian-only through the convex-hull strategy bound. Empty input
/// returns an empty polygon; one- and two-point inputs return a closed,
/// zero-area degenerate rectangle.
#[inline]
#[must_use]
pub fn minimum_rotated_rect<G, P>(geometry: &G) -> Polygon<P>
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    MonotoneChain: ConvexHullStrategy<G, Output = Ring<P, true, true>>,
{
    let hull = convex_hull(geometry);
    let mut points = hull.0;
    if points.len() > 1 && same_xy(points.first(), points.last()) {
        points.pop();
    }
    match points.len() {
        0 => return Polygon::default(),
        1 => {
            let point = points[0];
            return Polygon::new(Ring::from_vec(vec![point, point, point, point, point]));
        }
        2 => {
            let first = points[0];
            let second = points[1];
            return Polygon::new(Ring::from_vec(vec![first, first, second, second, first]));
        }
        _ => {}
    }

    let mut best: Option<Frame> = None;
    for index in 0..points.len() {
        let first = points[index];
        let second = points[(index + 1) % points.len()];
        let dx = second.get::<0>() - first.get::<0>();
        let dy = second.get::<1>() - first.get::<1>();
        let length = dx.hypot(dy);
        if length <= f64::EPSILON {
            continue;
        }
        let ux = dx / length;
        let uy = dy / length;
        let vx = -uy;
        let vy = ux;
        let mut min_u = f64::INFINITY;
        let mut max_u = f64::NEG_INFINITY;
        let mut min_v = f64::INFINITY;
        let mut max_v = f64::NEG_INFINITY;
        for point in &points {
            let x = point.get::<0>();
            let y = point.get::<1>();
            let along = x * ux + y * uy;
            let across = x * vx + y * vy;
            min_u = min_u.min(along);
            max_u = max_u.max(along);
            min_v = min_v.min(across);
            max_v = max_v.max(across);
        }
        let frame = Frame {
            ux,
            uy,
            vx,
            vy,
            min_u,
            max_u,
            min_v,
            max_v,
        };
        if best.is_none_or(|current| frame.area() < current.area()) {
            best = Some(frame);
        }
    }

    let Some(frame) = best else {
        return Polygon::default();
    };
    let lower_left = frame.point::<P>(frame.min_u, frame.min_v);
    let upper_left = frame.point::<P>(frame.min_u, frame.max_v);
    let upper_right = frame.point::<P>(frame.max_u, frame.max_v);
    let lower_right = frame.point::<P>(frame.max_u, frame.min_v);
    Polygon::new(Ring::from_vec(vec![
        lower_left,
        upper_left,
        upper_right,
        lower_right,
        lower_left,
    ]))
}

#[derive(Clone, Copy)]
struct Frame {
    ux: f64,
    uy: f64,
    vx: f64,
    vy: f64,
    min_u: f64,
    max_u: f64,
    min_v: f64,
    max_v: f64,
}

impl Frame {
    fn area(self) -> f64 {
        (self.max_u - self.min_u) * (self.max_v - self.min_v)
    }

    fn point<P>(self, along: f64, across: f64) -> P
    where
        P: Point<Scalar = f64> + PointMut + Default,
    {
        let mut point = P::default();
        point.set::<0>(along * self.ux + across * self.vx);
        point.set::<1>(along * self.uy + across * self.vy);
        point
    }
}

#[allow(
    clippy::float_cmp,
    reason = "coordinate identity is used only to detect the closing duplicate"
)]
fn same_xy<P: Point<Scalar = f64>>(first: Option<&P>, second: Option<&P>) -> bool {
    first.zip(second).is_some_and(|(first, second)| {
        first.get::<0>() == second.get::<0>() && first.get::<1>() == second.get::<1>()
    })
}

#[cfg(test)]
mod tests {
    use geometry_cs::Cartesian;
    use geometry_model::{MultiPoint, Point2D};

    use super::minimum_rotated_rect;
    use crate::area::area;

    #[test]
    fn diamond_has_area_two() {
        type P = Point2D<f64, Cartesian>;
        let points = MultiPoint::from_vec(alloc::vec![
            P::new(0.0, 1.0),
            P::new(1.0, 0.0),
            P::new(0.0, -1.0),
            P::new(-1.0, 0.0),
        ]);
        let rectangle = minimum_rotated_rect(&points);
        assert!((area(&rectangle).abs() - 2.0).abs() < 1e-12);
    }
}
