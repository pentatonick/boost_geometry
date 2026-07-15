//! Native ear-clipping triangulation for Cartesian polygons.
//!
//! Boost.Geometry has no triangulation entry. This implementation follows the
//! classic Meisters ear-clipping method, using the adaptive exact-sign
//! [`geometry_coords::precise_math::orient2d`] predicate for every turn test.
//! Interior rings are connected to the exterior by non-crossing visibility
//! bridges before clipping.

use alloc::{vec, vec::Vec};

use geometry_coords::precise_math;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{Polygon as ModelPolygon, Ring as ModelRing};
use geometry_tag::SameAs;
use geometry_trait::{Point, Polygon, Ring};

/// Triangulate a Cartesian polygon into owned clockwise, closed stock polygons.
///
/// Degenerate rings and interiors that cannot be bridged into the exterior
/// return an empty vector. Every returned polygon has exactly three distinct
/// vertices plus the closing duplicate.
#[inline]
#[must_use]
pub fn triangulate_earcut<Pg, P>(polygon: &Pg) -> Vec<ModelPolygon<P>>
where
    Pg: Polygon<Point = P>,
    P: Point<Scalar = f64> + Copy,
    P::Cs: CoordinateSystem,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let mut exterior = ring_vertices(polygon.exterior());
    if exterior.len() < 3 {
        return Vec::new();
    }
    make_counter_clockwise(&mut exterior);

    let mut holes: Vec<Vec<P>> = polygon
        .interiors()
        .map(ring_vertices)
        .filter(|ring| ring.len() >= 3)
        .collect();
    for hole in &mut holes {
        make_clockwise(hole);
    }
    for hole_index in 0..holes.len() {
        let Some(merged) = bridge_hole(&exterior, &holes[hole_index], &holes) else {
            return Vec::new();
        };
        exterior = merged;
    }

    clip_ears(&exterior)
}

fn ring_vertices<R, P>(ring: &R) -> Vec<P>
where
    R: Ring<Point = P>,
    P: Point<Scalar = f64> + Copy,
{
    let mut points: Vec<P> = ring.points().copied().collect();
    while points.len() > 1 && same_point(&points[0], points.last().unwrap_or(&points[0])) {
        points.pop();
    }
    points.dedup_by(|second, first| same_point(first, second));
    points
}

fn make_counter_clockwise<P: Point<Scalar = f64>>(points: &mut [P]) {
    if signed_area(points) < 0.0 {
        points.reverse();
    }
}

fn make_clockwise<P: Point<Scalar = f64>>(points: &mut [P]) {
    if signed_area(points) > 0.0 {
        points.reverse();
    }
}

fn signed_area<P: Point<Scalar = f64>>(points: &[P]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for index in 0..points.len() {
        let first = &points[index];
        let second = &points[(index + 1) % points.len()];
        area += first.get::<0>() * second.get::<1>() - second.get::<0>() * first.get::<1>();
    }
    area / 2.0
}

fn bridge_hole<P>(exterior: &[P], hole: &[P], holes: &[Vec<P>]) -> Option<Vec<P>>
where
    P: Point<Scalar = f64> + Copy,
{
    let hole_vertex = hole
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| {
            left.get::<0>()
                .total_cmp(&right.get::<0>())
                .then_with(|| right.get::<1>().total_cmp(&left.get::<1>()))
        })?
        .0;
    let source = hole[hole_vertex];

    let exterior_vertex = (0..exterior.len())
        .filter(|&index| bridge_is_visible(source, exterior[index], index, exterior, hole, holes))
        .min_by(|&left, &right| {
            squared_distance(source, exterior[left])
                .total_cmp(&squared_distance(source, exterior[right]))
        })?;

    let mut merged = Vec::with_capacity(exterior.len() + hole.len() + 2);
    merged.extend_from_slice(&exterior[..=exterior_vertex]);
    merged.push(source);
    for offset in 1..hole.len() {
        merged.push(hole[(hole_vertex + offset) % hole.len()]);
    }
    merged.push(source);
    merged.push(exterior[exterior_vertex]);
    merged.extend_from_slice(&exterior[exterior_vertex + 1..]);
    Some(merged)
}

fn bridge_is_visible<P>(
    source: P,
    target: P,
    target_index: usize,
    exterior: &[P],
    source_hole: &[P],
    holes: &[Vec<P>],
) -> bool
where
    P: Point<Scalar = f64> + Copy,
{
    if same_point(&source, &target) {
        return false;
    }
    for edge in 0..exterior.len() {
        let next = (edge + 1) % exterior.len();
        if edge == target_index || next == target_index {
            continue;
        }
        if segments_intersect(source, target, exterior[edge], exterior[next]) {
            return false;
        }
    }
    for ring in holes {
        for edge in 0..ring.len() {
            let next = (edge + 1) % ring.len();
            if core::ptr::eq(ring.as_slice(), source_hole)
                && (same_point(&ring[edge], &source) || same_point(&ring[next], &source))
            {
                continue;
            }
            if segments_intersect(source, target, ring[edge], ring[next]) {
                return false;
            }
        }
    }
    let midpoint = [
        source.get::<0>() / 2.0 + target.get::<0>() / 2.0,
        source.get::<1>() / 2.0 + target.get::<1>() / 2.0,
    ];
    point_in_ring(midpoint, exterior)
        && holes.iter().all(|ring| {
            core::ptr::eq(ring.as_slice(), source_hole) || !point_in_ring(midpoint, ring)
        })
}

fn clip_ears<P>(points: &[P]) -> Vec<ModelPolygon<P>>
where
    P: Point<Scalar = f64> + Copy,
{
    if points.len() < 3 {
        return Vec::new();
    }
    let mut indices: Vec<usize> = (0..points.len()).collect();
    let mut triangles = Vec::with_capacity(points.len().saturating_sub(2));
    while indices.len() > 3 {
        let mut clipped = false;
        for position in 0..indices.len() {
            let previous = indices[(position + indices.len() - 1) % indices.len()];
            let current = indices[position];
            let next = indices[(position + 1) % indices.len()];
            if !is_ear(points, &indices, previous, current, next) {
                continue;
            }
            triangles.push(triangle(points[previous], points[current], points[next]));
            indices.remove(position);
            clipped = true;
            break;
        }
        if !clipped {
            if let Some(position) = removable_collinear(points, &indices) {
                indices.remove(position);
            } else {
                return Vec::new();
            }
        }
    }
    if orientation(points[indices[0]], points[indices[1]], points[indices[2]]) == 0.0 {
        return Vec::new();
    }
    triangles.push(triangle(
        points[indices[0]],
        points[indices[1]],
        points[indices[2]],
    ));
    triangles
}

fn is_ear<P>(points: &[P], polygon: &[usize], previous: usize, current: usize, next: usize) -> bool
where
    P: Point<Scalar = f64> + Copy,
{
    let a = points[previous];
    let b = points[current];
    let c = points[next];
    if orientation(a, b, c) <= 0.0 {
        return false;
    }
    if diagonal_crosses_polygon(points, polygon, previous, next) {
        return false;
    }
    !polygon.iter().copied().any(|index| {
        index != previous
            && index != current
            && index != next
            && !same_point(&points[index], &a)
            && !same_point(&points[index], &b)
            && !same_point(&points[index], &c)
            && point_in_triangle(points[index], a, b, c)
    })
}

fn diagonal_crosses_polygon<P>(points: &[P], polygon: &[usize], first: usize, second: usize) -> bool
where
    P: Point<Scalar = f64> + Copy,
{
    for edge in 0..polygon.len() {
        let edge_first = polygon[edge];
        let edge_second = polygon[(edge + 1) % polygon.len()];
        if edge_first == first
            || edge_second == first
            || edge_first == second
            || edge_second == second
            || same_point(&points[edge_first], &points[first])
            || same_point(&points[edge_second], &points[first])
            || same_point(&points[edge_first], &points[second])
            || same_point(&points[edge_second], &points[second])
        {
            continue;
        }
        if segments_intersect(
            points[first],
            points[second],
            points[edge_first],
            points[edge_second],
        ) {
            return true;
        }
    }
    false
}

fn removable_collinear<P>(points: &[P], polygon: &[usize]) -> Option<usize>
where
    P: Point<Scalar = f64> + Copy,
{
    (0..polygon.len()).find(|&position| {
        let previous = polygon[(position + polygon.len() - 1) % polygon.len()];
        let current = polygon[position];
        let next = polygon[(position + 1) % polygon.len()];
        same_point(&points[previous], &points[current])
            || same_point(&points[current], &points[next])
            || orientation(points[previous], points[current], points[next]) == 0.0
    })
}

fn triangle<P: Point<Scalar = f64> + Copy>(a: P, b: P, c: P) -> ModelPolygon<P> {
    // Ear clipping works counter-clockwise; swap the final two vertices so the
    // stock polygon's default clockwise order reports positive area.
    ModelPolygon::new(ModelRing::from_vec(vec![a, c, b, a]))
}

fn point_in_triangle<P: Point<Scalar = f64> + Copy>(point: P, a: P, b: P, c: P) -> bool {
    orientation(a, b, point) >= 0.0
        && orientation(b, c, point) >= 0.0
        && orientation(c, a, point) >= 0.0
}

fn point_in_ring<P: Point<Scalar = f64>>(point: [f64; 2], ring: &[P]) -> bool {
    let mut inside = false;
    for index in 0..ring.len() {
        let first = &ring[index];
        let second = &ring[(index + 1) % ring.len()];
        let crosses = (first.get::<1>() > point[1]) != (second.get::<1>() > point[1]);
        if crosses {
            let x = (second.get::<0>() - first.get::<0>()) * (point[1] - first.get::<1>())
                / (second.get::<1>() - first.get::<1>())
                + first.get::<0>();
            if point[0] < x {
                inside = !inside;
            }
        }
    }
    inside
}

fn segments_intersect<P>(a: P, b: P, c: P, d: P) -> bool
where
    P: Point<Scalar = f64> + Copy,
{
    let ab_c = orientation(a, b, c);
    let ab_d = orientation(a, b, d);
    let cd_a = orientation(c, d, a);
    let cd_b = orientation(c, d, b);
    if ab_c == 0.0 && on_segment(a, b, c) {
        return true;
    }
    if ab_d == 0.0 && on_segment(a, b, d) {
        return true;
    }
    if cd_a == 0.0 && on_segment(c, d, a) {
        return true;
    }
    if cd_b == 0.0 && on_segment(c, d, b) {
        return true;
    }
    (ab_c > 0.0) != (ab_d > 0.0) && (cd_a > 0.0) != (cd_b > 0.0)
}

#[allow(
    clippy::needless_pass_by_value,
    reason = "ear clipping operates on Copy point handles throughout"
)]
fn orientation<P: Point<Scalar = f64>>(a: P, b: P, c: P) -> f64 {
    precise_math::orient2d(
        [a.get::<0>(), a.get::<1>()],
        [b.get::<0>(), b.get::<1>()],
        [c.get::<0>(), c.get::<1>()],
    )
}

fn on_segment<P: Point<Scalar = f64> + Copy>(a: P, b: P, point: P) -> bool {
    point.get::<0>() >= a.get::<0>().min(b.get::<0>())
        && point.get::<0>() <= a.get::<0>().max(b.get::<0>())
        && point.get::<1>() >= a.get::<1>().min(b.get::<1>())
        && point.get::<1>() <= a.get::<1>().max(b.get::<1>())
}

fn squared_distance<P: Point<Scalar = f64> + Copy>(first: P, second: P) -> f64 {
    let dx = second.get::<0>() - first.get::<0>();
    let dy = second.get::<1>() - first.get::<1>();
    dx * dx + dy * dy
}

#[allow(
    clippy::float_cmp,
    reason = "coordinate identity, not approximate geometric equality, is required"
)]
fn same_point<P: Point<Scalar = f64>>(first: &P, second: &P) -> bool {
    first.get::<0>() == second.get::<0>() && first.get::<1>() == second.get::<1>()
}

#[cfg(test)]
mod tests {
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, Ring};

    use super::triangulate_earcut;
    use crate::area::area;

    #[test]
    fn concave_pentagon_becomes_three_triangles() {
        type P = Point2D<f64, Cartesian>;
        let polygon: Polygon<P> = Polygon::new(Ring::from_vec(alloc::vec![
            P::new(0.0, 0.0),
            P::new(0.0, 2.0),
            P::new(1.0, 1.0),
            P::new(2.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 0.0),
        ]));
        let triangles = triangulate_earcut(&polygon);
        assert_eq!(triangles.len(), 3);
        let sum: f64 = triangles.iter().map(|triangle| area(triangle).abs()).sum();
        assert!((sum - area(&polygon).abs()).abs() < 1e-12);
    }

    #[test]
    fn square_with_square_hole_preserves_area() {
        type P = Point2D<f64, Cartesian>;
        let outer: Ring<P> = Ring::from_vec(alloc::vec![
            P::new(0.0, 0.0),
            P::new(0.0, 4.0),
            P::new(4.0, 4.0),
            P::new(4.0, 0.0),
            P::new(0.0, 0.0),
        ]);
        let hole: Ring<P> = Ring::from_vec(alloc::vec![
            P::new(1.0, 1.0),
            P::new(3.0, 1.0),
            P::new(3.0, 3.0),
            P::new(1.0, 3.0),
            P::new(1.0, 1.0),
        ]);
        let polygon = Polygon::with_inners(outer, alloc::vec![hole]);

        let triangles = triangulate_earcut(&polygon);

        assert_eq!(triangles.len(), 8);
        let sum: f64 = triangles.iter().map(|triangle| area(triangle).abs()).sum();
        assert!((sum - area(&polygon).abs()).abs() < 1e-12);
    }
}
