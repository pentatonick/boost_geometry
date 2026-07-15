//! Concave hulls derived from a planar point set.
//!
//! Boost.Geometry has no concave-hull algorithm. The edge-refinement shape is
//! based on Park & Oh (2012), while the `k`-nearest entry exposes the candidate
//! breadth described by Moreira & Santos (2007). Both variants begin with the
//! existing convex hull and preserve a simple clockwise boundary.

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use geometry_coords::math::Float;
use geometry_coords::precise_math;
use geometry_model::{Polygon, Ring};
use geometry_strategy::{CollectPoints, ConvexHullStrategy, MonotoneChain};
use geometry_trait::{Point, PointMut};

use crate::convex_hull::convex_hull;

/// Parameters controlling edge-refinement concave hulls.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcaveHullParams {
    /// Maximum ratio `(a→candidate + candidate→b) / a→b` accepted for an edge.
    /// Values below `1` are clamped to `1` at the algorithm boundary.
    pub concavity: f64,
    /// Edges no longer than this threshold are left unchanged.
    pub length_threshold: f64,
}

impl Default for ConcaveHullParams {
    fn default() -> Self {
        Self {
            concavity: 2.0,
            length_threshold: 0.0,
        }
    }
}

/// Construct a concave hull with [`ConcaveHullParams::default`].
#[inline]
#[must_use]
pub fn concave_hull<G, P>(geometry: &G) -> Polygon<P>
where
    G: CollectPoints<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    MonotoneChain: ConvexHullStrategy<G, Output = Ring<P, true, true>>,
{
    concave_hull_with(geometry, ConcaveHullParams::default())
}

/// Construct a concave hull using explicit edge-refinement parameters.
#[inline]
#[must_use]
pub fn concave_hull_with<G, P>(geometry: &G, parameters: ConcaveHullParams) -> Polygon<P>
where
    G: CollectPoints<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    MonotoneChain: ConvexHullStrategy<G, Output = Ring<P, true, true>>,
{
    refine_hull(geometry, parameters, None)
}

/// Construct a concave hull while considering at most `k` nearest candidates
/// for each boundary edge.
///
/// Candidate distance is measured from the edge midpoint. `k == 0` leaves the
/// convex hull unchanged; larger values admit progressively more refinements.
#[inline]
#[must_use]
pub fn k_nearest_concave_hull<G, P>(geometry: &G, k: usize) -> Polygon<P>
where
    G: CollectPoints<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    MonotoneChain: ConvexHullStrategy<G, Output = Ring<P, true, true>>,
{
    if k == 0 {
        return Polygon::new(convex_hull(geometry));
    }
    refine_hull(
        geometry,
        ConcaveHullParams {
            concavity: f64::INFINITY,
            length_threshold: 0.0,
        },
        Some(k),
    )
}

fn refine_hull<G, P>(
    geometry: &G,
    parameters: ConcaveHullParams,
    nearest_limit: Option<usize>,
) -> Polygon<P>
where
    G: CollectPoints<Point = P>,
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    MonotoneChain: ConvexHullStrategy<G, Output = Ring<P, true, true>>,
{
    let mut all_points = Vec::new();
    geometry.collect_points(&mut all_points);
    deduplicate(&mut all_points);

    let mut boundary = convex_hull(geometry).0;
    while boundary.len() > 1 && same_xy(boundary.first(), boundary.last()) {
        boundary.pop();
    }
    if boundary.len() < 3 {
        close(&mut boundary);
        return Polygon::new(Ring::from_vec(boundary));
    }

    let mut candidates: Vec<P> = all_points
        .into_iter()
        .filter(|point| !boundary.iter().any(|hull| same_point(point, hull)))
        .collect();
    let concavity = parameters.concavity.max(1.0);
    let length_threshold = parameters.length_threshold.max(0.0);

    while !candidates.is_empty() {
        let mut best: Option<Insertion> = None;
        for edge in 0..boundary.len() {
            let first = boundary[edge];
            let second = boundary[(edge + 1) % boundary.len()];
            let edge_length = distance(first, second);
            if edge_length <= length_threshold.max(f64::EPSILON) {
                continue;
            }

            let mut candidate_indices: Vec<usize> = (0..candidates.len()).collect();
            candidate_indices.sort_by(|&left, &right| {
                midpoint_distance(first, second, candidates[left]).total_cmp(&midpoint_distance(
                    first,
                    second,
                    candidates[right],
                ))
            });
            if let Some(limit) = nearest_limit {
                candidate_indices.truncate(limit.min(candidate_indices.len()));
            }

            for candidate_index in candidate_indices {
                let candidate = candidates[candidate_index];
                let detour =
                    (distance(first, candidate) + distance(candidate, second)) / edge_length;
                if detour > concavity
                    || point_segment_distance(candidate, first, second) <= f64::EPSILON
                    || !insertion_is_simple(&boundary, edge, candidate)
                {
                    continue;
                }
                let score = point_segment_distance(candidate, first, second);
                let insertion = Insertion {
                    edge,
                    candidate: candidate_index,
                    score,
                };
                if best.is_none_or(|current| insertion.score < current.score) {
                    best = Some(insertion);
                }
            }
        }

        let Some(insertion) = best else {
            break;
        };
        let point = candidates.swap_remove(insertion.candidate);
        boundary.insert(insertion.edge + 1, point);
    }

    close(&mut boundary);
    Polygon::new(Ring::from_vec(boundary))
}

#[derive(Clone, Copy)]
struct Insertion {
    edge: usize,
    candidate: usize,
    score: f64,
}

fn deduplicate<P: Point<Scalar = f64> + Copy>(points: &mut Vec<P>) {
    let mut unique = Vec::with_capacity(points.len());
    for point in points.iter().copied() {
        if !unique.iter().any(|other| same_point(&point, other)) {
            unique.push(point);
        }
    }
    *points = unique;
}

fn close<P: Copy>(points: &mut Vec<P>) {
    if let Some(first) = points.first().copied() {
        points.push(first);
    }
}

fn insertion_is_simple<P>(boundary: &[P], edge: usize, candidate: P) -> bool
where
    P: Point<Scalar = f64> + Copy,
{
    let first = boundary[edge];
    let second_index = (edge + 1) % boundary.len();
    let second = boundary[second_index];
    for other_edge in 0..boundary.len() {
        if other_edge == edge {
            continue;
        }
        let other_first_index = other_edge;
        let other_second_index = (other_edge + 1) % boundary.len();
        let other_first = boundary[other_first_index];
        let other_second = boundary[other_second_index];

        let first_segment_shares_endpoint = other_first_index == edge || other_second_index == edge;
        if !first_segment_shares_endpoint
            && segments_intersect(first, candidate, other_first, other_second)
        {
            return false;
        }
        let second_segment_shares_endpoint =
            other_first_index == second_index || other_second_index == second_index;
        if !second_segment_shares_endpoint
            && segments_intersect(candidate, second, other_first, other_second)
        {
            return false;
        }
    }
    true
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
    reason = "the hull operates on Copy point handles throughout"
)]
fn orientation<P: Point<Scalar = f64>>(first: P, second: P, third: P) -> f64 {
    precise_math::orient2d(
        [first.get::<0>(), first.get::<1>()],
        [second.get::<0>(), second.get::<1>()],
        [third.get::<0>(), third.get::<1>()],
    )
}

#[allow(
    clippy::needless_pass_by_value,
    reason = "the hull operates on Copy point handles throughout"
)]
fn on_segment<P: Point<Scalar = f64>>(first: P, second: P, point: P) -> bool {
    point.get::<0>() >= first.get::<0>().min(second.get::<0>())
        && point.get::<0>() <= first.get::<0>().max(second.get::<0>())
        && point.get::<1>() >= first.get::<1>().min(second.get::<1>())
        && point.get::<1>() <= first.get::<1>().max(second.get::<1>())
}

#[allow(
    clippy::needless_pass_by_value,
    reason = "the hull operates on Copy point handles throughout"
)]
fn midpoint_distance<P: Point<Scalar = f64>>(first: P, second: P, point: P) -> f64 {
    let x = first.get::<0>() / 2.0 + second.get::<0>() / 2.0 - point.get::<0>();
    let y = first.get::<1>() / 2.0 + second.get::<1>() / 2.0 - point.get::<1>();
    x.hypot(y)
}

#[allow(
    clippy::needless_pass_by_value,
    reason = "the hull operates on Copy point handles throughout"
)]
fn point_segment_distance<P: Point<Scalar = f64>>(point: P, first: P, second: P) -> f64 {
    let dx = second.get::<0>() - first.get::<0>();
    let dy = second.get::<1>() - first.get::<1>();
    let length_squared = dx * dx + dy * dy;
    if length_squared <= f64::EPSILON {
        return distance(point, first);
    }
    let projection = ((point.get::<0>() - first.get::<0>()) * dx
        + (point.get::<1>() - first.get::<1>()) * dy)
        / length_squared;
    let projection = projection.clamp(0.0, 1.0);
    let x = first.get::<0>() + projection * dx;
    let y = first.get::<1>() + projection * dy;
    (point.get::<0>() - x).hypot(point.get::<1>() - y)
}

#[allow(
    clippy::needless_pass_by_value,
    reason = "the hull operates on Copy point handles throughout"
)]
fn distance<P: Point<Scalar = f64>>(first: P, second: P) -> f64 {
    (second.get::<0>() - first.get::<0>()).hypot(second.get::<1>() - first.get::<1>())
}

fn same_xy<P: Point<Scalar = f64>>(first: Option<&P>, second: Option<&P>) -> bool {
    first
        .zip(second)
        .is_some_and(|(first, second)| same_point(first, second))
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
    use geometry_model::{MultiPoint, Point2D};

    use super::*;
    use crate::area::area;

    #[test]
    fn square_digs_toward_an_interior_point() {
        type P = Point2D<f64, Cartesian>;
        let points = MultiPoint::from_vec(alloc::vec![
            P::new(0.0, 0.0),
            P::new(0.0, 4.0),
            P::new(4.0, 4.0),
            P::new(4.0, 0.0),
            P::new(2.0, 1.0),
        ]);
        let hull = concave_hull_with(
            &points,
            ConcaveHullParams {
                concavity: 1.2,
                length_threshold: 0.0,
            },
        );
        assert!(hull.outer.0.contains(&P::new(2.0, 1.0)));
        assert!(area(&hull).abs() < 16.0);
    }

    #[test]
    fn private_intersection_guards_cover_invalid_insertions() {
        type P = Point2D<f64, Cartesian>;
        let boundary = [
            P::new(0.0, 0.0),
            P::new(0.0, 4.0),
            P::new(4.0, 4.0),
            P::new(4.0, 0.0),
        ];
        assert!(!insertion_is_simple(&boundary, 0, P::new(5.0, 2.0)));
        assert!(!insertion_is_simple(&boundary, 0, P::new(5.0, -1.0)));

        assert!(segments_intersect(
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
            P::new(1.0, 0.0),
            P::new(1.0, 1.0),
        ));
        assert!(segments_intersect(
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
            P::new(1.0, 1.0),
            P::new(1.0, 0.0),
        ));
        assert!(segments_intersect(
            P::new(1.0, 0.0),
            P::new(1.0, 1.0),
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
        ));
        assert!(segments_intersect(
            P::new(1.0, 1.0),
            P::new(1.0, 0.0),
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
        ));
        let distance = point_segment_distance(P::new(3.0, 4.0), P::new(0.0, 0.0), P::new(0.0, 0.0));
        assert!((distance - 5.0).abs() < f64::EPSILON);
    }
}
