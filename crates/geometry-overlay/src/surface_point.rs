//! OVL6.T3 — `point_on_surface`.
//!
//! Mirrors `boost/geometry/algorithms/point_on_surface.hpp`: pick a
//! point **guaranteed to lie in the interior** of an areal geometry.
//! Boost takes the polygon's extreme points — its topmost vertex and the
//! two edges leaving it, cut level at the lower of their ends — narrows
//! them below any part of the boundary that reaches up into them, and
//! averages what is left.
//!
//! Unlike the centroid, this point is always inside a valid polygon even
//! for non-convex or holed shapes — which is exactly why labelling needs
//! it. The overlay samples its own rings with a horizontal sweep instead
//! (`sweep_interior_point`), which needs nothing of a ring's orientation.

use alloc::vec::Vec;
use core::cmp::Ordering;

use geometry_coords::CoordinateScalar;
use geometry_trait::{Point, PointMut, PointOrder, Polygon as PolygonTrait, Ring as RingTrait};

/// A point guaranteed to lie in the interior of a valid `polygon`, or
/// `None` where Boost finds no extreme points to average: an exterior of
/// fewer than three points, or one every point of which is the same.
///
/// Takes the extreme points in `y` — or, where those cannot be had, in `x`
/// — and averages them: the topmost vertex where the boundary turns right
/// and the two edges leaving it, the lower of those cut level with the
/// higher; lowered beneath the highest vertex of any part of the boundary
/// that reaches up between them, or, where such a part reaches the top
/// itself, replaced by a triangle under it.
///
/// Mirrors `boost::geometry::point_on_surface`
/// (`algorithms/point_on_surface.hpp`, `algorithms/detail/extreme_points.hpp`)
/// with Boost's Cartesian side strategy.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::surface_point::point_on_surface;
/// use geometry_trait::Point as _;
///
/// type P = Point2D<f64, Cartesian>;
/// let pg: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
/// let p = point_on_surface(&pg).unwrap();
/// // The representative point is inside the square.
/// assert!(p.get::<0>() > 0.0 && p.get::<0>() < 4.0);
/// assert!(p.get::<1>() > 0.0 && p.get::<1>() < 4.0);
/// ```
#[inline]
#[must_use]
pub fn point_on_surface<G, P>(polygon: &G) -> Option<P>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar<Measure = P::Scalar>,
{
    // C++: first in dimension 1, which succeeds for every valid polygon,
    // then in dimension 0.
    surface_point(polygon, 1).or_else(|| surface_point(polygon, 0))
}

/// C++: `calculate_point_on_surface<Dimension>`.
fn surface_point<G, P>(polygon: &G, dimension: usize) -> Option<P>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    let exterior = SurfaceRing::of(polygon.exterior(), dimension);
    let (mut extremes, left, right) = exterior.extremes()?;
    let mut intruders = Vec::new();
    exterior.intruders(left, right, &extremes, &mut intruders);
    for ring in polygon.interiors() {
        let ring = SurfaceRing::of(ring, dimension);
        if ring.points.len() >= 3 {
            ring.intruders(0, 1, &extremes, &mut intruders);
        }
    }
    if extremes.len() < 3 {
        return None;
    }
    if let Some(max_intruder) = highest(&intruders, dimension, None) {
        let max_extreme = coordinate(&extremes[topmost(&extremes, dimension)], dimension);
        if max_extreme > max_intruder {
            move_range_along(&mut extremes, max_intruder, dimension);
        } else {
            extremes = self_tangency_triangle(extremes, intruders, max_intruder, dimension);
        }
    }
    Some(average(&extremes))
}

/// One ring as `extreme_points_on_ring` walks it: its points as stored,
/// closing point included, visited round and round.
struct SurfaceRing<P> {
    points: Vec<P>,
    dimension: usize,
    /// `1` for a clockwise ring, `-1` for a counter-clockwise one.
    factor: i8,
}

impl<P> SurfaceRing<P>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    fn of<R: RingTrait<Point = P>>(ring: &R, dimension: usize) -> Self {
        Self {
            points: ring.points().copied().collect(),
            dimension,
            factor: if ring.point_order() == PointOrder::Clockwise {
                1
            } else {
                -1
            },
        }
    }

    /// C++: `ever_circling_range_iterator` moved one point, backwards for
    /// a negative `step`.
    fn step(&self, index: usize, step: isize) -> usize {
        let n = self.points.len();
        if step < 0 {
            (index + n - 1) % n
        } else {
            (index + 1) % n
        }
    }

    fn value(&self, index: usize) -> P::Scalar {
        coordinate(&self.points[index], self.dimension)
    }

    /// The side of `p` from `p1` to `p2`, against the ring's orientation:
    /// `1` to the left of a clockwise ring.
    fn side(&self, p1: &P, p2: &P, p: &P) -> i8 {
        let side = match P::Scalar::side_by_triangle(xy(p1), xy(p2), xy(p)) {
            Ordering::Greater => 1,
            Ordering::Equal => 0,
            Ordering::Less => -1,
        };
        side * self.factor
    }

    /// C++: `extend` — steps from `index` while the points stay level with
    /// `top`, collecting them into `points` when given; where it stopped,
    /// and whether it found a point that is not level before going round.
    fn extend(
        &self,
        mut index: usize,
        top: P::Scalar,
        direction: isize,
        mut points: Option<&mut Vec<P>>,
    ) -> (usize, bool) {
        let n = self.points.len();
        let mut safe = 0;
        loop {
            index = self.step(index, direction);
            if let Some(points) = points.as_mut() {
                points.push(self.points[index]);
            }
            if safe >= n {
                return (index, false);
            }
            safe += 1;
            if !self.value(index).tolerant_eq(top) {
                return (index, true);
            }
        }
    }

    /// C++: `right_turn` — whether the boundary turns right at `index`,
    /// level stretches beside it skipped.
    fn right_turn(&self, index: usize) -> bool {
        let top = self.value(index);
        let (left, found) = self.extend(index, top, -1, None);
        if !found {
            return false;
        }
        let (right, found) = self.extend(index, top, 1, None);
        if !found {
            return false;
        }
        let first = self.side(
            &self.points[self.step(right, -1)],
            &self.points[right],
            &self.points[left],
        );
        let last = self.side(
            &self.points[left],
            &self.points[self.step(left, 1)],
            &self.points[right],
        );
        first != 1 && last != 1
    }

    /// C++: `collect` — the points level with `index`, from the leftmost to
    /// the rightmost, and one step past each end; with where the walk
    /// stopped either side, and whether both found their end.
    fn collect(&self, index: usize) -> (Vec<P>, usize, usize, bool) {
        let top = self.value(index);
        let mut points = Vec::new();
        let (left, found) = self.extend(index, top, -1, Some(&mut points));
        if !found {
            return (points, left, index, false);
        }
        points.reverse();
        points.push(self.points[index]);
        let (right, found) = self.extend(index, top, 1, Some(&mut points));
        (points, left, right, found)
    }

    /// C++: `extreme_points_on_ring::apply` — the first highest vertex
    /// where the boundary turns right, its neighbours levelled at the
    /// higher of them; with where the walk stopped either side.
    fn extremes(&self) -> Option<(Vec<P>, usize, usize)> {
        let n = self.points.len();
        if n < 3 {
            return None;
        }
        let mut top = 0;
        for index in 1..n {
            if self.value(top) < self.value(index) && self.right_turn(index) {
                top = index;
            }
        }
        let (mut points, left, right, found) = self.collect(top);
        if !found {
            return None;
        }
        let front = coordinate(&points[0], self.dimension);
        let back = coordinate(&points[points.len() - 1], self.dimension);
        let base = if front < back { back } else { front };
        let last = points.len() - 1;
        if front < back {
            let neighbour = points[1];
            move_along(&mut points[0], &neighbour, base, self.dimension);
        } else {
            let neighbour = points[last - 1];
            move_along(&mut points[last], &neighbour, base, self.dimension);
        }
        Some((points, left, right))
    }

    /// C++: `get_intruders` — every part of this ring, walking on from
    /// `right` round to `left`, that rises above the extremes' base between
    /// their sides, levelled down to that base.
    fn intruders(&self, left: usize, mut right: usize, extremes: &[P], out: &mut Vec<Vec<P>>) {
        if extremes.len() < 3 {
            return;
        }
        let dimension = self.dimension;
        let other = 1 - dimension;
        let min_value = coordinate(&extremes[lowest(extremes, dimension)], dimension);
        let other_min = coordinate(&extremes[lowest(extremes, other)], other);
        let other_max = coordinate(&extremes[topmost(extremes, other)], other);
        let n = self.points.len();
        let mut checked = 0;
        while left != right && checked < n {
            let point = self.points[right];
            let along = coordinate(&point, dimension);
            let across = coordinate(&point, other);
            if along > min_value && across > other_min && across < other_max {
                let first = self.side(&point, &extremes[0], &extremes[1]);
                let last = self.side(
                    &point,
                    &extremes[extremes.len() - 2],
                    &extremes[extremes.len() - 1],
                );
                if first != 1 && last != 1 {
                    let (mut intruder, _, end, _) = self.collect(right);
                    right = end;
                    move_range_along(&mut intruder, min_value, dimension);
                    out.push(intruder);
                    right = self.step(right, -1);
                }
            }
            right = self.step(right, 1);
            checked += 1;
        }
    }
}

/// C++: `replace_extremes_for_self_tangencies` — where a part of the
/// boundary reaches the very top, the triangle of the extremes' first two
/// points and the end of the leftmost such part, all levelled beneath the
/// highest part that does not reach the top.
fn self_tangency_triangle<P>(
    mut extremes: Vec<P>,
    mut intruders: Vec<Vec<P>>,
    max_intruder: P::Scalar,
    dimension: usize,
) -> Vec<P>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    if let Some(penultimate) = highest(&intruders, dimension, Some(max_intruder)) {
        intruders.retain(|intruder| {
            !intruder.is_empty() && {
                let top = coordinate(&intruder[topmost(intruder, dimension)], dimension);
                !(top.tolerant_eq(penultimate) || top < penultimate)
            }
        });
        for intruder in &mut intruders {
            move_range_along(intruder, penultimate, dimension);
        }
        move_range_along(&mut extremes, penultimate, dimension);
    }
    let other = 1 - dimension;
    intruders.sort_by(|one, two| {
        coordinate(&one[lowest(one, other)], other)
            .partial_cmp(&coordinate(&two[lowest(two, other)], other))
            .unwrap_or(Ordering::Equal)
    });
    match intruders.first().and_then(|intruder| intruder.last()) {
        Some(&end) => alloc::vec![extremes[0], extremes[1], end],
        None => extremes,
    }
}

/// C++: `max_value` — the highest of the intruders' highest points, those
/// level with `skipped` counting below every other (`specific_coordinate_first`).
fn highest<P>(
    intruders: &[Vec<P>],
    dimension: usize,
    skipped: Option<P::Scalar>,
) -> Option<P::Scalar>
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let below = |one: P::Scalar, two: P::Scalar| match skipped {
        Some(skipped) if two.tolerant_eq(skipped) => false,
        Some(skipped) if one.tolerant_eq(skipped) => true,
        _ => one < two,
    };
    let mut best: Option<P::Scalar> = None;
    for intruder in intruders.iter().filter(|intruder| !intruder.is_empty()) {
        let mut top = coordinate(&intruder[0], dimension);
        for point in &intruder[1..] {
            let value = coordinate(point, dimension);
            if below(top, value) {
                top = value;
            }
        }
        if best.is_none_or(|best| top > best) {
            best = Some(top);
        }
    }
    best
}

/// C++: `move_along_vector` over a range — both ends of three or more
/// points, each along its leg toward its neighbour, up to `base`.
fn move_range_along<P>(points: &mut [P], base: P::Scalar, dimension: usize)
where
    P: PointMut + Copy,
    P::Scalar: CoordinateScalar,
{
    let n = points.len();
    if n >= 3 {
        let second = points[1];
        move_along(&mut points[0], &second, base, dimension);
        let penultimate = points[n - 2];
        move_along(&mut points[n - 1], &penultimate, base, dimension);
    }
}

/// C++: `move_along_vector` — `point` moved along the leg from `extreme`
/// until it is level with `base`, where it is below it.
fn move_along<P>(point: &mut P, extreme: &P, base: P::Scalar, dimension: usize)
where
    P: PointMut,
    P::Scalar: CoordinateScalar,
{
    if coordinate(point, dimension) >= base {
        return;
    }
    let vector = (
        point.get::<0>() - extreme.get::<0>(),
        point.get::<1>() - extreme.get::<1>(),
    );
    let diff = if dimension == 0 { vector.0 } else { vector.1 };
    if diff.tolerant_eq(P::Scalar::ZERO) {
        return;
    }
    let base_diff = base - coordinate(extreme, dimension);
    point.set::<0>(extreme.get::<0>() + vector.0 * base_diff / diff);
    point.set::<1>(extreme.get::<1>() + vector.1 * base_diff / diff);
}

/// C++: `calculate_average`.
fn average<P>(points: &[P]) -> P
where
    P: PointMut + Default,
    P::Scalar: CoordinateScalar,
{
    let mut x = P::Scalar::ZERO;
    let mut y = P::Scalar::ZERO;
    let mut count = P::Scalar::ZERO;
    for point in points {
        x = x + point.get::<0>();
        y = y + point.get::<1>();
        count = count + P::Scalar::ONE;
    }
    let mut result = P::default();
    result.set::<0>(x / count);
    result.set::<1>(y / count);
    result
}

fn coordinate<P: Point>(point: &P, dimension: usize) -> P::Scalar {
    if dimension == 0 {
        point.get::<0>()
    } else {
        point.get::<1>()
    }
}

fn xy<P: Point>(point: &P) -> (P::Scalar, P::Scalar) {
    (point.get::<0>(), point.get::<1>())
}

/// The index of the first highest point in `dimension` (`std::max_element`).
fn topmost<P>(points: &[P], dimension: usize) -> usize
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    (1..points.len()).fold(0, |best, index| {
        if coordinate(&points[best], dimension) < coordinate(&points[index], dimension) {
            index
        } else {
            best
        }
    })
}

/// The index of the first lowest point in `dimension` (`std::min_element`).
fn lowest<P>(points: &[P], dimension: usize) -> usize
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    (1..points.len()).fold(0, |best, index| {
        if coordinate(&points[index], dimension) < coordinate(&points[best], dimension) {
            index
        } else {
            best
        }
    })
}

/// A point inside `polygon`: the middle of the widest span the horizontal
/// line halfway up the exterior cuts out of it, holes included; `None`
/// where the line cuts no span.
///
/// The overlay samples its own traced rings with it, needing only some
/// interior point, not [`point_on_surface`]'s. The sweep divides in the
/// coordinate scalar, so an integer one would misplace every crossing;
/// the overlay's rings are only ever built from non-integer coordinates.
pub(crate) fn sweep_interior_point<G, P>(polygon: &G) -> Option<P>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    let outer: Vec<P> = polygon.exterior().points().copied().collect();
    if outer.len() < 3 {
        return None;
    }

    // Representative sweep height: the average of the exterior's y-range.
    let mut ymin = outer[0].get::<1>();
    let mut ymax = ymin;
    for p in &outer {
        let y = p.get::<1>();
        if y < ymin {
            ymin = y;
        }
        if y > ymax {
            ymax = y;
        }
    }
    let two = P::Scalar::ONE + P::Scalar::ONE;
    let sweep_y = (ymin + ymax) / two;

    // Collect x-crossings of the sweep line with every ring.
    let mut xs: Vec<P::Scalar> = Vec::new();
    collect_crossings(&outer, sweep_y, &mut xs);
    for hole in polygon.interiors() {
        let hpts: Vec<P> = hole.points().copied().collect();
        collect_crossings(&hpts, sweep_y, &mut xs);
    }

    if xs.len() < 2 {
        return None;
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));

    // The interior spans are the odd gaps (between crossing 0-1, 2-3, …).
    // Take the midpoint of the widest such span.
    let mut best: Option<(P::Scalar, P::Scalar)> = None; // (width, mid_x)
    let mut i = 0;
    while i + 1 < xs.len() {
        let lo = xs[i];
        let hi = xs[i + 1];
        let width = hi - lo;
        let mid = (lo + hi) / two;
        match best {
            Some((bw, _)) if bw >= width => {}
            _ => best = Some((width, mid)),
        }
        i += 2;
    }

    let (_, mid_x) = best?;
    let mut p = P::default();
    p.set::<0>(mid_x);
    p.set::<1>(sweep_y);
    Some(p)
}

/// Append the x-coordinates where the horizontal line `y = sweep_y`
/// crosses the edges of the vertex ring `pts`.
fn collect_crossings<P>(pts: &[P], sweep_y: P::Scalar, out: &mut Vec<P::Scalar>)
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let n = pts.len();
    if n < 2 {
        return;
    }
    for k in 0..n {
        let a = &pts[k];
        let b = &pts[(k + 1) % n];
        let ay = a.get::<1>();
        let by = b.get::<1>();
        // Half-open crossing test to avoid double-counting a shared
        // vertex: the edge crosses the sweep if exactly one endpoint is
        // strictly above it.
        if (ay > sweep_y) != (by > sweep_y) {
            let ax = a.get::<0>();
            let bx = b.get::<0>();
            let t = (sweep_y - ay) / (by - ay);
            out.push(ax + t * (bx - ax));
        }
    }
}

#[cfg(test)]
mod tests {
    //! OVL6.T3 done-when: the returned point is inside the polygon.
    //! Mirrors `test/algorithms/point_on_surface.cpp`.

    use super::point_on_surface;
    use geometry_algorithm::{covered_by, within};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};
    use geometry_trait::Point as _;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn inside_a_square() {
        let pg: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
        let p = point_on_surface(&pg).unwrap();
        assert!(within(&p, &pg));
    }

    /// Non-convex L, clockwise: the centroid could fall outside, but the
    /// extremes of its top — the arm's level top cut down its two sides —
    /// average to `(1 4)` inside it, as in Boost (`aed7bc3`).
    #[test]
    fn inside_an_l_shape() {
        let pg: Polygon<P> = polygon![[
            (0.0, 0.0),
            (0.0, 6.0),
            (2.0, 6.0),
            (2.0, 2.0),
            (6.0, 2.0),
            (6.0, 0.0),
            (0.0, 0.0)
        ]];
        let p = point_on_surface(&pg).unwrap();
        assert_eq!((p.get::<0>(), p.get::<1>()), (1.0, 4.0));
        assert!(within(&p, &pg));
    }

    /// A big square with a big central hole: the top of the hole reaches up
    /// between the square's top corners, so the extremes are lowered past
    /// it to `(5 8.5)`, in the ring of material, as in Boost (`aed7bc3`).
    #[test]
    fn avoids_a_hole() {
        let pg: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (0.0, 0.0)
            ],
            [(3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0), (3.0, 3.0)]
        ];
        let p = point_on_surface(&pg).unwrap();
        assert_eq!((p.get::<0>(), p.get::<1>()), (5.0, 8.5));
        assert!(within(&p, &pg));
    }

    /// Wound against its declared order the L is not valid, and Boost's
    /// right turns are its left ones: Boost (`aed7bc3`) returns `(2.4 2.4)`,
    /// outside it, and so does the port.
    #[test]
    fn a_ring_wound_the_wrong_way_is_not_guaranteed() {
        let pg: Polygon<P> = polygon![[
            (0.0, 0.0),
            (6.0, 0.0),
            (6.0, 2.0),
            (2.0, 2.0),
            (2.0, 6.0),
            (0.0, 6.0),
            (0.0, 0.0)
        ]];
        let p = point_on_surface(&pg).unwrap();
        assert!((p.get::<0>() - 2.4).abs() < 1e-15 && (p.get::<1>() - 2.4).abs() < 1e-15);
        assert!(!within(&p, &pg));
    }

    /// A counter-clockwise ring turns the other way: the same square
    /// declared anticlockwise still averages to its middle, `(2 2)`, as in
    /// Boost (`6b76894`).
    #[test]
    fn inside_a_counter_clockwise_square() {
        let pg: Polygon<P, false> =
            polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
        let p = point_on_surface(&pg).unwrap();
        assert_eq!((p.get::<0>(), p.get::<1>()), (2.0, 2.0));
        assert!(within(&p, &pg));
    }

    /// A hole touching the apex reaches the very top, so the extremes are
    /// replaced by the triangle beneath it, the lower holes levelling it
    /// first. Boost (`6b76894`) returns `(4 8)`, on the exterior rather
    /// than inside it, and so does the port.
    #[test]
    fn a_hole_reaching_the_top_takes_the_self_tangency_triangle() {
        let pg: Polygon<P> = polygon![
            [(0.0, 0.0), (5.0, 10.0), (10.0, 0.0), (0.0, 0.0)],
            [(5.0, 10.0), (4.5, 4.0), (5.5, 4.0), (5.0, 10.0)],
            [(2.0, 1.0), (3.0, 3.0), (4.0, 1.0), (2.0, 1.0)],
            [(6.0, 1.0), (7.0, 3.0), (8.0, 1.0), (6.0, 1.0)]
        ];
        let p = point_on_surface(&pg).unwrap();
        assert_eq!((p.get::<0>(), p.get::<1>()), (4.0, 8.0));
        assert!(!within(&p, &pg) && covered_by(&p, &pg));
    }
}
