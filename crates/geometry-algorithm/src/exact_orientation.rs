//! Exact orientation of three points and the closed-segment tests built on
//! it, shared by the port-only [`concave_hull`](crate::concave_hull) and
//! [`triangulate_earcut`](crate::triangulate_earcut).
//!
//! Every turn is decided by the adaptive exact-sign
//! [`geometry_coords::precise_math::orient2d`], so a point that lies on a
//! line is collinear rather than on whichever side rounding leaves it.

use geometry_coords::precise_math;
use geometry_trait::Point;

/// Whether the closed segments `a b` and `c d` share a point: a crossing,
/// an endpoint on the other segment, or a collinear overlap.
pub(crate) fn segments_intersect<P>(a: P, b: P, c: P, d: P) -> bool
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

/// Positive when `c` lies to the left of `a → b`, negative to its right,
/// and zero when the three points are collinear.
#[allow(
    clippy::needless_pass_by_value,
    reason = "both algorithms operate on Copy point handles throughout"
)]
pub(crate) fn orientation<P: Point<Scalar = f64>>(a: P, b: P, c: P) -> f64 {
    precise_math::orient2d(
        [a.get::<0>(), a.get::<1>()],
        [b.get::<0>(), b.get::<1>()],
        [c.get::<0>(), c.get::<1>()],
    )
}

/// Whether `point`, collinear with `a b`, lies on the closed segment.
fn on_segment<P: Point<Scalar = f64> + Copy>(a: P, b: P, point: P) -> bool {
    point.get::<0>() >= a.get::<0>().min(b.get::<0>())
        && point.get::<0>() <= a.get::<0>().max(b.get::<0>())
        && point.get::<1>() >= a.get::<1>().min(b.get::<1>())
        && point.get::<1>() <= a.get::<1>().max(b.get::<1>())
}
