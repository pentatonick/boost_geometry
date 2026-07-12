//! OVL1.T2 — the in-circle predicate.
//!
//! Given a triangle `a, b, c` and a fourth point `d`, decide whether
//! `d` lies inside, on, or outside the circle through `a, b, c`. This
//! is the classic Delaunay in-circle determinant; `is_valid` and the
//! overlay turn graph use it to reason about the local geometry around
//! a turn.
//!
//! Mirrors the robust in-circle policy in
//! `boost/geometry/policies/robustness/` (the same 4×4 determinant
//! Boost's Delaunay / validity code forms). The sign convention is
//! stated relative to the orientation of `a, b, c`, so the caller must
//! know that orientation to read the result — see [`in_circle_2d`].
//!
//! # Robustness
//!
//! Like [`orientation`](super::orientation) the determinant is exact
//! only inside the safe-magnitude range
//!; the products here are of
//! *squared* coordinate differences, so the safe range is tighter than
//! the orientation predicate's. Callers route inputs through
//! [`range_guard`](super::range_guard) first.

use geometry_coords::CoordinateScalar;
use geometry_trait::Point;

use super::orientation::Sign;

/// Position of `d` relative to the circle through `a, b, c`.
///
/// The determinant computed below is positive when `d` is inside the
/// circle **and** `a, b, c` are in counter-clockwise order. To make the
/// answer independent of the input triangle's winding, the raw sign is
/// combined with the orientation of `a, b, c`:
///
/// * [`Sign::Positive`] → `d` is **inside** the circle.
/// * [`Sign::Negative`] → `d` is **outside** the circle.
/// * [`Sign::Collinear`] → `d` is **on** the circle (cocircular), or
///   `a, b, c` are themselves collinear (degenerate circle).
///
/// Mirrors the in-circle test used by Boost's Delaunay / validity code
/// (`boost/geometry/policies/robustness/`). Cartesian only.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_overlay::predicate::in_circle::in_circle_2d;
/// use geometry_overlay::predicate::orientation::Sign;
///
/// type P = Point2D<f64, Cartesian>;
/// // Unit circle sampled at three points (CCW).
/// let a = P::new(1.0, 0.0);
/// let b = P::new(0.0, 1.0);
/// let c = P::new(-1.0, 0.0);
///
/// // Origin is inside.
/// assert_eq!(in_circle_2d(&a, &b, &c, &P::new(0.0, 0.0)), Sign::Positive);
/// // A far point is outside.
/// assert_eq!(in_circle_2d(&a, &b, &c, &P::new(5.0, 5.0)), Sign::Negative);
/// // A fourth point on the same unit circle is cocircular.
/// assert_eq!(in_circle_2d(&a, &b, &c, &P::new(0.0, -1.0)), Sign::Collinear);
/// ```
#[must_use]
pub fn in_circle_2d<P>(a: &P, b: &P, c: &P, d: &P) -> Sign
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let ax = a.get::<0>();
    let ay = a.get::<1>();
    let bx = b.get::<0>();
    let by = b.get::<1>();
    let cx = c.get::<0>();
    let cy = c.get::<1>();
    let dx = d.get::<0>();
    let dy = d.get::<1>();

    // Translate so `d` is the origin, then the in-circle test is the
    // sign of the 3×3 determinant of rows
    // `[ex, ey, ex² + ey²]` for e ∈ {a, b, c}. This is the standard
    // reduction of the 4×4 in-circle determinant (Guibas & Stolfi).
    let adx = ax - dx;
    let ady = ay - dy;
    let bdx = bx - dx;
    let bdy = by - dy;
    let cdx = cx - dx;
    let cdy = cy - dy;

    let a_lift = adx * adx + ady * ady;
    let b_lift = bdx * bdx + bdy * bdy;
    let c_lift = cdx * cdx + cdy * cdy;

    let det = adx * (bdy * c_lift - b_lift * cdy) - ady * (bdx * c_lift - b_lift * cdx)
        + a_lift * (bdx * cdy - bdy * cdx);

    // `det > 0` ⇔ d inside, *when a,b,c are CCW*. For a CW triangle the
    // determinant negates, so fold in the base orientation to give a
    // winding-independent answer.
    let base = orientation_of(adx, ady, bdx, bdy, cdx, cdy);
    match base {
        Sign::Collinear => Sign::Collinear,   // degenerate circle
        Sign::Positive => sign_of(det),       // CCW: det>0 = inside
        Sign::Negative => flip(sign_of(det)), // CW: invert
    }
}

/// Sign of the signed area of the triangle whose vertices, already
/// translated to be relative to `d`, are `(ax, ay)`, `(bx, by)`,
/// `(cx, cy)`. Local to the in-circle reduction.
fn orientation_of<T: CoordinateScalar>(ax: T, ay: T, bx: T, by: T, cx: T, cy: T) -> Sign {
    let area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    sign_of(area)
}

fn sign_of<T: CoordinateScalar>(v: T) -> Sign {
    if v > T::ZERO {
        Sign::Positive
    } else if v < T::ZERO {
        Sign::Negative
    } else {
        Sign::Collinear
    }
}

fn flip(s: Sign) -> Sign {
    match s {
        Sign::Positive => Sign::Negative,
        Sign::Negative => Sign::Positive,
        Sign::Collinear => Sign::Collinear,
    }
}

#[cfg(test)]
mod tests {
    //! Three hand-picked triangles (OVL1.T2 done-when): a CCW triangle,
    //! its CW mirror, and a degenerate collinear triple — each probed
    //! with an inside, outside, and on-circle point.

    use super::in_circle_2d;
    use crate::predicate::orientation::Sign;
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn ccw_unit_circle() {
        let a = P::new(1.0, 0.0);
        let b = P::new(0.0, 1.0);
        let c = P::new(-1.0, 0.0);
        assert_eq!(in_circle_2d(&a, &b, &c, &P::new(0.0, 0.0)), Sign::Positive);
        assert_eq!(
            in_circle_2d(&a, &b, &c, &P::new(0.0, -1.0)),
            Sign::Collinear
        );
        assert_eq!(in_circle_2d(&a, &b, &c, &P::new(3.0, 3.0)), Sign::Negative);
    }

    #[test]
    fn cw_triangle_gives_same_answer() {
        // Same circle, vertices listed clockwise — the winding-fold
        // must keep "inside" reported as inside.
        let a = P::new(-1.0, 0.0);
        let b = P::new(0.0, 1.0);
        let c = P::new(1.0, 0.0);
        assert_eq!(in_circle_2d(&a, &b, &c, &P::new(0.0, 0.0)), Sign::Positive);
        assert_eq!(
            in_circle_2d(&a, &b, &c, &P::new(0.0, -1.0)),
            Sign::Collinear
        );
        assert_eq!(in_circle_2d(&a, &b, &c, &P::new(9.0, 0.0)), Sign::Negative);
    }

    #[test]
    fn collinear_triangle_is_degenerate() {
        let a = P::new(0.0, 0.0);
        let b = P::new(1.0, 0.0);
        let c = P::new(2.0, 0.0);
        // No circle through three collinear points → always Collinear.
        assert_eq!(in_circle_2d(&a, &b, &c, &P::new(0.5, 5.0)), Sign::Collinear);
    }

    #[test]
    fn larger_circle_containment() {
        // Circle of radius 5 centred at origin, sampled CCW.
        let a = P::new(5.0, 0.0);
        let b = P::new(0.0, 5.0);
        let c = P::new(-5.0, 0.0);
        assert_eq!(in_circle_2d(&a, &b, &c, &P::new(3.0, 3.0)), Sign::Positive); // r≈4.24
        assert_eq!(in_circle_2d(&a, &b, &c, &P::new(4.0, 4.0)), Sign::Negative); // r≈5.66
    }
}
