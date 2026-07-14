//! OVL1.T1 — the orientation (side) predicate.
//!
//! Given three points `p`, `q`, `r`, decide whether `r` lies to the
//! left of, to the right of, or on the directed line `p → q`. This is
//! the signed area of the triangle `(p, q, r)`, reduced to its sign.
//!
//! Mirrors `boost::geometry::strategy::side::side_by_triangle`
//! (`boost/geometry/strategy/cartesian/side_by_triangle.hpp`). Boost's
//! `side_value` computes the same signed area
//! `(qx - px)(ry - py) - (qy - py)(rx - px)`; its result sign is the
//! side, with `+1` = left, `-1` = right, `0` = collinear — the
//! convention the spherical side test spells out explicitly
//! (`test/strategies/spherical_side.cpp:55-56`: `side == 1 ? 'L' :
//! side == -1 ? 'R'`).
//!
//! # Robustness
//!
//! The sign is computed on the raw input coordinates (no rescale) by the
//! adaptive expansion arithmetic in
//! [`geometry_coords::precise_math::orient2d`]. This mirrors Boost's robust
//! side strategy and produces the exact sign for finite `f32`/`f64` inputs.
//! Boost's
//! `side_by_triangle` additionally treats any coincident pair among the
//! three points as collinear
//! (`side_by_triangle.hpp:159-164`); this predicate does the same,
//! because a zero-length base line has no well-defined side.

use geometry_coords::{CoordinateScalar, precise_math};
use geometry_trait::Point;

/// The three possible outcomes of the [`orientation_2d`] side test.
///
/// Mirrors the `+1 / 0 / -1` return of Boost's `side_by_triangle`
/// (`boost/geometry/strategy/cartesian/side_by_triangle.hpp`), named
/// so call sites read as topology rather than as integers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Sign {
    /// `r` lies to the **left** of the directed line `p → q`
    /// (counter-clockwise turn). Boost's `+1`, the `'L'` case in
    /// `test/strategies/spherical_side.cpp`.
    Positive,
    /// `r` lies to the **right** of the directed line `p → q`
    /// (clockwise turn). Boost's `-1`, the `'R'` case.
    Negative,
    /// `p`, `q`, `r` are **collinear** (or two of them coincide).
    /// Boost's `0`, the `'|'` case.
    Collinear,
}

/// Sign of the signed area of the triangle `(p, q, r)` — i.e. which
/// side of the directed line `p → q` the point `r` lies on.
///
/// Returns [`Sign::Positive`] for a left turn (counter-clockwise),
/// [`Sign::Negative`] for a right turn (clockwise), and
/// [`Sign::Collinear`] when the three points are collinear or any two
/// coincide.
///
/// Mirrors `side_by_triangle::apply`
/// (`boost/geometry/strategy/cartesian/side_by_triangle.hpp:144-147`),
/// computing `(qx - px)(ry - py) - (qy - py)(rx - px)` and taking its
/// sign. Cartesian only.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_overlay::predicate::orientation::{orientation_2d, Sign};
///
/// type P = Point2D<f64, Cartesian>;
/// let p = P::new(0.0, 0.0);
/// let q = P::new(1.0, 0.0);
///
/// // A point above the x-axis is to the left of p → q.
/// assert_eq!(orientation_2d(&p, &q, &P::new(0.5, 1.0)), Sign::Positive);
/// // Below is to the right.
/// assert_eq!(orientation_2d(&p, &q, &P::new(0.5, -1.0)), Sign::Negative);
/// // On the axis is collinear.
/// assert_eq!(orientation_2d(&p, &q, &P::new(2.0, 0.0)), Sign::Collinear);
/// ```
#[must_use]
pub fn orientation_2d<P>(p: &P, q: &P, r: &P) -> Sign
where
    P: Point,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let px = p.get::<0>();
    let py = p.get::<1>();
    let qx = q.get::<0>();
    let qy = q.get::<1>();
    let rx = r.get::<0>();
    let ry = r.get::<1>();

    // Signed area of (p, q, r). Boost's `side_by_triangle::side_value`
    // computes the identical determinant
    // (`side_by_triangle.hpp` `side_value`).
    let area = precise_math::orient2d(
        [px.into(), py.into()],
        [qx.into(), qy.into()],
        [rx.into(), ry.into()],
    );

    if area > 0.0 {
        Sign::Positive
    } else if area < 0.0 {
        Sign::Negative
    } else {
        Sign::Collinear
    }
}

#[cfg(test)]
mod tests {
    //! Reproduces the left / right / collinear convention asserted in
    //! `test/strategies/spherical_side.cpp:55-56` (`1 = 'L'`,
    //! `-1 = 'R'`, else collinear), on the Cartesian predicate.

    use super::{Sign, orientation_2d};
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn left_right_collinear_unit_segment() {
        let p = P::new(0.0, 0.0);
        let q = P::new(1.0, 0.0);
        assert_eq!(orientation_2d(&p, &q, &P::new(0.5, 1.0)), Sign::Positive);
        assert_eq!(orientation_2d(&p, &q, &P::new(0.5, -1.0)), Sign::Negative);
        assert_eq!(orientation_2d(&p, &q, &P::new(0.5, 0.0)), Sign::Collinear);
    }

    #[test]
    fn sign_flips_with_base_direction() {
        // Reversing the directed base line flips left ↔ right — the
        // signed area negates. `side_by_triangle` has the same
        // antisymmetry.
        let a = P::new(0.0, 0.0);
        let b = P::new(4.0, 4.0);
        let c = P::new(4.0, 0.0);
        assert_eq!(orientation_2d(&a, &b, &c), Sign::Negative);
        assert_eq!(orientation_2d(&b, &a, &c), Sign::Positive);
    }

    #[test]
    fn coincident_points_are_collinear() {
        // Boost returns 0 whenever two of the three points coincide
        // (`side_by_triangle.hpp:159-164`) — a zero-length base line
        // has no side.
        let p = P::new(2.0, 3.0);
        let r = P::new(9.0, 9.0);
        assert_eq!(orientation_2d(&p, &p, &r), Sign::Collinear);
        assert_eq!(orientation_2d(&p, &r, &p), Sign::Collinear);
        assert_eq!(orientation_2d(&r, &p, &p), Sign::Collinear);
    }

    #[test]
    fn diagonal_line_sides() {
        // Line y = x, direction (0,0) → (2,2).
        let p = P::new(0.0, 0.0);
        let q = P::new(2.0, 2.0);
        // (0,2) is above the line → left.
        assert_eq!(orientation_2d(&p, &q, &P::new(0.0, 2.0)), Sign::Positive);
        // (2,0) is below the line → right.
        assert_eq!(orientation_2d(&p, &q, &P::new(2.0, 0.0)), Sign::Negative);
        // (5,5) is on the line → collinear.
        assert_eq!(orientation_2d(&p, &q, &P::new(5.0, 5.0)), Sign::Collinear);
    }
}
