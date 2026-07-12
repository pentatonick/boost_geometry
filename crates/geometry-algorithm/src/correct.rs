//! `correct(&mut g)` — fix ring closure and orientation in place.
//!
//! Mirrors `boost::geometry::correct` from
//! `boost/geometry/algorithms/correct.hpp` and the closure-fix helper
//! at `algorithms/correct_closure.hpp`.
//!
//! Per-kind:
//!
//! * `Ring<P, CW, true>`   → push closing vertex if missing
//! * `Ring<P, CW, false>`  → pop closing vertex if duplicated
//! * exterior ring         → reverse so its *strategy-level* signed
//!   area (which already folds the declared `PointOrder`) is positive
//!   — i.e. the stored order matches the declaration, for CW- and
//!   CCW-declared rings alike
//! * `Polygon` outer       → as above; inners → opposite of outer
//! * `MultiPolygon`        → correct each member
//!
//! Empty and 1-point rings are left unchanged (silent no-op, matching
//! Boost). Cartesian-only: the orientation test uses the Cartesian
//! shoelace area.

use geometry_coords::CoordinateScalar;
use geometry_cs::CoordinateSystem;
use geometry_model::{MultiPolygon, Polygon, Ring};
use geometry_strategy::{AreaStrategy, ShoelaceArea};
use geometry_trait::{Closure, Point as PointTrait, Ring as RingTrait};

/// Fix closure and orientation of `g` in place.
///
/// Mirrors `boost::geometry::correct(g)` from
/// `boost/geometry/algorithms/correct.hpp`.
pub fn correct<G: Correct>(g: &mut G) {
    g.correct();
}

/// Per-kind correction dispatch.
#[doc(hidden)]
pub trait Correct {
    fn correct(&mut self);
}

/// Add or drop the closing vertex so the stored point sequence matches
/// the ring's `CLOSED` const-generic.
fn fix_closure<P, const CW: bool, const CL: bool>(r: &mut Ring<P, CW, CL>)
where
    P: PointTrait + Copy,
{
    // Rings of two or fewer points are degenerate — closing one would
    // just append a spurious `[a, b, a]`. Boost leaves them untouched
    // (`algorithms/correct_closure.hpp:59`, `if (size <= 2) return;`).
    if r.0.len() <= 2 {
        return;
    }
    let first = r.0[0];
    let last = *r.0.last().unwrap();
    let should_be_closed = matches!(r.closure(), Closure::Closed);
    let already_closed = coords_equal(&first, &last);
    match (should_be_closed, already_closed) {
        (true, false) => r.0.push(first), // close it
        (false, true) => {
            r.0.pop(); // open it
        }
        _ => {}
    }
}

/// Coordinate-wise equality — `Point<T, D, Cs>` does not derive a
/// usable `PartialEq` (the derive would demand `Cs: PartialEq`), so we
/// compare per dimension via `get::<D>`.
fn coords_equal<P: PointTrait>(a: &P, b: &P) -> bool {
    geometry_trait::fold_dims(true, a, |acc, _p, d| {
        acc && match d {
            0 => a.get::<0>() == b.get::<0>(),
            1 => a.get::<1>() == b.get::<1>(),
            2 => a.get::<2>() == b.get::<2>(),
            3 => a.get::<3>() == b.get::<3>(),
            _ => unreachable!("fold_dims caps at MAX_DIM"),
        }
    })
}

/// Reverse `r` if its signed area sign disagrees with `want_positive`.
fn fix_orientation<P, const CW: bool, const CL: bool>(r: &mut Ring<P, CW, CL>, want_positive: bool)
where
    P: PointTrait,
    ShoelaceArea: AreaStrategy<Ring<P, CW, CL>, Out = P::Scalar>,
{
    let a = ShoelaceArea.area(&*r);
    let zero = <P::Scalar as CoordinateScalar>::ZERO;
    let is_positive = a > zero;
    let is_negative = a < zero;
    // Only reverse when the sign is decisively wrong; a zero-area
    // (degenerate) ring is left as-is.
    if (want_positive && is_negative) || (!want_positive && is_positive) {
        r.0.reverse();
    }
}

impl<P, const CW: bool, const CL: bool> Correct for Ring<P, CW, CL>
where
    P: PointTrait + Copy,
    P::Cs: CoordinateSystem,
    ShoelaceArea: AreaStrategy<Ring<P, CW, CL>, Out = P::Scalar>,
{
    fn correct(&mut self) {
        fix_closure(self);
        // `ShoelaceArea` already folds the declared `PointOrder` into
        // its sign: a ring stored in its declared direction has a
        // POSITIVE strategy-level area for CW-declared and
        // CCW-declared rings alike. So the target sign of a corrected
        // standalone ring is always positive — passing `CW` here would
        // double-apply the declaration flip and reverse correctly
        // wound CCW rings.
        fix_orientation(self, true);
    }
}

impl<P, const CW: bool, const CL: bool> Correct for Polygon<P, CW, CL>
where
    P: PointTrait + Copy,
    P::Cs: CoordinateSystem,
    ShoelaceArea: AreaStrategy<Ring<P, CW, CL>, Out = P::Scalar>,
{
    fn correct(&mut self) {
        fix_closure(&mut self.outer);
        // Exterior: stored order must match the declaration —
        // strategy-level area positive (see the Ring impl above).
        fix_orientation(&mut self.outer, true);
        for inner in &mut self.inners {
            fix_closure(inner);
            // Interior rings wind opposite the exterior, i.e. opposite
            // their own declared order — strategy-level area negative.
            // That is the state `ShoelacePolygonArea`'s plain ring-sum
            // relies on (holes arrive negatively signed).
            fix_orientation(inner, false);
        }
    }
}

impl<Pg: Correct + geometry_trait::Polygon> Correct for MultiPolygon<Pg> {
    fn correct(&mut self) {
        for p in &mut self.0 {
            p.correct();
        }
    }
}

#[cfg(test)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/correct.cpp`: a
    //! counter-clockwise-stored exterior of a clockwise-declared ring
    //! is reversed so its signed area becomes positive.

    #![allow(clippy::float_cmp, reason = "Areas are exact integer literals.")]

    use super::correct;
    use crate::area::ring_area;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Ring};

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn ccw_exterior_of_cw_ring_is_reversed() {
        // A 2×2 square stored counter-clockwise. Declared CW (default),
        // so its signed area is negative until `correct` reverses it.
        let mut r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
            P::new(2.0, 2.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ]);
        assert!(ring_area(&r) < 0.0, "precondition: CCW ring is negative");
        correct(&mut r);
        assert_eq!(ring_area(&r), 4.0);
    }

    #[test]
    fn already_correct_ring_is_unchanged() {
        // Same square stored clockwise — already positive; correct is a
        // no-op on orientation.
        let mut r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.0, 2.0),
            P::new(2.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 0.0),
        ]);
        assert_eq!(ring_area(&r), 4.0);
        correct(&mut r);
        assert_eq!(ring_area(&r), 4.0);
    }

    #[test]
    fn two_point_ring_is_left_untouched() {
        // Regression: a degenerate 2-point ring must NOT be "closed" into
        // a spurious [a, b, a]. Boost leaves rings of size <= 2 alone
        // (correct_closure.hpp:59).
        use geometry_trait::Ring as _;
        let mut r: Ring<P> = Ring::from_vec(vec![P::new(0.0, 0.0), P::new(1.0, 1.0)]);
        correct(&mut r);
        assert_eq!(r.points().count(), 2, "2-point ring must stay 2 points");
    }

    #[test]
    fn ccw_ring_correctly_wound_is_a_noop() {
        // Regression: `fix_orientation(self, CW)` used to reverse a
        // CORRECTLY wound CCW-declared ring (+2 → −2), because
        // `ShoelaceArea` already folds the declared order into its
        // sign. Fixture mirrors geometry-strategy's
        // `ccw_declared_ccw_traversed_diamond_is_2`.
        let mut r: Ring<P, false> = Ring::from_vec(vec![
            P::new(1.0, 0.0),
            P::new(0.0, 1.0),
            P::new(-1.0, 0.0),
            P::new(0.0, -1.0),
            P::new(1.0, 0.0),
        ]);
        assert_eq!(ring_area(&r), 2.0, "precondition: correctly wound");
        correct(&mut r);
        assert_eq!(ring_area(&r), 2.0, "correct() must be a no-op");
    }

    #[test]
    fn ccw_ring_wrongly_wound_is_reversed() {
        // The same diamond stored clockwise under a CCW declaration:
        // strategy area −2 → correct() reverses → +2.
        let mut r: Ring<P, false> = Ring::from_vec(vec![
            P::new(1.0, 0.0),
            P::new(0.0, -1.0),
            P::new(-1.0, 0.0),
            P::new(0.0, 1.0),
            P::new(1.0, 0.0),
        ]);
        assert_eq!(ring_area(&r), -2.0, "precondition: wrongly wound");
        correct(&mut r);
        assert_eq!(ring_area(&r), 2.0);
    }

    #[test]
    fn ccw_polygon_with_hole_correctly_wound_is_a_noop() {
        // Outer 4x4 stored CCW (matches declaration, ring_area +16),
        // hole 1x1 stored CW (opposite, ring_area −1). correct() must
        // change nothing: this is exactly the state
        // ShoelacePolygonArea's ring-sum (+15) relies on.
        use geometry_model::Polygon;
        let outer: Ring<P, false> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(4.0, 0.0),
            P::new(4.0, 4.0),
            P::new(0.0, 4.0),
            P::new(0.0, 0.0),
        ]);
        let hole: Ring<P, false> = Ring::from_vec(vec![
            P::new(1.0, 1.0),
            P::new(1.0, 2.0),
            P::new(2.0, 2.0),
            P::new(2.0, 1.0),
            P::new(1.0, 1.0),
        ]);
        let mut pg: Polygon<P, false> = Polygon::new(outer);
        pg.inners.push(hole);
        assert_eq!(ring_area(&pg.outer), 16.0, "precondition: outer CCW-stored");
        assert_eq!(
            ring_area(&pg.inners[0]),
            -1.0,
            "precondition: hole CW-stored"
        );
        correct(&mut pg);
        assert_eq!(ring_area(&pg.outer), 16.0, "outer must be untouched");
        assert_eq!(ring_area(&pg.inners[0]), -1.0, "hole must be untouched");
    }

    #[test]
    fn ccw_polygon_wrongly_wound_is_fixed() {
        // Outer stored CW (wrong for a CCW declaration), hole stored
        // CCW (wrong for a hole): correct() reverses both.
        use geometry_model::Polygon;
        let outer: Ring<P, false> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.0, 4.0),
            P::new(4.0, 4.0),
            P::new(4.0, 0.0),
            P::new(0.0, 0.0),
        ]);
        let hole: Ring<P, false> = Ring::from_vec(vec![
            P::new(1.0, 1.0),
            P::new(2.0, 1.0),
            P::new(2.0, 2.0),
            P::new(1.0, 2.0),
            P::new(1.0, 1.0),
        ]);
        let mut pg: Polygon<P, false> = Polygon::new(outer);
        pg.inners.push(hole);
        assert_eq!(ring_area(&pg.outer), -16.0, "precondition: outer wrong");
        assert_eq!(ring_area(&pg.inners[0]), 1.0, "precondition: hole wrong");
        correct(&mut pg);
        assert_eq!(ring_area(&pg.outer), 16.0);
        assert_eq!(ring_area(&pg.inners[0]), -1.0);
    }
}
