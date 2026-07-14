//! `clear(&mut g)` — empty the backing storage in place.
//!
//! Mirrors `boost::geometry::clear` from
//! `boost/geometry/algorithms/clear.hpp`. Per-kind:
//!
//! * `Linestring`, `Ring`, `MultiPoint` → `Vec::clear` on the backing.
//! * `Polygon` → clear the outer ring and drop every inner ring.
//! * `MultiLinestring`, `MultiPolygon` → `Vec::clear` on the member
//!   vector.
//! * `Point`, `Box`, `Segment` → no impl (fixed-size storage — there
//!   is nothing to clear).
//!
//! Boost ships the fixed-size case as a silent no-op fall-through
//! (`clear.hpp`); the Rust port instead simply provides no [`Clear`]
//! impl for `Point` / `Box` / `Segment`, so `clear(&mut point)` is a
//! compile error — the type system says "call this on something that
//! *has* clearable storage."

use geometry_model::{Linestring, MultiLinestring, MultiPoint, MultiPolygon, Polygon, Ring};

/// Empty the backing storage of `g` in place.
///
/// Mirrors `boost::geometry::clear(g)` from
/// `boost/geometry/algorithms/clear.hpp`. Not defined for `Point`,
/// `Box`, or `Segment` (fixed-size storage — see the module docs).
pub fn clear<G: Clear>(g: &mut G) {
    g.clear();
}

/// Per-kind clear dispatch. Implemented only for the variable-length
/// kinds; fixed-size kinds (`Point`, `Box`, `Segment`) are
/// intentionally absent so clearing them fails to compile.
#[doc(hidden)]
pub trait Clear {
    fn clear(&mut self);
}

impl<P: geometry_trait::Point> Clear for Linestring<P> {
    fn clear(&mut self) {
        self.0.clear();
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> Clear for Ring<P, CW, CL> {
    fn clear(&mut self) {
        self.0.clear();
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> Clear for Polygon<P, CW, CL> {
    fn clear(&mut self) {
        self.outer.0.clear();
        self.inners.clear();
    }
}

impl<P: geometry_trait::Point> Clear for MultiPoint<P> {
    fn clear(&mut self) {
        self.0.clear();
    }
}

impl<L: geometry_trait::Linestring> Clear for MultiLinestring<L> {
    fn clear(&mut self) {
        self.0.clear();
    }
}

impl<Pg: geometry_trait::Polygon> Clear for MultiPolygon<Pg> {
    fn clear(&mut self) {
        self.0.clear();
    }
}

#[cfg(test)]
mod tests {
    //! Behaviour cross-checked against
    //! [`crate::is_empty`]: a cleared geometry reports
    //! empty, and `Polygon::clear` drops both the outer ring's points
    //! and every inner ring.

    use super::clear;
    use crate::is_empty::is_empty;
    use geometry_cs::Cartesian;
    use geometry_model::{
        Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring, linestring,
        polygon,
    };

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn clear_linestring() {
        let mut ls: Linestring<Pt> = linestring![(0., 0.), (1., 1.), (2., 2.)];
        clear(&mut ls);
        assert!(is_empty(&ls));
    }

    #[test]
    fn clear_polygon_drops_both_rings_and_holes() {
        let mut pg: Polygon<Pt> = polygon![
            [(0., 0.), (5., 0.), (5., 5.), (0., 0.)],
            [(1., 1.), (2., 1.), (2., 2.), (1., 1.)],
        ];
        clear(&mut pg);
        assert!(is_empty(&pg));
        assert_eq!(pg.inners.len(), 0);
    }

    #[test]
    fn clear_multi_point() {
        let mut mp = MultiPoint(alloc::vec![Pt::new(0., 0.), Pt::new(1., 1.)]);
        clear(&mut mp);
        assert_eq!(mp.0.len(), 0);
    }

    #[test]
    fn clear_ring() {
        let mut r: Ring<Pt> = Ring::from_vec(alloc::vec![
            Pt::new(0., 0.),
            Pt::new(1., 0.),
            Pt::new(1., 1.)
        ]);
        clear(&mut r);
        assert!(is_empty(&r));
    }

    #[test]
    fn clear_multi_linestring_drops_all_members() {
        let mut mls: MultiLinestring<Linestring<Pt>> = MultiLinestring(alloc::vec![
            linestring![(0., 0.), (1., 1.)],
            linestring![(2., 2.), (3., 3.)],
        ]);
        clear(&mut mls);
        assert_eq!(mls.0.len(), 0);
        assert!(is_empty(&mls));
    }

    #[test]
    fn clear_multi_polygon_drops_all_members() {
        let member: Polygon<Pt> = polygon![[(0., 0.), (5., 0.), (5., 5.), (0., 0.)]];
        let mut mpg: MultiPolygon<Polygon<Pt>> = MultiPolygon(alloc::vec![member.clone(), member]);
        clear(&mut mpg);
        assert_eq!(mpg.0.len(), 0);
        assert!(is_empty(&mpg));
    }
}
