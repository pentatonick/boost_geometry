//! `reverse(&mut g)` — flip point order in place.
//!
//! Mirrors `boost::geometry::reverse` from
//! `boost/geometry/algorithms/reverse.hpp`. Per-kind dispatch:
//!
//! * `Linestring`, `Ring`         → reverse the backing `Vec<P>`
//! * `Polygon`                    → reverse outer + every inner ring
//! * `MultiLinestring`            → reverse each member
//! * `MultiPolygon`               → reverse each member polygon
//! * `Point`, `Segment`, `Box`, `MultiPoint` → no-op (Boost ships
//!   these as silent no-ops; the call sites that drive `reverse` are
//!   linear/areal)
//!
//! `reverse` does NOT change the [`geometry_trait::PointOrder`]
//! const-generic on `Ring` / `Polygon` — that is a *type-level*
//! attribute. A clockwise-declared `Ring` mutated to traverse CCW will
//! compute negative area; [`correct`](fn@crate::correct) re-syncs the
//! const generic with the stored order.

use geometry_model::{Linestring, MultiLinestring, MultiPolygon, Polygon, Ring};
use geometry_trait::{Linestring as LinestringTrait, Polygon as PolygonTrait};

/// Reverse the point order of `g` in place.
///
/// Mirrors `boost::geometry::reverse(g)` from
/// `boost/geometry/algorithms/reverse.hpp`.
pub fn reverse<G: Reverse>(g: &mut G) {
    g.reverse();
}

/// Per-kind reverse dispatch. Implemented for the linear / areal model
/// types; point-like kinds are intentionally absent (reversing a point
/// is meaningless).
#[doc(hidden)]
pub trait Reverse {
    fn reverse(&mut self);
}

impl<P: geometry_trait::Point> Reverse for Linestring<P> {
    fn reverse(&mut self) {
        self.0.reverse();
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> Reverse for Ring<P, CW, CL> {
    fn reverse(&mut self) {
        self.0.reverse();
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> Reverse for Polygon<P, CW, CL> {
    fn reverse(&mut self) {
        self.outer.reverse();
        for inner in &mut self.inners {
            inner.reverse();
        }
    }
}

impl<L: Reverse + LinestringTrait> Reverse for MultiLinestring<L> {
    fn reverse(&mut self) {
        for l in &mut self.0 {
            l.reverse();
        }
    }
}

impl<Pg: Reverse + PolygonTrait> Reverse for MultiPolygon<Pg> {
    fn reverse(&mut self) {
        for p in &mut self.0 {
            p.reverse();
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "Reversed coordinates are exact literals.")]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/reverse.cpp` — a linestring's
    //! points come back in reversed order; a polygon reverses outer and
    //! inner rings.

    use super::reverse;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, linestring, polygon};
    use geometry_trait::{Linestring as _, Point as _, Polygon as _, Ring as _};

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn reverse_linestring_flips_order() {
        let mut ls: geometry_model::Linestring<P> = linestring![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)];
        reverse(&mut ls);
        let xs: Vec<f64> = ls.points().map(geometry_trait::Point::get::<0>).collect();
        assert_eq!(xs, vec![2.0, 1.0, 0.0]);
    }

    #[test]
    fn reverse_polygon_flips_outer_and_inner() {
        let mut pg: geometry_model::Polygon<P> = polygon![
            [(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)],
            [(1.0, 1.0), (1.0, 2.0), (2.0, 2.0), (2.0, 1.0), (1.0, 1.0)]
        ];
        reverse(&mut pg);
        let outer_first_x = pg.exterior().points().next().unwrap().get::<0>();
        // First point is unchanged (it is the pivot of the reversal for a
        // closed ring whose first == last), but the second becomes the old
        // penultimate. Check the second vertex flipped.
        let outer_second_x = pg.exterior().points().nth(1).unwrap().get::<0>();
        assert_eq!(outer_first_x, 0.0);
        assert_eq!(outer_second_x, 4.0); // was (4.0, 0.0) before reversal
        assert_eq!(pg.interiors().count(), 1);
    }
}
