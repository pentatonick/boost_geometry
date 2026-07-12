//! `append(&mut g, point)` and the ring-index variant.
//!
//! Mirrors `boost::geometry::append` from
//! `boost/geometry/algorithms/append.hpp`. Boost ships a single
//! overloaded `append(...)` that dispatches on the source geometry
//! kind; the Rust port uses two free functions — [`append`] for the
//! common linear cases and [`append_to_ring`] for the polygon
//! ring-index overload — so each call site disambiguates without a
//! defaulted argument (which Rust methods cannot express).

use geometry_model::{Linestring, Polygon, Ring};
use geometry_trait::Point as PointTrait;

/// Push `point` to the end of a [`Linestring`] or [`Ring`].
///
/// Mirrors `boost::geometry::append(g, point)` from
/// `boost/geometry/algorithms/append.hpp` for the linear kinds.
pub fn append<G, P>(g: &mut G, point: P)
where
    G: Append<P>,
{
    g.append(point);
}

/// Push `point` onto the polygon's outer ring (`ring_index = None`)
/// or its `ring_index`-th interior ring (`ring_index = Some(i)`).
///
/// Mirrors the `append(polygon, point, ring_index)` overload of
/// `boost::geometry::append` from
/// `boost/geometry/algorithms/append.hpp`, where the outer ring is
/// index `-1` and interior rings are `0..n`.
///
/// An out-of-range interior `ring_index` is a **no-op**, matching Boost:
/// `append.hpp:96-104` guards the interior write with
/// `else if (ring_index < num_interior_rings(polygon))` and has no
/// fall-through, so an index past the last hole silently does nothing.
pub fn append_to_ring<P, const CW: bool, const CL: bool>(
    polygon: &mut Polygon<P, CW, CL>,
    point: P,
    ring_index: Option<usize>,
) where
    P: PointTrait,
{
    match ring_index {
        None => polygon.outer.0.push(point),
        Some(i) if i < polygon.inners.len() => polygon.inners[i].0.push(point),
        // Out-of-range interior index: no-op, as Boost.
        Some(_) => {}
    }
}

/// Per-kind append dispatch for the linear kinds. Implemented for
/// [`Linestring`] and [`Ring`]; the [`Polygon`] ring-index overload
/// lives on the separate [`append_to_ring`] free function.
#[doc(hidden)]
pub trait Append<P> {
    fn append(&mut self, point: P);
}

impl<P: PointTrait> Append<P> for Linestring<P> {
    fn append(&mut self, point: P) {
        self.0.push(point);
    }
}

impl<P: PointTrait, const CW: bool, const CL: bool> Append<P> for Ring<P, CW, CL> {
    fn append(&mut self, point: P) {
        self.0.push(point);
    }
}

#[cfg(test)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/append.cpp` — appending grows
    //! the target ring by one point.

    use super::{append, append_to_ring};
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, Polygon, linestring, polygon};
    use geometry_trait::{Linestring as _, Polygon as _, Ring as _};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn append_to_linestring() {
        let mut ls: Linestring<Pt> = linestring![(0., 0.), (1., 1.)];
        append(&mut ls, Pt::new(2., 2.));
        assert_eq!(ls.points().count(), 3);
    }

    #[test]
    fn append_to_polygon_outer() {
        let mut pg: Polygon<Pt> = polygon![[(0., 0.), (4., 0.), (4., 3.)]];
        append_to_ring(&mut pg, Pt::new(0., 3.), None);
        assert_eq!(pg.exterior().points().count(), 4);
    }

    #[test]
    fn append_to_polygon_interior_ring_by_index() {
        let mut pg: Polygon<Pt> = polygon![
            [(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)],
            [(1., 1.), (2., 1.), (2., 2.)],
        ];
        append_to_ring(&mut pg, Pt::new(1., 2.), Some(0));
        assert_eq!(pg.interiors().next().unwrap().points().count(), 4);
    }

    #[test]
    fn append_to_out_of_range_interior_is_a_noop() {
        // Regression: an out-of-range interior index must NOT panic — it
        // is a silent no-op, matching Boost (append.hpp:96-104 guards the
        // interior write and falls through on OOB).
        let mut pg: Polygon<Pt> = polygon![[(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)]];
        // No interior rings; index 0 is out of range.
        append_to_ring(&mut pg, Pt::new(1., 1.), Some(0));
        assert_eq!(pg.interiors().count(), 0, "no ring created, no panic");
        // Outer untouched.
        assert_eq!(pg.exterior().points().count(), 5);
    }
}
