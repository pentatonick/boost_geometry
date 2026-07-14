//! `remove_spikes(&mut g)` — drop collinear-and-reversed vertices.
//!
//! Mirrors `boost::geometry::remove_spikes` from
//! `boost/geometry/algorithms/remove_spikes.hpp`. A spike is a triple
//! `(a, b, c)` where `(b-a) × (c-b) == 0` (collinear) AND
//! `(b-a) · (c-b) < 0` (reversed). The middle vertex `b` is removed;
//! the walk repeats until no spike remains, because collapsing one
//! spike can create a new one at the now-adjacent pair.
//!
//! Per-kind:
//! * `Linestring`, `Ring`  → spike-walk the backing `Vec<P>`
//! * `Polygon`             → walk outer + every inner ring
//! * `MultiPolygon`        → walk each member
//!
//! Cartesian-only: the collinearity / reversal predicate is the 2D
//! cross/dot product. Spherical / geographic spike detection needs
//! angle-aware predicates; deferred until a downstream caller appears.

use geometry_coords::CoordinateScalar;
use geometry_model::{Linestring, MultiPolygon, Polygon, Ring};
use geometry_trait::Point as PointTrait;

/// Remove spikes from `g` in place.
///
/// Mirrors `boost::geometry::remove_spikes(g)` from
/// `boost/geometry/algorithms/remove_spikes.hpp`.
pub fn remove_spikes<G: RemoveSpikes>(g: &mut G) {
    g.remove_spikes();
}

/// Per-kind spike-removal dispatch.
#[doc(hidden)]
pub trait RemoveSpikes {
    fn remove_spikes(&mut self);
}

/// True iff `b` is a spike between `a` and `c` (2D cross `== 0` AND
/// dot `< 0`).
fn is_spike_2d<P: PointTrait>(a: &P, b: &P, c: &P) -> bool {
    let ux = b.get::<0>() - a.get::<0>();
    let uy = b.get::<1>() - a.get::<1>();
    let vx = c.get::<0>() - b.get::<0>();
    let vy = c.get::<1>() - b.get::<1>();
    let cross = ux * vy - uy * vx;
    let dot = ux * vx + uy * vy;
    let zero = <P::Scalar as CoordinateScalar>::ZERO;
    cross == zero && dot < zero
}

fn walk_spikes<P: PointTrait>(pts: &mut alloc::vec::Vec<P>) {
    let mut changed = true;
    while changed && pts.len() >= 3 {
        changed = false;
        let mut i = 1;
        while i + 1 < pts.len() {
            if is_spike_2d(&pts[i - 1], &pts[i], &pts[i + 1]) {
                pts.remove(i);
                changed = true;
                // Do not advance `i`: the new `pts[i]` (was `pts[i+1]`)
                // may now form a spike with `pts[i-1]`.
                if i > 1 {
                    i -= 1;
                }
            } else {
                i += 1;
            }
        }
    }
}

impl<P: PointTrait> RemoveSpikes for Linestring<P> {
    fn remove_spikes(&mut self) {
        walk_spikes(&mut self.0);
    }
}

/// Spike-walk a **ring**: the interior linear pass plus the wrap-around
/// seam that a linestring does not have.
///
/// Mirrors `detail::remove_spikes::range_remove_spikes::apply`
/// (`algorithms/remove_spikes.hpp:99-141`). After the interior pass,
/// Boost drops the closing point of a closed ring, then repeatedly
/// removes a spike formed at the *first* vertex — the triple
/// `(back-1, back, front)` — and at the *second* — `(back, front,
/// front+1)` — until neither fires, and re-adds the closing point. The
/// interior [`walk_spikes`] alone never forms those seam triples, so a
/// spike sitting on the ring's first/last vertex would otherwise survive.
///
/// `closed` is `true` when the backing vector repeats its first vertex as
/// its last (the model's `CLOSED` const generic).
fn walk_ring_spikes<P: PointTrait + Copy>(pts: &mut alloc::vec::Vec<P>, closed: bool) {
    // Interior pass first.
    walk_spikes(pts);

    // Work on the open sequence: drop the duplicated closing vertex, if
    // any, so `first` and `last` are distinct ring vertices.
    let had_closing = closed && pts.len() >= 2 && same_point(&pts[0], &pts[pts.len() - 1]);
    if had_closing {
        pts.pop();
    }

    // Seam cleanup: alternately peel a spike off the back (last vertex)
    // and the front (first vertex) until the seam is clean.
    let mut found = true;
    while found {
        found = false;
        // Spike at the first point: (prev = back-1, back, front).
        while pts.len() >= 3 && is_spike_2d(&pts[pts.len() - 2], &pts[pts.len() - 1], &pts[0]) {
            pts.pop();
            found = true;
        }
        // Spike at the second point: (back, front, front+1).
        while pts.len() >= 3 && is_spike_2d(&pts[pts.len() - 1], &pts[0], &pts[1]) {
            pts.remove(0);
            found = true;
        }
    }

    // Re-add the closing vertex we removed, restoring the ring's closure.
    if had_closing && !pts.is_empty() {
        let first = pts[0];
        pts.push(first);
    }
}

/// Coordinate equality of two points (2D).
fn same_point<P: PointTrait>(a: &P, b: &P) -> bool {
    a.get::<0>() == b.get::<0>() && a.get::<1>() == b.get::<1>()
}

impl<P: PointTrait + Copy, const CW: bool, const CL: bool> RemoveSpikes for Ring<P, CW, CL> {
    fn remove_spikes(&mut self) {
        walk_ring_spikes(&mut self.0, CL);
    }
}

impl<P: PointTrait + Copy, const CW: bool, const CL: bool> RemoveSpikes for Polygon<P, CW, CL> {
    fn remove_spikes(&mut self) {
        walk_ring_spikes(&mut self.outer.0, CL);
        for inner in &mut self.inners {
            walk_ring_spikes(&mut inner.0, CL);
        }
    }
}

impl<Pg: RemoveSpikes + geometry_trait::Polygon> RemoveSpikes for MultiPolygon<Pg> {
    fn remove_spikes(&mut self) {
        for p in &mut self.0 {
            p.remove_spikes();
        }
    }
}

#[cfg(test)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/remove_spikes.cpp`: an
    //! out-and-back spur on a linestring is collapsed to its base
    //! vertex.

    use super::remove_spikes;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, linestring};
    use geometry_trait::Linestring as _;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn out_and_back_spur_is_removed() {
        // (0,0) → (1,0) → (3,0) → (2,0): the tip (3,0) is a reversed
        // collinear overshoot between (1,0) and (2,0), so it is dropped,
        // leaving the monotone run (0,0) → (1,0) → (2,0).
        let mut ls: geometry_model::Linestring<P> =
            linestring![(0.0, 0.0), (1.0, 0.0), (3.0, 0.0), (2.0, 0.0)];
        remove_spikes(&mut ls);
        let xs: Vec<f64> = ls.points().map(geometry_trait::Point::get::<0>).collect();
        assert_eq!(xs, vec![0.0, 1.0, 2.0]);
    }

    #[test]
    fn spike_free_linestring_is_unchanged() {
        let mut ls: geometry_model::Linestring<P> = linestring![(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)];
        remove_spikes(&mut ls);
        assert_eq!(ls.points().count(), 3);
    }

    /// A cascading spike: removing the inner tip exposes a second spike
    /// at the now-adjacent pair, which the non-advancing backtrack
    /// (`i -= 1`) then also removes. All overshoots collapse to the base
    /// monotone run.
    #[test]
    fn cascading_spikes_all_collapse() {
        // (0,0) → (2,0) → (5,0) → (3,0) → (1,0): both (5,0) and the
        // resulting reversed vertices are collinear overshoots along the
        // x-axis. After the walk only a monotone sequence survives.
        let mut ls: geometry_model::Linestring<P> =
            linestring![(0.0, 0.0), (2.0, 0.0), (5.0, 0.0), (3.0, 0.0), (1.0, 0.0)];
        remove_spikes(&mut ls);
        let xs: Vec<f64> = ls.points().map(geometry_trait::Point::get::<0>).collect();
        // The sequence is strictly monotone (no reversal remains): each
        // step moves in one direction only.
        for w in xs.windows(3) {
            let d1 = w[1] - w[0];
            let d2 = w[2] - w[1];
            assert!(d1 * d2 >= 0.0, "residual reversal in {xs:?}");
        }
    }

    /// A `Polygon` removes spikes from its exterior *and* every interior
    /// ring.
    #[test]
    fn polygon_removes_spikes_in_outer_and_holes() {
        use geometry_model::{Polygon, Ring};
        use geometry_trait::{Point as _, Polygon as _, Ring as _};
        // Outer square with a spur vertex (5,0) on the bottom edge.
        let outer = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(4.0, 0.0),
            P::new(5.0, 0.0), // reversed-collinear overshoot then back
            P::new(4.0, 0.0),
            P::new(4.0, 4.0),
            P::new(0.0, 4.0),
            P::new(0.0, 0.0),
        ]);
        // Hole with its own spur.
        let hole = Ring::from_vec(vec![
            P::new(1.0, 1.0),
            P::new(2.0, 1.0),
            P::new(3.0, 1.0), // overshoot
            P::new(2.0, 1.0),
            P::new(2.0, 2.0),
            P::new(1.0, 1.0),
        ]);
        let mut pg: Polygon<P> = Polygon::with_inners(outer, vec![hole]);
        remove_spikes(&mut pg);
        // The (5,0) and (3,1) overshoot vertices are gone.
        let ext: Vec<(f64, f64)> = pg
            .exterior()
            .points()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        assert!(!ext.contains(&(5.0, 0.0)), "outer spike survived: {ext:?}");
        let hole_pts: Vec<(f64, f64)> = pg
            .interiors()
            .next()
            .unwrap()
            .points()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        assert!(!hole_pts.contains(&(3.0, 1.0)), "hole spike survived");
    }

    /// A `MultiPolygon` removes spikes from each member polygon.
    #[test]
    fn multipolygon_removes_spikes_from_each_member() {
        use geometry_model::{MultiPolygon, Polygon, Ring};
        use geometry_trait::{Point as _, Polygon as _, Ring as _};
        let spiky = || {
            Polygon::<P>::new(Ring::from_vec(vec![
                P::new(0.0, 0.0),
                P::new(4.0, 0.0),
                P::new(5.0, 0.0),
                P::new(4.0, 0.0),
                P::new(4.0, 4.0),
                P::new(0.0, 4.0),
                P::new(0.0, 0.0),
            ]))
        };
        let mut mpg: MultiPolygon<Polygon<P>> = MultiPolygon(vec![spiky(), spiky()]);
        remove_spikes(&mut mpg);
        for pg in &mpg.0 {
            let pts: Vec<(f64, f64)> = pg
                .exterior()
                .points()
                .map(|p| (p.get::<0>(), p.get::<1>()))
                .collect();
            assert!(!pts.contains(&(5.0, 0.0)), "member spike survived");
        }
    }

    #[test]
    fn ring_seam_spike_is_removed() {
        // A closed ring whose FIRST vertex is a reversed-collinear spike
        // straddling the seam — a triple the interior pass never inspects.
        // Vertices: (0,0)[seam], (2,0), (2,2), (0,2), (1,0), close(0,0).
        // Dropping the closing duplicate leaves the open loop
        //   [(0,0), (2,0), (2,2), (0,2), (1,0)].
        // Seam triple at the first vertex is (back=(1,0), front=(0,0),
        // front+1=(2,0)): u=(0,0)−(1,0)=(−1,0), v=(2,0)−(0,0)=(2,0),
        // cross=0 and dot=−2<0 → a spike at (0,0). Boost removes it; the
        // wrap-around seam cleanup must too.
        use geometry_model::Ring;
        use geometry_trait::{Point as _, Ring as _};

        let mut r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
            P::new(2.0, 2.0),
            P::new(0.0, 2.0),
            P::new(1.0, 0.0),
            P::new(0.0, 0.0),
        ]);
        remove_spikes(&mut r);

        let pts: Vec<(f64, f64)> = r.points().map(|p| (p.get::<0>(), p.get::<1>())).collect();
        // The seam spike vertex (0,0) was dropped.
        assert!(
            !pts.contains(&(0.0, 0.0)),
            "seam spike vertex (0,0) must be gone: {pts:?}"
        );
        // Ring stays closed and non-degenerate.
        assert!(r.points().count() >= 4);
        assert_eq!(pts.first(), pts.last(), "ring must remain closed");
    }
}
