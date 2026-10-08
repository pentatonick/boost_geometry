//! `remove_spikes(&mut g)` — drop collinear-and-reversed vertices.
//!
//! Mirrors `boost::geometry::remove_spikes` from
//! `boost/geometry/algorithms/remove_spikes.hpp`. The predicate is
//! Boost's `point_is_spike_or_equal`, and the `or_equal` half carries
//! its weight: a triple `(a, b, c)` qualifies when it is collinear and `c`
//! does not lie beyond `b` along `a → b` — `(b-a) · (c-b) <= 0`, which
//! covers both a reversal and a zero-length step, that is, a repeated
//! vertex. The middle
//! vertex `b` is removed; the walk repeats until nothing qualifies,
//! because collapsing one spike can create a new one at the
//! now-adjacent pair, and peeling a spike off a ring routinely leaves a
//! repeated vertex behind.
//!
//! Per-kind:
//! * `Ring`                → spike-walk the backing `Vec<P>`, then its seam
//! * `Polygon`             → walk outer + every inner ring
//! * `MultiPolygon`        → walk each member
//! * `Linestring`          → the walk without a seam. Boost dispatches only
//!   the areal kinds and leaves a linestring untouched; this arm is the
//!   port's extension.
//!
//! Cartesian only: the collinearity / reversal predicate is the 2D
//! cross/dot product, so angular input does not compile. Boost also takes
//! spherical and geographic rings, with the spherical side formula and a
//! direction test of its own, which this port does not carry.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{Linestring, MultiPolygon, Polygon, Ring};
use geometry_tag::SameAs;
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

/// True iff `b` is a spike between `a` and `c`, **or** duplicates one of
/// them: on one line by Boost's side test, and `c` not beyond `b`.
///
/// Mirrors `detail::point_is_spike_or_equal`
/// (`algorithms/detail/point_is_spike_or_equal.hpp:46-65`), whose
/// `direction_code(a, b, c) < 1` is the sign of `(b-a) · (c-b)` as Boost
/// rounds it. Requiring a reversal instead would leave every repeated
/// vertex in place, including the ones this function creates: removing
/// the apex of `(4,0) (6,0) (4,0)` leaves `(4,0) (4,0)` adjacent, and
/// Boost collapses that.
fn is_spike_or_equal_2d<P: PointTrait>(a: &P, b: &P, c: &P) -> bool {
    let (a2, b2, c2) = (
        (a.get::<0>(), a.get::<1>()),
        (b.get::<0>(), b.get::<1>()),
        (c.get::<0>(), c.get::<1>()),
    );
    // The collinearity half is Boost's `side_by_triangle`, which calls three
    // points collinear whenever any *two* of them are equal by `math::equals`
    // — a relative epsilon — before it looks at any determinant, and the
    // determinant zero within an epsilon of it. A hairline whose two ends
    // are a few last bits apart at a large coordinate is a spike to Boost
    // and a genuine sliver to an exact cross product, which is how one
    // survived into a tile that the reference drew as nothing.
    P::Scalar::side_by_triangle(a2, b2, c2) == core::cmp::Ordering::Equal
        && P::Scalar::direction_code(a2, b2, c2) != core::cmp::Ordering::Greater
}

fn walk_spikes<P: PointTrait>(pts: &mut alloc::vec::Vec<P>) {
    let mut changed = true;
    while changed && pts.len() >= 3 {
        changed = false;
        let mut i = 1;
        while i + 1 < pts.len() {
            if is_spike_or_equal_2d(&pts[i - 1], &pts[i], &pts[i + 1]) {
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

impl<P> RemoveSpikes for Linestring<P>
where
    P: PointTrait,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn remove_spikes(&mut self) {
        walk_spikes(&mut self.0);
    }
}

/// Spike-walk a **ring**: the interior linear pass plus the wrap-around
/// seam that a linestring does not have.
///
/// Mirrors `detail::remove_spikes::range_remove_spikes::apply`
/// (`algorithms/remove_spikes.hpp:66-161`). A ring one point short of its
/// closure's minimum size is left as it is. After the interior pass, Boost
/// drops a closed ring's last point — its closing point, whatever it holds
/// — then repeatedly removes a spike formed at the *first* vertex — the
/// triple `(back-1, back, front)` — and at the *second* — `(back, front,
/// front+1)` — until neither fires. Of two points left the second is, by
/// definition, a spike (Boost ticket #9871), so only the first stays; a
/// closed ring is closed again on it. The interior [`walk_spikes`] alone
/// never forms the seam triples, so a spike sitting on the ring's
/// first/last vertex would otherwise survive.
///
/// `closed` is `true` for a ring whose backing vector repeats its first
/// vertex as its last (the model's `CLOSED` const generic).
fn walk_ring_spikes<P: PointTrait + Copy>(pts: &mut alloc::vec::Vec<P>, closed: bool) {
    // A polygon with only one spike comes out as one point, so only rings
    // shorter than that keep every point.
    let minimum = if closed { 3 } else { 2 };
    if pts.len() < minimum {
        return;
    }

    // Interior pass first.
    walk_spikes(pts);

    // Work on the open sequence, so `first` and `last` are distinct ring
    // vertices.
    if closed {
        pts.pop();
    }

    // Seam cleanup: alternately peel a spike off the back (last vertex)
    // and the front (first vertex) until the seam is clean.
    let mut found = true;
    while found {
        found = false;
        // Spike at the first point: (prev = back-1, back, front).
        while pts.len() >= 3
            && is_spike_or_equal_2d(&pts[pts.len() - 2], &pts[pts.len() - 1], &pts[0])
        {
            pts.pop();
            found = true;
        }
        // Spike at the second point: (back, front, front+1).
        while pts.len() >= 3 && is_spike_or_equal_2d(&pts[pts.len() - 1], &pts[0], &pts[1]) {
            pts.remove(0);
            found = true;
        }
    }

    if pts.len() == 2 {
        pts.pop();
    }

    // Close the ring again on its first vertex.
    if closed {
        let first = pts[0];
        pts.push(first);
    }
}

impl<P, const CW: bool, const CL: bool> RemoveSpikes for Ring<P, CW, CL>
where
    P: PointTrait + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn remove_spikes(&mut self) {
        walk_ring_spikes(&mut self.0, CL);
    }
}

impl<P, const CW: bool, const CL: bool> RemoveSpikes for Polygon<P, CW, CL>
where
    P: PointTrait + Copy,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
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
    //! Ring and polygon behaviour follows
    //! `boost/geometry/test/algorithms/remove_spikes.cpp`. Boost only
    //! dispatches areal kinds and leaves a linestring untouched;
    //! collapsing an out-and-back spur on a linestring to its base
    //! vertex is this port's extension, not a Boost fixture.

    use super::remove_spikes;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Ring, linestring};
    use geometry_trait::{Linestring as _, Point as _, Ring as _};

    type P = Point2D<f64, Cartesian>;

    fn spike_ring(points: &[(f64, f64)]) -> Ring<P> {
        let mut ring = Ring::new();
        for &(x, y) in points {
            ring.push(P::new(x, y));
        }
        ring
    }

    /// A hairline whose two ends are four last bits apart at a coordinate of
    /// 3540, which is inside one epsilon of it.
    ///
    /// C++: `side_by_triangle` calls three points collinear when any two of
    /// them are `math::equals` — a *relative* epsilon — before it computes any
    /// determinant, so Boost sees a spike here and collapses the ring to a
    /// single repeated point. An exact cross product sees a sliver with real
    /// area and keeps it, which is how one survived into a monaco tile the
    /// reference drew as nothing.
    #[test]
    fn a_hairline_within_an_epsilon_is_a_spike() {
        let mut ring = spike_ring(&[
            (3_539.999_999_999_999_5, 482.199_999_999_999_76),
            (3540.0, 482.199_999_999_999_8),
            (3540.0, 479.0),
            (3_539.999_999_999_999_5, 482.199_999_999_999_76),
        ]);
        remove_spikes(&mut ring);
        assert_eq!(ring.0.len(), 2, "{:?}", ring.0);
    }

    /// The same ring with its two ends far enough apart to be two points,
    /// where the sliver has real area and stays.
    #[test]
    fn a_sliver_wider_than_an_epsilon_is_kept() {
        let mut ring = spike_ring(&[
            (3_539.999_999_9, 482.199_999_9),
            (3540.0, 482.2),
            (3540.0, 479.0),
            (3_539.999_999_9, 482.199_999_9),
        ]);
        remove_spikes(&mut ring);
        assert_eq!(ring.0.len(), 4, "{:?}", ring.0);
    }

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
        assert_eq!(xs, vec![0.0, 1.0]);
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

    /// Boost collapses a repeated vertex the same way it collapses a
    /// spike — `point_is_spike_or_equal` covers both. Expected values from
    /// `boost::geometry::remove_spikes` on a clockwise `model::polygon`
    /// (Boost 1.83):
    ///
    /// ```text
    /// consecutive dup  -> (0,0) (0,4) (4,4) (4,0) (0,0)
    /// dup at start     -> (0,0) (0,4) (4,4) (4,0) (0,0)
    /// triple dup       -> (0,0) (0,4) (4,4) (4,0) (0,0)
    /// real spike       -> (0,0) (0,4) (4,4) (4,0) (0,0)
    /// dup + spike      -> (0,0) (0,4) (4,4) (4,0) (0,0)
    /// ```
    ///
    /// The `real spike` row is the one that shows why: removing the apex
    /// of `(4,0) (6,0) (4,0)` leaves `(4,0) (4,0)` adjacent, so a
    /// spike-only predicate makes duplicates out of its own output.
    #[test]
    fn repeated_vertices_are_collapsed() {
        let square = [(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)];

        for (name, input) in [
            (
                "consecutive dup",
                vec![
                    (0.0, 0.0),
                    (0.0, 4.0),
                    (4.0, 4.0),
                    (4.0, 4.0),
                    (4.0, 0.0),
                    (0.0, 0.0),
                ],
            ),
            (
                "dup at start",
                vec![
                    (0.0, 0.0),
                    (0.0, 0.0),
                    (0.0, 4.0),
                    (4.0, 4.0),
                    (4.0, 0.0),
                    (0.0, 0.0),
                ],
            ),
            (
                "triple dup",
                vec![
                    (0.0, 0.0),
                    (0.0, 4.0),
                    (4.0, 4.0),
                    (4.0, 4.0),
                    (4.0, 4.0),
                    (4.0, 0.0),
                    (0.0, 0.0),
                ],
            ),
            (
                "real spike",
                vec![
                    (0.0, 0.0),
                    (0.0, 4.0),
                    (4.0, 4.0),
                    (4.0, 0.0),
                    (6.0, 0.0),
                    (4.0, 0.0),
                    (0.0, 0.0),
                ],
            ),
            (
                "dup + spike",
                vec![
                    (0.0, 0.0),
                    (0.0, 4.0),
                    (4.0, 4.0),
                    (4.0, 4.0),
                    (4.0, 0.0),
                    (6.0, 0.0),
                    (4.0, 0.0),
                    (0.0, 0.0),
                ],
            ),
        ] {
            let mut ring: Ring<P> =
                Ring::from_vec(input.iter().map(|&(x, y)| P::new(x, y)).collect());
            remove_spikes(&mut ring);
            let pts: Vec<(f64, f64)> = ring
                .points()
                .map(|p| (p.get::<0>(), p.get::<1>()))
                .collect();
            assert_eq!(pts, square, "{name}");
        }
    }

    /// Two points left of an open ring are one point and a spike to it
    /// (Boost ticket #9871), so only the first stays: `(0 0) (2 0) (1 0)`
    /// comes out as `(0 0)`, as in Boost (`aed7bc3`).
    #[test]
    fn an_open_ring_of_two_points_keeps_one() {
        let mut ring: Ring<P, true, false> =
            Ring::from_vec(vec![P::new(0.0, 0.0), P::new(2.0, 0.0), P::new(1.0, 0.0)]);
        remove_spikes(&mut ring);
        let pts: Vec<(f64, f64)> = ring
            .points()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        assert_eq!(pts, vec![(0.0, 0.0)]);
    }

    /// A closed ring's last point is its closing point, whatever it holds:
    /// Boost (`aed7bc3`) closes `(0 0) (0 4) (4 4) (4 0) (1 0)` on `(0 0)`.
    #[test]
    fn a_closed_rings_last_point_closes_it() {
        let mut ring = spike_ring(&[(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (1.0, 0.0)]);
        remove_spikes(&mut ring);
        let pts: Vec<(f64, f64)> = ring
            .points()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        assert_eq!(
            pts,
            vec![(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)]
        );
    }

    /// Of two vertices a last bit apart, Boost (`aed7bc3`) keeps the one
    /// its rounding of the direction test leaves: here the second.
    #[test]
    fn near_duplicates_collapse_where_boost_rounds_them() {
        let a = (15_682.546_124_542_228, 174_769.657_699_793_93);
        let c = (259_765.440_433_603_83, 585_953.745_039_905_3);
        let mut ring = spike_ring(&[
            a,
            (-630_679.312_290_246_7, 23_817.278_083_611_003),
            (-630_679.312_290_246_7, 23_817.278_083_611),
            c,
            a,
        ]);
        remove_spikes(&mut ring);
        let pts: Vec<(f64, f64)> = ring
            .points()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        assert_eq!(
            pts,
            vec![a, (-630_679.312_290_246_7, 23_817.278_083_611), c, a]
        );
    }
}
