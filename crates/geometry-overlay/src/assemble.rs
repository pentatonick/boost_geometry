//! OVL4 — output assembly: turn traversed rings into a `MultiPolygon`.
//!
//! Mirrors `boost/geometry/algorithms/detail/overlay/assign_parents.hpp`
//! and `add_rings.hpp`. Traversal ([`traverse`](crate::traverse))
//! produces a flat list of rings; this stage classifies each as an
//! outer boundary or a hole and nests the holes under their containing
//! outer, building `model::Polygon`s and collecting them into a
//! `model::MultiPolygon`.
//!
//! # Classification
//!
//! Each ring's role is decided by **containment**, not winding: a ring
//! contained by no other ring is an outer boundary; a ring contained by
//! exactly one outer is that outer's hole. Containment is tested with
//! [`within`](fn@geometry_algorithm::within) on a representative point, and
//! ties broken toward the **smallest** container — Boost's
//! `assign_parents` area-sorted containment search. Deeper nesting
//! (islands inside holes) is a deferred case.

use alloc::vec::Vec;

use geometry_algorithm::{ring_area, within};
use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{MultiPolygon, Polygon, Ring};
use geometry_tag::SameAs;
use geometry_trait::{PointMut, Ring as RingTrait};

use crate::surface_point::point_on_surface;

/// Assemble traversed rings into a `MultiPolygon`, nesting holes under
/// their containing outer rings.
///
/// A ring contained by no other ring is an outer boundary; a ring
/// contained by exactly one outer becomes that outer's hole. Attachment
/// uses the smallest containing ring.
///
/// Mirrors `assign_parents` + `add_rings`
/// (`detail/overlay/assign_parents.hpp`, `add_rings.hpp`).
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{Point2D, Ring};
/// use geometry_overlay::assemble::assemble_multipolygon;
/// use geometry_trait::MultiPolygon as _;
///
/// type P = Point2D<f64, Cartesian>;
/// // One CCW outer square, no holes → a MultiPolygon of one polygon.
/// let outer: Ring<P> = Ring::from_vec(vec![
///     P::new(0.0, 0.0), P::new(2.0, 0.0), P::new(2.0, 2.0), P::new(0.0, 2.0), P::new(0.0, 0.0),
/// ]);
/// let mp = assemble_multipolygon(&[outer]);
/// assert_eq!(mp.polygons().count(), 1);
/// ```
#[must_use]
pub fn assemble_multipolygon<P>(rings: &[Ring<P>]) -> MultiPolygon<Polygon<P>>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    // Classify each ring by containment depth rather than by signed
    // area: the overlay may emit rings in either winding, but a ring's
    // role is unambiguous from *how many other rings contain it*. Depth
    // 0 (contained by nothing) is an outer; depth 1 (contained by
    // exactly one outer) is a hole of that outer. Deeper nesting
    // (islands in holes) is a deferred case — see the module docs.
    let n = rings.len();
    let mut container_of: Vec<Option<usize>> = alloc::vec![None; n];
    for i in 0..n {
        let Some(rep) = representative_point(&rings[i]) else {
            continue;
        };
        container_of[i] = smallest_ring_container(rings, i, &rep);
    }

    // Outers are rings with no container; each becomes a polygon.
    let mut outer_slot: Vec<Option<usize>> = alloc::vec![None; n];
    let mut polygons: Vec<Polygon<P>> = Vec::new();
    for (i, ring) in rings.iter().enumerate() {
        if container_of[i].is_none() {
            outer_slot[i] = Some(polygons.len());
            polygons.push(Polygon::new(ring.clone()));
        }
    }

    // Holes attach to their (outer) container.
    for (i, ring) in rings.iter().enumerate() {
        if let Some(parent_ring) = container_of[i] {
            if let Some(slot) = outer_slot[parent_ring] {
                polygons[slot].inners.push(ring.clone());
            }
        }
    }

    MultiPolygon(polygons)
}

/// Index of the smallest *other* ring that contains ring `i`, or `None`
/// if no other ring contains it.
///
/// A container is a ring that (a) has **strictly larger** area than ring
/// `i` and (b) contains ring `i`'s representative point. The strict-area
/// gate is what keeps the containment relation acyclic: a nested ring is
/// always a proper subregion of its container, so `area(container) >
/// area(contained)`. Without it, two rings that geometrically nest but
/// happen to share a representative point (e.g. an outer boundary and
/// the hole it encloses, both sampled at a point that lies inside both
/// *as solids*) would each appear to contain the other, collapsing the
/// outer/hole classification and dropping the whole polygon.
fn smallest_ring_container<P>(rings: &[Ring<P>], i: usize, rep: &P) -> Option<usize>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let own_area = ring_area(&rings[i]).abs();
    let mut best: Option<(usize, P::Scalar)> = None;
    for (j, other) in rings.iter().enumerate() {
        if j == i {
            continue;
        }
        let a = ring_area(other).abs();
        // A container must be strictly larger — this breaks any spurious
        // mutual-containment cycle from a shared representative point.
        if a <= own_area {
            continue;
        }
        let pg = Polygon::new(other.clone());
        if within(rep, &pg) {
            match best {
                Some((_, ba)) if ba <= a => {}
                _ => best = Some((j, a)),
            }
        }
    }
    best.map(|(j, _)| j)
}

/// A representative point strictly **inside** the ring.
///
/// The parent-outer test uses the strict-interior `within` predicate, so
/// the representative must be an interior point — a *vertex* would not
/// do: an overlay-produced hole routinely shares a vertex or edge with
/// its containing outer, and that vertex lies on the outer's boundary,
/// where `within` returns `false`. Taking an interior point via
/// [`point_on_surface`](crate::surface_point::point_on_surface) (Boost's
/// `point_on_border` role in `assign_parents.hpp`) sidesteps that: an
/// interior point of a non-self-crossing overlay ring cannot lie on the
/// boundary of a ring that contains it. Falls back to the first vertex
/// only for a degenerate ring `point_on_surface` cannot sample.
fn representative_point<P>(ring: &Ring<P>) -> Option<P>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    let pg = Polygon::new(ring.clone());
    point_on_surface(&pg).or_else(|| ring.points().next().copied())
}

#[cfg(test)]
mod tests {
    use super::assemble_multipolygon;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Ring};
    use geometry_trait::{MultiPolygon as _, Point as _, Polygon as _, Ring as _};

    type P = Point2D<f64, Cartesian>;

    fn ccw_square(x: f64, y: f64, s: f64) -> Ring<P> {
        Ring::from_vec(vec![
            P::new(x, y),
            P::new(x + s, y),
            P::new(x + s, y + s),
            P::new(x, y + s),
            P::new(x, y),
        ])
    }

    fn cw_square(x: f64, y: f64, s: f64) -> Ring<P> {
        // Reverse winding → negative signed area → treated as a hole.
        Ring::from_vec(vec![
            P::new(x, y),
            P::new(x, y + s),
            P::new(x + s, y + s),
            P::new(x + s, y),
            P::new(x, y),
        ])
    }

    #[test]
    fn single_outer_no_holes() {
        let mp = assemble_multipolygon(&[ccw_square(0.0, 0.0, 4.0)]);
        assert_eq!(mp.polygons().count(), 1);
        let pg = mp.polygons().next().unwrap();
        assert_eq!(pg.interiors().count(), 0);
    }

    #[test]
    fn two_disjoint_outers() {
        let mp = assemble_multipolygon(&[ccw_square(0.0, 0.0, 1.0), ccw_square(5.0, 5.0, 1.0)]);
        assert_eq!(mp.polygons().count(), 2);
    }

    #[test]
    fn hole_nested_in_outer() {
        // A large CCW outer with a small CW hole inside it.
        let outer = ccw_square(0.0, 0.0, 10.0);
        let hole = cw_square(3.0, 3.0, 2.0);
        let mp = assemble_multipolygon(&[outer, hole]);
        assert_eq!(mp.polygons().count(), 1);
        let pg = mp.polygons().next().unwrap();
        assert_eq!(pg.interiors().count(), 1);
        // The hole's first vertex is inside the outer.
        let hv = pg.interiors().next().unwrap().points().next().unwrap();
        assert!(hv.get::<0>() > 0.0 && hv.get::<0>() < 10.0);
    }

    #[test]
    fn same_winding_outer_and_hole_do_not_form_a_cycle() {
        // Regression: an outer boundary and the hole it encloses can have
        // the SAME winding (both positive area, as the overlay traversal
        // emits for a union-with-hole). If both rings' interior sample
        // points fall inside each other *as solids*, a naive containment
        // test makes each "contain" the other, dropping the whole polygon.
        // The strict-larger-area container gate must keep it acyclic: the
        // big ring is the outer, the small one its hole.
        let outer = ccw_square(0.0, 0.0, 6.0); // area 36
        // A same-winding (CCW) inner square — area 4, well inside.
        let hole = ccw_square(2.0, 2.0, 2.0); // area 4, SAME winding
        let mp = assemble_multipolygon(&[outer, hole]);
        assert_eq!(mp.polygons().count(), 1, "must not drop the polygon");
        assert_eq!(mp.polygons().next().unwrap().interiors().count(), 1);
    }

    #[test]
    fn hole_sharing_a_vertex_with_outer_still_nests() {
        // Regression: a hole whose FIRST vertex lies on the outer's
        // boundary (here the outer's left edge x = 0). Classifying it by
        // that boundary vertex with strict `within` would (wrongly) make
        // it a separate outer; an interior representative point fixes it.
        let outer = ccw_square(0.0, 0.0, 10.0);
        let hole: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 4.0), // on the outer's x = 0 edge
            P::new(0.0, 6.0),
            P::new(2.0, 6.0),
            P::new(2.0, 4.0),
            P::new(0.0, 4.0),
        ]);
        let mp = assemble_multipolygon(&[outer, hole]);
        assert_eq!(
            mp.polygons().count(),
            1,
            "hole must not become a separate outer"
        );
        assert_eq!(mp.polygons().next().unwrap().interiors().count(), 1);
    }
}
