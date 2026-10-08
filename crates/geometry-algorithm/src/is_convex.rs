//! `is_convex(&g) -> bool`.
//!
//! Mirrors `boost::geometry::is_convex(g)` from
//! `boost/geometry/algorithms/is_convex.hpp`. A ring is convex iff,
//! walked clockwise, it never turns left (a straight run is allowed —
//! collinear vertices don't disqualify), so a ring wound against its
//! declared point order is not convex. A polygon is convex iff its outer
//! ring is convex AND it has no interior rings, and a multi-polygon iff it
//! has no member or one convex member.
//!
//! Cartesian only, and angular input does not compile: Boost also tests
//! spherical and geographic rings, with the spherical side formula and a
//! geodesic side strategy, which this port does not carry.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{MultiPolygon, Polygon, Ring};
use geometry_tag::SameAs;
use geometry_trait::{Point as PointTrait, Polygon as _, Ring as RingTrait};

/// True iff `g` is convex.
///
/// Mirrors `boost::geometry::is_convex(g)` from
/// `boost/geometry/algorithms/is_convex.hpp`.
#[must_use]
pub fn is_convex<G: IsConvex>(g: &G) -> bool {
    g.is_convex()
}

/// Per-kind convexity dispatch. Implemented for [`Ring`], [`Polygon`],
/// and [`MultiPolygon`].
#[doc(hidden)]
pub trait IsConvex {
    /// True iff `self` is convex.
    fn is_convex(&self) -> bool;
}

/// Convexity test for a ring: walked clockwise, no left turn.
///
/// C++: `ring_is_convex` (`algorithms/is_convex.hpp:46-120`). A ring
/// below its closure's minimum size is convex. The walk goes round the
/// ring as `closed_clockwise_view` presents it — closed, and reversed when
/// counter-clockwise — on an ever-circling iterator: from the first point
/// to the next one not equal to it by `math::equals`, and on, each step to
/// the next point not equal to the current one, testing every turn with
/// Boost's exact `side_robust`. Which of two points equal within an epsilon
/// is kept depends on the direction of the walk, and the exact side test
/// can tell them apart, so the walk keeps Boost's direction.
fn ring_is_convex<P: PointTrait, const CW: bool, const CL: bool>(ring: &Ring<P, CW, CL>) -> bool {
    let n = ring.points().len();
    let minimum_size = if CL { 4 } else { 3 };
    if n < minimum_size {
        return true;
    }
    let mut view: Vec<&P> = ring.points().collect();
    if !CL {
        view.push(view[0]);
    }
    if !CW {
        view.reverse();
    }
    let at = |index: usize| view[index % view.len()];
    let same = |a: &P, b: &P| {
        a.get::<0>().tolerant_eq(b.get::<0>()) && a.get::<1>().tolerant_eq(b.get::<1>())
    };
    let xy = |q: &P| (q.get::<0>(), q.get::<1>());

    let mut previous = 0;
    let mut current = 1;
    while same(at(current), at(previous)) && current < n {
        current += 1;
    }
    if current == n {
        // All points are equal.
        return true;
    }
    let mut next = current + 1;
    while same(at(current), at(next)) {
        next += 1;
    }
    for _ in 0..n {
        // A left turn on a clockwise ring is a reflex corner.
        if P::Scalar::side_robust(xy(at(previous)), xy(at(current)), xy(at(next)))
            == core::cmp::Ordering::Greater
        {
            return false;
        }
        previous = current;
        current = next;
        next += 1;
        while same(at(current), at(next)) {
            next += 1;
        }
    }
    true
}

impl<P, const CW: bool, const CL: bool> IsConvex for Ring<P, CW, CL>
where
    P: PointTrait,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn is_convex(&self) -> bool {
        ring_is_convex(self)
    }
}

impl<P, const CW: bool, const CL: bool> IsConvex for Polygon<P, CW, CL>
where
    P: PointTrait,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    fn is_convex(&self) -> bool {
        self.interiors().count() == 0 && ring_is_convex(self.exterior())
    }
}

/// C++: `multi_polygon_is_convex` — two members never make one convex
/// region, so only an empty multi-polygon or a single convex member is.
impl<Pg: IsConvex + geometry_trait::Polygon> IsConvex for MultiPolygon<Pg> {
    fn is_convex(&self) -> bool {
        match self.0.as_slice() {
            [] => true,
            [polygon] => polygon.is_convex(),
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/is_convex.cpp` — a triangle /
    //! square is convex; a reflex polygon and any polygon with a hole
    //! are not.

    use super::is_convex;
    use geometry_cs::Cartesian;
    use geometry_model::{MultiPolygon, Point2D, Polygon, Ring, polygon};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn triangle_is_convex() {
        let pg: Polygon<Pt> = polygon![[(1., 1.), (1., 4.), (5., 1.), (1., 1.)]];
        assert!(is_convex(&pg));
    }

    #[test]
    fn square_is_convex() {
        let pg: Polygon<Pt> = polygon![[(1., 1.), (1., 4.), (4., 4.), (4., 1.), (1., 1.)]];
        assert!(is_convex(&pg));
    }

    /// `concave1` from `is_convex.cpp`: clockwise, with a notch.
    #[test]
    fn notched_rectangle_is_not_convex() {
        let pg: Polygon<Pt> = polygon![[
            (1., 1.),
            (1., 4.),
            (3., 4.),
            (3., 3.),
            (4., 3.),
            (4., 4.),
            (5., 4.),
            (5., 1.),
            (1., 1.)
        ]];
        assert!(!is_convex(&pg));
    }

    /// Boost walks a ring in its declared order and fails on a turn
    /// against it, so a triangle wound counter-clockwise is convex only
    /// where the polygon declares that winding.
    #[test]
    fn a_ring_wound_against_its_declared_order_is_not_convex() {
        let counter_clockwise = [(1., 1.), (5., 1.), (1., 4.), (1., 1.)];
        let declared_clockwise: Polygon<Pt> = Polygon::new(Ring::from_vec(
            counter_clockwise
                .iter()
                .map(|&(x, y)| Pt::new(x, y))
                .collect(),
        ));
        assert!(!is_convex(&declared_clockwise));
        let declared_counter_clockwise: Polygon<Pt, false> = Polygon::new(Ring::from_vec(
            counter_clockwise
                .iter()
                .map(|&(x, y)| Pt::new(x, y))
                .collect(),
        ));
        assert!(is_convex(&declared_counter_clockwise));
    }

    #[test]
    fn reflex_polygon_is_not_convex() {
        let pg: Polygon<Pt> =
            polygon![[(0., 0.), (4., 0.), (2., 1.), (4., 4.), (0., 4.), (0., 0.)]];
        assert!(!is_convex(&pg));
    }

    #[test]
    fn polygon_with_hole_is_not_convex() {
        let pg: Polygon<Pt> = polygon![
            [(0., 0.), (4., 0.), (4., 4.), (0., 4.), (0., 0.)],
            [(1., 1.), (2., 1.), (2., 2.), (1., 2.), (1., 1.)],
        ];
        assert!(!is_convex(&pg));
    }

    /// A ring with fewer than 3 distinct vertices is trivially convex
    /// (the `len < 3` guard).
    #[test]
    fn two_point_ring_is_trivially_convex() {
        let r: Ring<Pt> = Ring::from_vec(alloc::vec![Pt::new(0., 0.), Pt::new(1., 0.)]);
        assert!(is_convex(&r));
    }

    /// The walk starts at the first point not equal to the first: a
    /// repeated start is stepped over, and a ring of one repeated point
    /// has no turn at all and is convex (`algorithms/is_convex.hpp:69-79`).
    #[test]
    fn repeated_points_are_stepped_over() {
        let repeated_start: Polygon<Pt> =
            polygon![[(1., 1.), (1., 1.), (1., 4.), (4., 4.), (4., 1.), (1., 1.)]];
        assert!(is_convex(&repeated_start));
        let one_point: Polygon<Pt> = polygon![[(2., 2.), (2., 2.), (2., 2.), (2., 2.)]];
        assert!(is_convex(&one_point));
    }

    /// All vertices collinear: every cross-product is zero, no sign is
    /// ever set, and the ring counts as convex (matches Boost, where
    /// degenerate/collinear rings are not rejected as concave).
    #[test]
    fn collinear_ring_is_convex() {
        let r: Ring<Pt> = Ring::from_vec(alloc::vec![
            Pt::new(0., 0.),
            Pt::new(1., 1.),
            Pt::new(2., 2.)
        ]);
        assert!(is_convex(&r));
    }

    /// A convex (non-degenerate) `Ring` exercises the `Ring` impl's
    /// positive path directly, not via `Polygon`.
    #[test]
    fn convex_ring_direct() {
        let r: Ring<Pt> = Ring::from_vec(alloc::vec![
            Pt::new(0., 0.),
            Pt::new(0., 4.),
            Pt::new(4., 4.),
            Pt::new(4., 0.),
        ]);
        assert!(is_convex(&r));
    }

    /// `mpoly1` / `mpoly2` from `is_convex.cpp`: a multi-polygon is
    /// convex only as a single convex member — two convex members never
    /// make one convex region.
    #[test]
    fn multi_polygon_is_convex_only_as_one_convex_member() {
        let convex: Polygon<Pt> = polygon![[(1., 1.), (1., 4.), (5., 1.), (1., 1.)]];
        let other: Polygon<Pt> = polygon![[(3., 0.), (3., 1.), (4., 0.), (3., 0.)]];
        assert!(is_convex(&MultiPolygon::<Polygon<Pt>>(alloc::vec![])));
        assert!(is_convex(&MultiPolygon(alloc::vec![convex.clone()])));
        assert!(!is_convex(&MultiPolygon(alloc::vec![convex, other])));
    }

    /// The turn at the seam vertex counts: a polygon whose only reflex
    /// vertex is its first (and closing) vertex is not convex.
    #[test]
    fn reflex_vertex_at_closed_ring_seam_is_not_convex() {
        let pg: Polygon<Pt> =
            polygon![[(2., 1.), (4., 4.), (0., 4.), (0., 0.), (4., 0.), (2., 1.)]];
        assert!(!is_convex(&pg));
        let r: Ring<Pt> = Ring::from_vec(alloc::vec![
            Pt::new(2., 1.),
            Pt::new(4., 4.),
            Pt::new(0., 4.),
            Pt::new(0., 0.),
            Pt::new(4., 0.),
            Pt::new(2., 1.),
        ]);
        assert!(!is_convex(&r));
    }

    /// Below two vertices there is no pair to compare for a closing
    /// duplicate, so the seam-trimming step is skipped entirely. Both
    /// degenerate rings still have to answer — convex, by the same
    /// `len < 3` rule that covers the two-point ring — rather than
    /// index into an empty sequence.
    #[test]
    fn rings_below_two_vertices_are_trivially_convex() {
        let empty: Ring<Pt> = Ring::from_vec(alloc::vec![]);
        assert!(is_convex(&empty));

        let single: Ring<Pt> = Ring::from_vec(alloc::vec![Pt::new(3., 7.)]);
        assert!(is_convex(&single));
    }

    /// Of two vertices a last bit apart Boost keeps the one its clockwise
    /// walk reaches first, and the exact side test tells them apart: this
    /// open counter-clockwise ring turns left at the one kept walking it
    /// backwards, so it is not convex in Boost (`aed7bc3`).
    #[test]
    fn near_duplicates_are_walked_in_boosts_direction() {
        let ring: Ring<Pt, false, false> = Ring::from_vec(vec![
            Pt::new(-124.961_656_060_410_52, -276.892_766_837_829_87),
            Pt::new(-849.223_249_721_270_6, 86.135_835_343_888_52),
            Pt::new(-1_146.321_623_485_671_1, -1_591.643_032_235_419),
            Pt::new(-1_195.115_813_186_1, -1_867.194_390_200_627_6),
            Pt::new(-1_195.115_813_186_100_2, -1_867.194_390_200_627_8),
            Pt::new(-96.609_059_576_314_4, -1_512.500_968_388_648_5),
            Pt::new(-96.609_059_576_314_38, -1_512.500_968_388_648_5),
        ]);
        assert!(!is_convex(&ring));
    }
}
