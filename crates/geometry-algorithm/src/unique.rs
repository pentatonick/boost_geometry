//! `unique(&mut g)` — collapse consecutive duplicate points.
//!
//! Mirrors `boost::geometry::unique` from
//! `boost/geometry/algorithms/unique.hpp`. Boost uses `std::unique`
//! and walks the same kind hierarchy as `reverse`. Per-kind:
//!
//! * `Linestring`, `Ring`  → `Vec::dedup` on the backing vec
//! * `Polygon`             → dedup outer + every inner ring
//! * `MultiLinestring`     → dedup each member
//! * `MultiPolygon`        → dedup each member polygon
//!
//! Two points are equal by Boost's default point comparison,
//! `geometry::equal_to` (`policies/compare.hpp:370-470`), which
//! [`EqualTo`] ports: `math::equals` in every dimension of a Cartesian
//! point; on a spherical or geographic one, `−180°` and `180°` name one
//! meridian and every longitude at a pole names the pole
//! (`strategies/spherical/compare.hpp`).

use geometry_cs::CoordinateSystem;
use geometry_model::{Linestring, MultiLinestring, MultiPolygon, Polygon, Ring};
use geometry_strategy::compare::ComparisonFamily;
use geometry_strategy::{ALL_DIMENSIONS, EqualTo};
use geometry_trait::{Linestring as LinestringTrait, Point as PointTrait, Polygon as PolygonTrait};

/// Collapse runs of equal consecutive points in `g`.
///
/// Mirrors `boost::geometry::unique(g)` from
/// `boost/geometry/algorithms/unique.hpp`.
pub fn unique<G: Unique>(g: &mut G) {
    g.unique();
}

/// Per-kind dedup dispatch.
#[doc(hidden)]
pub trait Unique {
    fn unique(&mut self);
}

/// Drops each point equal to the last one kept, as `std::unique` with
/// Boost's `equal_to` does.
fn dedup_vec<P>(v: &mut alloc::vec::Vec<P>)
where
    P: PointTrait,
    <P::Cs as CoordinateSystem>::Family: ComparisonFamily<P, P>,
{
    v.dedup_by(|next, kept| EqualTo::<ALL_DIMENSIONS>.apply(kept, next));
}

impl<P> Unique for Linestring<P>
where
    P: PointTrait,
    <P::Cs as CoordinateSystem>::Family: ComparisonFamily<P, P>,
{
    fn unique(&mut self) {
        dedup_vec(&mut self.0);
    }
}

impl<P, const CW: bool, const CL: bool> Unique for Ring<P, CW, CL>
where
    P: PointTrait,
    <P::Cs as CoordinateSystem>::Family: ComparisonFamily<P, P>,
{
    fn unique(&mut self) {
        dedup_vec(&mut self.0);
    }
}

impl<P, const CW: bool, const CL: bool> Unique for Polygon<P, CW, CL>
where
    P: PointTrait,
    <P::Cs as CoordinateSystem>::Family: ComparisonFamily<P, P>,
{
    fn unique(&mut self) {
        dedup_vec(&mut self.outer.0);
        for inner in &mut self.inners {
            dedup_vec(&mut inner.0);
        }
    }
}

impl<L: Unique + LinestringTrait> Unique for MultiLinestring<L> {
    fn unique(&mut self) {
        for l in &mut self.0 {
            l.unique();
        }
    }
}

impl<Pg: Unique + PolygonTrait> Unique for MultiPolygon<Pg> {
    fn unique(&mut self) {
        for p in &mut self.0 {
            p.unique();
        }
    }
}

#[cfg(test)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/unique.cpp`: consecutive
    //! duplicate points collapse to one; non-consecutive duplicates are
    //! left alone (Boost only removes *consecutive* runs).

    use super::unique;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, linestring};
    use geometry_trait::Linestring as _;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn consecutive_duplicates_collapse() {
        let mut ls: geometry_model::Linestring<P> = linestring![
            (0.0, 0.0),
            (0.0, 0.0),
            (1.0, 1.0),
            (1.0, 1.0),
            (1.0, 1.0),
            (2.0, 2.0)
        ];
        unique(&mut ls);
        assert_eq!(ls.points().count(), 3);
    }

    #[test]
    fn non_consecutive_duplicates_are_kept() {
        let mut ls: geometry_model::Linestring<P> = linestring![(0.0, 0.0), (1.0, 1.0), (0.0, 0.0)];
        unique(&mut ls);
        assert_eq!(ls.points().count(), 3);
    }

    use geometry_model::{MultiLinestring, MultiPolygon, Point, Point3D, Polygon, Ring, polygon};
    use geometry_trait::{
        MultiLinestring as _, MultiPolygon as _, PointMut as _, Polygon as _, Ring as _,
    };

    /// A `Ring` collapses consecutive duplicate vertices.
    #[test]
    fn ring_dedups_consecutive_vertices() {
        let mut r: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.0, 0.0),
            P::new(1.0, 0.0),
            P::new(1.0, 1.0),
            P::new(1.0, 1.0),
        ]);
        unique(&mut r);
        assert_eq!(r.points().count(), 3);
    }

    /// A `Polygon` dedups its exterior *and* every interior ring.
    #[test]
    fn polygon_dedups_outer_and_holes() {
        let mut p: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(2.0, 2.0), (2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 2.0)]
        ];
        unique(&mut p);
        assert_eq!(p.exterior().points().count(), 5); // one leading dup dropped
        let hole = p.interiors().next().unwrap();
        assert_eq!(hole.points().count(), 4); // one leading dup dropped
    }

    /// A `MultiLinestring` dedups each member independently.
    #[test]
    fn multi_linestring_dedups_each_member() {
        let mut mls: MultiLinestring<geometry_model::Linestring<P>> = MultiLinestring(vec![
            linestring![(0.0, 0.0), (0.0, 0.0), (1.0, 1.0)],
            linestring![(2.0, 2.0), (3.0, 3.0), (3.0, 3.0)],
        ]);
        unique(&mut mls);
        let counts: Vec<usize> = mls.linestrings().map(|l| l.points().count()).collect();
        assert_eq!(counts, vec![2, 2]);
    }

    /// A `MultiPolygon` dedups each member polygon.
    #[test]
    fn multi_polygon_dedups_each_member() {
        let member: Polygon<P> =
            polygon![[(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]];
        let mut mpg: MultiPolygon<Polygon<P>> = MultiPolygon(vec![member.clone(), member]);
        unique(&mut mpg);
        for pg in mpg.polygons() {
            assert_eq!(pg.exterior().points().count(), 4);
        }
    }

    /// The third ordinate of a 3D point counts: two points equal in x,y
    /// but differing in z are *not* merged.
    #[test]
    fn three_d_points_compare_all_three_ordinates() {
        type P3 = Point3D<f64, Cartesian>;
        let mut ls: geometry_model::Linestring<P3> = geometry_model::Linestring(vec![
            P3::new(0.0, 0.0, 0.0),
            P3::new(0.0, 0.0, 1.0), // same x,y — different z: kept
            P3::new(0.0, 0.0, 1.0), // exact duplicate: dropped
        ]);
        unique(&mut ls);
        assert_eq!(ls.points().count(), 2);
    }

    /// So does the fourth of a 4D point (`MAX_DIM`): two points
    /// differing only in it are distinct.
    #[test]
    fn four_d_points_compare_the_fourth_ordinate() {
        type P4 = Point<f64, 4, Cartesian>;
        let mut a = P4::default();
        a.set::<0>(1.0);
        a.set::<1>(2.0);
        a.set::<2>(3.0);
        a.set::<3>(4.0);
        let mut b = a;
        b.set::<3>(9.0); // differ only in the 4th ordinate
        let dup = a;
        let mut ls: geometry_model::Linestring<P4> = geometry_model::Linestring(vec![a, b, dup]);
        unique(&mut ls);
        // a, b differ (4th ordinate); dup == a but is not adjacent to a,
        // so nothing collapses.
        assert_eq!(ls.points().count(), 3);
    }

    /// On a sphere `−180°` and `180°` are one meridian and every longitude
    /// at a pole is the pole, so Boost (`aed7bc3`) collapses both pairs;
    /// `370°` is not normalised to `10°`, and stays.
    #[test]
    fn spherical_points_on_one_meridian_or_pole_collapse() {
        use geometry_cs::{Degree, Spherical};
        use geometry_trait::Point as _;
        type S = Point2D<f64, Spherical<Degree>>;
        let mut ls: geometry_model::Linestring<S> = geometry_model::Linestring(vec![
            S::new(180.0, 10.0),
            S::new(-180.0, 10.0),
            S::new(0.0, 90.0),
            S::new(45.0, 90.0),
            S::new(10.0, 20.0),
            S::new(370.0, 20.0),
            S::new(-190.0, -90.0),
            S::new(170.0, -90.0),
        ]);
        unique(&mut ls);
        let kept: Vec<(f64, f64)> = ls.points().map(|p| (p.get::<0>(), p.get::<1>())).collect();
        assert_eq!(
            kept,
            vec![
                (180.0, 10.0),
                (0.0, 90.0),
                (10.0, 20.0),
                (370.0, 20.0),
                (-190.0, -90.0)
            ]
        );
    }
}
