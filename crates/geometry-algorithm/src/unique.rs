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
//! Two points are equal iff every coordinate matches. Boost uses `==`
//! on the coordinate type, which for floats is exact equality; we
//! mirror that via [`Point::get`](geometry_trait::Point::get) per
//! dimension, driven by [`geometry_trait::fold_dims`].

use geometry_model::{Linestring, MultiLinestring, MultiPolygon, Polygon, Ring};
use geometry_trait::{
    Linestring as LinestringTrait, Point as PointTrait, Polygon as PolygonTrait, fold_dims,
};

/// Collapse runs of coordinate-equal consecutive points in `g`.
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

/// Coordinate-wise equality. Mirrors Boost's `operator==` path through
/// `traits::access<P, D>::get` — exact (bitwise-via-`==`) per Boost.
fn points_equal<P: PointTrait>(a: &P, b: &P) -> bool {
    // `fold_dims` recurses over the dimensions with a hard-coded const
    // `D`; the closure receives the runtime index only as a label, so
    // we re-issue `get::<D>` with matching literals.
    fold_dims(true, a, |acc, _p, d| {
        acc && match d {
            0 => a.get::<0>() == b.get::<0>(),
            1 => a.get::<1>() == b.get::<1>(),
            2 => a.get::<2>() == b.get::<2>(),
            3 => a.get::<3>() == b.get::<3>(),
            _ => unreachable!("fold_dims caps at MAX_DIM"),
        }
    })
}

fn dedup_vec<P: PointTrait>(v: &mut alloc::vec::Vec<P>) {
    v.dedup_by(|a, b| points_equal::<P>(a, b));
}

impl<P: PointTrait> Unique for Linestring<P> {
    fn unique(&mut self) {
        dedup_vec(&mut self.0);
    }
}

impl<P: PointTrait, const CW: bool, const CL: bool> Unique for Ring<P, CW, CL> {
    fn unique(&mut self) {
        dedup_vec(&mut self.0);
    }
}

impl<P: PointTrait, const CW: bool, const CL: bool> Unique for Polygon<P, CW, CL> {
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

    use geometry_model::{
        MultiLinestring, MultiPolygon, Point, Point3D, Polygon, Ring, polygon,
    };
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
        let counts: Vec<usize> = mls
            .linestrings()
            .map(|l| l.points().count())
            .collect();
        assert_eq!(counts, vec![2, 2]);
    }

    /// A `MultiPolygon` dedups each member polygon.
    #[test]
    fn multi_polygon_dedups_each_member() {
        let member: Polygon<P> = polygon![[
            (0.0, 0.0),
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 0.0)
        ]];
        let mut mpg: MultiPolygon<Polygon<P>> = MultiPolygon(vec![member.clone(), member]);
        unique(&mut mpg);
        for pg in mpg.polygons() {
            assert_eq!(pg.exterior().points().count(), 4);
        }
    }

    /// `points_equal` compares the third ordinate for 3D points — the
    /// `2 =>` arm. Two points equal in x,y but differing in z are *not*
    /// merged.
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

    /// `points_equal` reaches the `3 =>` arm for 4D points (`MAX_DIM)`:
    /// two points differing only in the fourth ordinate are distinct.
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
        let mut ls: geometry_model::Linestring<P4> =
            geometry_model::Linestring(vec![a, b, dup]);
        unique(&mut ls);
        // a, b differ (4th ordinate); dup == a but is not adjacent to a,
        // so nothing collapses.
        assert_eq!(ls.points().count(), 3);
    }
}
