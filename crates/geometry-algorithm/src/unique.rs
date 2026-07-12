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
}
