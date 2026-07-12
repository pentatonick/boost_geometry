//! Geometry visitors — `for_each_point` and `for_each_segment`.
//!
//! Mirror `boost::geometry::for_each_point` and
//! `boost::geometry::for_each_segment` from
//! `boost/geometry/algorithms/for_each.hpp`. Both pass each visited
//! element to a user closure by `&` (read-only). Boost also exposes
//! mutating variants; we ship the read-only forms — mutation is the
//! KC1 `PointMut` concern, not the visitor's job.

use geometry_model::{Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon, Ring};
use geometry_trait::{
    Closure, Linestring as LinestringTrait, MultiPoint as MultiPointTrait, Polygon as PolygonTrait,
    Ring as RingTrait,
};

/// Apply `f` to every stored point of `g`, recursively.
///
/// Mirrors `boost::geometry::for_each_point(g, f)` from
/// `boost/geometry/algorithms/for_each.hpp`. The closure is invoked
/// once per *stored* point — a closed ring visits its closing vertex
/// once (it appears once in storage).
pub fn for_each_point<G, F>(g: &G, mut f: F)
where
    G: ForEachPoint,
    F: FnMut(&G::Point),
{
    g.for_each_point(&mut f);
}

/// Per-kind point-visitor dispatch.
#[doc(hidden)]
pub trait ForEachPoint {
    type Point: geometry_trait::Point;
    fn for_each_point<F: FnMut(&Self::Point)>(&self, f: &mut F);
}

impl<T, const D: usize, Cs> ForEachPoint for Point<T, D, Cs>
where
    T: geometry_coords::CoordinateScalar,
    Cs: geometry_cs::CoordinateSystem,
{
    type Point = Self;
    fn for_each_point<F: FnMut(&Self::Point)>(&self, f: &mut F) {
        f(self);
    }
}

impl<P: geometry_trait::Point> ForEachPoint for Linestring<P> {
    type Point = P;
    fn for_each_point<F: FnMut(&P)>(&self, f: &mut F) {
        for p in self.points() {
            f(p);
        }
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> ForEachPoint for Ring<P, CW, CL> {
    type Point = P;
    fn for_each_point<F: FnMut(&P)>(&self, f: &mut F) {
        for p in self.points() {
            f(p);
        }
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> ForEachPoint for Polygon<P, CW, CL> {
    type Point = P;
    fn for_each_point<F: FnMut(&P)>(&self, f: &mut F) {
        self.exterior().for_each_point(f);
        for inner in self.interiors() {
            inner.for_each_point(f);
        }
    }
}

impl<P: geometry_trait::Point> ForEachPoint for MultiPoint<P> {
    type Point = P;
    fn for_each_point<F: FnMut(&P)>(&self, f: &mut F) {
        for p in self.points() {
            f(p);
        }
    }
}

impl<L> ForEachPoint for MultiLinestring<L>
where
    L: LinestringTrait + ForEachPoint<Point = <L as geometry_trait::Geometry>::Point>,
{
    type Point = <L as geometry_trait::Geometry>::Point;
    fn for_each_point<F: FnMut(&Self::Point)>(&self, f: &mut F) {
        for l in &self.0 {
            l.for_each_point(f);
        }
    }
}

impl<Pg> ForEachPoint for MultiPolygon<Pg>
where
    Pg: PolygonTrait + ForEachPoint<Point = <Pg as geometry_trait::Geometry>::Point>,
{
    type Point = <Pg as geometry_trait::Geometry>::Point;
    fn for_each_point<F: FnMut(&Self::Point)>(&self, f: &mut F) {
        for p in &self.0 {
            p.for_each_point(f);
        }
    }
}

/// Apply `f` to every segment of `g` as a `(&start, &end)` pair,
/// recursively.
///
/// Mirrors `boost::geometry::for_each_segment(g, f)` from
/// `boost/geometry/algorithms/for_each.hpp`. A closed ring visits its
/// closing edge via the repeated last-vertex already in storage; an
/// open ring emits the implicit `(last, first)` closing edge.
pub fn for_each_segment<G, F>(g: &G, mut f: F)
where
    G: ForEachSegment,
    F: FnMut(&G::Point, &G::Point),
{
    g.for_each_segment(&mut f);
}

/// Per-kind segment-visitor dispatch.
#[doc(hidden)]
pub trait ForEachSegment {
    type Point: geometry_trait::Point;
    fn for_each_segment<F: FnMut(&Self::Point, &Self::Point)>(&self, f: &mut F);
}

impl<P: geometry_trait::Point> ForEachSegment for Linestring<P> {
    type Point = P;
    fn for_each_segment<F: FnMut(&P, &P)>(&self, f: &mut F) {
        let pts: alloc::vec::Vec<&P> = self.points().collect();
        for w in pts.windows(2) {
            f(w[0], w[1]);
        }
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> ForEachSegment for Ring<P, CW, CL> {
    type Point = P;
    fn for_each_segment<F: FnMut(&P, &P)>(&self, f: &mut F) {
        let pts: alloc::vec::Vec<&P> = self.points().collect();
        if pts.len() < 2 {
            return;
        }
        for w in pts.windows(2) {
            f(w[0], w[1]);
        }
        // Open ring: add the implicit closing edge. A closed ring
        // already stores its closing vertex, so the windowed walk
        // emitted the closing edge.
        if matches!(self.closure(), Closure::Open) {
            f(pts[pts.len() - 1], pts[0]);
        }
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> ForEachSegment
    for Polygon<P, CW, CL>
{
    type Point = P;
    fn for_each_segment<F: FnMut(&P, &P)>(&self, f: &mut F) {
        self.exterior().for_each_segment(f);
        for inner in self.interiors() {
            inner.for_each_segment(f);
        }
    }
}

impl<L> ForEachSegment for MultiLinestring<L>
where
    L: LinestringTrait + ForEachSegment<Point = <L as geometry_trait::Geometry>::Point>,
{
    type Point = <L as geometry_trait::Geometry>::Point;
    fn for_each_segment<F: FnMut(&Self::Point, &Self::Point)>(&self, f: &mut F) {
        for l in &self.0 {
            l.for_each_segment(f);
        }
    }
}

impl<Pg> ForEachSegment for MultiPolygon<Pg>
where
    Pg: PolygonTrait + ForEachSegment<Point = <Pg as geometry_trait::Geometry>::Point>,
{
    type Point = <Pg as geometry_trait::Geometry>::Point;
    fn for_each_segment<F: FnMut(&Self::Point, &Self::Point)>(&self, f: &mut F) {
        for p in &self.0 {
            p.for_each_segment(f);
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "Visited coordinates are exact literals.")]
mod tests {
    //! Reference from `boost/geometry/test/algorithms/for_each.cpp`.

    use super::{for_each_point, for_each_segment};
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, MultiPoint, Point2D, Polygon, linestring, polygon};
    use geometry_trait::Point as _;

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn point_visits_once() {
        let p = Pt::new(3.0, 4.0);
        let mut sum = 0.0;
        for_each_point(&p, |q| sum += q.get::<0>() + q.get::<1>());
        assert_eq!(sum, 7.0);
    }

    #[test]
    fn linestring_visits_in_order() {
        let ls: Linestring<Pt> = linestring![(0.0, 0.0), (3.0, 4.0), (4.0, 3.0)];
        let mut xs = alloc::vec::Vec::new();
        for_each_point(&ls, |p| xs.push(p.get::<0>()));
        assert_eq!(xs, vec![0.0, 3.0, 4.0]);
    }

    #[test]
    fn polygon_visits_outer_then_inner() {
        let pg: Polygon<Pt> = polygon![
            [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)]
        ];
        let mut count = 0;
        for_each_point(&pg, |_| count += 1);
        assert_eq!(count, 5 + 4); // outer + inner stored points
    }

    #[test]
    fn multipoint_visits_each() {
        let mp = MultiPoint(vec![Pt::new(0.0, 0.0), Pt::new(1.0, 1.0)]);
        let mut count = 0;
        for_each_point(&mp, |_| count += 1);
        assert_eq!(count, 2);
    }

    #[test]
    fn linestring_segments_are_consecutive_pairs() {
        let ls: Linestring<Pt> = linestring![(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)];
        let mut edges = alloc::vec::Vec::new();
        for_each_segment(&ls, |a, b| edges.push((a.get::<0>(), b.get::<0>())));
        assert_eq!(edges, vec![(0.0, 3.0), (3.0, 3.0)]);
    }

    #[test]
    fn closed_ring_segment_count_equals_edges() {
        // A closed 5-point square ring stores its closing vertex, so a
        // windowed walk emits 4 edges.
        let pg: Polygon<Pt> =
            polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let mut count = 0;
        for_each_segment(&pg, |_, _| count += 1);
        assert_eq!(count, 4);
    }
}
