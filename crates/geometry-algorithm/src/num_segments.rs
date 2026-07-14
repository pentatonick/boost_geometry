//! `num_segments(&g)` — total edge count.
//!
//! Mirrors `boost::geometry::num_segments` from
//! `boost/geometry/algorithms/num_segments.hpp`.
//!
//! | Kind | Segment count |
//! |---|---|
//! | `Point` / `MultiPoint` | `0` |
//! | `Segment` | `1` |
//! | `Box` | `4` (an axis-aligned rectangle has four edges) |
//! | `Linestring` with `n` points | `n - 1` (`0` if `n < 2`) |
//! | `Ring` with `n` points | `n - 1` if closed, `n` if open |
//! | `Polygon` | Σ over exterior + interior rings |
//! | `MultiLinestring` / `MultiPolygon` | sum over members |

use geometry_model::{
    Box, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon, Ring, Segment,
};
use geometry_trait::{Closure, Linestring as _, Polygon as _, Ring as _};

/// Total number of segments (edges) in `g`.
///
/// Mirrors `boost::geometry::num_segments(g)` from
/// `boost/geometry/algorithms/num_segments.hpp`.
#[inline]
#[must_use]
pub fn num_segments<G: NumSegments>(g: &G) -> usize {
    g.num_segments()
}

/// Public-but-implementation-detail trait dispatching by concrete
/// model type (per the LA0.T2 coherence note). One impl per
/// `geometry-model` struct; users call [`num_segments`].
#[doc(hidden)]
pub trait NumSegments {
    /// Total number of segments (edges) in `self`.
    fn num_segments(&self) -> usize;
}

impl<T, const D: usize, Cs> NumSegments for Point<T, D, Cs>
where
    T: geometry_coords::CoordinateScalar,
    Cs: geometry_cs::CoordinateSystem,
{
    fn num_segments(&self) -> usize {
        0
    }
}

impl<P: geometry_trait::Point> NumSegments for MultiPoint<P> {
    fn num_segments(&self) -> usize {
        0
    }
}

impl<P: geometry_trait::Point> NumSegments for Segment<P> {
    fn num_segments(&self) -> usize {
        1
    }
}

impl<P: geometry_trait::Point> NumSegments for Box<P> {
    fn num_segments(&self) -> usize {
        4
    }
}

impl<P: geometry_trait::Point> NumSegments for Linestring<P> {
    fn num_segments(&self) -> usize {
        let n = self.points().count();
        if n < 2 { 0 } else { n - 1 }
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> NumSegments for Ring<P, CW, CL> {
    fn num_segments(&self) -> usize {
        let n = self.points().count();
        // Fewer than two points → no segments, regardless of closure.
        // Mirrors `num_segments.hpp:46-48` (`if (n <= 1) return 0;`).
        if n <= 1 {
            0
        } else if matches!(self.closure(), Closure::Closed) {
            n - 1
        } else {
            n
        }
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> NumSegments for Polygon<P, CW, CL> {
    fn num_segments(&self) -> usize {
        let mut n = NumSegments::num_segments(self.exterior());
        for inner in self.interiors() {
            n += NumSegments::num_segments(inner);
        }
        n
    }
}

impl<L: geometry_trait::Linestring + NumSegments> NumSegments for MultiLinestring<L> {
    fn num_segments(&self) -> usize {
        self.0.iter().map(NumSegments::num_segments).sum()
    }
}

impl<Pg: geometry_trait::Polygon + NumSegments> NumSegments for MultiPolygon<Pg> {
    fn num_segments(&self) -> usize {
        self.0.iter().map(NumSegments::num_segments).sum()
    }
}

#[cfg(test)]
mod tests {
    //! Reference values from
    //! `geometry/test/algorithms/num_segments.cpp`.

    use super::num_segments;
    use geometry_cs::Cartesian;
    use geometry_model::{
        Box, Linestring, MultiPoint, Point2D, Polygon, Ring, Segment, linestring, polygon,
    };

    type Pt = Point2D<f64, Cartesian>;
    type Ls = Linestring<Pt>;
    type Poly = Polygon<Pt>;

    /// `num_segments.cpp:104` — `POINT(0 0)` → 0.
    #[test]
    fn point_has_no_segments() {
        assert_eq!(num_segments(&Pt::new(0.0, 0.0)), 0);
    }

    /// `num_segments.cpp:139` — a multipoint has no edges → 0.
    #[test]
    fn multi_point_has_no_segments() {
        let mp = MultiPoint(vec![Pt::new(0.0, 0.0), Pt::new(1.0, 1.0)]);
        assert_eq!(num_segments(&mp), 0);
    }

    /// `num_segments.cpp:109` — `SEGMENT(0 0,1 1)` → 1.
    #[test]
    fn segment_is_one() {
        let s = Segment::new(Pt::new(0.0, 0.0), Pt::new(1.0, 1.0));
        assert_eq!(num_segments(&s), 1);
    }

    /// `num_segments.cpp:114` — `BOX(0 0,1 1)` → 4.
    #[test]
    fn box_is_four() {
        let b = Box::from_corners(Pt::new(0.0, 0.0), Pt::new(1.0, 1.0));
        assert_eq!(num_segments(&b), 4);
    }

    /// `num_segments.cpp:128` — `LINESTRING(0 0,0 0)` (2 points) → 1.
    #[test]
    fn linestring_two_points_is_one_segment() {
        let ls: Ls = linestring![(0.0, 0.0), (1.0, 1.0)];
        assert_eq!(num_segments(&ls), 1);
    }

    /// `num_segments.cpp:130` — a 4-point linestring → 3.
    #[test]
    fn linestring_n_points_is_n_minus_one() {
        let ls: Ls = linestring![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 3.0)];
        assert_eq!(num_segments(&ls), 3);
    }

    /// `num_segments.cpp:177` — a closed ring stores the closing vertex
    /// explicitly, so `n` points → `n - 1` edges. 5 points → 4 edges.
    #[test]
    fn closed_ring_rectangle_has_4_edges() {
        let mut r = Ring::<Pt>::new();
        r.push(Pt::new(0.0, 0.0));
        r.push(Pt::new(4.0, 0.0));
        r.push(Pt::new(4.0, 3.0));
        r.push(Pt::new(0.0, 3.0));
        r.push(Pt::new(0.0, 0.0));
        assert_eq!(num_segments(&r), 4);
    }

    /// `num_segments.cpp:164` — an open ring leaves the closing edge
    /// implicit, so `n` points → `n` edges. 4 points → 4 edges.
    #[test]
    fn open_ring_rectangle_has_4_edges() {
        let mut r = Ring::<Pt, true, false>::new();
        r.push(Pt::new(0.0, 0.0));
        r.push(Pt::new(4.0, 0.0));
        r.push(Pt::new(4.0, 3.0));
        r.push(Pt::new(0.0, 3.0));
        assert_eq!(num_segments(&r), 4);
    }

    /// `num_segments.cpp:221` —
    /// `POLYGON((0 0,10 0,10 10,0 10,0 0),(1 1,2 1,1 2,1 1))` → 7
    /// (closed outer 5 pts = 4 edges, closed hole 4 pts = 3 edges).
    #[test]
    fn polygon_with_one_hole_sums() {
        let pg: Poly = polygon![
            [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)],
            [(1.0, 1.0), (2.0, 1.0), (1.0, 2.0), (1.0, 1.0)],
        ];
        assert_eq!(num_segments(&pg), 7);
    }

    /// `num_segments.cpp:126-127` — an empty or 1-point linestring has
    /// no edges (the `n < 2` guard).
    #[test]
    fn short_linestrings_have_no_segments() {
        let empty: Ls = linestring![];
        assert_eq!(num_segments(&empty), 0);
        let single: Ls = linestring![(0.0, 0.0)];
        assert_eq!(num_segments(&single), 0);
    }

    /// `num_segments.cpp:143-156` — a multi-linestring sums its members.
    #[test]
    fn multi_linestring_sums_members() {
        let mls = geometry_model::MultiLinestring::<Ls>(vec![
            linestring![(0.0, 0.0), (1.0, 1.0)], // 1 edge
            linestring![(2.0, 2.0), (3.0, 3.0), (4.0, 4.0), (5.0, 5.0)], // 3 edges
        ]);
        assert_eq!(num_segments(&mls), 4);
    }

    /// `num_segments.cpp:242-262` — a multi-polygon sums its members.
    #[test]
    fn multi_polygon_sums_members() {
        let plain: Poly = polygon![[(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)]]; // 4
        let holed: Poly = polygon![
            [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)], // 4
            [(1.0, 1.0), (2.0, 1.0), (1.0, 2.0), (1.0, 1.0)],             // 3
        ];
        let mpg = geometry_model::MultiPolygon(vec![plain, holed]);
        assert_eq!(num_segments(&mpg), 11);
    }

    #[test]
    fn single_point_open_ring_has_no_segments() {
        // Regression: a 1-point ring has zero segments regardless of
        // closure. Boost guards `if (n <= 1) return 0;`
        // (num_segments.hpp:46-48). The port previously returned 1 for
        // the open case.
        use geometry_model::Ring;
        let open_one: Ring<Point2D<f64, Cartesian>, true, false> =
            Ring::from_vec(vec![Point2D::new(0.0, 0.0)]);
        assert_eq!(num_segments(&open_one), 0);
    }
}
