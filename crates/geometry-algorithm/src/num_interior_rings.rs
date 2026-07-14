//! `num_interior_rings(&g)` — Σ interior-ring counts.
//!
//! Mirrors `boost::geometry::num_interior_rings` from
//! `boost/geometry/algorithms/num_interior_rings.hpp`. Every kind that
//! is not a polygon (or multi-polygon) returns `0` by convention; for
//! polygons the count is the number of holes (`interior_rings`
//! accessor); a multi-polygon sums over its members.

use geometry_model::{
    Box, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon, Ring, Segment,
};
use geometry_trait::Polygon as _;

/// Number of interior rings (holes) in `g`.
///
/// * `Polygon` → `interiors().count()`
/// * `MultiPolygon` → sum over members
/// * everything else → `0`
///
/// Mirrors `boost::geometry::num_interior_rings(g)` from
/// `boost/geometry/algorithms/num_interior_rings.hpp`.
#[inline]
#[must_use]
pub fn num_interior_rings<G: NumInteriorRings>(g: &G) -> usize {
    g.num_interior_rings()
}

/// Public-but-implementation-detail trait dispatching by concrete
/// model type (per the LA0.T2 coherence note — the zero default is
/// spelled out per concrete type rather than a blanket, so it does not
/// collide with the polygon impl). Users call [`num_interior_rings`].
#[doc(hidden)]
pub trait NumInteriorRings {
    /// Number of interior rings (holes) in `self`.
    fn num_interior_rings(&self) -> usize;
}

// Zero for every kind that is not polygon-shaped.
impl<T, const D: usize, Cs> NumInteriorRings for Point<T, D, Cs>
where
    T: geometry_coords::CoordinateScalar,
    Cs: geometry_cs::CoordinateSystem,
{
    fn num_interior_rings(&self) -> usize {
        0
    }
}

impl<P: geometry_trait::Point> NumInteriorRings for Linestring<P> {
    fn num_interior_rings(&self) -> usize {
        0
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> NumInteriorRings
    for Ring<P, CW, CL>
{
    fn num_interior_rings(&self) -> usize {
        0
    }
}

impl<P: geometry_trait::Point> NumInteriorRings for Segment<P> {
    fn num_interior_rings(&self) -> usize {
        0
    }
}

impl<P: geometry_trait::Point> NumInteriorRings for Box<P> {
    fn num_interior_rings(&self) -> usize {
        0
    }
}

impl<P: geometry_trait::Point> NumInteriorRings for MultiPoint<P> {
    fn num_interior_rings(&self) -> usize {
        0
    }
}

impl<L: geometry_trait::Linestring> NumInteriorRings for MultiLinestring<L> {
    fn num_interior_rings(&self) -> usize {
        0
    }
}

// Polygon-shaped kinds count holes.
impl<P: geometry_trait::Point, const CW: bool, const CL: bool> NumInteriorRings
    for Polygon<P, CW, CL>
{
    fn num_interior_rings(&self) -> usize {
        self.interiors().count()
    }
}

impl<Pg: geometry_trait::Polygon + NumInteriorRings> NumInteriorRings for MultiPolygon<Pg> {
    fn num_interior_rings(&self) -> usize {
        self.0
            .iter()
            .map(NumInteriorRings::num_interior_rings)
            .sum()
    }
}

#[cfg(test)]
mod tests {
    //! Reference values from
    //! `geometry/test/algorithms/num_interior_rings.cpp`.

    use super::num_interior_rings;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, MultiPolygon, Point2D, Polygon, linestring, polygon};

    type Pt = Point2D<f64, Cartesian>;
    type Ls = Linestring<Pt>;
    type Poly = Polygon<Pt>;

    /// `num_interior_rings.cpp:66` — `POINT(0 0)` → 0.
    #[test]
    fn point_has_zero() {
        assert_eq!(num_interior_rings(&Pt::new(0.0, 0.0)), 0);
    }

    /// `num_interior_rings.cpp:81` — `LINESTRING(0 0,1 1)` → 0.
    #[test]
    fn linestring_has_zero() {
        let ls: Ls = linestring![(0.0, 0.0), (1.0, 1.0)];
        assert_eq!(num_interior_rings(&ls), 0);
    }

    /// `num_interior_rings.cpp:109` — an outer-only polygon → 0.
    #[test]
    fn polygon_outer_only_has_zero() {
        let pg: Poly = polygon![[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]];
        assert_eq!(num_interior_rings(&pg), 0);
    }

    /// `num_interior_rings.cpp:111` — a polygon with two holes → 2.
    #[test]
    fn polygon_with_two_holes() {
        let pg: Poly = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)],
            [(5.0, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 5.0)],
        ];
        assert_eq!(num_interior_rings(&pg), 2);
    }

    /// Every non-polygon kind reports zero interior rings by convention.
    #[test]
    fn non_polygon_kinds_have_zero() {
        use geometry_model::{Box, MultiLinestring, MultiPoint, Ring, Segment};
        let ring: Ring<Pt> =
            Ring::from_vec(vec![Pt::new(0.0, 0.0), Pt::new(1.0, 0.0), Pt::new(0.0, 0.0)]);
        assert_eq!(num_interior_rings(&ring), 0);
        assert_eq!(
            num_interior_rings(&Segment::new(Pt::new(0.0, 0.0), Pt::new(1.0, 1.0))),
            0
        );
        assert_eq!(
            num_interior_rings(&Box::from_corners(Pt::new(0.0, 0.0), Pt::new(1.0, 1.0))),
            0
        );
        assert_eq!(
            num_interior_rings(&MultiPoint(vec![Pt::new(0.0, 0.0)])),
            0
        );
        let mls: MultiLinestring<Ls> = MultiLinestring(vec![linestring![(0.0, 0.0), (1.0, 1.0)]]);
        assert_eq!(num_interior_rings(&mls), 0);
    }

    /// `num_interior_rings.cpp:121` —
    /// `MULTIPOLYGON(<1 hole>,<0 holes>)` → 1 (sum over members).
    #[test]
    fn multi_polygon_sums_holes() {
        let mpg: MultiPolygon<Poly> = MultiPolygon(vec![
            polygon![
                [
                    (0.0, 0.0),
                    (10.0, 0.0),
                    (10.0, 10.0),
                    (0.0, 10.0),
                    (0.0, 0.0)
                ],
                [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)],
            ],
            polygon![[(20.0, 0.0), (30.0, 0.0), (30.0, 10.0), (20.0, 0.0)]],
        ]);
        assert_eq!(num_interior_rings(&mpg), 1);
    }
}
