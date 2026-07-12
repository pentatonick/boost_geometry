//! `num_geometries(&g)` — number of distinct geometries in `g`.
//!
//! Mirrors `boost::geometry::num_geometries` from
//! `boost/geometry/algorithms/num_geometries.hpp`. Single geometries
//! return `1`; multi-geometries return the member count (Boost's same
//! convention — no recursive flatten).

use geometry_model::{
    Box, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon, Ring, Segment,
};

/// Number of distinct geometries in `g`.
///
/// * `Point` / `Linestring` / `Ring` / `Polygon` / `Segment` / `Box`
///   → `1`
/// * `Multi*` → member count
///
/// Mirrors `boost::geometry::num_geometries(g)` from
/// `boost/geometry/algorithms/num_geometries.hpp`.
#[inline]
#[must_use]
pub fn num_geometries<G: NumGeometries>(g: &G) -> usize {
    g.num_geometries()
}

/// Public-but-implementation-detail trait dispatching by concrete
/// model type (per the LA0.T2 coherence note). One impl per
/// `geometry-model` struct; users call [`num_geometries`].
#[doc(hidden)]
pub trait NumGeometries {
    /// Number of distinct geometries in `self`.
    fn num_geometries(&self) -> usize;
}

// Singles return 1.
impl<T, const D: usize, Cs> NumGeometries for Point<T, D, Cs>
where
    T: geometry_coords::CoordinateScalar,
    Cs: geometry_cs::CoordinateSystem,
{
    fn num_geometries(&self) -> usize {
        1
    }
}

impl<P: geometry_trait::Point> NumGeometries for Linestring<P> {
    fn num_geometries(&self) -> usize {
        1
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> NumGeometries for Ring<P, CW, CL> {
    fn num_geometries(&self) -> usize {
        1
    }
}

impl<P: geometry_trait::Point, const CW: bool, const CL: bool> NumGeometries
    for Polygon<P, CW, CL>
{
    fn num_geometries(&self) -> usize {
        1
    }
}

impl<P: geometry_trait::Point> NumGeometries for Segment<P> {
    fn num_geometries(&self) -> usize {
        1
    }
}

impl<P: geometry_trait::Point> NumGeometries for Box<P> {
    fn num_geometries(&self) -> usize {
        1
    }
}

// Multis return member counts.
impl<P: geometry_trait::Point> NumGeometries for MultiPoint<P> {
    fn num_geometries(&self) -> usize {
        self.0.len()
    }
}

impl<L: geometry_trait::Linestring> NumGeometries for MultiLinestring<L> {
    fn num_geometries(&self) -> usize {
        self.0.len()
    }
}

impl<Pg: geometry_trait::Polygon> NumGeometries for MultiPolygon<Pg> {
    fn num_geometries(&self) -> usize {
        self.0.len()
    }
}

#[cfg(test)]
mod tests {
    //! Reference values from
    //! `geometry/test/algorithms/num_geometries.cpp`.

    use super::num_geometries;
    use geometry_cs::Cartesian;
    use geometry_model::{
        Linestring, MultiPoint, MultiPolygon, Point2D, Polygon, linestring, polygon,
    };

    type Pt = Point2D<f64, Cartesian>;
    type Ls = Linestring<Pt>;
    type Poly = Polygon<Pt>;

    /// `num_geometries.cpp:66` — `POINT(0 0)` → 1.
    #[test]
    fn point_is_one() {
        assert_eq!(num_geometries(&Pt::new(0.0, 0.0)), 1);
    }

    /// `num_geometries.cpp:81` — `LINESTRING(0 0,1 1,2 2)` → 1.
    #[test]
    fn linestring_is_one() {
        let ls: Ls = linestring![(0.0, 0.0), (1.0, 1.0)];
        assert_eq!(num_geometries(&ls), 1);
    }

    /// `num_geometries.cpp:114` — a polygon with a hole is still 1.
    #[test]
    fn polygon_with_holes_is_one() {
        let pg: Poly = polygon![
            [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)],
        ];
        assert_eq!(num_geometries(&pg), 1);
    }

    /// `num_geometries.cpp:91` — `MULTIPOINT(0 0,0 0,1 1)` → 3.
    #[test]
    fn multi_point_returns_member_count() {
        let mp = MultiPoint(vec![
            Pt::new(0.0, 0.0),
            Pt::new(1.0, 1.0),
            Pt::new(2.0, 2.0),
        ]);
        assert_eq!(num_geometries(&mp), 3);
    }

    /// `num_geometries.cpp:124` — a 2-member multipolygon → 2.
    #[test]
    fn multi_polygon_returns_member_count() {
        let mpg: MultiPolygon<Poly> = MultiPolygon(vec![
            polygon![[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]],
            polygon![[(2.0, 2.0), (3.0, 2.0), (3.0, 3.0), (2.0, 2.0)]],
        ]);
        assert_eq!(num_geometries(&mpg), 2);
    }
}
