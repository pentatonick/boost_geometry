//! Coordinate mapping for the stock geometry models.

use geometry_model::{
    Box as ModelBox, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point as ModelPoint,
    Polygon, Ring, Segment,
};
use geometry_trait::{Geometry, Point as PointTrait};

/// Map every point in a stock geometry into a newly constructed geometry.
///
/// The point closure may change the scalar or coordinate-system type while the
/// geometry's topology and ring parameters are preserved.
pub fn map_coords<G, Q, F>(geometry: &G, mut map: F) -> G::Output
where
    G: MapCoords<Q>,
    Q: PointTrait,
    F: FnMut(&G::Point) -> Q,
{
    geometry.map_coords(&mut map)
}

/// Stock geometry support for [`map_coords`].
#[doc(hidden)]
pub trait MapCoords<Q: PointTrait>: Geometry {
    /// The mapped stock geometry.
    type Output;

    /// Reconstruct this geometry by mapping each point.
    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q;
}

impl<A, const D: usize, Cs, Q> MapCoords<Q> for ModelPoint<A, D, Cs>
where
    A: geometry_coords::CoordinateScalar,
    Cs: geometry_cs::CoordinateSystem,
    Q: PointTrait,
{
    type Output = Q;

    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q,
    {
        map(self)
    }
}

impl<P, Q> MapCoords<Q> for Linestring<P>
where
    P: PointTrait,
    Q: PointTrait,
{
    type Output = Linestring<Q>;

    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q,
    {
        Linestring::from_vec(self.0.iter().map(map).collect())
    }
}

impl<P, Q, const CW: bool, const CL: bool> MapCoords<Q> for Ring<P, CW, CL>
where
    P: PointTrait,
    Q: PointTrait,
{
    type Output = Ring<Q, CW, CL>;

    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q,
    {
        Ring::from_vec(self.0.iter().map(map).collect())
    }
}

impl<P, Q, const CW: bool, const CL: bool> MapCoords<Q> for Polygon<P, CW, CL>
where
    P: PointTrait,
    Q: PointTrait,
{
    type Output = Polygon<Q, CW, CL>;

    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q,
    {
        Polygon::with_inners(
            Ring::from_vec(self.outer.0.iter().map(&mut *map).collect()),
            self.inners
                .iter()
                .map(|ring| Ring::from_vec(ring.0.iter().map(&mut *map).collect()))
                .collect(),
        )
    }
}

impl<P, Q> MapCoords<Q> for MultiPoint<P>
where
    P: PointTrait,
    Q: PointTrait,
{
    type Output = MultiPoint<Q>;

    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q,
    {
        MultiPoint::from_vec(self.0.iter().map(map).collect())
    }
}

impl<P, Q> MapCoords<Q> for MultiLinestring<Linestring<P>>
where
    P: PointTrait,
    Q: PointTrait,
{
    type Output = MultiLinestring<Linestring<Q>>;

    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q,
    {
        MultiLinestring::from_vec(
            self.0
                .iter()
                .map(|line| Linestring::from_vec(line.0.iter().map(&mut *map).collect()))
                .collect(),
        )
    }
}

impl<P, Q, const CW: bool, const CL: bool> MapCoords<Q> for MultiPolygon<Polygon<P, CW, CL>>
where
    P: PointTrait,
    Q: PointTrait,
{
    type Output = MultiPolygon<Polygon<Q, CW, CL>>;

    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q,
    {
        MultiPolygon::from_vec(
            self.0
                .iter()
                .map(|polygon| polygon.map_coords(&mut *map))
                .collect(),
        )
    }
}

impl<P, Q> MapCoords<Q> for ModelBox<P>
where
    P: PointTrait,
    Q: PointTrait,
{
    type Output = ModelBox<Q>;

    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q,
    {
        ModelBox::from_corners(map(self.min()), map(self.max()))
    }
}

impl<P, Q> MapCoords<Q> for Segment<P>
where
    P: PointTrait,
    Q: PointTrait,
{
    type Output = Segment<Q>;

    fn map_coords<F>(&self, map: &mut F) -> Self::Output
    where
        F: FnMut(&Self::Point) -> Q,
    {
        Segment::new(map(self.start()), map(self.end()))
    }
}

/// Mutate every stored point in a stock geometry in place.
pub fn map_coords_in_place<G, F>(geometry: &mut G, mut map: F)
where
    G: MapCoordsInPlace,
    F: FnMut(&mut G::Point),
{
    geometry.map_coords_in_place(&mut map);
}

/// Stock geometry support for [`map_coords_in_place`].
#[doc(hidden)]
pub trait MapCoordsInPlace: Geometry {
    /// Visit every stored point mutably.
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point);
}

impl<A, const D: usize, Cs> MapCoordsInPlace for ModelPoint<A, D, Cs>
where
    A: geometry_coords::CoordinateScalar,
    Cs: geometry_cs::CoordinateSystem,
{
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point),
    {
        map(self);
    }
}

impl<P: PointTrait> MapCoordsInPlace for Linestring<P> {
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point),
    {
        self.0.iter_mut().for_each(map);
    }
}

impl<P: PointTrait, const CW: bool, const CL: bool> MapCoordsInPlace for Ring<P, CW, CL> {
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point),
    {
        self.0.iter_mut().for_each(map);
    }
}

impl<P: PointTrait, const CW: bool, const CL: bool> MapCoordsInPlace for Polygon<P, CW, CL> {
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point),
    {
        self.outer.0.iter_mut().for_each(&mut *map);
        for ring in &mut self.inners {
            ring.0.iter_mut().for_each(&mut *map);
        }
    }
}

impl<P: PointTrait> MapCoordsInPlace for MultiPoint<P> {
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point),
    {
        self.0.iter_mut().for_each(map);
    }
}

impl<P: PointTrait> MapCoordsInPlace for MultiLinestring<Linestring<P>> {
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point),
    {
        for line in &mut self.0 {
            line.0.iter_mut().for_each(&mut *map);
        }
    }
}

impl<P: PointTrait, const CW: bool, const CL: bool> MapCoordsInPlace
    for MultiPolygon<Polygon<P, CW, CL>>
{
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point),
    {
        for polygon in &mut self.0 {
            polygon.map_coords_in_place(&mut *map);
        }
    }
}

impl<P> MapCoordsInPlace for ModelBox<P>
where
    P: PointTrait + Clone,
{
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point),
    {
        let mut min = self.min().clone();
        let mut max = self.max().clone();
        map(&mut min);
        map(&mut max);
        *self = ModelBox::from_corners(min, max);
    }
}

impl<P> MapCoordsInPlace for Segment<P>
where
    P: PointTrait + Clone,
{
    fn map_coords_in_place<F>(&mut self, map: &mut F)
    where
        F: FnMut(&mut Self::Point),
    {
        let mut start = self.start().clone();
        let mut end = self.end().clone();
        map(&mut start);
        map(&mut end);
        *self = Segment::new(start, end);
    }
}

#[cfg(test)]
mod tests {
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D};
    use geometry_trait::{Point as _, PointMut as _};

    use super::{map_coords, map_coords_in_place};

    #[test]
    #[allow(
        clippy::cast_possible_truncation,
        reason = "small exact fixtures intentionally exercise scalar rebinding"
    )]
    fn linestring_rebinds_and_mutates() {
        let line = Linestring::from_vec(alloc::vec![
            Point2D::<f64, Cartesian>::new(1.0, 2.0),
            Point2D::new(3.0, 4.0),
        ]);
        let mapped: Linestring<Point2D<f32, Cartesian>> = map_coords(&line, |point| {
            Point2D::new(point.get::<0>() as f32, point.get::<1>() as f32)
        });
        assert_eq!(mapped.0[0], Point2D::new(1.0, 2.0));

        let mut shifted = line;
        map_coords_in_place(&mut shifted, |point| {
            point.set::<0>(point.get::<0>() + 1.0);
        });
        assert_eq!(shifted.0[0], Point2D::new(2.0, 2.0));
    }
}
