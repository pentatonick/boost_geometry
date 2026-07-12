//! `transform(&g, &strategy)` — return a fresh geometry whose every
//! point has been mapped through `strategy`.
//!
//! Mirrors `boost::geometry::transform(g_src, g_dst, strategy)` from
//! `boost/geometry/algorithms/transform.hpp`. The Boost overload
//! mutates `g_dst` through an out-parameter; the Rust port returns the
//! result by value (same observable behaviour).

use geometry_model::{Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon, Ring};
use geometry_strategy::TransformStrategy;
use geometry_trait::{
    Linestring as LinestringTrait, MultiPoint as MultiPointTrait, Point as PointTrait,
    Polygon as PolygonTrait, Ring as RingTrait,
};

/// Map every point of `g` through `s`, returning a new geometry of the
/// same kind (with the strategy's output point type).
///
/// Mirrors `boost::geometry::transform(src, dst, strategy)` from
/// `boost/geometry/algorithms/transform.hpp`.
pub fn transform<G, S>(g: &G, s: &S) -> G::Output
where
    G: Transform<S>,
{
    g.transform(s)
}

/// Per-kind transform dispatch.
#[doc(hidden)]
pub trait Transform<S> {
    type Output;
    fn transform(&self, s: &S) -> Self::Output;
}

impl<T, const D: usize, Cs, S> Transform<S> for Point<T, D, Cs>
where
    T: geometry_coords::CoordinateScalar,
    Cs: geometry_cs::CoordinateSystem,
    Self: PointTrait,
    S: TransformStrategy<Self>,
{
    type Output = S::Output;
    fn transform(&self, s: &S) -> Self::Output {
        s.transform(self)
    }
}

impl<P, S> Transform<S> for Linestring<P>
where
    P: PointTrait,
    S: TransformStrategy<P>,
{
    type Output = Linestring<S::Output>;
    fn transform(&self, s: &S) -> Self::Output {
        Linestring(self.points().map(|p| s.transform(p)).collect())
    }
}

impl<P, S, const CW: bool, const CL: bool> Transform<S> for Ring<P, CW, CL>
where
    P: PointTrait,
    S: TransformStrategy<P>,
{
    type Output = Ring<S::Output, CW, CL>;
    fn transform(&self, s: &S) -> Self::Output {
        Ring::from_vec(self.points().map(|p| s.transform(p)).collect())
    }
}

impl<P, S, const CW: bool, const CL: bool> Transform<S> for Polygon<P, CW, CL>
where
    P: PointTrait,
    S: TransformStrategy<P>,
{
    type Output = Polygon<S::Output, CW, CL>;
    fn transform(&self, s: &S) -> Self::Output {
        Polygon::with_inners(
            self.exterior().transform(s),
            self.interiors().map(|r| r.transform(s)).collect(),
        )
    }
}

impl<P, S> Transform<S> for MultiPoint<P>
where
    P: PointTrait,
    S: TransformStrategy<P>,
{
    type Output = MultiPoint<S::Output>;
    fn transform(&self, s: &S) -> Self::Output {
        MultiPoint(self.points().map(|p| s.transform(p)).collect())
    }
}

impl<L, S> Transform<S> for MultiLinestring<L>
where
    L: LinestringTrait + Transform<S>,
    <L as Transform<S>>::Output: LinestringTrait,
{
    type Output = MultiLinestring<<L as Transform<S>>::Output>;
    fn transform(&self, s: &S) -> Self::Output {
        MultiLinestring(self.0.iter().map(|l| l.transform(s)).collect())
    }
}

impl<Pg, S> Transform<S> for MultiPolygon<Pg>
where
    Pg: PolygonTrait + Transform<S>,
    <Pg as Transform<S>>::Output: PolygonTrait,
{
    type Output = MultiPolygon<<Pg as Transform<S>>::Output>;
    fn transform(&self, s: &S) -> Self::Output {
        MultiPolygon(self.0.iter().map(|p| p.transform(s)).collect())
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Affine outputs of integer inputs are exact."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/transform.cpp`.

    use super::transform;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};
    use geometry_strategy::Affine2;
    use geometry_trait::{Linestring as _, Point as _};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn point_identity_is_unchanged() {
        let s = Affine2::<f64>::identity();
        let q = transform(&Pt::new(3.0, 4.0), &s);
        assert_eq!((q.get::<0>(), q.get::<1>()), (3.0, 4.0));
    }

    #[test]
    fn linestring_translated() {
        let ls: Linestring<Pt> = linestring![(0.0, 0.0), (1.0, 1.0)];
        let s = Affine2::translation(10.0, 20.0);
        let out = transform(&ls, &s);
        let pts: Vec<(f64, f64)> = out.points().map(|p| (p.get::<0>(), p.get::<1>())).collect();
        assert_eq!(pts, vec![(10.0, 20.0), (11.0, 21.0)]);
    }
}
