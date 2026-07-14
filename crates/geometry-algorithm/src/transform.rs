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

    use geometry_model::{MultiLinestring, MultiPoint, MultiPolygon, Polygon, Ring, polygon};
    use geometry_trait::{
        MultiLinestring as _, MultiPoint as _, MultiPolygon as _, Polygon as _, Ring as _,
    };

    /// A `Ring` transforms every vertex through the strategy, preserving
    /// order.
    #[test]
    fn ring_scaled() {
        let r: Ring<Pt> = Ring::from_vec(vec![Pt::new(1.0, 1.0), Pt::new(2.0, 3.0)]);
        let out = transform(&r, &Affine2::scale(2.0, 10.0));
        let pts: Vec<(f64, f64)> = out.points().map(|p| (p.get::<0>(), p.get::<1>())).collect();
        assert_eq!(pts, vec![(2.0, 10.0), (4.0, 30.0)]);
    }

    /// A `Polygon` transforms its exterior *and* every interior ring.
    #[test]
    fn polygon_translated_exterior_and_holes() {
        let pg: Polygon<Pt> = polygon![
            [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)]
        ];
        let out = transform(&pg, &Affine2::translation(100.0, 0.0));
        // Exterior first vertex shifted by +100 in x.
        let ext0 = out.exterior().points().next().unwrap();
        assert_eq!((ext0.get::<0>(), ext0.get::<1>()), (100.0, 0.0));
        // The hole is present and also shifted.
        let hole0 = out.interiors().next().unwrap().points().next().unwrap();
        assert_eq!((hole0.get::<0>(), hole0.get::<1>()), (101.0, 1.0));
    }

    /// A `MultiPoint` transforms each member point.
    #[test]
    fn multipoint_scaled() {
        let mp = MultiPoint(vec![Pt::new(1.0, 1.0), Pt::new(2.0, 2.0)]);
        let out = transform(&mp, &Affine2::scale(3.0, 3.0));
        let pts: Vec<(f64, f64)> = out.points().map(|p| (p.get::<0>(), p.get::<1>())).collect();
        assert_eq!(pts, vec![(3.0, 3.0), (6.0, 6.0)]);
    }

    /// A `MultiLinestring` transforms each member line string.
    #[test]
    fn multilinestring_translated() {
        let mls: MultiLinestring<Linestring<Pt>> = MultiLinestring(vec![
            linestring![(0.0, 0.0), (1.0, 0.0)],
            linestring![(0.0, 1.0), (1.0, 1.0)],
        ]);
        let out = transform(&mls, &Affine2::translation(0.0, 5.0));
        let first = out.linestrings().next().unwrap().points().next().unwrap();
        assert_eq!((first.get::<0>(), first.get::<1>()), (0.0, 5.0));
        assert_eq!(out.linestrings().count(), 2);
    }

    /// A `MultiPolygon` transforms each member polygon.
    #[test]
    fn multipolygon_scaled() {
        let member: Polygon<Pt> = polygon![[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)]];
        let mpg: MultiPolygon<Polygon<Pt>> = MultiPolygon(vec![member.clone(), member]);
        let out = transform(&mpg, &Affine2::scale(10.0, 10.0));
        assert_eq!(out.polygons().count(), 2);
        let v = out
            .polygons()
            .next()
            .unwrap()
            .exterior()
            .points()
            .next()
            .unwrap();
        assert_eq!((v.get::<0>(), v.get::<1>()), (10.0, 10.0));
    }

    // ---- Affine3: the 4×4 homogeneous 3D transforms ------------------

    use geometry_model::Point3D;
    use geometry_strategy::Affine3;

    type P3 = Point3D<f64, Cartesian>;

    /// The 3D identity leaves every ordinate untouched.
    #[test]
    fn affine3_identity_is_a_noop() {
        let out = transform(&P3::new(2.0, 3.0, 4.0), &Affine3::identity());
        assert_eq!(out.get::<0>(), 2.0);
        assert_eq!(out.get::<1>(), 3.0);
        assert_eq!(out.get::<2>(), 4.0);
    }

    /// 3D translation adds `(tx, ty, tz)` to the point.
    #[test]
    fn affine3_translation_shifts_all_three_axes() {
        let t = Affine3::translation(1.0, 2.0, 3.0);
        let out = transform(&P3::new(10.0, 20.0, 30.0), &t);
        assert_eq!(out.get::<0>(), 11.0);
        assert_eq!(out.get::<1>(), 22.0);
        assert_eq!(out.get::<2>(), 33.0);
    }

    /// 3D non-uniform scale multiplies each axis independently.
    #[test]
    fn affine3_scale_multiplies_each_axis() {
        let s = Affine3::scale(2.0, 3.0, 4.0);
        let out = transform(&P3::new(1.0, 1.0, 1.0), &s);
        assert_eq!(out.get::<0>(), 2.0);
        assert_eq!(out.get::<1>(), 3.0);
        assert_eq!(out.get::<2>(), 4.0);
    }

    /// A hand-built 4×4 matrix combining scale and translation applies
    /// the full `M·[x y z 1]ᵀ` computation across all rows.
    #[test]
    fn affine3_full_matrix_applies_scale_and_translation() {
        // Scale by (2,2,2) then translate by (1,2,3): the row-major
        // matrix has diagonal 2s and the translation in the last column.
        let mut a = Affine3::scale(2.0, 2.0, 2.0);
        a.m[3] = 1.0;
        a.m[7] = 2.0;
        a.m[11] = 3.0;
        let out = transform(&P3::new(1.0, 1.0, 1.0), &a);
        assert_eq!(out.get::<0>(), 3.0); // 2*1 + 1
        assert_eq!(out.get::<1>(), 4.0); // 2*1 + 2
        assert_eq!(out.get::<2>(), 5.0); // 2*1 + 3
    }
}
