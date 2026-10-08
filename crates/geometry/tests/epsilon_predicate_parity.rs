//! Public-facade parity tests for points a rounding error off a vertex or
//! an edge.
//!
//! Boost decides whether a point is on a line with `side_by_triangle`,
//! which calls a determinant within an epsilon of zero zero, and whether
//! two points are one with `math::equals`, a relative epsilon; its convex
//! hull and `is_convex` decide sides exactly, with `side_robust`. Each case
//! is a point interpolated onto an edge, or a last bit off another point,
//! with the answer Boost (`aed7bc3`) gives for it — the same from every
//! predicate.

use boost_geometry::algorithm::{
    convex_hull, covered_by, equals, intersects, is_convex, is_simple, unique, within,
};
use boost_geometry::model::{Linestring, MultiPoint, Point2D, Polygon, Ring, Segment};
use boost_geometry::overlay::relate_matrix;
use boost_geometry::prelude::Cartesian;
use boost_geometry::trait_::{Point as _, Ring as _};

type P = Point2D<f64, Cartesian>;

fn polygon(points: &[P]) -> Polygon<P> {
    Polygon::new(Ring::from_vec(points.to_vec()))
}

/// `(u, v, w)` is a clockwise triangle and `p` is interpolated onto
/// `u → v`, at longitude/latitude scale. Boost puts `p` on the edge: on the
/// segment, on the triangle's boundary, and on the line.
#[test]
fn a_point_interpolated_onto_an_edge_is_on_it() {
    let u = P::new(-73.982_100_731_919_99, -73.973_981_824_580_3);
    let v = P::new(-73.981_107_578_878_99, -73.971_288_265_565_92);
    let w = P::new(-73.972_422_666_793_25, -73.988_050_913_805_38);
    let p = P::new(-73.981_885_230_679_44, -73.973_397_357_448_75);
    let triangle = polygon(&[u, v, w, u]);
    let line = Linestring::from_vec(vec![u, v]);

    assert!(intersects(&Segment::new(u, v), &p));
    assert!(!within(&p, &triangle));
    assert!(covered_by(&p, &triangle));
    let on_boundary = Ok(true);
    assert_eq!(
        relate_matrix(&p, &triangle).unwrap().matches("F0FFFF212"),
        on_boundary
    );
    let points = MultiPoint::from_vec(vec![p]);
    assert_eq!(
        relate_matrix(&points, &triangle)
            .unwrap()
            .matches("F0FFFF212"),
        on_boundary
    );
    assert_eq!(
        relate_matrix(&p, &line).unwrap().matches("0FFFFF102"),
        on_boundary
    );
}

/// The same interpolation at a larger scale leaves `p` further off the
/// edge than Boost's epsilon, which grows with the edge but not with its
/// square: inside the triangle, off the segment and the line.
#[test]
fn a_point_further_off_an_edge_than_the_epsilon_is_not_on_it() {
    let u = P::new(35.204_061_897_475_356, 92.661_116_077_251_48);
    let v = P::new(-95.563_506_201_838_02, -87.884_639_270_840_54);
    let w = P::new(-49.775_544_363_305_954, -8.737_574_072_724_158);
    let p = P::new(-6.644_879_526_836_128, 34.881_891_076_813_11);
    let triangle = polygon(&[u, v, w, u]);

    assert!(!intersects(&Segment::new(u, v), &p));
    assert!(within(&p, &triangle));
    assert_eq!(
        relate_matrix(&p, &triangle).unwrap().matches("0FFFFF212"),
        Ok(true)
    );
    assert_eq!(
        relate_matrix(&p, &Linestring::from_vec(vec![u, v]))
            .unwrap()
            .matches("FF0FFF102"),
        Ok(true)
    );
}

/// A point on an edge to `intersects` can still be a rounding error
/// outside it, and Boost's hull and `is_convex` see that.
#[test]
fn hull_and_convexity_decide_sides_exactly() {
    let u = P::new(-0.000_524_070_745_816_217_3, 8.845_845_059_190_366e-5);
    let v = P::new(-0.000_260_089_666_903_841_53, 0.000_207_840_077_192_388_92);
    let w = P::new(0.000_251_440_608_216_108_03, -0.000_868_942_281_520_373_8);
    let p = P::new(-0.000_302_994_753_968_636_77, 0.000_188_436_871_840_194_38);
    assert!(intersects(&Segment::new(u, v), &p));
    assert!(!is_convex(&polygon(&[u, p, v, w, u])));

    let u = P::new(0.991_289_671_020_925_6, -0.059_472_984_955_104_11);
    let v = P::new(-0.481_291_971_343_984_7, -0.531_338_077_906_607_3);
    let w = P::new(0.672_922_902_548_777_5, -0.047_293_582_601_330_1);
    let p = P::new(0.769_494_689_965_235, -0.130_543_617_876_987_26);
    assert!(intersects(&Segment::new(u, v), &p));
    let hull: Vec<(f64, f64)> = convex_hull(&MultiPoint::from_vec(vec![u, p, v, w]))
        .points()
        .map(|q| (q.get::<0>(), q.get::<1>()))
        .collect();
    assert_eq!(
        hull,
        [
            (-0.481_291_971_343_984_7, -0.531_338_077_906_607_3),
            (0.672_922_902_548_777_5, -0.047_293_582_601_330_1),
            (0.991_289_671_020_925_6, -0.059_472_984_955_104_11),
            (0.769_494_689_965_235, -0.130_543_617_876_987_26),
            (-0.481_291_971_343_984_7, -0.531_338_077_906_607_3),
        ]
    );
}

/// Two points are one where every coordinate is equal within an epsilon of
/// the larger magnitude, or of `1`.
#[test]
fn points_a_last_bit_apart_are_one_point() {
    let p = P::new(0.1, 0.2);
    let next = P::new(0.100_000_000_000_000_02, 0.2);
    assert!(equals(&p, &next));
    assert!(intersects(&p, &next));
    assert_eq!(
        relate_matrix(&p, &next).unwrap().matches("0FFFFFFF2"),
        Ok(true)
    );

    let (one, apart) = (P::new(1.0, 1.0), P::new(1.000_000_000_000_000_4, 1.0));
    assert!(!equals(&one, &apart));
    assert!(!intersects(&one, &apart));
    assert_eq!(
        relate_matrix(&one, &apart).unwrap().matches("FF0FFF0F2"),
        Ok(true)
    );

    let mut line = Linestring::from_vec(vec![P::new(0.0, 0.0), p, next, P::new(1.0, 1.0)]);
    assert!(!is_simple(&line));
    unique(&mut line);
    assert_eq!(line.0.len(), 3);

    let square = polygon(&[
        P::new(0.0, 0.0),
        P::new(0.0, 1.0),
        P::new(1.0, 1.0),
        P::new(1.0, 0.0),
        P::new(0.0, 0.0),
    ]);
    let nudged = polygon(&[
        P::new(0.0, 0.0),
        P::new(0.0, 1.0),
        P::new(1.0, 1.0),
        P::new(1.0, f64::from_bits(1)),
        P::new(0.0, 0.0),
    ]);
    assert!(equals(&square, &nudged));
}

/// A `float` coordinate's side is decided in `double`, as Boost promotes
/// it: the cross product here is `-2`, which `f32` products round away.
#[test]
fn a_float_side_is_decided_in_double() {
    type F = Point2D<f32, Cartesian>;
    let segment = Segment::new(F::new(1.0, 1.0), F::new(16_777_216.0, 16_777_215.0));
    assert!(!intersects(&segment, &F::new(16_777_214.0, 16_777_213.0)));
}
