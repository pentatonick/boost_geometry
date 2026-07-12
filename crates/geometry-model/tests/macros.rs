//! Smoke tests for the `point!`, `linestring!`, and `polygon!`
//! declarative literal macros.
//!
//! The polygon literal mirrors the quickstart snippet — same
//! outer-ring coordinates, same
//! point type. The linestring and point literals exercise the simpler
//! arms.

use geometry_cs::Cartesian;
use geometry_model::{Linestring, Point2D, Point3D, Polygon, linestring, point, polygon};
use geometry_trait::{Linestring as _, Point as _, Polygon as _, Ring as _};

/// Bit-exact float compare — every value written and read in this file
/// is a literal that round-trips byte-for-byte, so this is the strictest
/// check we can apply while sidestepping `clippy::float_cmp`.
fn bit_eq(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits()
}

#[test]
fn polygon_literal_matches_quickstart() {
    let p: Polygon<Point2D<f64, Cartesian>> =
        polygon![[(0., 0.), (4., 0.), (4., 3.), (0., 3.), (0., 0.)]];
    assert_eq!(p.exterior().points().count(), 5);
    assert_eq!(p.interiors().count(), 0);

    // Round-trip the first and last x coordinates bit-exactly.
    let xs: Vec<u64> = p
        .exterior()
        .points()
        .map(|pt| pt.get::<0>().to_bits())
        .collect();
    assert_eq!(xs.first().copied(), Some(0.0_f64.to_bits()));
    assert_eq!(xs.last().copied(), Some(0.0_f64.to_bits()));
    assert_eq!(xs[2], 4.0_f64.to_bits());
}

#[test]
fn polygon_literal_with_holes() {
    let p: Polygon<Point2D<f64, Cartesian>> = polygon![
        [(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)],
        [(1., 1.), (2., 1.), (2., 2.), (1., 2.), (1., 1.)],
        [(5., 5.), (6., 5.), (6., 6.), (5., 6.), (5., 5.)],
    ];
    assert_eq!(p.exterior().points().count(), 5);
    assert_eq!(p.interiors().count(), 2);
    for hole in p.interiors() {
        assert_eq!(hole.points().count(), 5);
    }
}

#[test]
fn linestring_literal_three_points() {
    let ls: Linestring<Point2D<f64, Cartesian>> = linestring![(0., 0.), (1., 1.), (2., 0.)];
    assert_eq!(ls.points().count(), 3);
    let ys: Vec<u64> = ls.points().map(|p| p.get::<1>().to_bits()).collect();
    assert_eq!(
        ys,
        vec![0.0_f64.to_bits(), 1.0_f64.to_bits(), 0.0_f64.to_bits()]
    );
}

#[test]
fn point_literal_2d() {
    let p: Point2D<f64, Cartesian> = point!((1.5, 2.5));
    assert!(bit_eq(p.get::<0>(), 1.5));
    assert!(bit_eq(p.get::<1>(), 2.5));
}

#[test]
fn point_literal_3d() {
    let p: Point3D<f64, Cartesian> = point!((1.5, 2.5, 3.5));
    assert!(bit_eq(p.get::<0>(), 1.5));
    assert!(bit_eq(p.get::<1>(), 2.5));
    assert!(bit_eq(p.get::<2>(), 3.5));
}

#[test]
fn polygon_macro_with_one_hole() {
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};
    use geometry_trait::{Polygon as _, Ring as _};

    let p: Polygon<Point2D<f64, Cartesian>> = polygon![
        [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)],
        [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)],
    ];

    assert_eq!(p.exterior().points().count(), 5);
    assert_eq!(p.interiors().count(), 1);
    assert_eq!(p.interiors().next().unwrap().points().count(), 5);
}

#[test]
fn polygon_macro_with_two_holes() {
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};
    use geometry_trait::Polygon as _;

    let p: Polygon<Point2D<f64, Cartesian>> = polygon![
        [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ],
        [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)],
        [(5.0, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 6.0), (5.0, 5.0)],
    ];

    assert_eq!(p.interiors().count(), 2);
}
