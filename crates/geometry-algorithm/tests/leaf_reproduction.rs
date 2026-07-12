//! M-LA — per-algorithm reproduction against Boost reference values.
//!
//! One `#[test]` per `phase_02` leaf algorithm, each mirroring a fixture
//! in the matching `boost/geometry/test/algorithms/<name>.cpp`. If any
//! of these regresses, the shipped algorithm surface has drifted from
//! Boost.
#![allow(
    clippy::float_cmp,
    reason = "Reference values are exact integer-valued literals or compared with an explicit epsilon."
)]

use geometry_algorithm::{
    area, azimuth, centroid, convex_hull, correct, discrete_frechet_distance,
    discrete_hausdorff_distance, is_convex, is_empty, is_simple, length, line_interpolate,
    num_geometries, num_interior_rings, num_points, num_segments, remove_spikes, reverse, simplify,
    unique,
};
use geometry_cs::Cartesian;
use geometry_model::{Linestring, MultiPoint, Point2D, Polygon, Ring, linestring, polygon};
use geometry_trait::{Linestring as _, Point as _, Ring as _};

type Pt = Point2D<f64, Cartesian>;

fn close(got: f64, exp: f64) -> bool {
    (got - exp).abs() <= exp.abs() * 1e-9 + 1e-12
}

// === LA1: counters ===

#[test]
fn num_points_pentagon_with_hole_is_nine() {
    // num_points.cpp:89 — 5 outer + 4 inner stored points.
    let pg: Polygon<Pt> = polygon![
        [(0., 0.), (5., 0.), (5., 5.), (0., 5.), (0., 0.)],
        [(1., 1.), (2., 1.), (2., 2.), (1., 1.)]
    ];
    assert_eq!(num_points(&pg), 9);
}

#[test]
fn num_geometries_single_polygon_is_one() {
    let pg: Polygon<Pt> = polygon![[(0., 0.), (1., 0.), (1., 1.), (0., 0.)]];
    assert_eq!(num_geometries(&pg), 1);
}

#[test]
fn num_segments_closed_square_ring_is_four() {
    // num_segments.cpp:177 — a closed 5-point square ring has 4 edges.
    let pg: Polygon<Pt> = polygon![[(0., 0.), (1., 0.), (1., 1.), (0., 1.), (0., 0.)]];
    assert_eq!(num_segments(&pg), 4);
}

#[test]
fn num_interior_rings_two_hole_polygon_is_two() {
    let pg: Polygon<Pt> = polygon![
        [(0., 0.), (10., 0.), (10., 10.), (0., 10.), (0., 0.)],
        [(1., 1.), (2., 1.), (2., 2.), (1., 1.)],
        [(5., 5.), (6., 5.), (6., 6.), (5., 5.)]
    ];
    assert_eq!(num_interior_rings(&pg), 2);
}

#[test]
fn is_empty_default_linestring() {
    let ls: Linestring<Pt> = Linestring::new();
    assert!(is_empty(&ls));
}

// === LA2: transforms ===

#[test]
fn reverse_three_point_linestring() {
    // reverse.cpp — (0,0)→(1,1)→(2,2) becomes (2,2)→(1,1)→(0,0).
    let mut ls: Linestring<Pt> = linestring![(0., 0.), (1., 1.), (2., 2.)];
    reverse(&mut ls);
    let xs: Vec<f64> = ls.points().map(geometry_trait::Point::get::<0>).collect();
    assert_eq!(xs, vec![2., 1., 0.]);
}

#[test]
fn unique_collapses_consecutive_duplicates() {
    let mut ls: Linestring<Pt> = linestring![(0., 0.), (0., 0.), (1., 1.), (1., 1.)];
    unique(&mut ls);
    assert_eq!(ls.points().count(), 2);
}

#[test]
fn correct_makes_ccw_ring_positive() {
    // A CCW-stored square ring, declared clockwise, has negative signed
    // area until `correct` reverses it → +4.
    let mut r: Ring<Pt> = Ring::from_vec(vec![
        Pt::new(0., 0.),
        Pt::new(2., 0.),
        Pt::new(2., 2.),
        Pt::new(0., 2.),
        Pt::new(0., 0.),
    ]);
    correct(&mut r);
    assert!(close(geometry_algorithm::ring_area(&r), 4.0));
}

#[test]
fn remove_spikes_drops_overshoot() {
    // (0,0)→(1,0)→(3,0)→(2,0): the tip (3,0) is a reversed collinear
    // overshoot and is removed.
    let mut ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (3., 0.), (2., 0.)];
    remove_spikes(&mut ls);
    let xs: Vec<f64> = ls.points().map(geometry_trait::Point::get::<0>).collect();
    assert_eq!(xs, vec![0., 1., 2.]);
}

// === LA3/LA4: length, area, azimuth ===

#[test]
fn length_3_4_5_linestring_is_seven() {
    // LINESTRING(0 0, 3 4, 4 3) — 5 + √2.
    let ls: Linestring<Pt> = linestring![(0., 0.), (3., 4.), (4., 3.)];
    assert!(close(length(&ls), 5.0 + 2.0_f64.sqrt()));
}

#[test]
fn area_quickstart_polygon_is_3_015() {
    // quick_start.cpp — Area: 3.015.
    let pg: Polygon<Pt> = polygon![[(2., 1.3), (4.1, 3.), (5.3, 2.6), (2.9, 0.7), (2., 1.3)]];
    assert!((area(&pg) - 3.015).abs() < 1e-3);
}

#[test]
fn azimuth_due_north_is_zero() {
    // Boost measures the Cartesian bearing clockwise from +y ("North"),
    // so due North is 0 (azimuth.hpp:76). Due East is +π/2.
    use core::f64::consts::FRAC_PI_2;
    assert!(close(
        azimuth(&Pt::new(0., 0.), &Pt::new(0., 1.)) + 1.0,
        1.0
    ));
    assert!(close(
        azimuth(&Pt::new(0., 0.), &Pt::new(1., 0.)),
        FRAC_PI_2
    ));
}

// === LA6: centroid ===

#[test]
fn centroid_unit_square_is_half_half() {
    let pg: Polygon<Pt> = polygon![[(0., 0.), (1., 0.), (1., 1.), (0., 1.), (0., 0.)]];
    let c = centroid(&pg);
    assert!(close(c.get::<0>(), 0.5) && close(c.get::<1>(), 0.5));
}

// === LA7: simplify, convex_hull, is_convex ===

#[test]
fn simplify_collapses_collinear_line() {
    // A straight 3-point line collapses to its endpoints.
    let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (2., 0.)];
    let out = simplify(&ls, 0.5);
    assert_eq!(out.points().count(), 2);
}

#[test]
fn convex_hull_excludes_interior_point() {
    // Square plus a centre point → hull is the 4 corners (5 stored,
    // closing vertex repeated).
    let mp = MultiPoint(vec![
        Pt::new(0., 0.),
        Pt::new(4., 0.),
        Pt::new(4., 4.),
        Pt::new(0., 4.),
        Pt::new(2., 2.),
    ]);
    let hull = convex_hull(&mp);
    assert_eq!(hull.points().count(), 5);
}

#[test]
fn is_convex_square_true_reflex_false() {
    let square: Polygon<Pt> = polygon![[(0., 0.), (4., 0.), (4., 4.), (0., 4.), (0., 0.)]];
    assert!(is_convex(&square));
    let reflex: Polygon<Pt> =
        polygon![[(0., 0.), (4., 0.), (2., 1.), (4., 4.), (0., 4.), (0., 0.)]];
    assert!(!is_convex(&reflex));
}

// === LA9: densify, line_interpolate ===

#[test]
fn line_interpolate_midpoint() {
    let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
    let mid = line_interpolate(&ls, 0.5);
    assert!(close(mid.get::<0>(), 5.0) && close(mid.get::<1>() + 1.0, 1.0));
}

// === LA10: frechet, hausdorff ===

#[test]
fn frechet_identical_is_zero() {
    let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (1., 1.)];
    assert!(discrete_frechet_distance(&ls, &ls) < 1e-12);
}

#[test]
fn hausdorff_parallel_tracks_one_apart() {
    let a: Linestring<Pt> = linestring![(0., 0.), (5., 0.)];
    let b: Linestring<Pt> = linestring![(0., 1.), (5., 1.)];
    assert!(close(discrete_hausdorff_distance(&a, &b), 1.0));
}

// === LA12: is_simple ===

#[test]
fn is_simple_bowtie_is_not_simple() {
    // Self-crossing "bowtie" linestring.
    let ls: Linestring<Pt> = linestring![(0., 0.), (2., 2.), (2., 0.), (0., 2.)];
    assert!(!is_simple(&ls));
    let simple: Linestring<Pt> = linestring![(0., 0.), (1., 1.), (2., 0.)];
    assert!(is_simple(&simple));
}
