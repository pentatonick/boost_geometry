//! Public-API parity tests for the remaining standalone algorithms.
//!
//! Reference cases come from Boost.Geometry's
//! `test/algorithms/{envelope_expand,intersects,line_interpolate,perimeter}`
//! suites and the closure rules in
//! `include/boost/geometry/algorithms/correct_closure.hpp`.

#![allow(
    clippy::float_cmp,
    reason = "Cartesian integer-coordinate cases have exact binary results; the spherical case uses a tolerance"
)]

use boost_geometry::adapt::{Adapt, WithCs};
use boost_geometry::model::{
    Box as ModelBox, Linestring, Point as ModelPoint, Point2D, Point3D, Polygon, Ring, Segment,
};
use boost_geometry::prelude::{
    Cartesian, Degree, Spherical, comparable_distance_with, correct_closure, expand, expand_with,
    intersects, line_interpolate, perimeter, perimeter_with, ring_perimeter_with,
};
use boost_geometry::strategy::{CartesianPerimeter, EnvelopePoint, Pythagoras, SphericalPerimeter};
use boost_geometry::trait_::{IndexedAccess as _, Point as _, PointMut as _, Ring as _};

type P2 = Point2D<f64, Cartesian>;
type P3 = Point3D<f64, Cartesian>;

/// `test/algorithms/correct_closure.cpp:51-84` — fix closure without changing
/// winding.
#[test]
fn correct_closure_only_appends_the_first_vertex() {
    let mut ring: Ring<P2> = Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(1.0, 0.0),
        P2::new(1.0, 1.0),
        P2::new(0.0, 1.0),
    ]);

    correct_closure(&mut ring);

    let points: Vec<_> = ring.points().copied().collect();
    assert_eq!(points.len(), 5);
    assert_eq!(points[0].get::<0>(), points[4].get::<0>());
    assert_eq!(points[0].get::<1>(), points[4].get::<1>());
    assert_eq!(points[1].get::<0>(), 1.0);
    assert_eq!(points[1].get::<1>(), 0.0);
}

/// `test/algorithms/envelope_expand/expand.cpp:38-54` — cumulative 3-D point
/// expansion.
#[test]
fn expand_box_with_points_in_all_three_dimensions() {
    let first = P3::new(1.0, 2.0, 5.0);
    let mut bounds = ModelBox::from_corners(first, first);

    expand(&mut bounds, &P3::new(3.0, 4.0, 6.0));
    expand(&mut bounds, &P3::new(10.0, 10.0, 4.0));
    expand(&mut bounds, &P3::new(0.0, 2.0, 7.0));

    assert_eq!(bounds.get_indexed::<0, 0>(), 0.0);
    assert_eq!(bounds.get_indexed::<0, 1>(), 2.0);
    assert_eq!(bounds.get_indexed::<0, 2>(), 4.0);
    assert_eq!(bounds.get_indexed::<1, 0>(), 10.0);
    assert_eq!(bounds.get_indexed::<1, 1>(), 10.0);
    assert_eq!(bounds.get_indexed::<1, 2>(), 7.0);
}

/// `test/algorithms/envelope_expand/expand.cpp:91-97` — an inverse-corner box
/// and a segment both expand the existing bounds coordinate-wise.
#[test]
fn expand_box_with_box_and_segment_envelopes() {
    let mut bounds = ModelBox::from_corners(P2::new(1.0, 1.0), P2::new(2.0, 2.0));
    let inverse = ModelBox::from_corners(P2::new(3.0, 4.0), P2::new(0.0, 1.0));
    let segment = Segment::new(P2::new(5.0, 6.0), P2::new(7.0, 8.0));

    expand(&mut bounds, &inverse);
    expand(&mut bounds, &segment);

    assert_eq!(bounds.get_indexed::<0, 0>(), 0.0);
    assert_eq!(bounds.get_indexed::<0, 1>(), 1.0);
    assert_eq!(bounds.get_indexed::<1, 0>(), 7.0);
    assert_eq!(bounds.get_indexed::<1, 1>(), 8.0);
}

/// `test/algorithms/comparable_distance.cpp:34-50` and
/// `test/algorithms/envelope_expand/expand.cpp:38-54` — explicit-strategy
/// companions use the same public strategy contracts as their defaults.
#[test]
fn explicit_comparable_distance_and_expand_match_defaults() {
    let a = P2::new(0.0, 0.0);
    let b = P2::new(3.0, 4.0);
    assert_eq!(comparable_distance_with(&a, &b, Pythagoras), 25.0);

    let mut bounds = ModelBox::from_corners(P2::new(0.0, 0.0), P2::new(1.0, 1.0));
    expand_with(&mut bounds, &P2::new(-2.0, 3.0), EnvelopePoint);
    assert_eq!(bounds.get_indexed::<0, 0>(), -2.0);
    assert_eq!(bounds.get_indexed::<1, 1>(), 3.0);
}

/// `test/algorithms/perimeter/perimeter.cpp:16-39` — the default and explicit
/// strategy entries agree against a self-contained rectangle oracle.
#[test]
fn perimeter_has_explicit_polygon_and_ring_companions() {
    let ring: Ring<P2> = Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 3.0),
        P2::new(4.0, 3.0),
        P2::new(4.0, 0.0),
        P2::new(0.0, 0.0),
    ]);
    let polygon = Polygon::new(ring.clone());

    assert_eq!(perimeter(&polygon), 14.0);
    assert_eq!(perimeter_with(&polygon, CartesianPerimeter), 14.0);
    assert_eq!(ring_perimeter_with(&ring, CartesianPerimeter), 14.0);
}

/// `test/algorithms/perimeter/perimeter_sph.cpp:17-60` — the strategy-less
/// perimeter follows the coordinate-system family rather than silently using
/// Cartesian distance on angular coordinates.
#[test]
fn spherical_perimeter_uses_the_spherical_default() {
    type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;
    let point = |lon, lat| WithCs::new(Adapt([lon, lat]));
    let ring: Ring<Sp> = Ring::from_vec(vec![
        point(0.0, 0.0),
        point(0.0, 1.0),
        point(1.0, 1.0),
        point(1.0, 0.0),
        point(0.0, 0.0),
    ]);
    let polygon = Polygon::new(ring);

    let default = perimeter(&polygon);
    let explicit = perimeter_with(&polygon, SphericalPerimeter::default());
    assert!((default - explicit).abs() < 1e-9);
    assert!(default > 400_000.0);
}

fn square_ring(x: f64, y: f64, size: f64) -> Ring<P2> {
    Ring::from_vec(vec![
        P2::new(x, y),
        P2::new(x + size, y),
        P2::new(x + size, y + size),
        P2::new(x, y + size),
        P2::new(x, y),
    ])
}

fn assert_point2_close(actual: P2, expected: P2) {
    assert!((actual.get::<0>() - expected.get::<0>()).abs() < 1e-12);
    assert!((actual.get::<1>() - expected.get::<1>()).abs() < 1e-12);
}

/// `test/algorithms/intersects/intersects.cpp:23-30` — polygons wholly inside
/// a hole are disjoint, while crossing either polygon's hole boundary counts
/// as an intersection. In each positive case, the first tested vertices are
/// outside the other polygon, so the result comes from the intended ring loop
/// rather than the vertex-containment shortcut.
#[test]
fn intersects_polygon_polygon_checks_each_hole_owner() {
    let polygon_with_hole = Polygon::with_inners(
        square_ring(0.0, 0.0, 10.0),
        vec![square_ring(4.0, 4.0, 2.0)],
    );
    let inside_hole = Polygon::new(square_ring(4.5, 4.5, 1.0));
    assert!(!intersects(&polygon_with_hole, &inside_hole));

    let crosses_first_polygons_hole = Polygon::new(square_ring(4.5, 4.5, 4.0));
    assert!(intersects(&polygon_with_hole, &crosses_first_polygons_hole));

    let crosses_second_polygons_hole = Polygon::new(square_ring(4.5, 4.5, 2.0));
    assert!(intersects(
        &crosses_second_polygons_hole,
        &polygon_with_hole
    ));
}

/// `test/algorithms/intersects/intersects.cpp:60-80` — lines with no covered
/// vertex can still intersect by crossing the exterior or a hole boundary.
/// The concave-hole case starts and ends inside the hole, but passes through
/// polygon material between its two arms.
#[test]
fn intersects_linestring_polygon_checks_both_rings_and_holes() {
    let concave_hole = Ring::from_vec(vec![
        P2::new(3.0, 3.0),
        P2::new(7.0, 3.0),
        P2::new(7.0, 7.0),
        P2::new(6.0, 7.0),
        P2::new(6.0, 4.0),
        P2::new(4.0, 4.0),
        P2::new(4.0, 7.0),
        P2::new(3.0, 7.0),
        P2::new(3.0, 3.0),
    ]);
    let polygon = Polygon::with_inners(square_ring(0.0, 0.0, 10.0), vec![concave_hole]);

    let ls_outside: Linestring<P2> =
        Linestring::from_vec(vec![P2::new(-5.0, -5.0), P2::new(-1.0, -1.0)]);
    assert!(!intersects(&ls_outside, &polygon));

    let ls_crosses_exterior_boundary: Linestring<P2> =
        Linestring::from_vec(vec![P2::new(-1.0, 8.0), P2::new(11.0, 8.0)]);
    assert!(intersects(&ls_crosses_exterior_boundary, &polygon));

    let ls_crosses_hole_boundary: Linestring<P2> =
        Linestring::from_vec(vec![P2::new(3.5, 6.0), P2::new(6.5, 6.0)]);
    assert!(intersects(&ls_crosses_hole_boundary, &polygon));
}

/// `test/algorithms/line_interpolate.cpp:182-203` — fractional arc-length
/// interpolation walks across segment boundaries instead of treating `t` as
/// a per-segment fraction.
#[test]
fn line_interpolate_walks_to_an_interior_segment() {
    let ls: Linestring<P2> = Linestring::from_vec(vec![
        P2::new(1.0, 1.0),
        P2::new(2.0, 1.0),
        P2::new(2.0, 2.0),
        P2::new(1.0, 2.0),
        P2::new(1.0, 3.0),
    ]);

    assert_point2_close(line_interpolate(&ls, 0.6), P2::new(1.6, 2.0));
}

/// `test/algorithms/line_interpolate.cpp:148-180` — the public Rust API
/// clamps fractions and gives stable results for degenerate linestrings.
#[test]
fn line_interpolate_handles_clamped_and_degenerate_inputs() {
    let ls: Linestring<P2> = Linestring::from_vec(vec![P2::new(1.0, 1.0), P2::new(2.0, 2.0)]);
    assert_eq!(line_interpolate(&ls, -1.0), P2::new(1.0, 1.0));
    assert_eq!(line_interpolate(&ls, 2.0), P2::new(2.0, 2.0));

    let single = Linestring::from_vec(vec![P2::new(3.0, 4.0)]);
    assert_eq!(line_interpolate(&single, 0.5), P2::new(3.0, 4.0));

    let repeated = Linestring::from_vec(vec![P2::new(1.0, 1.0), P2::new(1.0, 1.0)]);
    assert_eq!(line_interpolate(&repeated, 0.5), P2::new(1.0, 1.0));

    let empty = Linestring::<P2>::default();
    assert_eq!(line_interpolate(&empty, 0.5), P2::default());
}

/// The public point model is const-generic, so interpolation must blend every
/// ordinate rather than silently truncating points above two dimensions.
#[test]
fn line_interpolate_blends_three_and_four_dimensions() {
    type P4 = ModelPoint<f64, 4, Cartesian>;

    let three_d = Linestring::from_vec(vec![P3::new(0.0, 0.0, 0.0), P3::new(10.0, 2.0, 4.0)]);
    assert_eq!(line_interpolate(&three_d, 0.5), P3::new(5.0, 1.0, 2.0));

    let mut start = P4::default();
    start.set::<3>(8.0);
    let mut end = P4::default();
    end.set::<0>(10.0);
    end.set::<1>(2.0);
    end.set::<2>(4.0);

    let four_d = Linestring::from_vec(vec![start, end]);
    let midpoint = line_interpolate(&four_d, 0.5);
    assert_eq!(midpoint.get::<0>(), 5.0);
    assert_eq!(midpoint.get::<1>(), 1.0);
    assert_eq!(midpoint.get::<2>(), 2.0);
    assert_eq!(midpoint.get::<3>(), 4.0);
}
