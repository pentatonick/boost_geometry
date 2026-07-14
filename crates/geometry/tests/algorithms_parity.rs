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
    Box as ModelBox, DynGeometry, DynGeometryCollection, Linestring, MultiLinestring, MultiPoint,
    MultiPolygon, Point as ModelPoint, Point2D, Point3D, Polygon, Ring, Segment,
};
use boost_geometry::prelude::{
    Cartesian, Degree, Spherical, area, assign_values, azimuth_with, centroid, centroid_with,
    closest_points, comparable_distance_with, correct, correct_closure, densify, distance_with,
    equals, expand, expand_with, for_each_segment, intersects, intersects_reversed, is_simple,
    line_interpolate, perimeter, perimeter_with, remove_spikes, ring_area, ring_perimeter_with,
    unique, within,
};
use boost_geometry::strategy::{
    CartesianAzimuth, CartesianBoxCentroid, CartesianPerimeter, EnvelopePoint, PointToSegment,
    Pythagoras, SphericalPerimeter,
};
use boost_geometry::trait_::{
    IndexedAccess as _, Point as _, PointMut as _, Polygon as _, Ring as _,
};

type P2 = Point2D<f64, Cartesian>;
type P3 = Point3D<f64, Cartesian>;
type P4 = ModelPoint<f64, 4, Cartesian>;
type P5 = ModelPoint<f64, 5, Cartesian>;
type D = DynGeometry<f64, Cartesian>;

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

/// `test/algorithms/correct_closure.cpp:35-100` — polygon, multi-polygon,
/// variant, collection, open-ring, and no-op kinds all use the public closure
/// dispatcher. The 4-D witness verifies coordinate-wise closure above 2-D.
#[test]
fn correct_closure_dispatches_across_public_models() {
    let open_outer = square_ring(0.0, 0.0, 4.0);
    let open_hole = square_ring(1.0, 1.0, 1.0);
    let mut polygon: Polygon<P2> = Polygon::with_inners(
        Ring::<P2>::from_vec(open_outer.0[..4].to_vec()),
        vec![Ring::<P2>::from_vec(open_hole.0[..4].to_vec())],
    );
    correct_closure(&mut polygon);
    assert_eq!(polygon.exterior().points().count(), 5);
    assert_eq!(polygon.interiors().next().unwrap().points().count(), 5);

    let mut multi_polygon: MultiPolygon<Polygon<P2>> = MultiPolygon::from_vec(vec![Polygon::new(
        Ring::<P2>::from_vec(square_ring(10.0, 0.0, 2.0).0[..4].to_vec()),
    )]);
    correct_closure(&mut multi_polygon);
    assert_eq!(multi_polygon.0[0].exterior().points().count(), 5);

    let mut point = P2::new(1.0, 2.0);
    let mut line = Linestring::from_vec(vec![P2::new(0.0, 0.0), P2::new(1.0, 1.0)]);
    let mut segment = Segment::new(P2::new(0.0, 0.0), P2::new(1.0, 1.0));
    let mut bounds = ModelBox::from_corners(P2::new(0.0, 0.0), P2::new(1.0, 1.0));
    let mut multi_point = MultiPoint::from_vec(vec![P2::new(0.0, 0.0)]);
    let mut multi_line = MultiLinestring::from_vec(vec![line.clone()]);
    correct_closure(&mut point);
    correct_closure(&mut line);
    correct_closure(&mut segment);
    correct_closure(&mut bounds);
    correct_closure(&mut multi_point);
    correct_closure(&mut multi_line);
    assert_eq!(point, P2::new(1.0, 2.0));
    assert_eq!(line.0.len(), 2);
    assert_eq!(multi_line.0[0].0.len(), 2);

    let unclosed = || -> Polygon<P2> {
        Polygon::new(Ring::from_vec(vec![
            P2::new(0.0, 0.0),
            P2::new(0.0, 1.0),
            P2::new(1.0, 1.0),
            P2::new(1.0, 0.0),
        ]))
    };
    let mut dynamic = D::GeometryCollection(vec![
        D::Point(P2::new(0.0, 0.0)),
        D::LineString(line.clone()),
        D::MultiPoint(multi_point),
        D::MultiLineString(multi_line),
        D::Polygon(unclosed()),
        D::MultiPolygon(MultiPolygon::from_vec(vec![unclosed()])),
        D::GeometryCollection(vec![D::Polygon(unclosed())]),
    ]);
    correct_closure(&mut dynamic);
    let D::GeometryCollection(items) = dynamic else {
        panic!("collection kind changed during correction")
    };
    let D::Polygon(corrected) = &items[4] else {
        panic!("polygon member kind changed")
    };
    assert_eq!(corrected.exterior().points().count(), 5);

    let mut collection = DynGeometryCollection(vec![D::Polygon(unclosed())]);
    correct_closure(&mut collection);
    let D::Polygon(corrected) = &collection.0[0] else {
        panic!("polygon member kind changed")
    };
    assert_eq!(corrected.exterior().points().count(), 5);

    let mut first = P4::default();
    first.set::<0>(1.0);
    first.set::<1>(2.0);
    first.set::<2>(3.0);
    first.set::<3>(4.0);
    let mut second = first;
    second.set::<0>(5.0);
    let mut third = first;
    third.set::<1>(6.0);
    let mut ring4: Ring<P4> = Ring::from_vec(vec![first, second, third]);
    correct_closure(&mut ring4);
    assert_eq!(ring4.0[0], ring4.0[3]);
}

/// `test/algorithms/correct.cpp` and `correct_closure.cpp:86-100` — complete
/// correction fixes both polygon winding and closure through multi dispatch.
#[test]
fn correct_fixes_polygon_and_multi_polygon_orientation() {
    let outer: Ring<P2> = Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(4.0, 0.0),
        P2::new(4.0, 4.0),
        P2::new(0.0, 4.0),
    ]);
    let hole: Ring<P2> = Ring::from_vec(vec![
        P2::new(1.0, 1.0),
        P2::new(1.0, 2.0),
        P2::new(2.0, 2.0),
        P2::new(2.0, 1.0),
    ]);
    let mut multi: MultiPolygon<Polygon<P2>> =
        MultiPolygon::from_vec(vec![Polygon::with_inners(outer, vec![hole])]);
    correct(&mut multi);
    assert_eq!(multi.0[0].exterior().points().count(), 5);
    assert_eq!(multi.0[0].interiors().next().unwrap().points().count(), 5);
    assert_eq!(area(&multi.0[0]), 15.0);
}

/// `test/algorithms/azimuth.cpp:26-33` and
/// `test/algorithms/centroid.cpp` — explicit public strategies agree with
/// their elementary Cartesian oracles.
#[test]
fn explicit_azimuth_and_centroid_strategies_are_public() {
    let azimuth = azimuth_with(&P2::new(0.0, 0.0), &P2::new(1.0, 0.0), CartesianAzimuth);
    assert!((azimuth - core::f64::consts::FRAC_PI_2).abs() < 1e-12);

    let bounds = ModelBox::from_corners(P2::new(-2.0, 4.0), P2::new(6.0, 10.0));
    assert_eq!(
        centroid_with(&bounds, CartesianBoxCentroid),
        P2::new(2.0, 7.0)
    );
}

/// `test/algorithms/is_simple.cpp:93-139` — duplicate, fold-back,
/// self-crossing, closed, and empty linestring paths plus areal duplicate
/// handling are observable through the public predicate.
#[test]
fn is_simple_covers_reference_edge_cases() {
    assert!(is_simple(&Linestring::<P2>::default()));
    assert!(!is_simple(&Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 0.0),
        P2::new(1.0, 0.0),
    ])));
    assert!(!is_simple(&Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(2.0, 0.0),
        P2::new(1.0, 0.0),
    ])));
    assert!(!is_simple(&Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(2.0, 2.0),
        P2::new(0.0, 2.0),
        P2::new(2.0, 0.0),
    ])));
    assert!(is_simple(&Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(1.0, 0.0),
        P2::new(1.0, 1.0),
        P2::new(0.0, 0.0),
    ])));

    let polygon_with_empty_hole: Polygon<P2> =
        Polygon::with_inners(square_ring(0.0, 0.0, 4.0), vec![Ring::<P2>::default()]);
    assert!(!is_simple(&polygon_with_empty_hole));

    let open_outer: Ring<P2, true, false> = Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 2.0),
        P2::new(2.0, 2.0),
        P2::new(0.0, 0.0),
    ]);
    assert!(!is_simple(&Polygon::<P2, true, false>::new(open_outer)));
}

/// `test/algorithms/remove_spikes.cpp:163-170` and
/// `test/algorithms/for_each{,_multi}.cpp:109-133,45-84` — multi dispatch
/// removes an areal seam spike and visits every exterior/interior segment.
#[test]
fn multi_spike_removal_and_segment_visitation_use_public_dispatch() {
    let spiked: Ring<P2> = Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 4.0),
        P2::new(4.0, 4.0),
        P2::new(4.0, 2.0),
        P2::new(6.0, 2.0),
        P2::new(4.0, 2.0),
        P2::new(4.0, 0.0),
        P2::new(0.0, 0.0),
    ]);
    let mut multi: MultiPolygon<Polygon<P2>> = MultiPolygon::from_vec(vec![Polygon::new(spiked)]);
    remove_spikes(&mut multi);
    assert_eq!(area(&multi.0[0]), 16.0);

    let polygon =
        Polygon::with_inners(square_ring(0.0, 0.0, 4.0), vec![square_ring(1.0, 1.0, 1.0)]);
    let mut polygon_segments = 0;
    for_each_segment(&polygon, |_, _| polygon_segments += 1);
    assert_eq!(polygon_segments, 8);

    let multi_line = MultiLinestring::from_vec(vec![Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(1.0, 1.0),
    ])]);
    let mut line_segments = 0;
    for_each_segment(&multi_line, |_, _| line_segments += 1);
    assert_eq!(line_segments, 1);

    let multi_polygon = MultiPolygon::from_vec(vec![polygon]);
    let mut multi_segments = 0;
    for_each_segment(&multi_polygon, |_, _| multi_segments += 1);
    assert_eq!(multi_segments, 8);
}

/// `test/algorithms/remove_spikes.cpp` runs every ring fixture for both open
/// and closed models. A spike at an open ring's last vertex exists only
/// across the implicit seam and therefore exercises the wrap-around pass.
#[test]
fn remove_spikes_cleans_the_implicit_open_ring_seam() {
    let mut ring: Ring<P2, true, false> = Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 4.0),
        P2::new(4.0, 4.0),
        P2::new(4.0, 0.0),
        P2::new(6.0, 0.0),
    ]);
    remove_spikes(&mut ring);
    assert_eq!(ring.0.len(), 4);
    assert!(!ring.0.contains(&P2::new(6.0, 0.0)));
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

/// `test/strategies/projected_point.cpp` exercises the projected-point
/// strategy through the public distance algorithm. The Rust point model also
/// exposes one through four dimensions, so each supported const-generic walk
/// is verified here; the documented four-dimensional ceiling is observable
/// as a public panic rather than silent ordinate truncation.
#[test]
fn projected_point_distance_supports_every_public_dimension() {
    type P0 = ModelPoint<f64, 0, Cartesian>;
    type P1 = ModelPoint<f64, 1, Cartesian>;
    type P4 = ModelPoint<f64, 4, Cartesian>;
    type P5 = ModelPoint<f64, 5, Cartesian>;

    let one_d = Segment::new(P1::new(0.0), P1::new(4.0));
    assert_eq!(
        distance_with(
            &P1::new(2.0),
            &one_d,
            PointToSegment::<Pythagoras>::default(),
        ),
        0.0
    );

    let three_d = Segment::new(P3::new(0.0, 0.0, 0.0), P3::new(2.0, 0.0, 0.0));
    let three_d_distance = distance_with(
        &P3::new(1.0, 1.0, 1.0),
        &three_d,
        PointToSegment::<Pythagoras>::default(),
    );
    assert!((three_d_distance - 2.0_f64.sqrt()).abs() < 1e-12);

    let mut point4 = P4::default();
    point4.set::<0>(1.0);
    point4.set::<1>(1.0);
    point4.set::<2>(1.0);
    point4.set::<3>(1.0);
    let mut end4 = P4::default();
    end4.set::<0>(2.0);
    let four_d = Segment::new(P4::default(), end4);
    assert!(
        (distance_with(&point4, &four_d, PointToSegment::<Pythagoras>::default(),)
            - 3.0_f64.sqrt())
        .abs()
            < 1e-12
    );
    assert_eq!(
        comparable_distance_with(&point4, &four_d, PointToSegment::<Pythagoras>::default(),),
        3.0
    );

    let unsupported = Segment::new(P5::default(), P5::default());
    let panic = std::panic::catch_unwind(|| {
        distance_with(
            &P5::default(),
            &unsupported,
            PointToSegment::<Pythagoras>::default(),
        )
    });
    assert!(panic.is_err());

    for unsupported in [
        std::panic::catch_unwind(|| distance_with(&P0::default(), &P0::default(), Pythagoras)),
        std::panic::catch_unwind(|| distance_with(&P5::default(), &P5::default(), Pythagoras)),
    ] {
        assert!(unsupported.is_err());
    }
}

/// The public const-generic point model supports four ordinates. Algorithms
/// that recurse over coordinates must reach the last ordinate consistently;
/// exceeding the stable-Rust kernel ceiling must fail explicitly.
#[test]
fn coordinate_wise_algorithms_reach_the_fourth_ordinate() {
    type P1 = ModelPoint<f64, 1, Cartesian>;
    type P4 = ModelPoint<f64, 4, Cartesian>;
    type P5 = ModelPoint<f64, 5, Cartesian>;

    let mut one = P1::default();
    assign_values(&mut one, &[7.0]);
    assert_eq!(one.get::<0>(), 7.0);

    let mut four = P4::default();
    assign_values(&mut four, &[1.0, 2.0, 3.0, 4.0]);
    assert_eq!(four.get::<3>(), 4.0);

    let origin = P4::default();
    let mut differs_only_in_w = origin;
    differs_only_in_w.set::<3>(1.0);
    assert!(!equals(&origin, &differs_only_in_w));

    let mut ring: Ring<P4> = Ring::from_vec(vec![origin, four, differs_only_in_w]);
    correct_closure(&mut ring);
    assert_eq!(ring.0.len(), 4);
    assert_eq!(ring.0[3], origin);

    let mut line = Linestring::from_vec(vec![origin, origin, differs_only_in_w, differs_only_in_w]);
    unique(&mut line);
    assert_eq!(line.0, vec![origin, differs_only_in_w]);

    let mut bounds = ModelBox::from_corners(origin, origin);
    expand(&mut bounds, &four);
    assert_eq!(bounds.get_indexed::<1, 3>(), 4.0);

    let unsupported = std::panic::catch_unwind(|| equals(&P5::default(), &P5::default()));
    assert!(unsupported.is_err());
}

/// Open rings carry an implicit closing edge in Boost's area and centroid
/// algorithms. Empty point collections use the public centroid's zero-value
/// fallback (`test/algorithms/centroid.cpp`).
#[test]
fn open_ring_area_and_centroid_include_the_implicit_edge() {
    let open: Ring<P2, true, false> = Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 2.0),
        P2::new(2.0, 2.0),
        P2::new(2.0, 0.0),
    ]);
    assert_eq!(ring_area(&open), 4.0);
    assert_point2_close(centroid(&open), P2::new(1.0, 1.0));

    let empty = MultiPoint::<P2>::default();
    assert_eq!(centroid(&empty), P2::default());
}

/// `test/algorithms/closest_points` and `densify.cpp` exercise the public
/// algorithms on non-crossing segment pairs and multi-dimensional points.
/// These cases also verify that the third and fourth ordinates are copied or
/// interpolated rather than discarded.
#[test]
fn closest_points_and_densify_cover_non_crossing_and_four_dimensional_paths() {
    let horizontal = Segment::new(P2::new(0.0, 0.0), P2::new(1.0, 0.0));
    let vertical = Segment::new(P2::new(2.0, 1.0), P2::new(2.0, 2.0));
    let (on_horizontal, on_vertical) = closest_points(&horizontal, &vertical);
    assert_eq!(on_horizontal, P2::new(1.0, 0.0));
    assert_eq!(on_vertical, P2::new(2.0, 1.0));

    let first = Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(1.0, 0.0),
        P2::new(10.0, 0.0),
    ]);
    let second = Linestring::from_vec(vec![P2::new(10.0, 1.0), P2::new(11.0, 1.0)]);
    let (on_first, on_second) = closest_points(&first, &second);
    assert_eq!(on_first, P2::new(10.0, 0.0));
    assert_eq!(on_second, P2::new(10.0, 1.0));

    let mut point = P4::default();
    point.set::<0>(1.0);
    point.set::<1>(1.0);
    point.set::<2>(2.0);
    point.set::<3>(3.0);
    let mut end = P4::default();
    end.set::<0>(2.0);
    let segment = Segment::new(P4::default(), end);
    let (input, foot) = closest_points(&point, &segment);
    assert_eq!(input, point);
    assert_eq!(foot.get::<0>(), 1.0);
    assert_eq!(foot.get::<1>(), 0.0);
    assert_eq!(foot.get::<2>(), 0.0);
    assert_eq!(foot.get::<3>(), 0.0);

    let dense = densify(&Linestring::from_vec(vec![P4::default(), point]), 2.0);
    assert_eq!(dense.0.len(), 3);
    assert_eq!(dense.0[1].get::<0>(), 0.5);
    assert_eq!(dense.0[1].get::<1>(), 0.5);
    assert_eq!(dense.0[1].get::<2>(), 1.0);
    assert_eq!(dense.0[1].get::<3>(), 1.5);
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

/// Empty and undersized areal inputs are total predicates, while the explicit
/// reversed entry point exposes Boost's reverse-dispatch contract directly.
/// The crossing rectangles have no contained first vertex, forcing the
/// exterior-edge intersection path.
#[test]
fn intersects_handles_empty_short_and_explicitly_reversed_inputs() {
    let empty_polygon = Polygon::<P2>::default();
    let square = Polygon::new(square_ring(0.0, 0.0, 2.0));
    assert!(!intersects(&empty_polygon, &square));
    assert!(!intersects(&square, &empty_polygon));

    let single_vertex: Polygon<P2> = Polygon::new(Ring::from_vec(vec![P2::new(10.0, 10.0)]));
    assert!(!intersects(&single_vertex, &square));

    let horizontal: Polygon<P2> = Polygon::new(Ring::from_vec(vec![
        P2::new(-2.0, -0.5),
        P2::new(-2.0, 0.5),
        P2::new(2.0, 0.5),
        P2::new(2.0, -0.5),
        P2::new(-2.0, -0.5),
    ]));
    let vertical: Polygon<P2> = Polygon::new(Ring::from_vec(vec![
        P2::new(-0.5, -2.0),
        P2::new(-0.5, 2.0),
        P2::new(0.5, 2.0),
        P2::new(0.5, -2.0),
        P2::new(-0.5, -2.0),
    ]));
    assert!(intersects(&horizontal, &vertical));

    let empty_line = Linestring::<P2>::default();
    assert!(!intersects(&empty_line, &empty_polygon));
    let crossing_line = Linestring::from_vec(vec![P2::new(-1.0, 1.0), P2::new(3.0, 1.0)]);
    assert!(intersects_reversed(&square, &crossing_line));
}

/// Segment endpoint order must not affect intersection, and point equality
/// must inspect every ordinate supported by the public const-generic model.
#[test]
fn intersects_covers_late_endpoint_checks_and_fourth_ordinate_equality() {
    let horizontal = Segment::new(P2::new(0.0, 0.0), P2::new(2.0, 0.0));
    let starts_on_horizontal = Segment::new(P2::new(1.0, 0.0), P2::new(1.0, 1.0));
    let ends_on_horizontal = Segment::new(P2::new(1.0, 1.0), P2::new(1.0, 0.0));
    assert!(intersects(&horizontal, &starts_on_horizontal));
    assert!(intersects(&horizontal, &ends_on_horizontal));

    let origin = P4::default();
    let mut differs_only_in_w = origin;
    differs_only_in_w.set::<3>(1.0);
    assert!(intersects(&origin, &origin));
    assert!(!intersects(&origin, &differs_only_in_w));

    let unsupported = std::panic::catch_unwind(|| intersects(&P5::default(), &P5::default()));
    assert!(unsupported.is_err());
}

/// Empty rings are outside by convention, and an open ring's implicit
/// closing edge participates in boundary classification.
#[test]
fn within_handles_empty_rings_and_the_implicit_closing_edge() {
    let empty = Polygon::<P2>::default();
    assert!(!within(&P2::new(0.0, 0.0), &empty));

    let open: Ring<P2, true, false> = Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 2.0),
        P2::new(2.0, 2.0),
        P2::new(2.0, 0.0),
    ]);
    let polygon = Polygon::<P2, true, false>::new(open);
    assert!(!within(&P2::new(1.0, 0.0), &polygon));

    assert!(equals(&Polygon::<P2>::default(), &Polygon::<P2>::default()));
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

    // A NaN fraction cannot identify a segment; the strategy's total
    // fallback is the final point rather than a panic or out-of-bounds read.
    assert_eq!(line_interpolate(&ls, f64::NAN), P2::new(2.0, 2.0));
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
