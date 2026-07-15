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
use boost_geometry::algorithm::ConcaveHullParams;
use boost_geometry::model::{
    Box as ModelBox, DynGeometry, DynGeometryCollection, Linestring, MultiLinestring, MultiPoint,
    MultiPolygon, Point as ModelPoint, Point2D, Point3D, Polygon, Ring, Segment,
};
use boost_geometry::overlay::{Dimension, relation};
use boost_geometry::prelude::{
    Cartesian, CoordinatePosition, Degree, Geographic, Spherical, area, area_with, assign_values,
    azimuth_with, centroid, centroid_with, chaikin_smoothing, closest_points, closest_points_with,
    comparable_distance_with, concave_hull, concave_hull_with, coordinate_position, correct,
    correct_closure, densify, destination, distance_with, envelope, equals, expand, expand_with,
    for_each_segment, intersects, intersects_reversed, is_simple, k_nearest_concave_hull,
    line_interpolate, line_locate_point, linestring_segmentize, linestring_segmentize_with,
    map_coords, map_coords_in_place, minimum_rotated_rect, monotone_subdivision, perimeter,
    perimeter_with, remove_spikes, rhumb_azimuth, rhumb_destination, rhumb_distance,
    rhumb_distance_with, rhumb_length, ring_area, ring_perimeter_with, simplify_with, transform,
    triangulate_earcut, unique, within,
};
use boost_geometry::strategy::{
    CartesianAzimuth, CartesianBoxCentroid, CartesianPerimeter, ChamberlainDuquetteArea,
    CrossTrack, EnvelopePoint, GeographicAzimuth, GeographicPerimeter, Haversine,
    HaversineClosestPoints, PointToSegment, Pythagoras, Rhumb, Rotate, Scale, Skew, SphericalArea,
    SphericalPerimeter, Translate, Vincenty, VisvalingamWhyatt, VisvalingamWhyattPreserve,
};
use boost_geometry::trait_::{
    IndexedAccess as _, Point as _, PointMut as _, Polygon as _, Ring as _, fold_dims, segment_end,
    segment_start,
};

type P2 = Point2D<f64, Cartesian>;
type P3 = Point3D<f64, Cartesian>;
type P4 = ModelPoint<f64, 4, Cartesian>;
type P5 = ModelPoint<f64, 5, Cartesian>;
type D = DynGeometry<f64, Cartesian>;

/// Chamberlain–Duquette is an opt-in spherical area strategy and leaves the
/// existing spherical default unchanged.
#[test]
fn chamberlain_duquette_area_is_public() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let square: Polygon<SphericalPoint> = Polygon::new(Ring::from_vec(vec![
        SphericalPoint::new(0.0, 0.0),
        SphericalPoint::new(0.0, 1.0),
        SphericalPoint::new(1.0, 1.0),
        SphericalPoint::new(1.0, 0.0),
        SphericalPoint::new(0.0, 0.0),
    ]));

    let solid_angle = area_with(&square, ChamberlainDuquetteArea::UNIT);
    assert!((solid_angle - 0.000_304_601_954_726_850_5).abs() < 1e-12);
}

/// The opt-in spherical area strategy handles default radius scaling,
/// interiors, declared counter-clockwise rings, and degenerate exteriors.
#[test]
fn chamberlain_duquette_covers_topology_and_orientation() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let outer: Ring<SphericalPoint> = Ring::from_vec(vec![
        SphericalPoint::new(0.0, 0.0),
        SphericalPoint::new(0.0, 2.0),
        SphericalPoint::new(2.0, 2.0),
        SphericalPoint::new(2.0, 0.0),
        SphericalPoint::new(0.0, 0.0),
    ]);
    let mut hole: Ring<SphericalPoint> = Ring::from_vec(vec![
        SphericalPoint::new(0.5, 0.5),
        SphericalPoint::new(0.5, 1.5),
        SphericalPoint::new(1.5, 1.5),
        SphericalPoint::new(1.5, 0.5),
        SphericalPoint::new(0.5, 0.5),
    ]);
    hole.0.reverse();
    let outer_area = area_with(&Polygon::new(outer.clone()), ChamberlainDuquetteArea::UNIT);
    let donut_area = area_with(
        &Polygon::with_inners(outer.clone(), vec![hole]),
        ChamberlainDuquetteArea::UNIT,
    );
    assert!(donut_area > 0.0 && donut_area < outer_area);

    let earth_area = area_with(
        &Polygon::new(outer.clone()),
        ChamberlainDuquetteArea::default(),
    );
    assert!(
        (earth_area - outer_area * ChamberlainDuquetteArea::EARTH.radius.powi(2)).abs()
            < earth_area * 1e-12
    );

    let mut ccw_points = outer.0;
    ccw_points.reverse();
    let ccw: Polygon<SphericalPoint, false, true> =
        Polygon::new(Ring::<SphericalPoint, false, true>::from_vec(ccw_points));
    assert!((area_with(&ccw, ChamberlainDuquetteArea::UNIT) - outer_area).abs() < 1e-12);

    let degenerate: Polygon<SphericalPoint> = Polygon::new(Ring::from_vec(vec![
        SphericalPoint::new(0.0, 0.0),
        SphericalPoint::new(1.0, 1.0),
    ]));
    assert_eq!(area_with(&degenerate, ChamberlainDuquetteArea::UNIT), 0.0);
}

/// The native Boost spherical-excess strategy honors the ring's declared
/// orientation and treats an empty open ring as zero area.
#[test]
fn spherical_area_covers_counter_clockwise_and_empty_open_rings() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let clockwise: Ring<SphericalPoint> = Ring::from_vec(vec![
        SphericalPoint::new(0.0, 0.0),
        SphericalPoint::new(0.0, 1.0),
        SphericalPoint::new(1.0, 1.0),
        SphericalPoint::new(1.0, 0.0),
        SphericalPoint::new(0.0, 0.0),
    ]);
    let mut reversed = clockwise.0.clone();
    reversed.reverse();
    let counter_clockwise: Ring<SphericalPoint, false, true> = Ring::from_vec(reversed);
    assert!(
        (area_with(&clockwise, SphericalArea::UNIT)
            - area_with(&counter_clockwise, SphericalArea::UNIT))
        .abs()
            < 1e-12
    );

    let empty_open: Ring<SphericalPoint, true, false> = Ring::from_vec(Vec::new());
    assert_eq!(area_with(&empty_open, SphericalArea::UNIT), 0.0);
}

/// Rhumb-line distance, bearing, destination, and linestring length are all
/// reachable through named public entries, with an explicit-radius companion.
#[test]
fn rhumb_measure_family_is_public() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let start = SphericalPoint::new(0.0, 0.0);
    let east = SphericalPoint::new(1.0, 0.0);
    let distance = rhumb_distance(&start, &east);
    assert!((distance - 111_195.080_233_532_9).abs() < 1e-6);
    assert!((rhumb_azimuth(&start, &east) - core::f64::consts::FRAC_PI_2).abs() < 1e-12);

    let destination = rhumb_destination(&start, core::f64::consts::FRAC_PI_2, distance);
    assert!((destination.get::<0>() - 1.0).abs() < 1e-10);
    assert!(destination.get::<1>().abs() < 1e-10);

    let line = Linestring::from_vec(vec![start, east, SphericalPoint::new(2.0, 0.0)]);
    assert!((rhumb_length(&line) - 2.0 * distance).abs() < 1e-6);
    assert!((rhumb_distance_with(&start, &east, Rhumb::UNIT) - 1.0_f64.to_radians()).abs() < 1e-12);
}

/// Rhumb strategies expose custom radii, comparable distance, meridional
/// motion, pole reflection, and the zero-scale destination guard publicly.
#[test]
fn rhumb_public_edge_cases_cover_poles_and_custom_radius() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let origin = SphericalPoint::new(0.0, 0.0);
    let east = SphericalPoint::new(1.0, 0.0);
    let custom = Rhumb::with_radius(2.0);
    let expected = 2.0 * 1.0_f64.to_radians();
    assert!((rhumb_distance_with(&origin, &east, custom) - expected).abs() < 1e-12);
    assert!((comparable_distance_with(&origin, &east, custom) - expected).abs() < 1e-12);

    let north = rhumb_destination(&origin, 0.0, Rhumb::EARTH.radius * 10.0_f64.to_radians());
    assert!((north.get::<1>() - 10.0).abs() < 1e-12);

    let reflected_north = rhumb_destination(
        &SphericalPoint::new(0.0, 80.0),
        0.0,
        Rhumb::EARTH.radius * 20.0_f64.to_radians(),
    );
    assert!((reflected_north.get::<1>() - 80.0).abs() < 1e-12);
    let reflected_south = rhumb_destination(
        &SphericalPoint::new(0.0, -80.0),
        core::f64::consts::PI,
        Rhumb::EARTH.radius * 20.0_f64.to_radians(),
    );
    assert!((reflected_south.get::<1>() + 80.0).abs() < 1e-12);

    let pole = SphericalPoint::new(30.0, 90.0);
    let along_pole = rhumb_destination(&pole, core::f64::consts::FRAC_PI_2, 0.1);
    assert!(along_pole.get::<0>().is_finite());
    assert!(along_pole.get::<1>().is_finite());
}

/// Minimum rectangles and both concave-hull parameter faces operate on public
/// stock models and retain an interior point when the requested concavity allows.
#[test]
fn derived_hulls_are_public() {
    let diamond = MultiPoint::from_vec(vec![
        P2::new(0.0, 1.0),
        P2::new(1.0, 0.0),
        P2::new(0.0, -1.0),
        P2::new(-1.0, 0.0),
        P2::new(0.0, 0.0),
    ]);
    let rectangle = minimum_rotated_rect(&diamond);
    assert!((area(&rectangle).abs() - 2.0).abs() < 1e-12);

    let points = MultiPoint::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 4.0),
        P2::new(4.0, 4.0),
        P2::new(4.0, 0.0),
        P2::new(2.0, 1.0),
    ]);
    let params = ConcaveHullParams {
        concavity: 1.2,
        length_threshold: 0.0,
    };
    let refined = concave_hull_with(&points, params);
    assert!(refined.outer.0.contains(&P2::new(2.0, 1.0)));
    assert!(area(&refined).abs() < 16.0);

    let defaulted = concave_hull(&points);
    assert!(defaulted.outer.0.contains(&P2::new(2.0, 1.0)));
    let knn = k_nearest_concave_hull(&points, 3);
    assert!(knn.outer.0.contains(&P2::new(2.0, 1.0)));
}

/// Empty, point, segment, and sub-epsilon point sets exercise the documented
/// degenerate minimum-rectangle contract through the public facade.
#[test]
fn minimum_rotated_rect_handles_degenerate_point_sets() {
    let empty = minimum_rotated_rect(&MultiPoint::<P2>::from_vec(vec![]));
    assert!(empty.outer.0.is_empty());

    let point = P2::new(2.0, 3.0);
    let single = minimum_rotated_rect(&MultiPoint::from_vec(vec![point]));
    assert_eq!(single.outer.0, vec![point; 5]);

    let first = P2::new(0.0, 0.0);
    let second = P2::new(2.0, 2.0);
    let segment = minimum_rotated_rect(&MultiPoint::from_vec(vec![first, second]));
    assert_eq!(segment.outer.0.len(), 5);
    assert_eq!(area(&segment), 0.0);
    assert!(segment.outer.0.contains(&first));
    assert!(segment.outer.0.contains(&second));

    let tiny = f64::EPSILON / 4.0;
    let sub_epsilon = minimum_rotated_rect(&MultiPoint::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(tiny, 0.0),
        P2::new(0.0, tiny),
    ]));
    assert!(sub_epsilon.outer.0.is_empty());
}

/// Degenerate point sets and an over-large edge threshold are reference cases
/// from the upstream concave-hull suite. They remain observable through the
/// public facade, including the documented `k == 0` convex-hull boundary.
#[test]
fn concave_hull_handles_degenerate_and_threshold_boundaries() {
    let empty = MultiPoint::<P2>::from_vec(vec![]);
    assert!(concave_hull(&empty).outer.0.is_empty());

    let repeated = MultiPoint::from_vec(vec![P2::new(1.0, 1.0); 4]);
    assert_eq!(
        concave_hull(&repeated).outer.0,
        vec![P2::new(1.0, 1.0), P2::new(1.0, 1.0)]
    );

    let collinear = MultiPoint::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(2.0, 2.0),
        P2::new(6.0, 6.0),
    ]);
    let collinear_hull = concave_hull(&collinear);
    assert_eq!(collinear_hull.outer.0.len(), 3);
    assert_eq!(
        collinear_hull.outer.0.first(),
        collinear_hull.outer.0.last()
    );
    assert!(collinear_hull.outer.0.contains(&P2::new(0.0, 0.0)));
    assert!(collinear_hull.outer.0.contains(&P2::new(6.0, 6.0)));

    let points = MultiPoint::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(2.0, 0.0),
        P2::new(1.5, 1.0),
        P2::new(2.0, 2.0),
        P2::new(0.0, 2.0),
    ]);
    let thresholded = concave_hull_with(
        &points,
        ConcaveHullParams {
            concavity: 1.2,
            length_threshold: 3.0,
        },
    );
    let convex = k_nearest_concave_hull(&points, 0);
    assert_eq!(thresholded, convex);
    assert_eq!(convex.outer.0.len(), 5);
}

/// The upstream consecutive-drilling fixture supplies several candidates per
/// edge. The public implementation must retain every point while producing a
/// closed, simple boundary.
#[test]
fn concave_hull_orders_multiple_edge_candidates() {
    let points = MultiPoint::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(2.0, 1.0),
        P2::new(4.0, 0.0),
        P2::new(3.0, 2.0),
        P2::new(4.0, 4.0),
        P2::new(2.0, 3.0),
        P2::new(0.0, 4.0),
        P2::new(1.0, 2.0),
    ]);

    let hull = k_nearest_concave_hull(&points, points.0.len());
    assert_eq!(hull.outer.0.first(), hull.outer.0.last());
    assert!(is_simple(&hull));
    for point in points.0 {
        assert!(hull.outer.0.contains(&point));
    }
}

/// Native ear clipping and monotone subdivision expose owned stock polygons;
/// the pieces preserve the source area through the public area entry.
#[test]
fn native_triangulation_and_monotone_subdivision_are_public() {
    let polygon: Polygon<P2> = Polygon::new(Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 2.0),
        P2::new(1.0, 1.0),
        P2::new(2.0, 2.0),
        P2::new(2.0, 0.0),
        P2::new(0.0, 0.0),
    ]));

    let triangles = triangulate_earcut(&polygon);
    assert_eq!(triangles.len(), 3);
    let triangle_area: f64 = triangles.iter().map(|triangle| area(triangle).abs()).sum();
    assert!((triangle_area - area(&polygon).abs()).abs() < 1e-12);

    let monotone = monotone_subdivision(&polygon);
    assert!(monotone.len() > 1);
    assert!(monotone.iter().all(|piece| piece.outer.0.len() == 4));
    let monotone_area: f64 = monotone.iter().map(|piece| area(piece).abs()).sum();
    assert!((monotone_area - area(&polygon).abs()).abs() < 1e-12);
}

/// A polygon whose declared interior cannot be bridged into its exterior is
/// rejected atomically instead of returning a partial exterior triangulation.
#[test]
fn earcut_rejects_an_unbridgeable_hole_atomically() {
    let polygon = Polygon::with_inners(
        square_ring(0.0, 0.0, 4.0),
        vec![square_ring(10.0, 10.0, 1.0)],
    );

    let triangles = triangulate_earcut(&polygon);
    assert_eq!(triangles.len(), 0);
}

/// Public ear clipping rejects undersized and collinear exteriors, ignores
/// redundant adjacent vertices, and removes collinear boundary vertices while
/// preserving the polygon's area.
#[test]
fn earcut_handles_degenerate_and_redundant_vertices() {
    let degenerate_exteriors: [Ring<P2>; 3] = [
        Ring::from_vec(vec![]),
        Ring::from_vec(vec![P2::new(0.0, 0.0), P2::new(1.0, 1.0)]),
        Ring::from_vec(vec![
            P2::new(0.0, 0.0),
            P2::new(1.0, 1.0),
            P2::new(2.0, 2.0),
            P2::new(0.0, 0.0),
        ]),
    ];
    for exterior in degenerate_exteriors {
        assert!(triangulate_earcut(&Polygon::new(exterior)).is_empty());
    }

    let polygon: Polygon<P2> = Polygon::new(Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 2.0),
        P2::new(2.0, 2.0),
        P2::new(2.0, 0.0),
        P2::new(1.0, 0.0),
        P2::new(1.0, 0.0),
        P2::new(0.0, 0.0),
    ]));
    let triangles = triangulate_earcut(&polygon);
    assert_eq!(triangles.len(), 3);
    assert!(triangles.iter().all(|triangle| area(triangle).abs() > 0.0));
    let triangle_area: f64 = triangles.iter().map(|triangle| area(triangle).abs()).sum();
    assert!((triangle_area - area(&polygon).abs()).abs() < 1e-12);
}

/// Boost has no supported triangulation entry. This self-intersecting
/// exterior exercises the native earcut contract recorded in
/// `specs/geos_parity/triangulation.md`: a clipping-stalled input is rejected
/// atomically instead of returning a partial triangulation.
#[test]
fn earcut_rejects_a_clipping_stalled_exterior_atomically() {
    let stalled: Polygon<P2> = Polygon::new(Ring::from_vec(vec![
        P2::new(0.0, 3.0),
        P2::new(3.0, 1.0),
        P2::new(2.0, -2.0),
        P2::new(-3.0, 1.0),
        P2::new(-2.0, -2.0),
        P2::new(0.0, -3.0),
        P2::new(0.0, 3.0),
    ]));

    assert_eq!(triangulate_earcut(&stalled).len(), 0);
}

/// Multiple independently visible holes exercise the public bridge selection
/// path. Their triangulation must cover the exterior minus both interiors.
#[test]
fn earcut_triangulates_multiple_holes() {
    let mut first_hole = square_ring(1.0, 1.0, 2.0);
    first_hole.0.reverse();
    let mut second_hole = square_ring(6.0, 6.0, 2.0);
    second_hole.0.reverse();
    let polygon = Polygon::with_inners(square_ring(0.0, 0.0, 10.0), vec![first_hole, second_hole]);

    let triangles = triangulate_earcut(&polygon);
    assert_eq!(triangles.len(), 14);
    let triangle_area: f64 = triangles.iter().map(|triangle| area(triangle).abs()).sum();
    assert!((triangle_area - area(&polygon).abs()).abs() < 1e-12);
}

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

/// Boost's Andoyer formula treats a pole's cosine as zero with
/// `math::equals`, so the public geographic strategy has deterministic pole
/// limits despite `cos(π/2)` not being exactly zero in binary floating point.
#[test]
fn geographic_azimuth_uses_boost_pole_limits() {
    type G = Point2D<f64, Geographic<Degree>>;
    let origin = G::new(0.0, 0.0);

    assert!(azimuth_with(&origin, &G::new(0.0, 90.0), GeographicAzimuth::WGS84).abs() < 1e-12);
    assert!(
        (azimuth_with(&origin, &G::new(0.0, -90.0), GeographicAzimuth::WGS84)
            - core::f64::consts::PI)
            .abs()
            < 1e-12
    );
    assert!(
        (azimuth_with(
            &G::new(0.0, 90.0),
            &G::new(90.0, 0.0),
            GeographicAzimuth::WGS84,
        ) - core::f64::consts::FRAC_PI_2)
            .abs()
            < 1e-12
    );
    let antipodal = azimuth_with(&origin, &G::new(180.0, 0.0), GeographicAzimuth::WGS84);
    assert!(antipodal.abs() < 1e-12, "got {antipodal}");

    assert_eq!(
        distance_with(&G::new(0.0, 10.0), &G::new(360.0, 10.0), Vincenty::WGS84,),
        0.0
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
    type P0 = ModelPoint<f64, 0, Cartesian>;
    type P1 = ModelPoint<f64, 1, Cartesian>;
    type P4 = ModelPoint<f64, 4, Cartesian>;
    type P5 = ModelPoint<f64, 5, Cartesian>;

    let mut zero = P0::default();
    assign_values(&mut zero, &[]);

    let mut one = P1::default();
    assign_values(&mut one, &[7.0]);
    assert_eq!(one.get::<0>(), 7.0);
    assert_eq!(fold_dims(0, &P0::default(), |count, _, _| count + 1), 0);
    assert_eq!(fold_dims(0, &one, |count, _, _| count + 1), 1);

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

    let line = Linestring::from_vec(vec![origin, four]);
    let line_bounds = envelope(&line);
    assert_eq!(line_bounds.get_indexed::<1, 3>(), 4.0);

    let zero_segment = Segment::new(P0::default(), P0::default());
    assert_eq!(
        fold_dims(0, &segment_start(&zero_segment), |count, _, _| count + 1),
        0
    );
    let one_segment = Segment::new(P1::default(), one);
    assert_eq!(segment_end(&one_segment).get::<0>(), 7.0);
    let four_segment = Segment::new(origin, four);
    assert_eq!(segment_end(&four_segment).get::<3>(), 4.0);

    let unsupported = std::panic::catch_unwind(|| equals(&P5::default(), &P5::default()));
    assert!(unsupported.is_err());
    let fold_unsupported =
        std::panic::catch_unwind(|| fold_dims((), &P5::default(), |(), _, _| ()));
    assert!(fold_unsupported.is_err());
    let segment_unsupported =
        std::panic::catch_unwind(|| segment_start(&Segment::new(P5::default(), P5::default())));
    assert!(segment_unsupported.is_err());
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

    let empty_open: Ring<P2, true, false> = Ring::from_vec(Vec::new());
    assert_eq!(ring_area(&empty_open), 0.0);
    assert_eq!(centroid(&empty_open), P2::default());

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

    let first_pair_is_best = Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(10.0, 0.0),
        P2::new(20.0, 0.0),
    ]);
    let short_parallel = Linestring::from_vec(vec![P2::new(0.0, 1.0), P2::new(1.0, 1.0)]);
    let (on_long, on_short) = closest_points(&first_pair_is_best, &short_parallel);
    assert_eq!(on_long, P2::new(0.0, 0.0));
    assert_eq!(on_short, P2::new(0.0, 1.0));

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

/// `test/algorithms/length/length_sph.cpp:53-58` and
/// `length_geo.cpp:95-100` define empty angular inputs as zero length. An
/// explicitly open empty ring exercises the same public perimeter strategy
/// contract without requiring a private strategy call.
#[test]
fn empty_open_angular_rings_have_zero_perimeter() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    type GeographicPoint = Point2D<f64, Geographic<Degree>>;

    let spherical = Ring::<SphericalPoint, true, false>::new();
    let geographic = Ring::<GeographicPoint, true, false>::new();
    assert_eq!(
        ring_perimeter_with(&spherical, SphericalPerimeter::default()),
        0.0
    );
    assert_eq!(
        ring_perimeter_with(&geographic, GeographicPerimeter::default()),
        0.0
    );
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

/// Visvalingam and Whyatt (1993), using the published `PostGIS` example: the
/// area-ranked strategy is selectable through the public explicit-strategy
/// entry while Douglas–Peucker remains the default.
#[test]
fn visvalingam_whyatt_is_selectable_through_the_public_facade() {
    let line = Linestring::from_vec(vec![
        P2::new(5.0, 2.0),
        P2::new(3.0, 8.0),
        P2::new(6.0, 20.0),
        P2::new(7.0, 25.0),
        P2::new(10.0, 10.0),
    ]);

    let simplified = simplify_with(&line, 30.0, VisvalingamWhyatt);
    assert_eq!(
        simplified.0,
        vec![P2::new(5.0, 2.0), P2::new(7.0, 25.0), P2::new(10.0, 10.0),]
    );
}

/// The topology-preserving variant follows the Davies refinement: when the
/// lowest-area removal creates a crossing, its preceding vertex is removed as
/// part of the same simplification sequence.
#[test]
fn visvalingam_whyatt_preserve_avoids_a_new_self_intersection() {
    let line = Linestring::from_vec(vec![
        P2::new(10.0, 60.0),
        P2::new(135.0, 68.0),
        P2::new(94.0, 48.0),
        P2::new(126.0, 31.0),
        P2::new(280.0, 19.0),
        P2::new(117.0, 48.0),
        P2::new(300.0, 40.0),
        P2::new(301.0, 10.0),
    ]);

    let simplified = simplify_with(&line, 668.6, VisvalingamWhyattPreserve);
    assert_eq!(
        simplified.0,
        vec![
            P2::new(10.0, 60.0),
            P2::new(126.0, 31.0),
            P2::new(280.0, 19.0),
            P2::new(117.0, 48.0),
            P2::new(300.0, 40.0),
            P2::new(301.0, 10.0),
        ]
    );
}

/// The tri-state entry exposes the same Cartesian winding result used by
/// `within` and `covered_by`, including both exterior and hole boundaries.
#[test]
fn coordinate_position_is_public_and_hole_aware() {
    let polygon = Polygon::with_inners(
        square_ring(0.0, 0.0, 10.0),
        vec![square_ring(3.0, 3.0, 4.0)],
    );

    assert_eq!(
        coordinate_position(&P2::new(1.0, 1.0), &polygon),
        CoordinatePosition::Inside
    );
    assert_eq!(
        coordinate_position(&P2::new(0.0, 5.0), &polygon),
        CoordinatePosition::OnBoundary
    );
    assert_eq!(
        coordinate_position(&P2::new(3.0, 5.0), &polygon),
        CoordinatePosition::OnBoundary
    );
    assert_eq!(
        coordinate_position(&P2::new(5.0, 5.0), &polygon),
        CoordinatePosition::Outside
    );
    assert_eq!(
        coordinate_position(&P2::new(11.0, 5.0), &polygon),
        CoordinatePosition::Outside
    );
}

/// Chaikin's 1974 corner-cutting rule is reachable through the public facade
/// and retains the endpoints of an open linestring.
#[test]
fn chaikin_smoothing_subdivides_an_open_linestring() {
    let line = Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(1.0, 0.0),
        P2::new(1.0, 1.0),
    ]);

    assert_eq!(
        chaikin_smoothing(&line, 1).0,
        vec![
            P2::new(0.0, 0.0),
            P2::new(0.25, 0.0),
            P2::new(0.75, 0.0),
            P2::new(1.0, 0.25),
            P2::new(1.0, 0.75),
            P2::new(1.0, 1.0),
        ]
    );
}

/// The public Chaikin dispatch preserves polygon topology, open-ring closure
/// declarations, degenerate inputs, and all four supported ordinates.
#[test]
fn chaikin_smoothing_covers_stock_topologies_and_dimensions() {
    let empty = Linestring::<P2>::from_vec(vec![]);
    assert!(chaikin_smoothing(&empty, 2).0.is_empty());
    let singleton = Linestring::from_vec(vec![P2::new(1.0, 2.0)]);
    assert_eq!(chaikin_smoothing(&singleton, 1), singleton);

    let short_ring: Ring<P2> = Ring::from_vec(vec![P2::new(0.0, 0.0), P2::new(1.0, 0.0)]);
    assert_eq!(chaikin_smoothing(&short_ring, 1), short_ring);

    let open_ring: Ring<P2, true, false> = Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(0.0, 2.0),
        P2::new(2.0, 0.0),
    ]);
    let smoothed_open = chaikin_smoothing(&open_ring, 1);
    assert_eq!(smoothed_open.0.len(), 6);
    assert_ne!(smoothed_open.0.first(), smoothed_open.0.last());

    let polygon =
        Polygon::with_inners(square_ring(0.0, 0.0, 4.0), vec![square_ring(1.0, 1.0, 1.0)]);
    let smoothed_polygon = chaikin_smoothing(&polygon, 1);
    assert_eq!(smoothed_polygon.outer.0.len(), 9);
    assert_eq!(smoothed_polygon.inners.len(), 1);
    assert_eq!(smoothed_polygon.inners[0].0.len(), 9);

    let line3 = Linestring::from_vec(vec![P3::new(0.0, 2.0, 4.0), P3::new(4.0, 6.0, 8.0)]);
    let smoothed3 = chaikin_smoothing(&line3, 1);
    assert_eq!(smoothed3.0[1], P3::new(1.0, 3.0, 5.0));

    let mut start4 = P4::default();
    start4.set::<3>(4.0);
    let mut end4 = P4::default();
    end4.set::<3>(8.0);
    let smoothed4 = chaikin_smoothing(&Linestring::from_vec(vec![start4, end4]), 1);
    assert_eq!(smoothed4.0[1].get::<3>(), 5.0);
}

/// Named strategies construct the existing affine matrix engine rather than
/// introducing separate transform implementations.
#[test]
fn named_affine_strategies_feed_the_public_transform_entry() {
    let point = P2::new(1.0, 2.0);
    assert_eq!(
        transform(&point, &Translate::by(3.0, 4.0)),
        P2::new(4.0, 6.0)
    );
    assert_eq!(transform(&point, &Scale::uniform(2.0)), P2::new(2.0, 4.0));
    assert_eq!(transform(&point, &Scale::by(2.0, 3.0)), P2::new(2.0, 6.0));
    assert_eq!(transform(&point, &Skew::by(2.0, 0.0)), P2::new(5.0, 2.0));
    let skewed_radians = transform(&point, &Skew::radians(0.0, core::f64::consts::FRAC_PI_4));
    assert!((skewed_radians.get::<0>() - 1.0).abs() < 1e-12);
    assert!((skewed_radians.get::<1>() - 3.0).abs() < 1e-12);
    let skewed_degrees = transform(&point, &Skew::degrees(45.0, 0.0));
    assert!((skewed_degrees.get::<0>() - 3.0).abs() < 1e-12);
    assert!((skewed_degrees.get::<1>() - 2.0).abs() < 1e-12);

    let rotated = transform(&P2::new(1.0, 0.0), &Rotate::degrees(90.0));
    assert!(rotated.get::<0>().abs() < 1e-12);
    assert!((rotated.get::<1>() - 1.0).abs() < 1e-12);
}

/// The default geographic destination entry routes through the ported direct
/// formula and returns the endpoint in the origin point's angular units.
#[test]
fn destination_is_available_through_the_public_facade() {
    type GeographicPoint = Point2D<f64, Geographic<Degree>>;

    let endpoint = destination(
        &GeographicPoint::new(0.0, 0.0),
        core::f64::consts::FRAC_PI_2,
        100_000.0,
    );
    assert!((endpoint.get::<0>() - 0.898_315_284_1).abs() < 1e-6);
    assert!(endpoint.get::<1>().abs() < 1e-8);
}

/// Locating and segmentizing are inverse-style public operations over the same
/// accumulated Cartesian arc length.
#[test]
fn locate_and_segmentize_preserve_a_bent_linestring() {
    let line = Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(2.0, 0.0),
        P2::new(2.0, 2.0),
    ]);

    assert_eq!(line_locate_point(&line, &P2::new(2.5, 1.0)), Some(0.75));
    assert_eq!(line_locate_point(&line, &P2::new(1.0, 0.25)), Some(0.25));
    let pieces = linestring_segmentize(&line, 2);
    assert_eq!(pieces.0.len(), 2);
    assert_eq!(pieces.0[0].0, vec![P2::new(0.0, 0.0), P2::new(2.0, 0.0)]);
    assert_eq!(pieces.0[1].0, vec![P2::new(2.0, 0.0), P2::new(2.0, 2.0)]);
}

/// `geo/src/algorithm/line_locate_point.rs:223-235` supplies the repeated-point
/// reference case. The public contract also defines the adjacent single-point
/// boundary: both have fractional position zero regardless of the query.
#[test]
fn line_locate_point_handles_single_and_zero_length_linestrings() {
    let query = P2::new(2.0, 2.0);
    let single = Linestring::from_vec(vec![P2::new(1.0, 1.0)]);
    assert_eq!(line_locate_point(&single, &query), Some(0.0));

    let repeated = Linestring::from_vec(vec![
        P2::new(1.0, 1.0),
        P2::new(1.0, 1.0),
        P2::new(1.0, 1.0),
    ]);
    assert_eq!(line_locate_point(&repeated, &query), Some(0.0));
}

/// Explicit segmentization follows great-circle interpolation when supplied a
/// spherical distance strategy.
#[test]
fn segmentize_with_haversine_uses_spherical_interpolation() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let line = Linestring::from_vec(vec![
        SphericalPoint::new(0.0, 0.0),
        SphericalPoint::new(2.0, 0.0),
    ]);

    let pieces = linestring_segmentize_with(&line, 2, Haversine::EARTH);
    assert_eq!(pieces.0.len(), 2);
    assert!((pieces.0[0].0[1].get::<0>() - 1.0).abs() < 1e-12);
    assert!((pieces.0[1].0[0].get::<0>() - 1.0).abs() < 1e-12);
}

/// Segmentization preserves all four Cartesian ordinates, retains a wholly
/// degenerate line as one piece, and uses its documented linear fallback for
/// antipodal spherical endpoints.
#[test]
fn segmentize_handles_dimensions_and_degenerate_metrics() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;

    assert!(
        linestring_segmentize(&Linestring::<P2>::from_vec(Vec::new()), 2)
            .0
            .is_empty()
    );
    assert!(
        linestring_segmentize(
            &Linestring::from_vec(vec![P2::new(0.0, 0.0), P2::new(1.0, 0.0)]),
            0,
        )
        .0
        .is_empty()
    );

    let mut end = P4::default();
    end.set::<0>(4.0);
    end.set::<1>(8.0);
    end.set::<2>(12.0);
    end.set::<3>(16.0);
    let pieces = linestring_segmentize(&Linestring::from_vec(vec![P4::default(), end]), 2);
    assert_eq!(pieces.0[0].0[1].get::<2>(), 6.0);
    assert_eq!(pieces.0[0].0[1].get::<3>(), 8.0);

    let repeated = Linestring::from_vec(vec![P2::new(1.0, 1.0), P2::new(1.0, 1.0)]);
    let degenerate = linestring_segmentize(&repeated, 3);
    assert_eq!(degenerate.0, vec![repeated]);

    let with_interior_vertex = Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(1.0, 0.0),
        P2::new(6.0, 0.0),
    ]);
    let split = linestring_segmentize(&with_interior_vertex, 2);
    assert_eq!(
        split.0[0].0,
        vec![P2::new(0.0, 0.0), P2::new(1.0, 0.0), P2::new(3.0, 0.0)]
    );

    let antipodal = Linestring::from_vec(vec![
        SphericalPoint::new(0.0, 0.0),
        SphericalPoint::new(180.0, 0.0),
    ]);
    let antipodal_pieces = linestring_segmentize_with(&antipodal, 2, Haversine::UNIT);
    assert!((antipodal_pieces.0[0].0[1].get::<0>() - 90.0).abs() < 1e-12);
}

/// The ported spherical cross-track and closest-point strategies share the
/// same projection and are both reachable through explicit public entries.
#[test]
fn spherical_cross_track_and_closest_point_are_public() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let point = SphericalPoint::new(1.0, 1.0);
    let segment = Segment::new(SphericalPoint::new(0.0, 0.0), SphericalPoint::new(2.0, 0.0));

    let distance = distance_with(&point, &segment, CrossTrack::EARTH);
    assert!((distance - 111_226.255).abs() < 100.0);

    let (source, projected) = closest_points_with(&point, &segment, HaversineClosestPoints::EARTH);
    assert!((source.get::<0>() - point.get::<0>()).abs() < 1e-12);
    assert!((source.get::<1>() - point.get::<1>()).abs() < 1e-12);
    assert!((projected.get::<0>() - 1.0).abs() < 1e-9);
    assert!(projected.get::<1>().abs() < 1e-9);
}

/// Reference cross-track cases cover degenerate segments, projections beyond
/// the minor arc, polar projection fallback, reversed dispatch, and comparable
/// strategy construction through the public facade.
#[test]
fn spherical_cross_track_public_edge_cases_choose_endpoints() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let point = SphericalPoint::new(1.0, 1.0);
    let endpoint = SphericalPoint::new(2.0, 2.0);
    let degenerate = Segment::new(endpoint, endpoint);
    let distance = distance_with(&point, &degenerate, CrossTrack::UNIT);
    assert!((distance - 0.024_678_3).abs() < 1e-6);
    let projected_degenerate =
        closest_points_with(&point, &degenerate, HaversineClosestPoints::UNIT).1;
    assert_eq!(projected_degenerate.get::<0>(), endpoint.get::<0>());
    assert_eq!(projected_degenerate.get::<1>(), endpoint.get::<1>());

    let segment = Segment::new(
        SphericalPoint::new(10.0, 15.0),
        SphericalPoint::new(30.0, 15.0),
    );
    let beyond = SphericalPoint::new(5.0, 10.0);
    let forward = distance_with(&beyond, &segment, CrossTrack::default());
    let reverse = distance_with(&segment, &beyond, CrossTrack::default());
    let expected_endpoint = distance_with(&beyond, segment.start(), Haversine::EARTH);
    assert!((forward - expected_endpoint).abs() < 1e-9);
    assert!((forward - reverse).abs() < 1e-9);
    assert_eq!(
        comparable_distance_with(&beyond, &segment, CrossTrack::default()),
        forward
    );
    assert_eq!(
        comparable_distance_with(&segment, &beyond, CrossTrack::default()),
        reverse
    );
    let (projected, source) =
        closest_points_with(&segment, &beyond, HaversineClosestPoints::default());
    assert_eq!(source.get::<0>(), beyond.get::<0>());
    assert_eq!(source.get::<1>(), beyond.get::<1>());
    assert_eq!(projected.get::<0>(), segment.start().get::<0>());
    assert_eq!(projected.get::<1>(), segment.start().get::<1>());

    let beyond_end = SphericalPoint::new(35.0, 10.0);
    let end_distance = distance_with(&beyond_end, &segment, CrossTrack::default());
    let expected_end_distance = distance_with(&beyond_end, segment.end(), Haversine::EARTH);
    assert!((end_distance - expected_end_distance).abs() < 1e-9);
    let projected_end =
        closest_points_with(&beyond_end, &segment, HaversineClosestPoints::default()).1;
    assert_eq!(projected_end.get::<0>(), segment.end().get::<0>());
    assert_eq!(projected_end.get::<1>(), segment.end().get::<1>());

    let equator = Segment::new(
        SphericalPoint::new(-10.0, 0.0),
        SphericalPoint::new(10.0, 0.0),
    );
    let pole = SphericalPoint::new(0.0, 90.0);
    let (_, projected_pole) = closest_points_with(&pole, &equator, HaversineClosestPoints::UNIT);
    assert!(
        (projected_pole.get::<0>() - equator.start().get::<0>()).abs() < 1e-12
            || (projected_pole.get::<0>() - equator.end().get::<0>()).abs() < 1e-12
    );
    assert!(projected_pole.get::<1>().abs() < 1e-12);
}

/// Non-positive and NaN area tolerances are public no-op cases for the
/// Visvalingam–Whyatt strategies.
#[test]
fn visvalingam_whyatt_non_positive_and_nan_tolerances_copy_through() {
    let line = Linestring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(1.0, 1.0),
        P2::new(2.0, 0.0),
    ]);
    assert_eq!(simplify_with(&line, 0.0, VisvalingamWhyatt), line);
    assert_eq!(
        simplify_with(&line, f64::NAN, VisvalingamWhyattPreserve),
        line
    );
}

/// Value mapping can change scalar type, while the in-place face mutates every
/// stored point through the public geometry model.
#[test]
#[allow(
    clippy::cast_possible_truncation,
    reason = "the test deliberately verifies scalar-type rebinding from f64 to f32"
)]
fn map_coords_supports_rebind_and_in_place_faces() {
    let line = Linestring::from_vec(vec![P2::new(1.0, 2.0), P2::new(3.0, 4.0)]);
    let mapped: Linestring<Point2D<f32, Cartesian>> = map_coords(&line, |point| {
        Point2D::new(point.get::<0>() as f32 * 2.0, point.get::<1>() as f32)
    });
    assert_eq!(mapped.0[0], Point2D::new(2.0_f32, 2.0));

    let mut shifted = line;
    map_coords_in_place(&mut shifted, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });
    assert_eq!(shifted.0[0], P2::new(11.0, 2.0));
    assert_eq!(shifted.0[1], P2::new(13.0, 4.0));
}

/// Every stock geometry implementation is reachable through the public
/// facade. Mapping preserves topology, including polygon interiors and each
/// member of multi-geometries; in-place mapping visits the same stored points.
#[test]
fn map_coords_supports_every_stock_geometry() {
    let shift = |point: &P2| P2::new(point.get::<0>() + 10.0, point.get::<1>() - 1.0);

    let point = P2::new(1.0, 2.0);
    assert_eq!(map_coords(&point, shift), P2::new(11.0, 1.0));

    let ring = square_ring(0.0, 0.0, 2.0);
    let mapped_ring: Ring<P2> = map_coords(&ring, shift);
    assert_eq!(mapped_ring.0.len(), ring.0.len());
    assert_eq!(mapped_ring.0[2], P2::new(12.0, 1.0));

    let polygon = Polygon::with_inners(ring.clone(), vec![square_ring(0.5, 0.5, 0.5)]);
    let mapped_polygon: Polygon<P2> = map_coords(&polygon, shift);
    assert_eq!(mapped_polygon.outer.0.len(), polygon.outer.0.len());
    assert_eq!(mapped_polygon.inners.len(), 1);
    assert_eq!(mapped_polygon.inners[0].0[0], P2::new(10.5, -0.5));

    let points = MultiPoint::from_vec(vec![P2::new(1.0, 2.0), P2::new(3.0, 4.0)]);
    let mapped_points: MultiPoint<P2> = map_coords(&points, shift);
    assert_eq!(
        mapped_points.0,
        vec![P2::new(11.0, 1.0), P2::new(13.0, 3.0)]
    );

    let lines = MultiLinestring::from_vec(vec![
        Linestring::from_vec(vec![P2::new(0.0, 0.0), P2::new(1.0, 1.0)]),
        Linestring::from_vec(vec![P2::new(2.0, 2.0)]),
    ]);
    let mapped_lines: MultiLinestring<Linestring<P2>> = map_coords(&lines, shift);
    assert_eq!(mapped_lines.0.len(), 2);
    assert_eq!(mapped_lines.0[1].0[0], P2::new(12.0, 1.0));

    let polygons = MultiPolygon::from_vec(vec![polygon.clone(), Polygon::new(ring.clone())]);
    let mapped_polygons: MultiPolygon<Polygon<P2>> = map_coords(&polygons, shift);
    assert_eq!(mapped_polygons.0.len(), 2);
    assert_eq!(mapped_polygons.0[0].inners.len(), 1);
    assert_eq!(mapped_polygons.0[1].outer.0[1], P2::new(12.0, -1.0));

    let bounds = ModelBox::from_corners(P2::new(-1.0, -2.0), P2::new(3.0, 4.0));
    let mapped_bounds: ModelBox<P2> = map_coords(&bounds, shift);
    assert_eq!(mapped_bounds.get_indexed::<0, 0>(), 9.0);
    assert_eq!(mapped_bounds.get_indexed::<1, 1>(), 3.0);

    let segment = Segment::new(P2::new(-2.0, 3.0), P2::new(4.0, 5.0));
    let mapped_segment: Segment<P2> = map_coords(&segment, shift);
    assert_eq!(mapped_segment.get_indexed::<0, 0>(), 8.0);
    assert_eq!(mapped_segment.get_indexed::<1, 1>(), 4.0);

    let mut point_mut = point;
    let mut line_mut = Linestring::from_vec(vec![point]);
    let mut ring_mut = ring;
    let mut polygon_mut = polygon;
    let mut multi_point_mut = points;
    let mut multi_line_mut = lines;
    let mut multi_polygon_mut = polygons;
    let mut bounds_mut = bounds;
    let mut segment_mut = segment;
    map_coords_in_place(&mut point_mut, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });
    map_coords_in_place(&mut line_mut, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });
    map_coords_in_place(&mut ring_mut, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });
    map_coords_in_place(&mut polygon_mut, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });
    map_coords_in_place(&mut multi_point_mut, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });
    map_coords_in_place(&mut multi_line_mut, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });
    map_coords_in_place(&mut multi_polygon_mut, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });
    map_coords_in_place(&mut bounds_mut, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });
    map_coords_in_place(&mut segment_mut, |point| {
        point.set::<0>(point.get::<0>() + 10.0);
    });

    assert_eq!(point_mut, P2::new(11.0, 2.0));
    assert_eq!(line_mut.0[0], P2::new(11.0, 2.0));
    assert_eq!(ring_mut.0[2], P2::new(12.0, 2.0));
    assert_eq!(polygon_mut.inners[0].0[0], P2::new(10.5, 0.5));
    assert_eq!(multi_point_mut.0[1], P2::new(13.0, 4.0));
    assert_eq!(multi_line_mut.0[1].0[0], P2::new(12.0, 2.0));
    assert_eq!(multi_polygon_mut.0[1].outer.0[1], P2::new(12.0, 0.0));
    assert_eq!(bounds_mut.get_indexed::<0, 0>(), 9.0);
    assert_eq!(segment_mut.get_indexed::<1, 0>(), 14.0);
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

/// A connected line needs only one representative point plus all boundary
/// crossings to determine whether it meets polygon material. These cases lock
/// the public predicate at the two transitions which a first-point-only
/// containment check must still discover through ring crossings.
#[test]
fn intersects_linestring_polygon_uses_boundary_crossings_after_the_first_point() {
    let polygon = Polygon::with_inners(
        square_ring(0.0, 0.0, 10.0),
        vec![square_ring(3.0, 3.0, 4.0)],
    );

    let exits_hole_into_material: Linestring<P2> =
        Linestring::from_vec(vec![P2::new(4.0, 5.0), P2::new(2.0, 5.0)]);
    assert!(intersects(&exits_hole_into_material, &polygon));

    let exits_exterior_into_material: Linestring<P2> =
        Linestring::from_vec(vec![P2::new(-1.0, 2.0), P2::new(2.0, 2.0)]);
    assert!(intersects(&exits_exterior_into_material, &polygon));

    let stays_in_hole: Linestring<P2> = Linestring::from_vec(vec![
        P2::new(4.0, 4.0),
        P2::new(5.0, 5.0),
        P2::new(6.0, 6.0),
    ]);
    assert!(!intersects(&stays_in_hole, &polygon));

    let stays_outside = Linestring::from_vec(
        (0..64)
            .map(|index| P2::new(20.0 + f64::from(index), f64::from(index % 3)))
            .collect(),
    );
    assert!(!intersects(&stays_outside, &polygon));

    assert!(!intersects(&P2::new(5.0, 5.0), &polygon));
    assert!(intersects(&P2::new(0.0, 5.0), &polygon));
}

/// Differential public-API oracle: the dedicated boolean predicate must agree
/// with the independently implemented DE-9IM relation for exterior, material,
/// boundary, and hole transitions in either segment direction.
#[test]
fn intersects_point_and_linestring_polygon_match_public_relation_grid() {
    let polygon = Polygon::with_inners(
        square_ring(-2.0, -2.0, 4.0),
        vec![square_ring(-1.0, -1.0, 2.0)],
    );
    let values = [-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0];

    for &x in &values {
        for &y in &values {
            let point = P2::new(x, y);
            let matrix = relation(&point, &polygon).unwrap();
            let related = matrix.m[0][0] != Dimension::Empty || matrix.m[0][1] != Dimension::Empty;
            assert_eq!(intersects(&point, &polygon), related, "point=({x}, {y})");
        }
    }

    for &x1 in &values {
        for &y1 in &values {
            for &x2 in &values {
                for &y2 in &values {
                    let line = Linestring::from_vec(vec![P2::new(x1, y1), P2::new(x2, y2)]);
                    let matrix = relation(&line, &polygon).unwrap();
                    let related = matrix.m[0][0] != Dimension::Empty
                        || matrix.m[0][1] != Dimension::Empty
                        || matrix.m[1][0] != Dimension::Empty
                        || matrix.m[1][1] != Dimension::Empty;
                    assert_eq!(
                        intersects(&line, &polygon),
                        related,
                        "line=({x1}, {y1})→({x2}, {y2})"
                    );
                }
            }
        }
    }
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

    let disjoint_line = Linestring::from_vec(vec![P2::new(3.0, 3.0), P2::new(4.0, 4.0)]);
    assert!(!intersects(&disjoint_line, &empty_polygon));
    assert!(!intersects(&disjoint_line, &square));

    let open_triangle: Polygon<P2, true, false> = Polygon::new(Ring::from_vec(vec![
        P2::new(0.0, 0.0),
        P2::new(2.0, 0.0),
        P2::new(2.0, 2.0),
    ]));
    let crosses_implicit_edge = Linestring::from_vec(vec![P2::new(0.5, 1.6), P2::new(1.8, 1.6)]);
    assert!(intersects(&crosses_implicit_edge, &open_triangle));

    let mut hole = square_ring(10.0, 10.0, 1.0);
    hole.0.reverse();
    let disjoint_with_hole = Polygon::with_inners(square_ring(9.0, 9.0, 3.0), vec![hole]);
    assert!(!intersects(&square, &disjoint_with_hole));
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
