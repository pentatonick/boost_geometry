//! Public-facade tests for Boost's composable buffer strategy family.

use boost_geometry::cs::{Cartesian, Degree, Geographic, Spherical, Spheroid};
use boost_geometry::model::{
    Box as ModelBox, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
    Segment, polygon,
};
use boost_geometry::overlay::{
    JoinStrategy, OverlayError, PointStrategy, buffer, buffer_convex_polygon, buffer_with,
    buffer_with_strategy, is_valid,
};
use boost_geometry::prelude::{area, covered_by, distance_with};
use boost_geometry::strategy::buffer::{
    BufferDistanceStrategy, BufferEndStrategy, BufferJoinStrategy, BufferPointStrategy,
    BufferSettings, BufferSideStrategy, GeographicBuffer, SphericalBuffer,
};
use boost_geometry::strategy::{Haversine, Vincenty};
use boost_geometry::trait_::{MultiPolygon as _, Point as _, Polygon as _, Ring as _};

type P = Point2D<f64, Cartesian>;

fn buffered_area(result: &MultiPolygon<Polygon<P>>) -> f64 {
    result.polygons().map(area).sum()
}

#[test]
fn default_buffer_settings_match_the_public_round_constructor() {
    assert_eq!(BufferSettings::default(), BufferSettings::round(1.0, 36));
    assert_eq!(SphericalBuffer::default(), SphericalBuffer::UNIT);
}

/// `test/algorithms/buffer/buffer_point.cpp:25-29` and
/// `buffer_with_strategies.cpp:88-106` — point radius scaling through the
/// explicit five-strategy interface.
#[test]
fn composed_point_buffer_strategies_flow_through_public_api() {
    let settings = BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(2.0),
        side: BufferSideStrategy::Straight,
        join: BufferJoinStrategy::Round {
            points_per_circle: 720,
        },
        end: BufferEndStrategy::Round {
            points_per_circle: 720,
        },
        point: BufferPointStrategy::Circle {
            points_per_circle: 720,
        },
    };
    let result = buffer_with(&P::new(0.0, 0.0), settings).unwrap();
    assert!((buffered_area(&result) - 4.0 * core::f64::consts::PI).abs() < 0.02);
}

/// `test/algorithms/buffer/buffer_linestring.cpp:142-161` — asymmetric linear
/// distances and flat ends; the rectangle area is the self-contained oracle.
#[test]
fn asymmetric_flat_linestring_buffer_matches_rectangle() {
    let line = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(4.0, 0.0)]);
    let settings = BufferSettings {
        distance: BufferDistanceStrategy::Asymmetric {
            left: 1.0,
            right: 2.0,
        },
        side: BufferSideStrategy::Straight,
        join: BufferJoinStrategy::Miter { limit: 5.0 },
        end: BufferEndStrategy::Flat,
        point: BufferPointStrategy::Square,
    };
    let result = buffer_with(&line, settings).unwrap();
    assert!((buffered_area(&result) - 12.0).abs() < 1e-9);
}

/// `test/algorithms/buffer/buffer_polygon.cpp:660-680` and `823-838` — negative
/// symmetric distance is the polygon erosion arm of Boost's buffer.
#[test]
fn negative_miter_polygon_buffer_erodes_convex_polygon() {
    let square: Polygon<P> = polygon![[
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 10.0),
        (0.0, 10.0),
        (0.0, 0.0)
    ]];
    let settings = BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(-1.0),
        side: BufferSideStrategy::Straight,
        join: BufferJoinStrategy::Miter { limit: 5.0 },
        end: BufferEndStrategy::Flat,
        point: BufferPointStrategy::Square,
    };
    let result = buffer_with(&square, settings).unwrap();
    assert!((buffered_area(&result) - 64.0).abs() < 1e-9);
}

fn l_shape() -> Polygon<P> {
    polygon![[
        (0.0, 0.0),
        (0.0, 4.0),
        (1.0, 4.0),
        (1.0, 1.0),
        (4.0, 1.0),
        (4.0, 0.0),
        (0.0, 0.0)
    ]]
}

fn miter_settings(distance: f64) -> BufferSettings {
    BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(distance),
        side: BufferSideStrategy::Straight,
        join: BufferJoinStrategy::Miter { limit: 5.0 },
        end: BufferEndStrategy::Flat,
        point: BufferPointStrategy::Square,
    }
}

/// `test/algorithms/buffer/buffer_ring.cpp:15-31` and
/// `buffer_polygon.cpp:828-846` — concave positive/negative offsets handle
/// reflex vertices instead of restricting the entry to a convex subset.
#[test]
fn positive_and_negative_miter_buffers_support_non_convex_polygons() {
    let grown = buffer_with(&l_shape(), miter_settings(1.0)).unwrap();
    assert!((buffered_area(&grown) - 27.0).abs() < 1e-9);

    let eroded = buffer_with(&l_shape(), miter_settings(-0.2)).unwrap();
    assert!((buffered_area(&eroded) - 3.96).abs() < 1e-9);
}

/// `test/algorithms/buffer/buffer_polygon.cpp:179-185,660-680` — exterior and
/// interior rings move in opposite topological directions.
#[test]
fn polygon_buffer_preserves_and_offsets_holes() {
    let outer: Ring<P> = Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(0.0, 10.0),
        P::new(10.0, 10.0),
        P::new(10.0, 0.0),
        P::new(0.0, 0.0),
    ]);
    let hole: Ring<P> = Ring::from_vec(vec![
        P::new(3.0, 3.0),
        P::new(7.0, 3.0),
        P::new(7.0, 7.0),
        P::new(3.0, 7.0),
        P::new(3.0, 3.0),
    ]);
    let donut = Polygon::with_inners(outer, vec![hole]);

    let grown = buffer_with(&donut, miter_settings(1.0)).unwrap();
    assert!((buffered_area(&grown) - 140.0).abs() < 1e-9);
    assert_eq!(grown.polygons().next().unwrap().interiors().count(), 1);

    let eroded = buffer_with(&donut, miter_settings(-1.0)).unwrap();
    assert!((buffered_area(&eroded) - 28.0).abs() < 1e-9);
    assert_eq!(eroded.polygons().next().unwrap().interiors().count(), 1);
}

/// `test/algorithms/buffer/buffer_linestring.cpp:142-151` — round and flat
/// endpoint variants; a two-point capsule has a closed-form area oracle.
#[test]
fn round_linestring_ends_form_a_capsule() {
    let line = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(4.0, 0.0)]);
    let result = buffer_with(&line, BufferSettings::round(1.0, 720)).unwrap();
    let expected = 8.0 + core::f64::consts::PI;
    assert!((buffered_area(&result) - expected).abs() < 0.01);
}

/// `test/strategies/buffer_join.cpp:58-93` — the configured limit shortens a
/// sharp miter instead of emitting an arbitrarily long one.
#[test]
fn linestring_miter_limit_caps_sharp_joins() {
    let line = Linestring::from_vec(vec![
        P::new(-1.0, 10.0),
        P::new(0.0, 0.0),
        P::new(1.0, 10.0),
    ]);
    let mut limited = miter_settings(1.0);
    limited.distance = BufferDistanceStrategy::Symmetric(1.0);
    limited.join = BufferJoinStrategy::Miter { limit: 2.0 };
    let mut unlimited = limited;
    unlimited.join = BufferJoinStrategy::Miter { limit: 20.0 };

    let limited = buffer_with(&line, limited).unwrap();
    let unlimited = buffer_with(&line, unlimited).unwrap();
    let limited_points = limited
        .polygons()
        .next()
        .unwrap()
        .exterior()
        .points()
        .count();
    let unlimited_points = unlimited
        .polygons()
        .next()
        .unwrap()
        .exterior()
        .points()
        .count();
    assert!(limited_points > unlimited_points);
}

/// `test/algorithms/buffer/buffer_multi_point.cpp:37-66`,
/// `buffer_multi_polygon.cpp:39-85`, and `buffer_ring.cpp:15-31` — every
/// homogeneous aggregate dispatches through the same public entry.
#[test]
fn ring_box_and_multi_geometries_use_public_buffer_dispatch() {
    let ring = l_shape().exterior().clone();
    let ring_result = buffer_with(&ring, miter_settings(1.0)).unwrap();
    assert!((buffered_area(&ring_result) - 27.0).abs() < 1e-9);

    let open_ring: Ring<P, true, false> = Ring::from_vec(ring.0[..ring.0.len() - 1].to_vec());
    let open_ring_result = buffer_with(&open_ring, miter_settings(1.0)).unwrap();
    assert!((buffered_area(&open_ring_result) - 27.0).abs() < 1e-9);

    let bounds = ModelBox::from_corners(P::new(0.0, 0.0), P::new(2.0, 4.0));
    let box_result = buffer_with(&bounds, miter_settings(1.0)).unwrap();
    assert!((buffered_area(&box_result) - 24.0).abs() < 1e-9);

    let segment = Segment::new(P::new(0.0, 0.0), P::new(4.0, 0.0));
    let segment_result = buffer_with(&segment, BufferSettings::round(1.0, 720)).unwrap();
    assert!((buffered_area(&segment_result) - (8.0 + core::f64::consts::PI)).abs() < 0.01);

    let points = MultiPoint::from_vec(vec![P::new(0.0, 0.0), P::new(10.0, 0.0)]);
    let points_result = buffer_with(&points, BufferSettings::round(1.0, 720)).unwrap();
    assert_eq!(points_result.polygons().count(), 2);
    assert!((buffered_area(&points_result) - 2.0 * core::f64::consts::PI).abs() < 0.02);

    let overlapping_points = MultiPoint::from_vec(vec![P::new(0.0, 0.0), P::new(1.0, 0.0)]);
    let dissolved = buffer_with(&overlapping_points, BufferSettings::round(1.0, 72)).unwrap();
    assert_eq!(dissolved.polygons().count(), 1);

    let lines = MultiLinestring::from_vec(vec![
        Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(2.0, 0.0)]),
        Linestring::from_vec(vec![P::new(10.0, 0.0), P::new(12.0, 0.0)]),
    ]);
    let lines_result = buffer_with(&lines, BufferSettings::round(1.0, 720)).unwrap();
    assert_eq!(lines_result.polygons().count(), 2);

    let polygons: MultiPolygon<Polygon<P>> = MultiPolygon::from_vec(vec![
        polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]],
        polygon![[
            (10.0, 0.0),
            (10.0, 2.0),
            (12.0, 2.0),
            (12.0, 0.0),
            (10.0, 0.0)
        ]],
    ]);
    let polygons_result = buffer_with(&polygons, miter_settings(1.0)).unwrap();
    assert_eq!(polygons_result.polygons().count(), 2);
    assert!((buffered_area(&polygons_result) - 32.0).abs() < 1e-9);
}

/// `test/algorithms/buffer/buffer_polygon.cpp:622-680,823-838` — negative
/// buffers remove collapsed components and positive buffers remove collapsed
/// holes.
#[test]
fn polygon_buffer_handles_offset_topology_collapse() {
    let square: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)]];
    let vanished = buffer_with(&square, miter_settings(-3.0)).unwrap();
    assert_eq!(vanished.polygons().count(), 0);

    let outer: Ring<P> = Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(0.0, 10.0),
        P::new(10.0, 10.0),
        P::new(10.0, 0.0),
        P::new(0.0, 0.0),
    ]);
    let hole: Ring<P> = Ring::from_vec(vec![
        P::new(3.0, 3.0),
        P::new(7.0, 3.0),
        P::new(7.0, 7.0),
        P::new(3.0, 7.0),
        P::new(3.0, 3.0),
    ]);
    let donut = Polygon::with_inners(outer, vec![hole]);
    let filled = buffer_with(&donut, miter_settings(3.0)).unwrap();
    assert_eq!(filled.polygons().next().unwrap().interiors().count(), 0);
    assert!((buffered_area(&filled) - 256.0).abs() < 1e-9);

    let fully_eroded = buffer_with(&donut, miter_settings(-3.0)).unwrap();
    assert_eq!(fully_eroded.polygons().count(), 0);
}

/// `test/algorithms/buffer/buffer_with_strategies.cpp:88-106` — inapplicable
/// distance strategies and degenerate inputs are rejected consistently.
#[test]
#[expect(
    clippy::too_many_lines,
    reason = "one contract per geometry kind, read as a table"
)]
fn public_buffer_error_and_empty_contract_is_consistent_across_kinds() {
    let asymmetric = BufferSettings {
        distance: BufferDistanceStrategy::Asymmetric {
            left: 1.0,
            right: 2.0,
        },
        ..miter_settings(1.0)
    };
    let not_finite = BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(f64::NAN),
        ..miter_settings(1.0)
    };
    let zero = BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(0.0),
        ..miter_settings(1.0)
    };

    let point = P::new(0.0, 0.0);
    assert_eq!(
        buffer_with(&point, asymmetric),
        Err(OverlayError::Unsupported)
    );
    assert_eq!(
        buffer_with(&point, not_finite),
        Err(OverlayError::Unsupported)
    );
    assert!(buffer_with(&point, zero).unwrap().0.is_empty());

    let polygon: Polygon<P> =
        polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]];
    assert_eq!(
        buffer_with(&polygon, asymmetric),
        Err(OverlayError::Unsupported)
    );
    assert_eq!(
        buffer_with(&polygon, not_finite),
        Err(OverlayError::Unsupported)
    );
    // A zero-width buffer of a polygon is not an error and not a no-op: C++
    // runs the whole `buffer_inserter` pipeline, every side offsets onto
    // itself, and this square comes back unchanged. It is what
    // `repair_one_polygon` falls back on when the dissolve gives up.
    assert_eq!(
        buffer_with(&polygon, zero),
        Ok(MultiPolygon(vec![polygon![[
            (0.0, 0.0),
            (0.0, 2.0),
            (2.0, 2.0),
            (2.0, 0.0),
            (0.0, 0.0)
        ]]]))
    );
    // Where the offsetted rings *do* meet each other, what survives rests on
    // `check_turn_in_original` and the buffer traversal, neither of which is
    // ported. This bow tie's ring crosses itself, so the zero-width arm says so
    // rather than handing back an answer it cannot stand behind — the same
    // contract every other unported case gets.
    let self_crossing: Polygon<P> =
        polygon![[(0.0, 0.0), (2.0, 2.0), (2.0, 0.0), (0.0, 2.0), (0.0, 0.0)]];
    assert_eq!(
        buffer_with(&self_crossing, zero),
        Err(OverlayError::Unsupported)
    );

    assert!(
        buffer_convex_polygon(&polygon, 0.0, JoinStrategy::Miter)
            .exterior()
            .0
            .is_empty()
    );
    assert!(
        buffer_convex_polygon(&polygon, f64::NAN, JoinStrategy::Miter)
            .exterior()
            .0
            .is_empty()
    );
    let short: Polygon<P> = Polygon::new(Ring::<P>::from_vec(vec![point]));
    assert!(
        buffer_convex_polygon(&short, 1.0, JoinStrategy::Miter)
            .exterior()
            .0
            .is_empty()
    );
    let collinear: Polygon<P> = Polygon::new(Ring::<P>::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(1.0, 0.0),
        P::new(2.0, 0.0),
        P::new(0.0, 0.0),
    ]));
    assert!(
        !buffer_convex_polygon(&collinear, 1.0, JoinStrategy::Miter)
            .exterior()
            .0
            .is_empty()
    );

    let ring = polygon.exterior().clone();
    assert_eq!(
        buffer_with(&ring, asymmetric),
        Err(OverlayError::Unsupported)
    );
    assert_eq!(
        buffer_with(&ring, not_finite),
        Err(OverlayError::Unsupported)
    );
    assert_eq!(buffer_with(&ring, zero), Err(OverlayError::Unsupported));

    let line = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(2.0, 0.0)]);
    let negative = BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(-1.0),
        ..miter_settings(1.0)
    };
    assert_eq!(buffer_with(&line, negative), Err(OverlayError::Unsupported));
    assert_eq!(
        buffer_with(&line, not_finite),
        Err(OverlayError::Unsupported)
    );
    assert!(buffer_with(&line, zero).unwrap().0.is_empty());
    assert_eq!(
        buffer_with(&Linestring::from_vec(vec![point]), miter_settings(1.0)),
        Err(OverlayError::Unsupported)
    );
    assert_eq!(
        buffer_with(
            &Linestring::from_vec(vec![point, point]),
            miter_settings(1.0)
        ),
        Err(OverlayError::Unsupported)
    );
}

/// `test/algorithms/buffer/buffer_point.cpp:25-29` — the convenience entry
/// maps the public circle policy into the composed point strategy.
#[test]
fn convenience_point_circle_strategy_uses_public_dispatch() {
    let result = buffer(
        &P::new(0.0, 0.0),
        1.0,
        JoinStrategy::Round {
            points_per_circle: 16,
        },
        PointStrategy::Circle {
            points_per_circle: 72,
        },
    )
    .unwrap();
    assert_eq!(result.0[0].outer.0.len(), 73);
}

/// `test/algorithms/buffer/buffer_linestring.cpp:254-287` — round joins,
/// collinear vertices, and a zero-width side remain valid composed policies.
#[test]
fn linear_round_join_covers_bends_collinearity_and_zero_width_side() {
    let bent = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(2.0, 0.0), P::new(2.0, 2.0)]);
    let round = BufferSettings::round(1.0, 36);
    assert!(buffered_area(&buffer_with(&bent, round).unwrap()) > 0.0);

    let collinear =
        Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(1.0, 0.0), P::new(2.0, 0.0)]);
    assert!(buffered_area(&buffer_with(&collinear, round).unwrap()) > 0.0);
    assert!(buffered_area(&buffer_with(&collinear, miter_settings(1.0)).unwrap()) > 0.0);

    let one_sided = BufferSettings {
        distance: BufferDistanceStrategy::Asymmetric {
            left: 0.0,
            right: 1.0,
        },
        ..round
    };
    assert!(buffered_area(&buffer_with(&bent, one_sided).unwrap()) > 0.0);
}

/// `test/algorithms/buffer/buffer_polygon.cpp:823-846` — redundant and
/// collinear vertices exercise the offset kernel's degenerate-edge handling.
#[test]
fn areal_offset_handles_collinear_duplicate_and_collapsed_boundaries() {
    let collinear: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 2.0),
        (1.0, 2.0),
        (2.0, 2.0),
        (2.0, 0.0),
        (0.0, 0.0)
    ]];
    let mut limited = miter_settings(1.0);
    limited.join = BufferJoinStrategy::Miter { limit: 1.0 };
    assert!(buffered_area(&buffer_with(&collinear, limited).unwrap()) > 0.0);

    let duplicate: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(0.0, 2.0),
        P::new(0.0, 2.0),
        P::new(2.0, 2.0),
        P::new(2.0, 0.0),
        P::new(0.0, 0.0),
    ]));
    let _ = buffer_with(&duplicate, miter_settings(-0.25));

    let collapsed: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(1.0, 0.0),
        P::new(2.0, 0.0),
        P::new(0.0, 0.0),
    ]));
    let collapsed_result = buffer_with(&collapsed, miter_settings(1.0)).unwrap();
    assert_eq!(collapsed_result.0.len(), 1);
    assert!((buffered_area(&collapsed_result) - 1.0).abs() < 1e-12);

    let near_parallel: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(1.0, 0.0),
        P::new(2.0, 1e-20),
        P::new(2.0, 2.0),
        P::new(0.0, 2.0),
        P::new(0.0, 0.0),
    ]));
    assert!(
        !buffer_with(&near_parallel, miter_settings(1.0))
            .unwrap()
            .0
            .is_empty()
    );

    let exact_collapse: Polygon<P> =
        polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]];
    assert!(
        buffer_with(&exact_collapse, miter_settings(-1.0))
            .unwrap()
            .0
            .is_empty()
    );
}

/// `test/algorithms/buffer/buffer_point_geo.cpp:34-49` — the default
/// geographic coordinate strategy interprets buffer distance in metres and
/// constructs a geodesic point circle through the public facade.
#[test]
fn geographic_point_buffer_uses_wgs84_by_default() {
    type GeographicPoint = Point2D<f64, Geographic<Degree>>;

    let center = GeographicPoint::new(4.9, 52.0);
    let result = buffer_with(&center, BufferSettings::round(10.0, 360)).unwrap();
    let polygon = result.polygons().next().unwrap();
    assert_eq!(polygon.exterior().points().count(), 361);
    for point in polygon.exterior().points().take(360) {
        let distance = distance_with(&center, point, Vincenty::WGS84);
        assert!((distance - 10.0).abs() < 0.05);
    }
    let observed_area = area(polygon).abs();
    assert!((observed_area - 314.15).abs() < 314.15 * 0.005);
}

/// `strategies/buffer/spherical.hpp:24-58` — an explicit sphere radius is
/// carried by the spherical strategy bundle. The great-circle distance of
/// each generated vertex is the self-contained oracle because Boost has no
/// spherical buffer-algorithm fixture.
#[test]
fn spherical_point_buffer_honors_the_explicit_radius_strategy() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;

    let radius = 6_371_008.8;
    let center = SphericalPoint::new(-113.49, 53.54);
    let result = buffer_with_strategy(
        &center,
        BufferSettings::round(1_000.0, 72),
        SphericalBuffer::new(radius),
    )
    .unwrap();
    let polygon = result.polygons().next().unwrap();
    for point in polygon.exterior().points().take(72) {
        let distance = distance_with(&center, point, Haversine { radius });
        assert!((distance - 1_000.0).abs() < 0.5);
    }
}

/// `test/algorithms/buffer/buffer_geo_spheroid.cpp:107-121` — a caller can
/// replace WGS84 with the alternate spheroid used by Boost's oracle fixture.
#[test]
fn geographic_point_buffer_accepts_an_explicit_spheroid() {
    type GeographicPoint = Point2D<f64, Geographic<Degree>>;

    let spheroid = Spheroid {
        equatorial_radius: 6_378_000.0,
        flattening: (6_378_000.0 - 6_375_000.0) / 6_378_000.0,
    };
    let center = GeographicPoint::new(10.393_775_9, 63.430_232_3);
    let result = buffer_with_strategy(
        &center,
        BufferSettings::round(100.0, 360),
        GeographicBuffer::new(spheroid),
    )
    .unwrap();
    let polygon = result.polygons().next().unwrap();
    let distance_strategy = Vincenty {
        spheroid,
        max_iterations: 1_000,
        tolerance: 1e-12,
    };
    for point in polygon.exterior().points().take(360) {
        let distance = distance_with(&center, point, distance_strategy);
        assert!((distance - 100.0).abs() < 0.5);
    }
    let observed_area = area(polygon).abs();
    assert!((observed_area - 31_414.33).abs() < 31_414.33 * 0.005);
}

/// `test/algorithms/buffer/buffer_linestring_geo.cpp:15-64` and
/// `buffer_polygon_geo.cpp:15-55` — geographic linear and areal inputs use
/// the same five public strategy roles as Cartesian inputs.
#[test]
fn geographic_linear_and_areal_buffers_use_public_strategy_roles() {
    type GeographicPoint = Point2D<f64, Geographic<Degree>>;

    let line = Linestring::from_vec(vec![
        GeographicPoint::new(10.396_562_8, 63.427_678_6),
        GeographicPoint::new(10.395_313_4, 63.429_963_4),
    ]);
    let line_settings = BufferSettings {
        end: BufferEndStrategy::Flat,
        ..BufferSettings::round(5.0, 360)
    };
    let line_result = buffer_with(&line, line_settings).unwrap();
    let line_area: f64 = line_result
        .polygons()
        .map(|polygon| area(polygon).abs())
        .sum();
    assert!((line_area - 2_622.0).abs() < 35.0);

    let polygon: Polygon<GeographicPoint> = Polygon::new(Ring::from_vec(vec![
        GeographicPoint::new(10.400_658_7, 63.437_798_2),
        GeographicPoint::new(10.405_090_4, 63.439_599_3),
        GeographicPoint::new(10.407_499_4, 63.438_252_7),
        GeographicPoint::new(10.400_658_7, 63.437_798_2),
    ]));
    let polygon_result = buffer_with(&polygon, BufferSettings::round(5.0, 36)).unwrap();
    let polygon_area: f64 = polygon_result
        .polygons()
        .map(|polygon| area(polygon).abs())
        .sum();
    assert!((polygon_area - 32_940.0).abs() < 600.0);
}

/// `test/algorithms/buffer/buffer_multi_linestring_geo.cpp:18-73` and
/// `buffer_multi_polygon_geo.cpp:59-122` — every static geometry-kind arm is
/// available with an angular coordinate strategy, not only point/polygon.
#[test]
fn angular_segment_ring_box_and_multi_dispatch_is_public() {
    type GeographicPoint = Point2D<f64, Geographic<Degree>>;
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let spherical = SphericalBuffer::new(6_371_008.8);
    let round = BufferSettings::round(100.0, 36);

    let segment = Segment::new(
        SphericalPoint::new(-113.50, 53.54),
        SphericalPoint::new(-113.49, 53.54),
    );
    assert!(
        !buffer_with_strategy(&segment, round, spherical)
            .unwrap()
            .0
            .is_empty()
    );

    let ring: Ring<SphericalPoint> = Ring::from_vec(vec![
        SphericalPoint::new(-113.50, 53.53),
        SphericalPoint::new(-113.50, 53.54),
        SphericalPoint::new(-113.49, 53.54),
        SphericalPoint::new(-113.49, 53.53),
        SphericalPoint::new(-113.50, 53.53),
    ]);
    assert!(
        !buffer_with_strategy(&ring, round, spherical)
            .unwrap()
            .0
            .is_empty()
    );

    let bounds = ModelBox::from_corners(
        SphericalPoint::new(-113.50, 53.53),
        SphericalPoint::new(-113.49, 53.54),
    );
    assert!(
        !buffer_with_strategy(&bounds, round, spherical)
            .unwrap()
            .0
            .is_empty()
    );

    let points = MultiPoint::from_vec(vec![
        SphericalPoint::new(-113.50, 53.54),
        SphericalPoint::new(-113.48, 53.54),
    ]);
    assert_eq!(
        buffer_with_strategy(&points, round, spherical)
            .unwrap()
            .polygons()
            .count(),
        2
    );

    let lines = MultiLinestring::from_vec(vec![
        Linestring::from_vec(vec![
            GeographicPoint::new(10.396, 63.427),
            GeographicPoint::new(10.399, 63.428),
        ]),
        Linestring::from_vec(vec![
            GeographicPoint::new(10.406, 63.427),
            GeographicPoint::new(10.409, 63.428),
        ]),
    ]);
    assert_eq!(
        buffer_with(&lines, BufferSettings::round(5.0, 36))
            .unwrap()
            .polygons()
            .count(),
        2
    );

    let polygons: MultiPolygon<Polygon<GeographicPoint>> = MultiPolygon::from_vec(vec![
        Polygon::new(Ring::from_vec(vec![
            GeographicPoint::new(10.396, 63.427),
            GeographicPoint::new(10.396, 63.428),
            GeographicPoint::new(10.397, 63.428),
            GeographicPoint::new(10.397, 63.427),
            GeographicPoint::new(10.396, 63.427),
        ])),
        Polygon::new(Ring::from_vec(vec![
            GeographicPoint::new(10.406, 63.427),
            GeographicPoint::new(10.406, 63.428),
            GeographicPoint::new(10.407, 63.428),
            GeographicPoint::new(10.407, 63.427),
            GeographicPoint::new(10.406, 63.427),
        ])),
    ]);
    assert_eq!(
        buffer_with(&polygons, BufferSettings::round(5.0, 36))
            .unwrap()
            .polygons()
            .count(),
        2
    );
}

/// Angular projection failures are observable through the public buffer
/// contract: invalid radii/spheroids, poles, empty inputs, and invalid
/// projected members all return `Unsupported` instead of producing non-finite
/// coordinates.
#[test]
#[allow(
    clippy::too_many_lines,
    reason = "one public contract case covers every angular projection rejection path"
)]
fn angular_buffer_rejects_invalid_projection_inputs() {
    type GeographicPoint = Point2D<f64, Geographic<Degree>>;
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;

    let point = SphericalPoint::new(0.0, 0.0);
    let round = BufferSettings::round(100.0, 36);
    for radius in [f64::NAN, 0.0, -1.0] {
        assert!(matches!(
            buffer_with_strategy(&point, round, SphericalBuffer::new(radius)),
            Err(OverlayError::Unsupported)
        ));
    }
    for strategy in [SphericalBuffer::UNIT, SphericalBuffer::new(6_371_008.8)] {
        for pole in [
            SphericalPoint::new(0.0, 90.0),
            SphericalPoint::new(0.0, -90.0),
        ] {
            assert!(matches!(
                buffer_with_strategy(&pole, round, strategy),
                Err(OverlayError::Unsupported)
            ));
        }
    }

    let geographic = GeographicPoint::new(0.0, 0.0);
    for spheroid in [
        Spheroid {
            equatorial_radius: f64::NAN,
            flattening: 0.0,
        },
        Spheroid {
            equatorial_radius: 0.0,
            flattening: 0.0,
        },
        Spheroid {
            equatorial_radius: 1.0,
            flattening: f64::NAN,
        },
        Spheroid {
            equatorial_radius: 1.0,
            flattening: -0.1,
        },
        Spheroid {
            equatorial_radius: 1.0,
            flattening: 1.0,
        },
    ] {
        assert!(matches!(
            buffer_with_strategy(&geographic, round, GeographicBuffer::new(spheroid)),
            Err(OverlayError::Unsupported)
        ));
    }
    let geographic_pole = buffer_with_strategy(
        &GeographicPoint::new(0.0, 90.0),
        round,
        GeographicBuffer::WGS84,
    );
    assert!(matches!(geographic_pole, Err(OverlayError::Unsupported)));

    let spherical = SphericalBuffer::new(6_371_008.8);
    let empty_line = Linestring::<SphericalPoint>::from_vec(Vec::new());
    let empty_ring = Ring::<SphericalPoint>::from_vec(Vec::new());
    let empty_polygon = Polygon::<SphericalPoint>::new(Ring::from_vec(Vec::new()));
    let empty_points = MultiPoint::<SphericalPoint>::from_vec(Vec::new());
    let empty_lines = MultiLinestring::<Linestring<SphericalPoint>>::from_vec(Vec::new());
    let empty_polygons = MultiPolygon::<Polygon<SphericalPoint>>::from_vec(Vec::new());
    assert!(matches!(
        buffer_with_strategy(&empty_line, round, spherical),
        Err(OverlayError::Unsupported)
    ));
    assert!(matches!(
        buffer_with_strategy(&empty_ring, round, spherical),
        Err(OverlayError::Unsupported)
    ));
    assert!(matches!(
        buffer_with_strategy(&empty_polygon, round, spherical),
        Err(OverlayError::Unsupported)
    ));
    assert!(matches!(
        buffer_with_strategy(&empty_points, round, spherical),
        Err(OverlayError::Unsupported)
    ));
    assert!(matches!(
        buffer_with_strategy(&empty_lines, round, spherical),
        Err(OverlayError::Unsupported)
    ));
    assert!(matches!(
        buffer_with_strategy(&empty_polygons, round, spherical),
        Err(OverlayError::Unsupported)
    ));

    let short_line = Linestring::from_vec(vec![point]);
    let short_ring: Ring<SphericalPoint> = Ring::from_vec(vec![point]);
    let short_polygon = Polygon::new(short_ring.clone());
    assert!(matches!(
        buffer_with_strategy(&short_line, round, spherical),
        Err(OverlayError::Unsupported)
    ));
    let short_ring_result = buffer_with_strategy(&short_ring, round, spherical);
    let short_polygon_result = buffer_with_strategy(&short_polygon, round, spherical);
    assert!(short_ring_result.unwrap().0.is_empty());
    assert!(short_polygon_result.unwrap().0.is_empty());

    let valid_ring: Ring<SphericalPoint> = Ring::from_vec(vec![
        SphericalPoint::new(-0.1, -0.1),
        SphericalPoint::new(-0.1, 0.1),
        SphericalPoint::new(0.1, 0.1),
        SphericalPoint::new(0.1, -0.1),
        SphericalPoint::new(-0.1, -0.1),
    ]);
    let valid_polygon = Polygon::new(valid_ring.clone());
    let asymmetric = BufferSettings {
        distance: BufferDistanceStrategy::Asymmetric {
            left: 10.0,
            right: 20.0,
        },
        ..round
    };
    assert!(matches!(
        buffer_with_strategy(&valid_ring, asymmetric, spherical),
        Err(OverlayError::Unsupported)
    ));
    assert!(matches!(
        buffer_with_strategy(&valid_polygon, asymmetric, spherical),
        Err(OverlayError::Unsupported)
    ));
}

/// The local angular projection must choose the short path across the date
/// line and normalize every generated longitude back into the public range.
/// A holed areal input also exercises interior-ring reprojection.
#[test]
fn angular_buffer_wraps_antimeridian_and_reprojects_holes() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let spherical = SphericalBuffer::new(6_371_008.8);

    for longitude in [179.999, -179.999] {
        let result = buffer_with_strategy(
            &SphericalPoint::new(longitude, 0.0),
            BufferSettings::round(1_000.0, 72),
            spherical,
        )
        .unwrap();
        let ring = result.polygons().next().unwrap().exterior();
        assert!(
            ring.points()
                .all(|point| (-180.0..=180.0).contains(&point.x()))
        );
        assert!(ring.points().any(|point| point.x().is_sign_positive()));
        assert!(ring.points().any(|point| point.x().is_sign_negative()));
    }

    for longitudes in [[-170.0, -170.0, 170.0], [170.0, 170.0, -170.0]] {
        let line = Linestring::from_vec(
            longitudes
                .into_iter()
                .zip([0.0, 0.01, 0.02])
                .map(|(longitude, latitude)| SphericalPoint::new(longitude, latitude))
                .collect(),
        );
        assert!(
            !buffer_with_strategy(&line, BufferSettings::round(100.0, 36), spherical)
                .unwrap()
                .0
                .is_empty()
        );
    }

    let outer: Ring<SphericalPoint> = Ring::from_vec(vec![
        SphericalPoint::new(-0.1, -0.1),
        SphericalPoint::new(-0.1, 0.1),
        SphericalPoint::new(0.1, 0.1),
        SphericalPoint::new(0.1, -0.1),
        SphericalPoint::new(-0.1, -0.1),
    ]);
    let inner: Ring<SphericalPoint> = Ring::from_vec(vec![
        SphericalPoint::new(-0.04, -0.04),
        SphericalPoint::new(0.04, -0.04),
        SphericalPoint::new(0.04, 0.04),
        SphericalPoint::new(-0.04, 0.04),
        SphericalPoint::new(-0.04, -0.04),
    ]);
    let donut = Polygon::with_inners(outer, vec![inner]);
    let result = buffer_with_strategy(&donut, BufferSettings::round(100.0, 36), spherical).unwrap();
    assert_eq!(result.polygons().next().unwrap().interiors().count(), 1);
}

fn round_settings(distance: f64) -> BufferSettings {
    BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(distance),
        side: BufferSideStrategy::Straight,
        join: BufferJoinStrategy::Round {
            points_per_circle: 720,
        },
        end: BufferEndStrategy::Flat,
        point: BufferPointStrategy::Square,
    }
}

/// Boost 1.83 `join_round`: eroding the L by 0.5 leaves one valid polygon
/// of area 9.05366 — the arc at the reflex corner sweeps the short way,
/// on the eroded side, not through the material.
#[test]
fn round_erosion_of_an_l_shape_rounds_the_reflex_corner_inward() {
    let l: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 6.0),
        (2.0, 6.0),
        (2.0, 2.0),
        (6.0, 2.0),
        (6.0, 0.0),
        (0.0, 0.0)
    ]];
    let result = buffer_with(&l, round_settings(-0.5)).unwrap();
    assert_eq!(result.polygons().count(), 1, "{result:?}");
    assert!(
        (buffered_area(&result) - 9.053_66).abs() < 0.01,
        "area {}",
        buffered_area(&result)
    );
    assert_eq!(is_valid(&result), Ok(()));
}

/// Boost 1.83 `join_round`: growing a square with an L-shaped hole by
/// 0.5 keeps one valid polygon with one hole, area 115.732.
#[test]
fn round_growth_of_a_square_with_an_l_hole_is_valid() {
    let holed: Polygon<P> = polygon![
        [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0)
        ],
        [
            (2.0, 2.0),
            (6.0, 2.0),
            (6.0, 4.0),
            (4.0, 4.0),
            (4.0, 6.0),
            (2.0, 6.0),
            (2.0, 2.0)
        ]
    ];
    let result = buffer_with(&holed, round_settings(0.5)).unwrap();
    assert!(
        (buffered_area(&result) - 115.732).abs() < 0.01,
        "area {}",
        buffered_area(&result)
    );
    assert_eq!(result.polygons().count(), 1);
    assert_eq!(result.polygons().next().unwrap().interiors().count(), 1);
    assert_eq!(is_valid(&result), Ok(()));
}

/// Distance from `p` to the nearest edge of `pg`.
fn boundary_distance(p: (f64, f64), pg: &Polygon<P>) -> f64 {
    let mut best = f64::INFINITY;
    for ring in core::iter::once(pg.exterior()).chain(pg.interiors()) {
        let pts: Vec<(f64, f64)> = ring
            .points()
            .map(|q| (q.get::<0>(), q.get::<1>()))
            .collect();
        for w in pts.windows(2) {
            let (dx, dy) = (w[1].0 - w[0].0, w[1].1 - w[0].1);
            let t =
                (((p.0 - w[0].0) * dx + (p.1 - w[0].1) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            best = best
                .min(((p.0 - w[0].0 - t * dx).powi(2) + (p.1 - w[0].1 - t * dy).powi(2)).sqrt());
        }
    }
    best
}

/// The polygon miter limit (five times the distance by default) caps the
/// spike of a 5.7° apex, whose uncapped miter would sit about 20·d away.
#[test]
fn miter_limit_is_honoured_for_polygons() {
    let spike: Polygon<P> = polygon![[(0.0, 0.0), (5.0, 100.0), (10.0, 0.0), (0.0, 0.0)]];
    let settings = BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(1.0),
        side: BufferSideStrategy::Straight,
        join: BufferJoinStrategy::Miter { limit: 5.0 },
        end: BufferEndStrategy::Flat,
        point: BufferPointStrategy::Square,
    };
    let result = buffer_with(&spike, settings).unwrap();
    let farthest = result
        .polygons()
        .flat_map(|pg| pg.exterior().points())
        .map(|p| boundary_distance((p.get::<0>(), p.get::<1>()), &spike))
        .fold(0.0_f64, f64::max);
    assert!(
        farthest <= 5.0 + 1e-9,
        "a vertex sits {farthest} from the input, beyond the 5·d limit"
    );
}

// ---- Offsets whose offsetted ring crosses itself -------------------------
//
// Boost never trusts the offsetted ring: `buffered_piece_collection` finds
// the turns between the pieces and traverses them. Where a notch is narrower
// than twice the distance, or a neck thinner than that, the raw offsetted
// ring crosses itself, and the port rebuilds the offset from the same pieces
// through the overlay engine.

fn u_shape() -> Polygon<P> {
    polygon![[
        (0.0, 0.0),
        (0.0, 6.0),
        (2.0, 6.0),
        (2.0, 2.0),
        (4.0, 2.0),
        (4.0, 6.0),
        (6.0, 6.0),
        (6.0, 0.0),
        (0.0, 0.0)
    ]]
}

fn dumbbell() -> Polygon<P> {
    polygon![[
        (0.0, 0.0),
        (0.0, 10.0),
        (10.0, 10.0),
        (10.0, 5.5),
        (20.0, 5.5),
        (20.0, 10.0),
        (30.0, 10.0),
        (30.0, 0.0),
        (20.0, 0.0),
        (20.0, 4.5),
        (10.0, 4.5),
        (10.0, 0.0),
        (0.0, 0.0)
    ]]
}

/// Boost 1.83 `join_round`: growing the U by 1.5 closes its 2-wide notch
/// into one valid polygon of area 78.8281. The join arcs at the notch's two
/// top corners meet at `(3, 7.118)`, which is where the outline dips.
#[test]
fn round_growth_of_a_u_closes_the_notch() {
    let result = buffer_with(&u_shape(), round_settings(1.5)).unwrap();
    assert_eq!(result.polygons().count(), 1, "{result:?}");
    assert_eq!(result.polygons().next().unwrap().interiors().count(), 0);
    assert!(
        (buffered_area(&result) - 78.8281).abs() < 0.01,
        "area {}",
        buffered_area(&result)
    );
    assert_eq!(is_valid(&result), Ok(()));
    let grown = result.polygons().next().unwrap();
    assert!(covered_by(&P::new(3.0, 4.0), grown), "the notch is filled");
    assert!(covered_by(&P::new(3.0, 7.0), grown), "below the dip");
    assert!(!covered_by(&P::new(3.0, 7.3), grown), "above the dip");

    // A ring is a polygon without holes, and takes the same path.
    let as_ring = buffer_with(u_shape().exterior(), round_settings(1.5)).unwrap();
    assert!((buffered_area(&as_ring) - buffered_area(&result)).abs() < 1e-9);
    assert_eq!(is_valid(&as_ring), Ok(()));
}

/// Eroding the dumbbell by 1 removes its 1-wide neck and leaves the two
/// 10×10 lobes as separate valid polygons; nothing of either is within 1 of
/// the neck's walls once the neck is gone, so each lobe is the 8×8 core plus
/// the sliver the round join leaves between the arcs at the neck's two
/// reflex corners: 64 + 0.0434. C++ Boost 1.83 keeps only one of the two
/// lobes here — with exactly that area, 64.0434 — where the erosion keeps
/// both.
#[test]
fn round_erosion_of_a_dumbbell_splits_it_into_its_lobes() {
    let result = buffer_with(&dumbbell(), round_settings(-1.0)).unwrap();
    assert_eq!(result.polygons().count(), 2, "{result:?}");
    for lobe in result.polygons() {
        assert!(
            (area(lobe) - 64.0434).abs() < 1e-3,
            "lobe area {}",
            area(lobe)
        );
        assert_eq!(lobe.interiors().count(), 0);
    }
    assert_eq!(is_valid(&result), Ok(()));
    assert!(
        result
            .polygons()
            .any(|lobe| covered_by(&P::new(5.0, 5.0), lobe))
    );
    assert!(
        result
            .polygons()
            .any(|lobe| covered_by(&P::new(25.0, 5.0), lobe))
    );
    assert!(
        !result
            .polygons()
            .any(|lobe| covered_by(&P::new(15.0, 5.0), lobe))
    );
}

/// The same erosion with a miter join: the offset lines at each reflex
/// corner meet at the lobe's corner, so the lobes are exactly 8×8.
#[test]
fn miter_erosion_of_a_dumbbell_leaves_two_squares() {
    let result = buffer_with(&dumbbell(), miter_settings(-1.0)).unwrap();
    assert_eq!(result.polygons().count(), 2, "{result:?}");
    for lobe in result.polygons() {
        assert!((area(lobe) - 64.0).abs() < 1e-9, "lobe area {}", area(lobe));
    }
    assert_eq!(is_valid(&result), Ok(()));
}
