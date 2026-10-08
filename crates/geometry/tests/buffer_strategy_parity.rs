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
use boost_geometry::prelude::{area, area_with, covered_by, distance_with};
use boost_geometry::strategy::buffer::{
    BufferDistanceStrategy, BufferEndStrategy, BufferJoinStrategy, BufferPointStrategy,
    BufferSettings, BufferSideStrategy, GeographicBuffer, SphericalBuffer,
};
use boost_geometry::strategy::{GeographicPolygonArea, Haversine, Vincenty};
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

/// Boost (`aed7bc3`) buffers this segment by 0.5 with 36-point round ends to
/// `22.527_078_167_967_673`: each end is the half of a 36-gon that `end_round`
/// starts square to the segment.
#[test]
fn round_ends_place_their_points_as_boost_does() {
    let line = Linestring::from_vec(vec![P::new(8.8, -4.1), P::new(-9.655_996, 7.4)]);
    let settings = BufferSettings {
        end: BufferEndStrategy::Round {
            points_per_circle: 36,
        },
        ..miter_settings(0.5)
    };
    let result = buffer_with(&line, settings).unwrap();
    assert!((buffered_area(&result) - 22.527_078_167_967_673).abs() < 1e-9);
}

/// Boost (`aed7bc3`) buffers this bend by 2.5 with flat ends to
/// `90.032_096_103_794_36`. Its first segment is shorter than the distance,
/// so on the concave side of the bend that segment's piece reaches past the
/// end edge of the next one, and cutting across where the two offsets cross
/// would leave part of it out.
#[test]
fn a_concave_bend_after_a_short_segment_keeps_its_piece() {
    let line = Linestring::from_vec(vec![
        P::new(5.4, -8.6),
        P::new(3.920_18, -8.7),
        P::new(-5.115_997, 5.0),
    ]);
    let settings = BufferSettings {
        join: BufferJoinStrategy::Miter { limit: 3.0 },
        ..miter_settings(2.5)
    };
    let result = buffer_with(&line, settings).unwrap();
    assert!((buffered_area(&result) - 90.032_096_103_794_36).abs() < 1e-9);
    assert_eq!(is_valid(&result), Ok(()));
}

/// `test/strategies/buffer_join.cpp:58-93` — the configured limit shortens a
/// sharp miter instead of emitting an arbitrarily long one: the miter point
/// is drawn back along the miter to the limit, not cut off to a bevel.
/// Boost (`aed7bc3`) buffers this V to `32.189_576_864_903_55` at a limit of 2,
/// its miter point two below the vertex, and to `40.199_502_484_483_57` at a
/// limit of 20.
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
    assert!((buffered_area(&limited) - 32.189_576_864_903_55).abs() < 1e-9);
    assert!((buffered_area(&unlimited) - 40.199_502_484_483_57).abs() < 1e-9);
    assert!(
        limited
            .polygons()
            .flat_map(|polygon| polygon.exterior().points())
            .any(|point| point.x().abs() < 1e-12 && (point.y() + 2.0).abs() < 1e-12)
    );
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
/// distance strategies are rejected consistently. A linear geometry's
/// negative distance is its magnitude, and one that simplifies to a single
/// point is buffered as that point: Boost (`aed7bc3`) returns the 2 by 2
/// rectangle for the segment at -1, and the 2 by 2 square for one point or
/// two equal ones.
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
    assert_eq!(buffer_with(&point, zero).unwrap().0.len(), 0);

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

    assert_eq!(
        buffer_convex_polygon(&polygon, 0.0, JoinStrategy::Miter)
            .exterior()
            .0
            .len(),
        0
    );
    assert_eq!(
        buffer_convex_polygon(&polygon, f64::NAN, JoinStrategy::Miter)
            .exterior()
            .0
            .len(),
        0
    );
    let short: Polygon<P> = Polygon::new(Ring::<P>::from_vec(vec![point]));
    assert_eq!(
        buffer_convex_polygon(&short, 1.0, JoinStrategy::Miter)
            .exterior()
            .0
            .len(),
        0
    );
    let collinear: Polygon<P> = Polygon::new(Ring::<P>::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(1.0, 0.0),
        P::new(2.0, 0.0),
        P::new(0.0, 0.0),
    ]));
    assert_ne!(
        buffer_convex_polygon(&collinear, 1.0, JoinStrategy::Miter)
            .exterior()
            .0
            .len(),
        0
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
    // C++: a ring takes the polygon inserter, zero width included; Boost
    // (`aed7bc3`) returns this square unchanged.
    assert_eq!(
        buffer_with(&ring, zero),
        Ok(MultiPolygon(vec![Polygon::new(ring.clone())]))
    );

    let line = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(2.0, 0.0)]);
    let negative = BufferSettings {
        distance: BufferDistanceStrategy::Symmetric(-1.0),
        ..miter_settings(1.0)
    };
    assert!((buffered_area(&buffer_with(&line, negative).unwrap()) - 4.0).abs() < 1e-12);
    assert_eq!(
        buffer_with(&line, not_finite),
        Err(OverlayError::Unsupported)
    );
    assert_eq!(buffer_with(&line, zero).unwrap().0.len(), 0);
    for points in [vec![point], vec![point, point]] {
        let square = buffer_with(&Linestring::from_vec(points), miter_settings(1.0)).unwrap();
        assert_eq!(square.0.len(), 1);
        assert!((buffered_area(&square) - 4.0).abs() < 1e-12);
    }
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
/// An exterior that collapses onto a line is buffered as its first point:
/// Boost (`aed7bc3`) returns the 2 by 2 square about `(0 0)`.
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
    assert!((buffered_area(&collapsed_result) - 4.0).abs() < 1e-12);

    let near_parallel: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(1.0, 0.0),
        P::new(2.0, 1e-20),
        P::new(2.0, 2.0),
        P::new(0.0, 2.0),
        P::new(0.0, 0.0),
    ]));
    assert_ne!(
        buffer_with(&near_parallel, miter_settings(1.0))
            .unwrap()
            .0
            .len(),
        0
    );

    let exact_collapse: Polygon<P> =
        polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]];
    assert_eq!(
        buffer_with(&exact_collapse, miter_settings(-1.0))
            .unwrap()
            .0
            .len(),
        0
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
    // Measured on the spheroid it was built on, as Boost's fixture does.
    let observed_area = area_with(polygon, GeographicPolygonArea { spheroid }).abs();
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

    // C++: an input that simplifies to fewer points than its kind needs is
    // buffered as its first point (`buffer_point`).
    let short_line = Linestring::from_vec(vec![point]);
    let short_ring: Ring<SphericalPoint> = Ring::from_vec(vec![point]);
    let short_polygon = Polygon::new(short_ring.clone());
    for result in [
        buffer_with_strategy(&short_line, round, spherical),
        buffer_with_strategy(&short_ring, round, spherical),
        buffer_with_strategy(&short_polygon, round, spherical),
    ] {
        assert_eq!(result.unwrap().0.len(), 1);
    }

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
/// of area 9.05365 — the arc at the reflex corner sweeps the short way,
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
        (buffered_area(&result) - 9.053_65).abs() < 0.01,
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
/// into one valid polygon of area 78.8284. The join arcs at the notch's two
/// top corners meet at `(3, 7.118)`, which is where the outline dips.
#[test]
fn round_growth_of_a_u_closes_the_notch() {
    let result = buffer_with(&u_shape(), round_settings(1.5)).unwrap();
    assert_eq!(result.polygons().count(), 1, "{result:?}");
    assert_eq!(result.polygons().next().unwrap().interiors().count(), 0);
    assert!(
        (buffered_area(&result) - 78.8284).abs() < 0.01,
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

// ---- Offsets cut past a short side ---------------------------------------
//
// At a concave corner the offsetted ring cuts across from one side's offset
// to the next. Where a side is shorter than that cut reaches, the cut runs
// the side's offset backwards, or a side piece's end edge reaches past its
// neighbour, and the ring stops being the outline even where it does not
// cross itself. The port then takes Boost's pieces, as Boost always does.

fn settings_with(distance: f64, join: BufferJoinStrategy) -> BufferSettings {
    BufferSettings {
        join,
        ..round_settings(distance)
    }
}

fn assert_buffer(result: &MultiPolygon<Polygon<P>>, expected: f64, holes: usize) {
    assert_eq!(result.polygons().count(), 1, "{result:?}");
    assert_eq!(
        result
            .polygons()
            .map(|pg| pg.interiors().count())
            .sum::<usize>(),
        holes
    );
    assert!(
        (buffered_area(result) - expected).abs() < 1e-9,
        "area {}, expected {expected}",
        buffered_area(result)
    );
    assert_eq!(is_valid(result), Ok(()));
}

/// Boost (`aed7bc3`) grows this by 0.5 with eight-point round joins to
/// `10.580_367_381_025_582`. Two of its sides are shorter than the distance;
/// the offsetted ring left two per cent of that out.
#[test]
fn growth_past_short_sides_is_the_union_of_the_pieces() {
    let shape: Polygon<P> = polygon![[
        (-6.4, 2.5),
        (-6.566_818_999_161_166, 2.371_782_772_246_169),
        (-6.5, 8.2),
        (-6.560_855_686_493_813, 8.151_072_297_973_963),
        (-5.8, 9.5),
        (-6.4, 2.5)
    ]];
    let settings = settings_with(
        0.5,
        BufferJoinStrategy::Round {
            points_per_circle: 8,
        },
    );
    assert_buffer(
        &buffer_with(&shape, settings).unwrap(),
        10.580_367_381_025_582,
        0,
    );
}

/// Boost (`aed7bc3`) grows this by 4 with a miter limit of 3 to one valid
/// polygon of area `256.184_121_475_311_14`. The side ending at `(-9 -7)` is
/// shorter than the cut at that concave corner reaches back, and the
/// offsetted ring ran back along the side's offset there: the same area,
/// with a spike in it.
#[test]
fn growth_past_a_short_side_leaves_no_spike() {
    let shape: Polygon<P> = polygon![[
        (-6.737_086_948_835_929_5, -14.816_662_952_386_594),
        (-10.0, -9.0),
        (-11.984_049_610_381_488, -5.609_385_502_139_951),
        (-10.852_458_143_080_957, -4.878_138_148_479_655),
        (-9.0, -7.0),
        (-4.0, -4.0),
        (-6.737_086_948_835_929_5, -14.816_662_952_386_594)
    ]];
    let settings = settings_with(4.0, BufferJoinStrategy::Miter { limit: 3.0 });
    assert_buffer(
        &buffer_with(&shape, settings).unwrap(),
        256.184_121_475_311_14,
        0,
    );
}

/// Boost (`aed7bc3`) erodes this by 0.5 with a miter limit of 2 to
/// `29.797_469_475_250_836`, its hole grown past the short side the hole has.
#[test]
fn erosion_grows_a_hole_past_its_short_side() {
    let shape: Polygon<P> = polygon![
        [
            (15.2, 4.3),
            (0.041_836_548_975_474_84, 1.609_244_691_789_168_5),
            (3.1, 6.4),
            (7.013_137_685_154_756, 11.696_745_729_896_794),
            (8.3, 6.4),
            (15.2, 4.3)
        ],
        [
            (6.631_286_076_217_49, 5.594_946_028_540_917),
            (5.345_850_176_741_91, 5.609_624_744_510_454),
            (6.900_871_300_868_399, 5.200_781_123_379_323),
            (6.923_088_694_856_561, 5.059_718_959_559_267),
            (7.082_449_234_064_768, 5.522_827_214_008_680_5),
            (6.631_286_076_217_49, 5.594_946_028_540_917)
        ]
    ];
    let settings = settings_with(-0.5, BufferJoinStrategy::Miter { limit: 2.0 });
    assert_buffer(
        &buffer_with(&shape, settings).unwrap(),
        29.797_469_475_250_836,
        1,
    );
}

/// Boost (`aed7bc3`) grows this by 0.1 with a miter limit of 10 to one valid
/// polygon of area `92.462_201_326_511_63` that keeps its hole; the
/// offsetted ring came out invalid.
#[test]
fn growth_with_a_hole_past_a_short_side_is_valid() {
    let shape: Polygon<P> = polygon![
        [
            (16.8, 4.1),
            (16.821_684_256_298_3, 4.039_191_302_666_736),
            (6.1, -0.3),
            (6.510_783_277_232_39, 5.507_193_496_949_913),
            (2.428_859_239_636_780_7, 8.953_831_407_172_91),
            (6.864_165_432_083_775, 8.560_242_906_754_157),
            (6.863_587_454_500_993, 8.560_422_695_511_972),
            (7.440_266_662_929_202_5, 9.982_070_077_373_171),
            (12.182_022_051_071_634, 10.645_174_189_373_417),
            (15.817_000_531_412_84, 9.644_567_588_167_39),
            (16.8, 4.1)
        ],
        [
            (8.751_102_010_171_202, 5.534_845_101_794_879),
            (8.654_743_892_882_472, 3.921_324_784_548_790_6),
            (9.348_302_203_594_628, 4.913_883_015_138_034),
            (8.751_102_010_171_202, 5.534_845_101_794_879)
        ]
    ];
    let settings = settings_with(0.1, BufferJoinStrategy::Miter { limit: 10.0 });
    assert_buffer(
        &buffer_with(&shape, settings).unwrap(),
        92.462_201_326_511_63,
        1,
    );
}

/// Boost (`aed7bc3`) erodes this sliver by 2 to nothing. Simplifying at a
/// thousandth of the distance drops its second vertex, and what erodes is
/// the polygon as simplified: the sliver cut off between the two is no part
/// of the result.
#[test]
fn erosion_is_of_the_polygon_as_simplified() {
    let sliver: Polygon<P> = polygon![[
        (-4.178_386_634_617_558, 4.471_206_730_971_897),
        (-4.179_382_397_289_544, 4.471_136_390_557_469_5),
        (-0.967_280_172_426_481_7, 5.803_455_091_551_505_5),
        (10.032_117_302_295_983, 6.669_941_162_806_324_5),
        (-4.178_386_634_617_558, 4.471_206_730_971_897)
    ]];
    let settings = settings_with(
        -2.0,
        BufferJoinStrategy::Round {
            points_per_circle: 90,
        },
    );
    assert_eq!(buffer_with(&sliver, settings).unwrap().0.len(), 0);
}
