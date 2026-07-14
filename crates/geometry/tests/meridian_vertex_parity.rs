//! Public-facade parity tests for Boost's meridian and geodesic-vertex formulas.

use boost_geometry::cs::Spheroid;
use boost_geometry::strategy::geographic::{
    KarneyInverse, Meridian, MeridianSegmentKind, VincentyDirect, geographic_vertex_latitude,
    geographic_vertex_longitude, spherical_vertex_latitude, spherical_vertex_longitude,
};

const D2R: f64 = core::f64::consts::PI / 180.0;
const R2D: f64 = 180.0 / core::f64::consts::PI;

/// `test/formulas/direct_meridian_cases.hpp:31-43` and
/// `formulas/quarter_meridian.hpp:54-76`.
#[test]
fn wgs84_meridian_arc_and_direct_cases_match_boost() {
    let meridian = Meridian::WGS84;

    assert!((meridian.quarter_length() - 10_001_965.729_312_722).abs() < 1e-6);
    assert!((meridian.arc_length(10.0 * D2R) - 1_105_854.833_234_372_3).abs() < 1e-5);
    assert!((meridian.latitude_at_arc(1_105_854.833_234_372_3) * R2D - 10.0).abs() < 1e-9);

    let short = meridian.direct(0.0, 0.0, 100_000.0, true);
    let short_reference = VincentyDirect::WGS84.apply(0.0, 0.0, 100_000.0, 0.0);
    assert!(short.lon2.abs() < 1e-15);
    assert!((short.lat2 * R2D - 0.904_355_377_82).abs() < 2e-5);
    assert!((short.lat2 - short_reference.lat2).abs() < 1e-10);

    let across_pole = meridian.direct(10.0 * D2R, 0.0, 11_000_000.0, true);
    assert!((across_pole.lon2 * R2D + 170.0).abs() < 1e-9);
    assert!((across_pole.lat2 * R2D - 81.063_836_359).abs() < 1e-8);
    assert!((across_pole.reverse_azimuth - core::f64::consts::PI).abs() < 1e-15);
}

/// `formulas/meridian_segment.hpp:35-73` and
/// `formulas/meridian_inverse.hpp:72-112`.
#[test]
fn meridian_segment_classification_and_inverse_cover_both_paths() {
    let meridian = Meridian::WGS84;
    assert_eq!(
        meridian.classify_segment(0.0, -10.0 * D2R, 0.0, 20.0 * D2R),
        MeridianSegmentKind::NotCrossingPole,
    );
    assert_eq!(
        meridian.classify_segment(0.0, 80.0 * D2R, core::f64::consts::PI, 70.0 * D2R),
        MeridianSegmentKind::CrossingPole,
    );
    assert_eq!(
        meridian.classify_segment(0.0, 0.0, 20.0 * D2R, 10.0 * D2R),
        MeridianSegmentKind::NonMeridian,
    );

    let same = meridian.inverse(0.0, -10.0 * D2R, 0.0, 20.0 * D2R);
    assert!(same.meridian);
    assert!((same.distance - 3_318_221.087_406_006).abs() < 1e-5);

    let across = meridian.inverse(0.0, 80.0 * D2R, core::f64::consts::PI, 70.0 * D2R);
    assert!(across.meridian);
    let across_reference = KarneyInverse::WGS84
        .apply(0.0, 80.0 * D2R, core::f64::consts::PI, 70.0 * D2R)
        .distance;
    assert!((across.distance - 3_349_810.858_917_966_5).abs() < 1e-5);
    assert!((across.distance - across_reference).abs() < 1e-5);

    assert_eq!(Meridian::default(), Meridian::WGS84);
    assert_eq!(
        meridian.classify_segment(
            0.0,
            -core::f64::consts::FRAC_PI_2,
            1.0,
            core::f64::consts::FRAC_PI_2,
        ),
        MeridianSegmentKind::NotCrossingPole
    );
    assert_eq!(
        meridian.classify_segment(
            0.0,
            core::f64::consts::FRAC_PI_2,
            1.0,
            -core::f64::consts::FRAC_PI_2,
        ),
        MeridianSegmentKind::NotCrossingPole
    );

    let non_meridian = meridian.inverse(0.0, 0.0, 20.0 * D2R, 10.0 * D2R);
    assert!(!non_meridian.meridian);
    assert_eq!(non_meridian.distance.to_bits(), 0.0_f64.to_bits());

    let across_south = meridian.inverse(0.0, -80.0 * D2R, core::f64::consts::PI, -70.0 * D2R);
    assert!(across_south.meridian);
    assert!(across_south.distance > 3_000_000.0);

    let south = meridian.direct(10.0 * D2R, 0.0, 11_000_000.0, false);
    assert!((south.lon2 * R2D + 170.0).abs() < 1e-9);
    assert!(south.lat2 < 0.0);
    assert_eq!(south.reverse_azimuth.to_bits(), 0.0_f64.to_bits());
}

/// First row of `test/formulas/vertex_longitude_cases.hpp:38-45`.
#[test]
fn spherical_and_geographic_vertices_match_boost() {
    let lon1 = 1.0 * D2R;
    let lat1 = 1.0 * D2R;
    let lon2 = 100.0 * D2R;
    let lat2 = 2.0 * D2R;
    let dlon = lon2 - lon1;
    let spherical_azimuth = (dlon.sin() * lat2.cos())
        .atan2(lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos());
    let spherical_lat = spherical_vertex_latitude(lat1, spherical_azimuth);
    let spherical_lon = spherical_vertex_longitude(lon1, lat1, lon2, lat2, spherical_lat);
    assert!((spherical_lon * R2D - 66.397_442_08).abs() < 1e-8);

    let inverse = KarneyInverse::WGS84.apply(lon1, lat1, lon2, lat2);
    let geographic_lat = geographic_vertex_latitude(lat1, inverse.azimuth, Spheroid::WGS84);
    let geographic_lon = geographic_vertex_longitude(
        lon1,
        lat1,
        lon2,
        lat2,
        geographic_lat,
        inverse.azimuth,
        Spheroid::WGS84,
    );
    assert!((geographic_lon * R2D - 66.255_942_73).abs() < 2e-7);
}

/// Endpoint, meridian, and polar shortcuts from
/// `vertex_longitude_cases.hpp:207-243` are part of the public formula
/// contract and avoid unstable longitude arithmetic at singularities.
#[test]
fn geographic_vertex_longitude_handles_endpoints_meridians_and_poles() {
    let spheroid = Spheroid::WGS84;
    let endpoint1 = geographic_vertex_longitude(1.0, 0.2, 2.0, 0.3, 0.2, 0.5, spheroid);
    assert!((endpoint1 - 1.0).abs() < 1e-12);

    let endpoint2 = geographic_vertex_longitude(1.0, 0.2, 2.0, 0.3, 0.3, 0.5, spheroid);
    assert!((endpoint2 - 2.0).abs() < 1e-12);

    let meridian = geographic_vertex_longitude(1.0, 0.2, 1.0, 0.3, 0.4, 0.5, spheroid);
    assert!((meridian - 1.0).abs() < 1e-12);

    let polar = geographic_vertex_longitude(
        0.7,
        core::f64::consts::FRAC_PI_2,
        1.2,
        0.3,
        0.4,
        0.5,
        spheroid,
    );
    assert!(polar.is_finite());
}
