//! Public-facade parity tests for direct geographic formulas.

use boost_geometry::cs::Spheroid;
use boost_geometry::strategy::geographic::{KarneyDirect, ThomasDirect, VincentyDirect};

const D2R: f64 = core::f64::consts::PI / 180.0;
const R2D: f64 = 180.0 / core::f64::consts::PI;

/// `test/formulas/direct_cases.hpp:45-53` and
/// `test/formulas/direct.cpp:78-89` — WGS84, `(0°,0°)`, 250 km at 45°.
#[test]
fn direct_formulas_match_reference_case() {
    assert_eq!(VincentyDirect::default().spheroid, Spheroid::WGS84);
    assert_eq!(KarneyDirect::default().spheroid, Spheroid::WGS84);

    let vincenty = VincentyDirect::WGS84.apply(0.0, 0.0, 250_000.0, 45.0 * D2R);
    assert!((vincenty.lon2 * R2D - 1.588_421_501_689_775_6).abs() < 1e-11);
    assert!((vincenty.lat2 * R2D - 1.598_504_192_670_177).abs() < 1e-11);
    assert!((vincenty.reverse_azimuth * R2D - 45.022_160_689_435_4).abs() < 1e-10);

    let thomas = ThomasDirect::WGS84.apply(0.0, 0.0, 250_000.0, 45.0 * D2R);
    assert!((thomas.lon2 * R2D - 1.588_421_499_588_542_6).abs() < 1e-10);
    assert!((thomas.lat2 * R2D - 1.598_504_190_565_435_4).abs() < 1e-10);
    assert!((thomas.reverse_azimuth * R2D - 45.022_160_689_377_01).abs() < 1e-9);

    let karney = KarneyDirect::WGS84.apply(0.0, 0.0, 250_000.0, 45.0 * D2R);
    assert!((karney.lon2 * R2D - 1.588_421_501_690_313_4).abs() < 1e-12);
    assert!((karney.lat2 * R2D - 1.598_504_192_671_097_7).abs() < 1e-12);
    assert!((karney.reverse_azimuth * R2D - 45.022_160_689_435_424).abs() < 1e-11);
}

/// `test/formulas/direct_cases.hpp:54-62` — an equatorial eastbound line
/// remains on the equator and both implementations normalize longitude.
#[test]
fn direct_equatorial_case_and_longitude_normalization() {
    let karney_equatorial =
        KarneyDirect::WGS84.apply(0.0, 0.0, 250_000.0, core::f64::consts::FRAC_PI_2);
    assert!((karney_equatorial.lon2 * R2D - 2.245_788_210_298_804).abs() < 1e-14);
    assert!(karney_equatorial.lat2.abs() < f64::EPSILON);
    assert!(
        (karney_equatorial.reverse_azimuth - core::f64::consts::FRAC_PI_2).abs() < f64::EPSILON
    );

    for result in [
        VincentyDirect::WGS84.apply(179.0 * D2R, 0.0, 250_000.0, 90.0 * D2R),
        ThomasDirect::WGS84.apply(179.0 * D2R, 0.0, 250_000.0, 90.0 * D2R),
        KarneyDirect::WGS84.apply(179.0 * D2R, 0.0, 250_000.0, 90.0 * D2R),
    ] {
        assert!(result.lat2.abs() < 1e-12);
        assert!(result.lon2 >= -core::f64::consts::PI);
        assert!(result.lon2 <= core::f64::consts::PI);
    }
}

/// `test/formulas/direct_cases.hpp` cases at ±135° and 180° exercise
/// Thomas's southward reflection, while the 0° case uses its meridian arm.
#[test]
fn thomas_direct_covers_reflections_meridians_poles_and_first_order() {
    let north = ThomasDirect::WGS84.apply(0.0, 0.0, 250_000.0, 0.0);
    assert!((north.lon2 * R2D).abs() < 1e-12);
    assert!((north.lat2 * R2D - 2.260_911_893_866_417_4).abs() < 1e-10);

    let southeast = ThomasDirect::WGS84.apply(0.0, 0.0, 250_000.0, 135.0 * D2R);
    assert!((southeast.lon2 * R2D - 1.588_421_499_588_542_6).abs() < 1e-10);
    assert!((southeast.lat2 * R2D + 1.598_504_190_565_436).abs() < 1e-10);

    let southwest = ThomasDirect::WGS84.apply(0.0, 0.0, 250_000.0, -135.0 * D2R);
    assert!((southwest.lon2 * R2D + 1.588_421_499_588_542_6).abs() < 1e-10);
    assert!((southwest.lat2 * R2D + 1.598_504_190_565_436).abs() < 1e-10);

    let south = ThomasDirect::WGS84.apply(0.0, 0.0, 250_000.0, core::f64::consts::PI);
    assert!((south.lat2 * R2D + 2.260_911_893_866_417_4).abs() < 1e-10);

    for latitude in [core::f64::consts::FRAC_PI_2, -core::f64::consts::FRAC_PI_2] {
        let result = ThomasDirect::WGS84.apply(0.0, latitude, 1_000.0, 0.0);
        assert!(result.lon2.is_finite());
        assert!(result.lat2.is_finite());
    }

    let first_order = ThomasDirect {
        spheroid: Spheroid::WGS84,
        second_order: false,
    }
    .apply(0.0, 0.0, 250_000.0, 45.0 * D2R);
    assert!((first_order.lon2 * R2D - 1.588_421_500_757_703_6).abs() < 1e-10);
    assert!((first_order.lat2 * R2D - 1.598_504_194_061_877_2).abs() < 1e-10);

    let default = ThomasDirect::default();
    assert!(default.second_order);
    assert_eq!(default.spheroid, Spheroid::WGS84);
}

/// `formulas/thomas_direct.hpp` — a due-south course spelled as `-π`
/// keeps the sign of its reverse azimuth (Boost 1.83: lon2 = 10°,
/// lat2 = 10.962936519310404°, reverse azimuth = −180°), and `+π`
/// reaches the same latitude with a `+180°` reverse azimuth.
#[test]
fn thomas_direct_negative_pi_azimuth_is_due_south() {
    let minus =
        ThomasDirect::WGS84.apply(10.0 * D2R, 20.0 * D2R, 1_000_000.0, -core::f64::consts::PI);
    let plus =
        ThomasDirect::WGS84.apply(10.0 * D2R, 20.0 * D2R, 1_000_000.0, core::f64::consts::PI);
    assert!(
        (minus.lon2 * R2D - 10.0).abs() < 1e-9,
        "lon2 {}",
        minus.lon2 * R2D
    );
    assert!(
        (minus.lat2 * R2D - 10.962_936_519_310_404).abs() < 1e-9,
        "lat2 {}",
        minus.lat2 * R2D
    );
    assert!(
        (minus.reverse_azimuth * R2D + 180.0).abs() < 1e-9,
        "reverse azimuth {}",
        minus.reverse_azimuth * R2D
    );
    assert!((minus.lat2 - plus.lat2).abs() < 1e-12);
    assert!((plus.reverse_azimuth * R2D - 180.0).abs() < 1e-9);
    let vincenty =
        VincentyDirect::WGS84.apply(10.0 * D2R, 20.0 * D2R, 1_000_000.0, -core::f64::consts::PI);
    assert!((minus.lat2 - vincenty.lat2).abs() * R2D < 1e-6);
}

/// `formulas/thomas_direct.hpp:94,130,179` test the pole, `sin θ0 = 0`, and
/// `M = 0` cases with `math::equals`, not exactly. From a pole `cos θ1` is
/// about 6e-17 rather than 0, so `M` must count as zero there; dividing by
/// it sent the destination latitude kilometres off. Boost: lon2 = −126°,
/// lat2 = 0.017 777 348 844 594°, reverse azimuth = −180° from the north
/// pole, and lon2 = 150°, lat2 = −45.153 161 682 740 134° from the south.
#[test]
fn thomas_direct_from_a_pole_follows_the_meridian() {
    let north = ThomasDirect::WGS84.apply(0.0, 90.0 * D2R, 10_000_000.0, -54.0 * D2R);
    assert!(
        (north.lon2 * R2D + 126.0).abs() < 1e-9,
        "lon2 {}",
        north.lon2 * R2D
    );
    assert!(
        (north.lat2 * R2D - 0.017_777_348_844_594).abs() < 1e-12,
        "lat2 {}",
        north.lat2 * R2D
    );
    assert!((north.reverse_azimuth * R2D + 180.0).abs() < 1e-9);

    let south = ThomasDirect::WGS84.apply(30.0 * D2R, -90.0 * D2R, 5_000_000.0, 120.0 * D2R);
    assert!(
        (south.lon2 * R2D - 150.0).abs() < 1e-9,
        "lon2 {}",
        south.lon2 * R2D
    );
    assert!(
        (south.lat2 * R2D + 45.153_161_682_740_134).abs() < 1e-12,
        "lat2 {}",
        south.lat2 * R2D
    );

    // Thomas is a second-order series; Karney is exact to round-off.
    let karney = KarneyDirect::WGS84.apply(0.0, 90.0 * D2R, 10_000_000.0, -54.0 * D2R);
    assert!((north.lat2 - karney.lat2).abs() * R2D < 1e-6);
}

/// Boost normalizes the destination longitude only after computing the
/// differential quantities (`vincenty_direct.hpp:169-176`). Folded first, an
/// equatorial line across the antimeridian measured its arc a turn short, a
/// turn the division by `1 − f` no longer cancels: 863 km of reduced length
/// instead of 996 km. Turning the start about the axis changes neither
/// quantity. Boost (`aed7bc3`): 995 880.535 435 932 6 m and
/// 0.987 651 801 508 027 2 from 3 rad, as from 0.
#[test]
fn equatorial_lines_across_the_antimeridian_keep_their_quantities() {
    let east = core::f64::consts::FRAC_PI_2;
    for (crossing, from_zero) in [
        (
            VincentyDirect::WGS84.apply(3.0, 0.0, 1_000_000.0, east),
            VincentyDirect::WGS84.apply(0.0, 0.0, 1_000_000.0, east),
        ),
        (
            ThomasDirect::WGS84.apply(3.0, 0.0, 1_000_000.0, east),
            ThomasDirect::WGS84.apply(0.0, 0.0, 1_000_000.0, east),
        ),
        (
            KarneyDirect::WGS84.apply(3.0, 0.0, 1_000_000.0, east),
            KarneyDirect::WGS84.apply(0.0, 0.0, 1_000_000.0, east),
        ),
    ] {
        assert!(crossing.lon2 < -3.0, "lon2 {}", crossing.lon2);
        assert!((crossing.reduced_length - 995_880.535_435_932_6).abs() < 1e-6);
        assert!((crossing.reduced_length - from_zero.reduced_length).abs() < 1e-6);
        assert!((crossing.geodesic_scale - 0.987_651_801_508_027_2).abs() < 1e-12);
        assert!((crossing.geodesic_scale - from_zero.geodesic_scale).abs() < 1e-12);
    }
}

/// Boost brings the destination longitude into `(−π, π]`: due north from
/// `−180°`, each formula arrives at `+180°` (Boost `aed7bc3`).
#[test]
fn a_line_from_the_negative_antimeridian_arrives_at_positive_180() {
    let pi = core::f64::consts::PI;
    for result in [
        VincentyDirect::WGS84.apply(-pi, 0.5, 1_000.0, 0.0),
        ThomasDirect::WGS84.apply(-pi, 0.5, 1_000.0, 0.0),
        KarneyDirect::WGS84.apply(-pi, 0.5, 1_000.0, 0.0),
    ] {
        assert_eq!(result.lon2, pi);
    }
}

/// Boost instantiates `differential_quantities` at the second order for
/// the Vincenty and Thomas formulas (`vincenty_direct.hpp:162`,
/// `thomas_direct.hpp:202`); the third order is 18 mm off here. Boost
/// (`aed7bc3`) gives these.
#[test]
fn vincenty_and_thomas_quantities_take_the_second_order() {
    let (lon1, lat1, distance, azimuth) = (
        0.872_819_066_926_347_8,
        -0.382_807_371_821_806_44,
        1_109_197.692_814_7,
        -2.747_077_888_584_542_4,
    );
    let vincenty = VincentyDirect::WGS84.apply(lon1, lat1, distance, azimuth);
    assert!((vincenty.reduced_length - 1_103_592.658_879_181_1).abs() < 1e-6);
    assert!((vincenty.geodesic_scale - 0.984_851_375_357_535_1).abs() < 1e-12);
    let thomas = ThomasDirect::WGS84.apply(lon1, lat1, distance, azimuth);
    assert!((thomas.reduced_length - 1_103_592.662_260_444).abs() < 1e-6);
    assert!((thomas.geodesic_scale - 0.984_851_375_264_021_8).abs() < 1e-12);
}

/// Boost's Karney formula takes its reduced length and geodesic scale from
/// its own order-8 series (`karney_direct.hpp:219-249`), not from the
/// flattening expansion the other formulas use, which is a millimetre off
/// over 15 000 km. Boost (`aed7bc3`) gives these.
#[test]
fn karney_quantities_follow_its_own_series() {
    let result = KarneyDirect::WGS84.apply(0.2, 0.5, 15_000_000.0, 0.7);
    assert!((result.reduced_length - 4_532_361.710_864_253).abs() < 1e-6);
    assert!((result.geodesic_scale + 0.703_318_048_269_265_1).abs() < 1e-12);
}
