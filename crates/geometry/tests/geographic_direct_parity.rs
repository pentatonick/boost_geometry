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
/// lat2 = 10.962936519310402°, reverse azimuth = −180°), and `+π`
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
        (minus.lat2 * R2D - 10.962_936_519_310_402).abs() < 1e-9,
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
