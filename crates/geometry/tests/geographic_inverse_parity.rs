//! Public-facade parity tests for the Karney inverse geodesic.

#![allow(
    clippy::float_cmp,
    reason = "identity/default contracts require exact zero, unit, and copied-strategy values"
)]

use boost_geometry::adapt::{Adapt, WithCs};
use boost_geometry::cs::{Degree, Geographic};
use boost_geometry::prelude::distance_with;
use boost_geometry::strategy::geographic::{KarneyDirect, KarneyInverse, Meridian, Vincenty};

const D2R: f64 = core::f64::consts::PI / 180.0;
const R2D: f64 = 180.0 / core::f64::consts::PI;
type DegreePoint = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;

/// `test/formulas/inverse_cases.hpp:44-50` and
/// `test/formulas/inverse_karney.cpp:32-49` — WGS84 `(0°,0°)→(2°,2°)`.
#[test]
fn karney_inverse_matches_reference_case() {
    let result = KarneyInverse::WGS84.apply(0.0, 0.0, 2.0 * D2R, 2.0 * D2R);
    assert!((result.distance - 313_775.709_429_184_2).abs() < 1e-5);
    assert!((result.azimuth * R2D - 45.174_888_586_484_67).abs() < 1e-9);
    assert!((result.reverse_azimuth * R2D - 45.209_802_308_036_75).abs() < 1e-9);
}

/// `test/formulas/inverse_cases_antipodal.hpp:37-42` and
/// `test/formulas/inverse_karney.cpp:52-72` — a near-antipodal case that
/// requires Karney's globally convergent path.
#[test]
fn karney_inverse_converges_near_antipodal() {
    let result = KarneyInverse::WGS84.apply(
        0.0,
        31.394_417_440_639 * D2R,
        179.615_601_631_202_9 * D2R,
        -31.275_540_610_835_466 * D2R,
    );
    assert!((result.distance - 19_980_218.405_539_9).abs() < 0.02);
    assert!((result.azimuth * R2D - 34.266_322_930_672).abs() < 1e-7);
    assert!((result.reverse_azimuth * R2D - 145.782_701_113_414_3).abs() < 1e-7);
}

/// The same difficult case must be usable through the public algorithm and
/// strategy API, not only through the formula object.
#[test]
fn karney_inverse_is_a_public_distance_strategy() {
    let first = WithCs::<_, Geographic<Degree>>::new(Adapt([0.0, 31.394_417_440_639]));
    let second = WithCs::<_, Geographic<Degree>>::new(Adapt([
        179.615_601_631_202_9,
        -31.275_540_610_835_466,
    ]));
    let distance = distance_with(&first, &second, KarneyInverse::WGS84);

    assert!((distance - 19_980_218.405_539_9).abs() < 0.02);
}

/// `test/formulas/inverse_cases.hpp` includes coincident endpoints as the
/// identity case; the Rust strategy also exposes the comparable strategy.
#[test]
fn karney_inverse_coincident_default_and_comparable_contract() {
    let result = KarneyInverse::WGS84.apply(1.0, 0.5, 1.0, 0.5);
    assert_eq!(result.distance, 0.0);
    assert_eq!(result.azimuth, 0.0);
    assert_eq!(result.reverse_azimuth, 0.0);
    assert!(result.converged);
    assert_eq!(result.reduced_length, 0.0);
    assert_eq!(result.geodesic_scale, 1.0);

    let strategy = KarneyInverse::default();
    let comparable = <KarneyInverse as boost_geometry::strategy::DistanceStrategy<
        DegreePoint,
        DegreePoint,
    >>::comparable(&strategy);
    assert_eq!(comparable.max_iterations, strategy.max_iterations);
    assert_eq!(comparable.tolerance, strategy.tolerance);
}

/// Several Newton seeds can converge onto a geodesic through both points,
/// and only the shortest is the inverse solution. For this pair one seed
/// converges along a 22 032 km route — longer than half a meridian, which
/// no shortest geodesic exceeds — and it used to win on endpoint residual.
#[test]
fn karney_inverse_returns_the_shortest_converged_route() {
    let (lon1, lat1) = (-146.896_445_514_961_95, -49.473_038_132_649_11);
    let (lon2, lat2) = (7.895_230_699_299_105_5, 60.807_377_236_927_806);
    let result = KarneyInverse::WGS84.apply(lon1 * D2R, lat1 * D2R, lon2 * D2R, lat2 * D2R);
    assert!(result.converged);
    assert!(result.distance < 2.0 * Meridian::WGS84.quarter_length());

    let first = WithCs::<_, Geographic<Degree>>::new(Adapt([lon1, lat1]));
    let second = WithCs::<_, Geographic<Degree>>::new(Adapt([lon2, lat2]));
    let vincenty = distance_with(&first, &second, Vincenty::WGS84);
    assert!(
        (result.distance - vincenty).abs() < 1e-3,
        "karney {} vincenty {vincenty}",
        result.distance
    );
}

/// A geodesic ending at a pole runs along the start's meridian: its length
/// is the meridian arc and it leaves due north (due south). Karney orders
/// the endpoints so the one nearer a pole comes first; solving towards the
/// pole instead left the iteration singular and kilometres off. The
/// arrival azimuth is the one walking back from the pole retraces.
#[test]
fn karney_inverse_reaches_a_pole_along_the_meridian() {
    let meridian = Meridian::WGS84;
    for (lon1, lat1, lon2, lat2, azimuth) in [
        (10.0, 35.0, 40.0, 90.0, 0.0),
        (-20.0, -50.0, 70.0, -90.0, 180.0),
    ] {
        let result = KarneyInverse::WGS84.apply(lon1 * D2R, lat1 * D2R, lon2 * D2R, lat2 * D2R);
        let arc = (meridian.arc_length(lat2 * D2R) - meridian.arc_length(lat1 * D2R)).abs();
        assert!(result.converged);
        assert!(
            (result.distance - arc).abs() < 1e-6,
            "distance {} arc {arc}",
            result.distance
        );
        assert!(
            ((result.azimuth * R2D).abs() - azimuth).abs() < 1e-9,
            "azimuth {}",
            result.azimuth * R2D
        );

        let back = KarneyDirect::WGS84.apply(
            lon2 * D2R,
            lat2 * D2R,
            result.distance,
            result.reverse_azimuth + core::f64::consts::PI,
        );
        assert!(
            (back.lat2 * R2D - lat1).abs() < 1e-9,
            "lat {}",
            back.lat2 * R2D
        );
        assert!(
            (back.lon2 * R2D - lon1).abs() < 1e-9,
            "lon {}",
            back.lon2 * R2D
        );
    }

    // Short of the pole the target is still the ill-conditioned end.
    let (lon1, lat1, lon2, lat2) = (-48.385, -75.463, 45.443, 90.0 - 1e-6);
    let near = KarneyInverse::WGS84.apply(lon1 * D2R, lat1 * D2R, lon2 * D2R, lat2 * D2R);
    let vincenty = distance_with(
        &WithCs::<_, Geographic<Degree>>::new(Adapt([lon1, lat1])),
        &WithCs::<_, Geographic<Degree>>::new(Adapt([lon2, lat2])),
        Vincenty::WGS84,
    );
    assert!(near.converged);
    assert!((near.distance - vincenty).abs() < 1e-3);
}

/// The inverse of a geodesic walked by `KarneyDirect` returns its length
/// and azimuth, however short or near a pole: centimetres at mid
/// latitudes, where the cosine rule's `acos` seeded a zero length, and
/// metres from a pole, where an error measured as a longitude difference
/// scaled by the target's cosine left the solve nothing to hold. Both
/// came back as a distance of zero.
#[test]
fn karney_inverse_recovers_short_and_polar_geodesics() {
    let pi = core::f64::consts::PI;
    for (lon1, lat1, distance, azimuth) in [
        (-11.6, -51.1, 0.07, 75.0),
        (12.6, -57.7, 0.06, 90.0),
        (120.25, 13.64, 0.000_6, 16.4),
        (23.5, 89.999_997, 0.29, 150.0),
        (0.0, 90.0, 1.1, 30.0),
        (-158.8, -89.999_999_999, 0.002, 10.0),
    ] {
        let end = KarneyDirect::WGS84.apply(lon1 * D2R, lat1 * D2R, distance, azimuth * D2R);
        let result = KarneyInverse::WGS84.apply(lon1 * D2R, lat1 * D2R, end.lon2, end.lat2);
        assert!(result.converged, "({lon1}, {lat1})");
        assert!(
            (result.distance - distance).abs() < 1e-8,
            "({lon1}, {lat1}): {} m",
            result.distance
        );
        let turn = (result.azimuth - azimuth * D2R + pi).rem_euclid(2.0 * pi) - pi;
        assert!(
            (turn * distance).abs() < 1e-8,
            "({lon1}, {lat1}): azimuth {}",
            result.azimuth * R2D
        );
    }
}
