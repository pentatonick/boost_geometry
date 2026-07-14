//! Public-facade parity tests for geodesic differential quantities/results.

#![allow(
    clippy::float_cmp,
    reason = "default result contracts require exact initialized zero and unit values"
)]

use boost_geometry::cs::Spheroid;
use boost_geometry::strategy::geographic::{
    DifferentialQuantities, DirectResult, InverseResult, KarneyDirect, KarneyInverse,
    differential_quantities,
};

const D2R: f64 = core::f64::consts::PI / 180.0;

/// `formulas/result_direct.hpp:29-42` and
/// `formulas/result_inverse.hpp:31-48` — Rust's initialized result state uses
/// zero values, unit scale, and reports an unsolved inverse as unconverged.
#[test]
fn geodesic_result_defaults_are_initialized() {
    let quantities = DifferentialQuantities::default();
    assert_eq!(quantities.reduced_length, 0.0);
    assert_eq!(quantities.geodesic_scale, 1.0);

    let direct = DirectResult::default();
    assert_eq!(direct.lon2, 0.0);
    assert_eq!(direct.lat2, 0.0);
    assert_eq!(direct.reverse_azimuth, 0.0);
    assert_eq!(direct.reduced_length, 0.0);
    assert_eq!(direct.geodesic_scale, 1.0);

    let inverse = InverseResult::default();
    assert_eq!(inverse.distance, 0.0);
    assert_eq!(inverse.azimuth, 0.0);
    assert_eq!(inverse.reverse_azimuth, 0.0);
    assert!(!inverse.converged);
    assert_eq!(inverse.reduced_length, 0.0);
    assert_eq!(inverse.geodesic_scale, 1.0);
}

/// Equatorial special case from `formulas/differential_quantities.hpp:76-96`.
#[test]
fn equatorial_differential_quantities_match_closed_form() {
    let longitude_difference = 1.0 * D2R;
    let result = differential_quantities(
        0.0,
        0.0,
        longitude_difference,
        0.0,
        core::f64::consts::FRAC_PI_2,
        core::f64::consts::FRAC_PI_2,
        Spheroid::WGS84,
    );
    let sigma12 = longitude_difference / (1.0 - Spheroid::WGS84.flattening);
    let expected_reduced_length = Spheroid::WGS84.polar_radius() * sigma12.sin();

    assert!((result.reduced_length - expected_reduced_length).abs() < 1e-9);
    assert!((result.geodesic_scale - sigma12.cos()).abs() < 1e-15);
}

/// `result_direct.hpp:22-42`, `result_inverse.hpp:22-42`, and the Karney
/// formulas: public formula results retain the two differential fields.
#[test]
fn direct_and_inverse_results_expose_differential_fields() {
    let direct =
        KarneyDirect::WGS84.apply(0.0, 0.0, 313_775.709_429_184_2, 45.174_888_586_484_67 * D2R);
    let direct_quantities = differential_quantities(
        0.0,
        0.0,
        direct.lon2,
        direct.lat2,
        45.174_888_586_484_67 * D2R,
        direct.reverse_azimuth,
        Spheroid::WGS84,
    );
    assert!((direct.reduced_length - direct_quantities.reduced_length).abs() < 1e-6);
    assert!((direct.geodesic_scale - direct_quantities.geodesic_scale).abs() < 1e-12);

    let inverse = KarneyInverse::WGS84.apply(0.0, 0.0, 2.0 * D2R, 2.0 * D2R);
    assert!((inverse.reduced_length - direct.reduced_length).abs() < 1e-5);
    assert!((inverse.geodesic_scale - direct.geodesic_scale).abs() < 1e-12);
}
