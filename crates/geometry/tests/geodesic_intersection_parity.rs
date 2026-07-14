//! Public-facade parity tests for spheroidal geodesic intersections.

#![allow(
    clippy::approx_constant,
    reason = "literal coordinates reproduce Boost's issue-612 regression input exactly"
)]

use boost_geometry::strategy::geographic::{Gnomonic, Sjoberg};

const D2R: f64 = core::f64::consts::PI / 180.0;
const R2D: f64 = 180.0 / core::f64::consts::PI;

fn radians(point: [f64; 2]) -> [f64; 2] {
    [point[0] * D2R, point[1] * D2R]
}

/// Fifth row of `test/formulas/intersection_cases.hpp`: both formula families
/// find the same WGS84 crossing of the two short geodesics.
#[test]
fn gnomonic_and_sjoberg_match_reference_crossing() {
    let first1 = radians([0.0, 0.0]);
    let first2 = radians([1.0, 1.0]);
    let second1 = radians([0.0, 1.0]);
    let second2 = radians([1.0, 0.0]);

    let gnomonic = Gnomonic::WGS84
        .intersection(first1, first2, second1, second2)
        .expect("short crossing is projectable");
    assert!((gnomonic.longitude * R2D - 0.5).abs() < 1e-10);
    assert!((gnomonic.latitude * R2D - 0.500_057_375_518_847).abs() < 1e-10);

    let sjoberg = Sjoberg::WGS84
        .intersection(first1, first2, second1, second2)
        .expect("short crossing has a Sjöberg solution");
    assert!((sjoberg.longitude * R2D - 0.500_000_000_031_326_6).abs() < 1e-8);
    assert!((sjoberg.latitude * R2D - 0.500_057_375_518_838_9).abs() < 1e-8);
}

/// `test/formulas/intersection.cpp:221-231`, regression for Boost issue 612.
#[test]
fn sjoberg_handles_nearly_equatorial_crossing() {
    let intersection = Sjoberg::WGS84
        .intersection(
            [-0.087_266_5, -0.087_266_5],
            [-0.087_266_5, 0.087_266_5],
            [0.0, 1.57e-7],
            [-0.392_699, 1.57e-7],
        )
        .expect("issue 612 crossing has a solution");
    assert!((intersection.longitude - (-0.087_266_500_535_674_75)).abs() < 1e-8);
    assert!((intersection.latitude - 1.589_249_913_962_292e-7).abs() < 1e-8);
}

/// Tenth row of `intersection_cases.hpp`, representative of the broad
/// Sjoberg-only table in Boost.
#[test]
fn sjoberg_handles_long_oblique_geodesics() {
    let intersection = Sjoberg::WGS84
        .intersection(
            radians([-25.0, -31.0]),
            radians([3.0, 44.0]),
            radians([-66.0, -14.0]),
            radians([-1.0, -1.0]),
        )
        .expect("long oblique geodesics have a solution");
    assert!((intersection.longitude * R2D - (-15.832_694_062_350_58)).abs() < 1e-7);
    assert!((intersection.latitude * R2D - (-4.877_464_502_624_33)).abs() < 1e-7);
}

/// Projection outside the gnomonic horizon and coincident projected lines do
/// not have finite, unique results. These are public `Option::None` outcomes,
/// not panics. The strategy defaults are WGS84.
#[test]
fn gnomonic_public_failure_contract_and_defaults_are_total() {
    assert_eq!(Gnomonic::default(), Gnomonic::WGS84);
    assert_eq!(Sjoberg::default(), Sjoberg::WGS84);

    assert!(
        Gnomonic::WGS84
            .forward([0.0, 0.0], [core::f64::consts::PI, 0.0])
            .is_none()
    );

    let first = radians([-1.0, 0.0]);
    let second = radians([1.0, 0.0]);
    assert!(
        Gnomonic::WGS84
            .intersection(first, second, first, second)
            .is_none()
    );

    assert!(
        Gnomonic::WGS84
            .reverse([0.0, 0.0], [f64::MAX, f64::MAX])
            .is_none()
    );
}
