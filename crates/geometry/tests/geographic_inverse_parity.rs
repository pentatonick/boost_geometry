//! Public-facade parity tests for the Karney inverse geodesic.

#![allow(
    clippy::float_cmp,
    reason = "identity/default contracts require exact zero, unit, and copied-strategy values"
)]

use boost_geometry::adapt::{Adapt, WithCs};
use boost_geometry::cs::{Degree, Geographic};
use boost_geometry::prelude::distance_with;
use boost_geometry::strategy::geographic::KarneyInverse;

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
