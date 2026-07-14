//! Public-facade parity tests for spheroidal box-coordinate normalization.

use boost_geometry::cs::{Degree, normalize_spheroidal_box_coordinates};

fn assert_close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 1e-12);
}

/// `util/normalize_spheroidal_box_coordinates.hpp:51-81` — normalized
/// antimeridian boxes keep an increasing longitude interval by extending the
/// maximum longitude into the following turn.
#[test]
fn antimeridian_box_becomes_an_increasing_interval() {
    let (mut lon1, mut lat1, mut lon2, mut lat2) = (170.0, -10.0, -170.0, 20.0);
    normalize_spheroidal_box_coordinates::<Degree, _>(&mut lon1, &mut lat1, &mut lon2, &mut lat2);
    assert_close(lon1, 170.0);
    assert_close(lat1, -10.0);
    assert_close(lon2, 190.0);
    assert_close(lat2, 20.0);
}

/// `util/normalize_spheroidal_box_coordinates.hpp:37-42,69-75` — an input
/// spanning at least one full turn represents a complete longitude band.
#[test]
fn full_turn_box_normalizes_to_a_longitude_band() {
    let (mut lon1, mut lat1, mut lon2, mut lat2) = (-200.0, -10.0, 200.0, 20.0);
    normalize_spheroidal_box_coordinates::<Degree, _>(&mut lon1, &mut lat1, &mut lon2, &mut lat2);
    assert_close(lon1, -180.0);
    assert_close(lat1, -10.0);
    assert_close(lon2, 180.0);
    assert_close(lat2, 20.0);
}

/// `util/normalize_spheroidal_box_coordinates.hpp:57-68` — longitude is
/// irrelevant when both corners collapse to the same pole.
#[test]
fn polar_box_canonicalizes_both_longitudes_to_zero() {
    let (mut lon1, mut lat1, mut lon2, mut lat2) = (20.0, 90.0, 160.0, 90.0);
    normalize_spheroidal_box_coordinates::<Degree, _>(&mut lon1, &mut lat1, &mut lon2, &mut lat2);
    assert_close(lon1, 0.0);
    assert_close(lat1, 90.0);
    assert_close(lon2, 0.0);
    assert_close(lat2, 90.0);
}

/// `test/util/math_normalize_spheroidal.cpp:74-80` — each finite ordinate is
/// normalized independently while a NaN ordinate remains a NaN.
#[test]
fn box_normalization_preserves_nan_and_normalizes_the_other_longitude() {
    let (mut lon1, mut lat1, mut lon2, mut lat2) = (f64::NAN, 10.0, 270.0, 20.0);
    normalize_spheroidal_box_coordinates::<Degree, _>(&mut lon1, &mut lat1, &mut lon2, &mut lat2);
    assert!(lon1.is_nan());
    assert_close(lat1, 10.0);
    assert_close(lon2, -90.0);
    assert_close(lat2, 20.0);
}

/// `test/util/math_normalize_spheroidal.cpp:84-90` — the same normalization
/// contract applies to integral degree coordinates without a floating cast.
#[test]
fn degree_box_normalization_supports_integral_coordinates() {
    let (mut lon1, mut lat1, mut lon2, mut lat2) = (170_i32, -10_i32, -170_i32, 20_i32);
    normalize_spheroidal_box_coordinates::<Degree, _>(&mut lon1, &mut lat1, &mut lon2, &mut lat2);
    assert_eq!((lon1, lat1, lon2, lat2), (170, -10, 190, 20));
}
