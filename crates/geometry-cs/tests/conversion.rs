//! Degree↔radian round-trip and WGS84 constants.
//!
//! Direct counterparts to the unit tests Boost.Geometry keeps under
//! `geometry/test/srs/` and the angle-conversion checks scattered
//! through `geometry/test/strategies/`.

use geometry_cs::{AngleUnit, Degree, FromF64, Radian, Spheroid};

#[test]
fn conversion_factor_scalars_accept_f64_constants() {
    assert_eq!(f32::from_f64(1.5).to_bits(), 1.5_f32.to_bits());
    assert_eq!(f64::from_f64(1.5).to_bits(), 1.5_f64.to_bits());
}

#[test]
fn degree_to_radian_180() {
    let r = Degree::to_radians(180.0_f64);
    assert!((r - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn degree_round_trip() {
    let v = 47.5_f64;
    let r = Degree::to_radians(v);
    let back = Degree::from_radians(r);
    assert!((back - v).abs() < 1e-12);
}

#[test]
fn radian_identity() {
    // `Radian::to_radians` / `from_radians` are the identity function
    // on `f64`, so a strict-equality check is the right test — there is
    // no rounding to tolerate.
    assert!((Radian::to_radians(1.0_f64) - 1.0).abs() < f64::EPSILON);
    assert!((Radian::from_radians(1.0_f64) - 1.0).abs() < f64::EPSILON);
}

#[test]
fn wgs84_constants() {
    let s = Spheroid::WGS84;
    assert!((s.equatorial_radius - 6_378_137.0).abs() < f64::EPSILON);
    assert!((s.flattening - 1.0 / 298.257_223_563).abs() < 1e-18);
    // Polar radius known value: 6_356_752.314_245 m.
    assert!((s.polar_radius() - 6_356_752.314_245).abs() < 1e-3);
}

#[test]
fn wgs84_eccentricity_squared() {
    // First eccentricity squared for WGS84 is ≈ 6.694_379_990_14e-3.
    let s = Spheroid::WGS84;
    assert!((s.eccentricity_squared() - 6.694_379_990_14e-3).abs() < 1e-12);
}
