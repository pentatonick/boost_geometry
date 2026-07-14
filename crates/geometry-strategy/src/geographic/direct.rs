//! Shared result shape for direct geodesic formulas.
//!
//! Mirrors `boost::geometry::formula::result_direct<CT>` from
//! `formulas/result_direct.hpp:24-47`.

use geometry_cs::Spheroid;

/// Coordinates and reverse azimuth produced by a direct geodesic solution.
///
/// All angular values are radians. Mirrors `formula::result_direct` from
/// `formulas/result_direct.hpp:29-42`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectResult {
    /// Destination longitude in normalized radians, `[-π, π]`.
    pub lon2: f64,
    /// Destination latitude in radians.
    pub lat2: f64,
    /// Final/reverse azimuth in radians.
    pub reverse_azimuth: f64,
    /// Reduced geodesic length in the spheroid's radius unit.
    pub reduced_length: f64,
    /// Dimensionless forward geodesic scale.
    pub geodesic_scale: f64,
}

impl Default for DirectResult {
    fn default() -> Self {
        Self {
            lon2: 0.0,
            lat2: 0.0,
            reverse_azimuth: 0.0,
            reduced_length: 0.0,
            geodesic_scale: 1.0,
        }
    }
}

impl DirectResult {
    #[cfg(feature = "std")]
    pub(crate) fn solved(
        longitude1: f64,
        latitude1: f64,
        azimuth1: f64,
        spheroid: Spheroid,
        lon2: f64,
        lat2: f64,
        reverse_azimuth: f64,
    ) -> Self {
        let quantities = super::differential::differential_quantities(
            longitude1,
            latitude1,
            lon2,
            lat2,
            azimuth1,
            reverse_azimuth,
            spheroid,
        );
        Self {
            lon2,
            lat2,
            reverse_azimuth,
            reduced_length: quantities.reduced_length,
            geodesic_scale: quantities.geodesic_scale,
        }
    }
}

#[cfg(feature = "std")]
pub(crate) fn normalize_longitude(mut longitude: f64) -> f64 {
    let pi = core::f64::consts::PI;
    let two_pi = 2.0 * pi;
    while longitude > pi {
        longitude -= two_pi;
    }
    while longitude < -pi {
        longitude += two_pi;
    }
    longitude
}
