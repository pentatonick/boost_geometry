//! Reduced length and geodesic scale between spheroidal endpoints.
//!
//! Mirrors `boost::geometry::formula::differential_quantities` with the
//! maximum third-order flattening expansion exposed by the C++ header.

use geometry_cs::Spheroid;

/// Differential quantities attached to a direct or inverse geodesic result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialQuantities {
    /// Reduced geodesic length in the spheroid's radius unit.
    pub reduced_length: f64,
    /// Dimensionless forward geodesic scale.
    pub geodesic_scale: f64,
}

impl Default for DifferentialQuantities {
    fn default() -> Self {
        Self {
            reduced_length: 0.0,
            geodesic_scale: 1.0,
        }
    }
}

/// Calculate reduced length and geodesic scale for a solved geodesic.
#[cfg(feature = "std")]
#[must_use]
#[allow(
    clippy::too_many_arguments,
    clippy::many_single_char_names,
    clippy::similar_names,
    reason = "the inputs and symbols mirror Boost's differential formula"
)]
pub fn differential_quantities(
    longitude1: f64,
    latitude1: f64,
    longitude2: f64,
    latitude2: f64,
    azimuth: f64,
    reverse_azimuth: f64,
    spheroid: Spheroid,
) -> DifferentialQuantities {
    let longitude_difference = longitude2 - longitude1;
    let sin_latitude1 = latitude1.sin();
    let cos_latitude1 = latitude1.cos();
    let sin_latitude2 = latitude2.sin();
    let cos_latitude2 = latitude2.cos();
    let one_minus_f = 1.0 - spheroid.flattening;
    let mut sin_beta1 = one_minus_f * sin_latitude1;
    let mut sin_beta2 = one_minus_f * sin_latitude2;

    if sin_beta1.abs() <= 1e-15 && sin_beta2.abs() <= 1e-15 {
        let sigma12 = longitude_difference / one_minus_f;
        let azimuth_sign = if azimuth >= 0.0 { 1.0 } else { -1.0 };
        return DifferentialQuantities {
            reduced_length: azimuth_sign * sigma12.sin() * spheroid.polar_radius(),
            geodesic_scale: sigma12.cos(),
        };
    }

    let f = spheroid.flattening;
    let e2 = f * (2.0 - f);
    let ep2 = e2 / (one_minus_f * one_minus_f);
    let sin_alpha1 = azimuth.sin();
    let cos_alpha1 = azimuth.cos();
    let cos_alpha2 = reverse_azimuth.cos();
    let mut cos_beta1 = cos_latitude1;
    let mut cos_beta2 = cos_latitude2;
    normalize(&mut sin_beta1, &mut cos_beta1);
    normalize(&mut sin_beta2, &mut cos_beta2);
    let mut sin_sigma1 = sin_beta1;
    let mut cos_sigma1 = cos_alpha1 * cos_beta1;
    let mut sin_sigma2 = sin_beta2;
    let mut cos_sigma2 = cos_alpha2 * cos_beta2;
    normalize(&mut sin_sigma1, &mut cos_sigma1);
    normalize(&mut sin_sigma2, &mut cos_sigma2);
    let sin_alpha0 = sin_alpha1 * cos_beta1;
    let cos_alpha0_squared = 1.0 - sin_alpha0 * sin_alpha0;
    let j12 = j12_flattening(
        sin_sigma1,
        cos_sigma1,
        sin_sigma2,
        cos_sigma2,
        cos_alpha0_squared,
        f,
    );
    let dn1 = (1.0 + ep2 * sin_beta1 * sin_beta1).sqrt();
    let dn2 = (1.0 + ep2 * sin_beta2 * sin_beta2).sqrt();
    let reduced_length = spheroid.polar_radius()
        * (dn2 * cos_sigma1 * sin_sigma2
            - dn1 * sin_sigma1 * cos_sigma2
            - cos_sigma1 * cos_sigma2 * j12);
    let cos_sigma12 = cos_sigma1 * cos_sigma2 + sin_sigma1 * sin_sigma2;
    let t = ep2 * (cos_beta1 - cos_beta2) * (cos_beta1 + cos_beta2) / (dn1 + dn2);
    let geodesic_scale = cos_sigma12 + (t * sin_sigma2 - cos_sigma2 * j12) * sin_sigma1 / dn1;
    DifferentialQuantities {
        reduced_length,
        geodesic_scale,
    }
}

#[cfg(feature = "std")]
#[allow(
    clippy::similar_names,
    reason = "sigma-indexed symbols mirror Boost's differential formula"
)]
fn j12_flattening(
    sin_sigma1: f64,
    cos_sigma1: f64,
    sin_sigma2: f64,
    cos_sigma2: f64,
    cos_alpha0_squared: f64,
    flattening: f64,
) -> f64 {
    let sigma12 = (cos_sigma1 * sin_sigma2 - sin_sigma1 * cos_sigma2)
        .atan2(cos_sigma1 * cos_sigma2 + sin_sigma1 * sin_sigma2);
    let sin_2sigma1 = 2.0 * cos_sigma1 * sin_sigma1;
    let sin_2sigma2 = 2.0 * cos_sigma2 * sin_sigma2;
    let sin_2sigma12 = sin_2sigma2 - sin_2sigma1;
    let l1 = sigma12 - sin_2sigma12 / 2.0;
    let sin_4sigma1 = 2.0 * sin_2sigma1 * (cos_sigma1 * cos_sigma1 - sin_sigma1 * sin_sigma1);
    let sin_4sigma2 = 2.0 * sin_2sigma2 * (cos_sigma2 * cos_sigma2 - sin_sigma2 * sin_sigma2);
    let sin_4sigma12 = sin_4sigma2 - sin_4sigma1;
    let l2 = -(cos_alpha0_squared * sin_4sigma12
        + (-8.0 * cos_alpha0_squared + 12.0) * sin_2sigma12
        + (12.0 * cos_alpha0_squared - 24.0) * sigma12)
        / 16.0;
    let cos_alpha0_fourth = cos_alpha0_squared * cos_alpha0_squared;
    let sin_2sigma1_cubed = sin_2sigma1 * sin_2sigma1 * sin_2sigma1;
    let sin_2sigma2_cubed = sin_2sigma2 * sin_2sigma2 * sin_2sigma2;
    let l3 = ((9.0 * cos_alpha0_fourth - 12.0 * cos_alpha0_squared) * sin_4sigma12
        + 4.0 * cos_alpha0_fourth * (sin_2sigma2_cubed - sin_2sigma1_cubed)
        + (-48.0 * cos_alpha0_fourth + 96.0 * cos_alpha0_squared - 64.0) * sin_2sigma12
        + (60.0 * cos_alpha0_fourth - 144.0 * cos_alpha0_squared + 128.0) * sigma12)
        / 64.0;
    cos_alpha0_squared * flattening * (l1 + flattening * (l2 + flattening * l3))
}

#[cfg(feature = "std")]
fn normalize(x: &mut f64, y: &mut f64) {
    let length = x.hypot(*y);
    *x /= length;
    *y /= length;
}
