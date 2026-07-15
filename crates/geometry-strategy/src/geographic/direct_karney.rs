//! Karney eighth-order solution of the direct geodesic problem.
//!
//! Mirrors `boost::geometry::formula::karney_direct` from
//! `formulas/karney_direct.hpp:52-255`, using the order-8 coefficient tables
//! from [`geometry_coords::series_expansion`].

use geometry_cs::Spheroid;

use super::direct::DirectResult;

#[cfg(feature = "std")]
use geometry_coords::series_expansion::{
    coefficients_a3, coefficients_c1, coefficients_c1p, coefficients_c3, evaluate_a1,
    sin_cos_series,
};

#[cfg(feature = "std")]
use super::direct::normalize_longitude;
#[cfg(feature = "std")]
use super::spheroid_calc::SpheroidCalc;

/// Karney's eighth-order direct geodesic formula.
///
/// Given a longitude/latitude, distance, and initial azimuth, computes the
/// destination and final azimuth. Inputs and outputs are radians; distance
/// uses the spheroid radius unit. Mirrors `formula::karney_direct<CT, ..., 8>`
/// from `formulas/karney_direct.hpp:52-255`.
#[derive(Debug, Clone, Copy)]
pub struct KarneyDirect {
    /// Reference ellipsoid.
    pub spheroid: Spheroid,
}

impl KarneyDirect {
    /// Eighth-order Karney direct on WGS84.
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };

    /// Solve the direct geodesic problem.
    ///
    /// Mirrors `karney_direct::apply` at
    /// `formulas/karney_direct.hpp:76-253`. The Rust calculation stays in
    /// radians instead of temporarily converting angles to degrees; both
    /// forms evaluate the same trigonometric quantities, while the final
    /// longitude is normalized identically.
    #[cfg(feature = "std")]
    #[inline]
    #[must_use]
    #[allow(
        clippy::many_single_char_names,
        clippy::similar_names,
        clippy::float_cmp,
        reason = "names and exact zero branches follow the cited Karney formula"
    )]
    pub fn apply(&self, lon1: f64, lat1: f64, distance: f64, azimuth12: f64) -> DirectResult {
        let calc = SpheroidCalc::from(self.spheroid);
        let b = calc.b;
        let f = calc.f;
        let one_minus_f = 1.0 - f;
        let two_minus_f = 2.0 - f;
        let n = f / two_minus_f;
        let e2 = f * two_minus_f;
        let ep2 = e2 / (one_minus_f * one_minus_f);

        let sin_alpha1 = azimuth12.sin();
        let sin_alpha1 = if sin_alpha1.abs() <= f64::EPSILON {
            0.0
        } else {
            sin_alpha1
        };
        let cos_alpha1 = azimuth12.cos();
        let cos_alpha1 = if cos_alpha1.abs() <= f64::EPSILON {
            0.0
        } else {
            cos_alpha1
        };
        let mut sin_beta1 = lat1.sin() * one_minus_f;
        let mut cos_beta1 = lat1.cos();
        let beta_norm = sin_beta1.hypot(cos_beta1);
        sin_beta1 /= beta_norm;
        cos_beta1 = (cos_beta1 / beta_norm).max(0.0);

        let sin_alpha0 = sin_alpha1 * cos_beta1;
        let cos_alpha0 = cos_alpha1.hypot(sin_alpha1 * sin_beta1);
        let k2 = cos_alpha0 * cos_alpha0 * ep2;
        let epsilon = k2 / (2.0 * (1.0 + (1.0 + k2).sqrt()) + k2);
        let expansion_a1 = evaluate_a1(epsilon);
        let coefficients_c1 = coefficients_c1(epsilon);
        let tau12 = distance / (b * (1.0 + expansion_a1));
        let sin_tau12 = tau12.sin();
        let cos_tau12 = tau12.cos();

        let mut sin_sigma1 = sin_beta1;
        let sin_omega1 = sin_alpha0 * sin_beta1;
        // Boost evaluates these terms with `sin_cos_degrees`, which returns an
        // exact zero for the equatorial due-east/west cases. The radian
        // trigonometric functions leave a sub-epsilon residue instead.
        let mut cos_sigma1 = if sin_beta1.abs() > f64::EPSILON || cos_alpha1.abs() > f64::EPSILON {
            cos_beta1 * cos_alpha1
        } else {
            1.0
        };
        let cos_omega1 = cos_sigma1;
        let sigma_norm = sin_sigma1.hypot(cos_sigma1);
        sin_sigma1 /= sigma_norm;
        cos_sigma1 /= sigma_norm;

        let b11 = sin_cos_series(sin_sigma1, cos_sigma1, &coefficients_c1);
        let sin_b11 = b11.sin();
        let cos_b11 = b11.cos();
        let sin_tau1 = sin_sigma1 * cos_b11 + cos_sigma1 * sin_b11;
        let cos_tau1 = cos_sigma1 * cos_b11 - sin_sigma1 * sin_b11;
        let coefficients_c1p = coefficients_c1p(epsilon);
        let b12 = -sin_cos_series(
            sin_tau1 * cos_tau12 + cos_tau1 * sin_tau12,
            cos_tau1 * cos_tau12 - sin_tau1 * sin_tau12,
            &coefficients_c1p,
        );
        let sigma12 = tau12 - (b12 - b11);
        let sin_sigma12 = sigma12.sin();
        let cos_sigma12 = sigma12.cos();
        let sin_sigma2 = sin_sigma1 * cos_sigma12 + cos_sigma1 * sin_sigma12;
        let cos_sigma2 = cos_sigma1 * cos_sigma12 - sin_sigma1 * sin_sigma12;

        let sin_alpha2 = sin_alpha0;
        let cos_alpha2 = cos_alpha0 * cos_sigma2;
        let reverse_azimuth = sin_alpha2.atan2(cos_alpha2);

        let sin_beta2 = cos_alpha0 * sin_sigma2;
        let cos_beta2 = sin_alpha0.hypot(cos_alpha0 * cos_sigma2);
        let lat2 = sin_beta2.atan2(one_minus_f * cos_beta2);

        let sin_omega2 = sin_alpha0 * sin_sigma2;
        let cos_omega2 = cos_sigma2;
        let omega12 = (sin_omega2 * cos_omega1 - cos_omega2 * sin_omega1)
            .atan2(cos_omega2 * cos_omega1 + sin_omega2 * sin_omega1);
        let coefficients_a3 = coefficients_a3(n);
        let a3 = coefficients_a3
            .as_slice()
            .iter()
            .rev()
            .fold(0.0, |value, coefficient| value * epsilon + coefficient);
        let a3c = -f * sin_alpha0 * a3;
        let coefficients_c3 = coefficients_c3(n, epsilon);
        let b31 = sin_cos_series(sin_sigma1, cos_sigma1, &coefficients_c3);
        let b32 = sin_cos_series(sin_sigma2, cos_sigma2, &coefficients_c3);
        let lambda12 = omega12 + a3c * (sigma12 + b32 - b31);

        DirectResult::solved(
            lon1,
            lat1,
            azimuth12,
            self.spheroid,
            normalize_longitude(lon1 + lambda12),
            lat2,
            reverse_azimuth,
        )
    }
}

impl Default for KarneyDirect {
    #[inline]
    fn default() -> Self {
        Self::WGS84
    }
}
