//! Vincenty solution of the direct geodesic problem on a spheroid.
//!
//! Mirrors `boost::geometry::formula::vincenty_direct` from
//! `formulas/vincenty_direct.hpp:43-178`.

use geometry_cs::Spheroid;

use super::direct::DirectResult;

#[cfg(feature = "std")]
use super::direct::normalize_longitude;
#[cfg(feature = "std")]
use super::spheroid_calc::SpheroidCalc;

/// Vincenty's iterative direct geodesic formula.
///
/// Given a longitude/latitude, distance, and initial azimuth, computes the
/// destination and final azimuth. Inputs and output angles are radians;
/// distance uses the spheroid radius unit. Mirrors
/// `formula::vincenty_direct<CT>` from
/// `formulas/vincenty_direct.hpp:43-178`.
#[derive(Debug, Clone, Copy)]
pub struct VincentyDirect {
    /// Reference ellipsoid.
    pub spheroid: Spheroid,
    /// Iteration limit, matching Boost's default of 1000.
    pub max_iterations: u32,
    /// Convergence threshold for auxiliary-sphere arc length.
    pub tolerance: f64,
}

impl VincentyDirect {
    /// Vincenty direct on WGS84 with Boost's iteration settings.
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
        max_iterations: 1000,
        tolerance: 1e-12,
    };

    /// Solve the direct geodesic problem.
    ///
    /// Mirrors `vincenty_direct::apply` from
    /// `formulas/vincenty_direct.hpp:68-176`, including the iterative
    /// `delta_sigma` correction and final longitude normalization.
    #[cfg(feature = "std")]
    #[inline]
    #[must_use]
    #[allow(
        clippy::many_single_char_names,
        clippy::similar_names,
        reason = "names follow Vincenty's equations and the cited Boost implementation"
    )]
    pub fn apply(&self, lon1: f64, lat1: f64, distance: f64, azimuth12: f64) -> DirectResult {
        let calc = SpheroidCalc::from(self.spheroid);
        let radius_a = calc.a;
        let radius_b = calc.b;
        let flattening = calc.f;

        let sin_azimuth12 = azimuth12.sin();
        let cos_azimuth12 = azimuth12.cos();
        let one_min_f = 1.0 - flattening;
        let tan_u1 = one_min_f * lat1.tan();
        let sigma1 = tan_u1.atan2(cos_azimuth12);
        let u1 = tan_u1.atan();
        let sin_u1 = u1.sin();
        let cos_u1 = u1.cos();

        let sin_alpha = cos_u1 * sin_azimuth12;
        let sin_alpha_sqr = sin_alpha * sin_alpha;
        let cos_alpha_sqr = 1.0 - sin_alpha_sqr;
        let b_sqr = radius_b * radius_b;
        let u_sqr = cos_alpha_sqr * (radius_a * radius_a - b_sqr) / b_sqr;
        let a = 1.0
            + (u_sqr / 16_384.0) * (4096.0 + u_sqr * (-768.0 + u_sqr * (320.0 - u_sqr * 175.0)));
        let b = (u_sqr / 1024.0) * (256.0 + u_sqr * (-128.0 + u_sqr * (74.0 - u_sqr * 47.0)));

        let s_div_ba = distance / (radius_b * a);
        let mut sigma = s_div_ba;
        let mut sin_sigma;
        let mut cos_sigma;
        let mut cos_2sigma_m;
        let mut cos_2sigma_m_sqr;
        let mut counter = 0;
        loop {
            let previous_sigma = sigma;
            let two_sigma_m = 2.0 * sigma1 + sigma;
            sin_sigma = sigma.sin();
            cos_sigma = sigma.cos();
            let sin_sigma_sqr = sin_sigma * sin_sigma;
            cos_2sigma_m = two_sigma_m.cos();
            cos_2sigma_m_sqr = cos_2sigma_m * cos_2sigma_m;
            let delta_sigma = b
                * sin_sigma
                * (cos_2sigma_m
                    + (b / 4.0)
                        * (cos_sigma * (-1.0 + 2.0 * cos_2sigma_m_sqr)
                            - (b / 6.0)
                                * cos_2sigma_m
                                * (-3.0 + 4.0 * sin_sigma_sqr)
                                * (-3.0 + 4.0 * cos_2sigma_m_sqr)));
            sigma = s_div_ba + delta_sigma;
            counter += 1;
            if (previous_sigma - sigma).abs() <= self.tolerance || counter >= self.max_iterations {
                break;
            }
        }

        let lat2 = (sin_u1 * cos_sigma + cos_u1 * sin_sigma * cos_azimuth12).atan2(
            one_min_f
                * (sin_alpha_sqr
                    + (sin_u1 * sin_sigma - cos_u1 * cos_sigma * cos_azimuth12).powi(2))
                .sqrt(),
        );
        let lambda = (sin_sigma * sin_azimuth12)
            .atan2(cos_u1 * cos_sigma - sin_u1 * sin_sigma * cos_azimuth12);
        let c =
            (flattening / 16.0) * cos_alpha_sqr * (4.0 + flattening * (4.0 - 3.0 * cos_alpha_sqr));
        let big_l = lambda
            - (1.0 - c)
                * flattening
                * sin_alpha
                * (sigma
                    + c * sin_sigma
                        * (cos_2sigma_m + c * cos_sigma * (-1.0 + 2.0 * cos_2sigma_m_sqr)));
        let reverse_azimuth =
            sin_alpha.atan2(-sin_u1 * sin_sigma + cos_u1 * cos_sigma * cos_azimuth12);

        DirectResult::solved(
            lon1,
            lat1,
            azimuth12,
            self.spheroid,
            normalize_longitude(lon1 + big_l),
            lat2,
            reverse_azimuth,
        )
    }
}

impl Default for VincentyDirect {
    #[inline]
    fn default() -> Self {
        Self::WGS84
    }
}
