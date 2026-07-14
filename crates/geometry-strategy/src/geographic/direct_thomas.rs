//! Thomas second-order solution of the direct geodesic problem.
//!
//! Mirrors `boost::geometry::formula::thomas_direct` from
//! `formulas/thomas_direct.hpp:39-238`.

use geometry_cs::Spheroid;

use super::direct::DirectResult;

#[cfg(feature = "std")]
use super::direct::normalize_longitude;
#[cfg(feature = "std")]
use super::spheroid_calc::SpheroidCalc;

/// Forsyth–Andoyer–Lambert direct approximation with Thomas's second-order
/// terms.
///
/// Inputs and output angles are radians. Mirrors
/// `formula::thomas_direct<CT, true>` from
/// `formulas/thomas_direct.hpp:39-238`.
#[derive(Debug, Clone, Copy)]
pub struct ThomasDirect {
    /// Reference ellipsoid.
    pub spheroid: Spheroid,
    /// Include Thomas's second-order correction terms.
    pub second_order: bool,
}

impl ThomasDirect {
    /// Second-order Thomas direct on WGS84.
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
        second_order: true,
    };

    /// Solve the direct geodesic problem.
    ///
    /// Mirrors `thomas_direct::apply` from
    /// `formulas/thomas_direct.hpp:66-205`, including its southward-azimuth
    /// vertical flip and longitude normalization.
    #[cfg(feature = "std")]
    #[inline]
    #[must_use]
    #[allow(
        clippy::float_cmp,
        clippy::if_not_else,
        clippy::many_single_char_names,
        clippy::similar_names,
        clippy::too_many_lines,
        reason = "names follow Thomas's equations and the cited Boost implementation"
    )]
    pub fn apply(&self, lon1: f64, lat1: f64, distance: f64, azimuth12: f64) -> DirectResult {
        let calc = SpheroidCalc::from(self.spheroid);
        let a = calc.a;
        let f = calc.f;
        let one_minus_f = 1.0 - f;
        let pi = core::f64::consts::PI;
        let pi_half = core::f64::consts::FRAC_PI_2;

        let mut azi12_alt = azimuth12;
        let mut lat1_alt = lat1;
        let alter_result = if azimuth12 > pi_half {
            azi12_alt = pi - azimuth12;
            lat1_alt = -lat1;
            true
        } else if azimuth12 < -pi_half {
            azi12_alt = -pi - azimuth12;
            lat1_alt = -lat1;
            true
        } else {
            false
        };

        let theta1 = if lat1_alt == pi_half || lat1_alt == -pi_half {
            lat1_alt
        } else {
            (one_minus_f * lat1_alt.tan()).atan()
        };
        let sin_theta1 = theta1.sin();
        let cos_theta1 = theta1.cos();
        let sin_a12 = azi12_alt.sin();
        let cos_a12 = azi12_alt.cos();
        let m = cos_theta1 * sin_a12;
        let theta0 = m.acos();
        let sin_theta0 = theta0.sin();
        let n = cos_theta1 * cos_a12;
        let c1 = f * m;
        let c2 = f * (1.0 - m * m) / 4.0;
        let (d_coefficient, p) = if self.second_order {
            let d = (1.0 - c2) * (1.0 - c2 - c1 * m);
            (d, c2 * (1.0 + c1 * m / 2.0) / d)
        } else {
            let d = 1.0 - 2.0 * c2 - c1 * m;
            (d, c2 / d)
        };

        let cos_sigma1 = if sin_theta0 == 0.0 {
            1.0
        } else {
            (sin_theta1 / sin_theta0).clamp(-1.0, 1.0)
        };
        let sigma1 = cos_sigma1.acos();
        let d = distance / (a * d_coefficient);
        let u = 2.0 * (sigma1 - d);
        let cos_d = d.cos();
        let sin_d = d.sin();
        let cos_u = u.cos();
        let sin_u = u.sin();
        let w = 1.0 - 2.0 * p * cos_u;
        let v = cos_u * cos_d - sin_u * sin_d;
        let y = 2.0 * p * v * w * sin_d;
        let mut d_sigma = d - y;
        if self.second_order {
            let x = c2 * c2 * sin_d * cos_d * (2.0 * v * v - 1.0);
            d_sigma += x;
        }
        let sin_d_sigma = d_sigma.sin();
        let cos_d_sigma = d_sigma.cos();

        let mut reverse_azimuth = m.atan2(n * cos_d_sigma - sin_theta1 * sin_d_sigma);
        if alter_result {
            reverse_azimuth = if reverse_azimuth == 0.0 {
                if azimuth12 >= 0.0 { pi } else { -pi }
            } else if reverse_azimuth > 0.0 {
                pi - reverse_azimuth
            } else {
                -pi - reverse_azimuth
            };
        }

        let s_sigma = 2.0 * sigma1 - d_sigma;
        let mut h = c1 * d_sigma;
        if self.second_order {
            h = h * (1.0 - c2) - c1 * c2 * sin_d_sigma * s_sigma.cos();
        }
        let d_eta = (sin_d_sigma * sin_a12)
            .atan2(cos_theta1 * cos_d_sigma - sin_theta1 * sin_d_sigma * cos_a12);
        let d_lambda = d_eta - h;
        let mut lat2 = if m != 0.0 {
            let sin_a21 = reverse_azimuth.sin();
            let tan_theta2 = (sin_theta1 * cos_d_sigma + n * sin_d_sigma) * sin_a21 / m;
            (tan_theta2 / one_minus_f).atan()
        } else {
            let sigma2 = s_sigma - sigma1;
            let tan_theta2 = sigma2.cos() / sigma2.sin().abs();
            (tan_theta2 / one_minus_f).atan()
        };
        if alter_result {
            lat2 = -lat2;
        }

        DirectResult::solved(
            lon1,
            lat1,
            azimuth12,
            self.spheroid,
            normalize_longitude(lon1 + d_lambda),
            lat2,
            reverse_azimuth,
        )
    }
}

impl Default for ThomasDirect {
    #[inline]
    fn default() -> Self {
        Self::WGS84
    }
}
