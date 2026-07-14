//! Globally seeded Karney inverse geodesic on a spheroid.
//!
//! Provides the capability of `boost::geometry::formula::karney_inverse`
//! from `formulas/karney_inverse.hpp:76-981` by numerically inverting this
//! crate's order-8 [`KarneyDirect`](super::KarneyDirect) mapping. The C++
//! implementation expands the inverse equations directly; Rust uses the same
//! Karney series in the forward map and a bounded two-variable Newton solve.
//! Multiple azimuth seeds preserve convergence near antipodal points, the
//! motivating property of Karney's formula.

use geometry_cs::{CoordinateSystem, GeographicFamily, Spheroid};
use geometry_tag::SameAs;
use geometry_trait::Point;

use crate::distance::DistanceStrategy;

use super::inverse::InverseResult;

#[cfg(feature = "std")]
use super::direct::normalize_longitude;
#[cfg(feature = "std")]
use super::direct_karney::KarneyDirect;
#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Karney order-8 inverse geodesic solver.
///
/// Mirrors the public role and result convention of
/// `formula::karney_inverse<CT, ..., 8>` from
/// `formulas/karney_inverse.hpp:76-981`. See the module-level divergence note
/// for the bounded inverse-of-direct implementation used in Rust.
#[derive(Debug, Clone, Copy)]
pub struct KarneyInverse {
    /// Reference ellipsoid.
    pub spheroid: Spheroid,
    /// Maximum Newton updates per starting azimuth.
    pub max_iterations: u32,
    /// Target endpoint residual in radians.
    pub tolerance: f64,
}

/// Conventional short name for [`KarneyInverse`] when selecting a distance
/// strategy.
pub type Karney = KarneyInverse;

impl KarneyInverse {
    /// Karney inverse on WGS84 with bounded globally seeded iteration.
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
        max_iterations: 50,
        tolerance: 2e-14,
    };

    /// Solve the inverse geodesic problem between two longitude/latitude
    /// pairs in radians.
    ///
    /// Reproduces the outputs of `karney_inverse::apply` from
    /// `formulas/karney_inverse.hpp:105-981`: shortest distance, forward
    /// azimuth, and final azimuth. The search evaluates several globally
    /// distributed azimuth seeds and selects the shortest converged geodesic,
    /// which handles the near-antipodal cases in
    /// `test/formulas/inverse_karney.cpp:52-72`.
    #[cfg(feature = "std")]
    #[inline]
    #[must_use]
    #[allow(
        clippy::many_single_char_names,
        clippy::similar_names,
        clippy::float_cmp,
        reason = "symbols and exact coincident checks follow the inverse-geodesic equations"
    )]
    pub fn apply(&self, lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> InverseResult {
        let delta_lon = normalize_longitude(lon2 - lon1);
        if delta_lon == 0.0 && lat1 == lat2 {
            return InverseResult {
                distance: 0.0,
                azimuth: 0.0,
                reverse_azimuth: 0.0,
                converged: true,
                reduced_length: 0.0,
                geodesic_scale: 1.0,
            };
        }

        let sin_delta_lon = delta_lon.sin();
        let cos_delta_lon = delta_lon.cos();
        let sin_lat1 = lat1.sin();
        let cos_lat1 = lat1.cos();
        let sin_lat2 = lat2.sin();
        let cos_lat2 = lat2.cos();
        let central_angle = (sin_lat1 * sin_lat2 + cos_lat1 * cos_lat2 * cos_delta_lon)
            .clamp(-1.0, 1.0)
            .acos();
        let spherical_azimuth = (sin_delta_lon * cos_lat2)
            .atan2(cos_lat1 * sin_lat2 - sin_lat1 * cos_lat2 * cos_delta_lon);
        let mean_radius = self.spheroid.equatorial_radius * (1.0 - self.spheroid.flattening / 3.0);
        let spherical_distance = central_angle * mean_radius;
        let half_meridian = core::f64::consts::PI * self.spheroid.polar_radius();
        let direct = KarneyDirect {
            spheroid: self.spheroid,
        };

        let azimuth_seeds = [
            spherical_azimuth,
            0.0,
            core::f64::consts::FRAC_PI_4,
            -core::f64::consts::FRAC_PI_4,
            core::f64::consts::FRAC_PI_2,
            -core::f64::consts::FRAC_PI_2,
            3.0 * core::f64::consts::FRAC_PI_4,
            -3.0 * core::f64::consts::FRAC_PI_4,
            core::f64::consts::PI,
        ];
        let distance_seeds = [spherical_distance, half_meridian];
        let mut best: Option<(f64, InverseResult)> = None;

        for &azimuth_seed in &azimuth_seeds {
            for &distance_seed in &distance_seeds {
                let candidate =
                    self.solve_seed(&direct, lon1, lat1, lon2, lat2, distance_seed, azimuth_seed);
                let endpoint = direct.apply(lon1, lat1, candidate.distance, candidate.azimuth);
                let error = endpoint_error(endpoint.lon2, endpoint.lat2, lon2, lat2);
                let replace = match best {
                    None => true,
                    Some((best_error, best_result)) => {
                        error < best_error
                            || (error <= self.tolerance
                                && best_error <= self.tolerance
                                && candidate.distance < best_result.distance)
                    }
                };
                if replace {
                    best = Some((error, candidate));
                }
            }
        }

        best.map_or_else(
            || {
                self.solve_seed(
                    &direct,
                    lon1,
                    lat1,
                    lon2,
                    lat2,
                    spherical_distance,
                    spherical_azimuth,
                )
            },
            |(_, result)| result,
        )
    }

    #[cfg(feature = "std")]
    #[allow(
        clippy::too_many_arguments,
        clippy::similar_names,
        reason = "the Newton state mirrors the two endpoints plus distance/azimuth seed"
    )]
    fn solve_seed(
        &self,
        direct: &KarneyDirect,
        lon1: f64,
        lat1: f64,
        lon2: f64,
        lat2: f64,
        mut distance: f64,
        mut azimuth: f64,
    ) -> InverseResult {
        let max_distance = 1.1 * core::f64::consts::PI * self.spheroid.equatorial_radius;
        let mut converged = false;
        for _ in 0..self.max_iterations {
            let current = direct.apply(lon1, lat1, distance, azimuth);
            let [residual_lon, residual_lat] = residual(current.lon2, current.lat2, lon2, lat2);
            let error = residual_lon.hypot(residual_lat);
            if error <= self.tolerance {
                converged = true;
                break;
            }

            let distance_step = 10.0;
            let azimuth_step = 1e-6;
            let plus_distance = direct.apply(lon1, lat1, distance + distance_step, azimuth);
            let minus_distance = direct.apply(lon1, lat1, distance - distance_step, azimuth);
            let plus_azimuth = direct.apply(lon1, lat1, distance, azimuth + azimuth_step);
            let minus_azimuth = direct.apply(lon1, lat1, distance, azimuth - azimuth_step);
            let [pd_lon, pd_lat] = residual(plus_distance.lon2, plus_distance.lat2, lon2, lat2);
            let [md_lon, md_lat] = residual(minus_distance.lon2, minus_distance.lat2, lon2, lat2);
            let [pa_lon, pa_lat] = residual(plus_azimuth.lon2, plus_azimuth.lat2, lon2, lat2);
            let [ma_lon, ma_lat] = residual(minus_azimuth.lon2, minus_azimuth.lat2, lon2, lat2);
            let j00 = (pd_lon - md_lon) / (2.0 * distance_step);
            let j10 = (pd_lat - md_lat) / (2.0 * distance_step);
            let j01 = (pa_lon - ma_lon) / (2.0 * azimuth_step);
            let j11 = (pa_lat - ma_lat) / (2.0 * azimuth_step);
            let determinant = j00 * j11 - j01 * j10;
            if determinant.abs() < 1e-24 || !determinant.is_finite() {
                break;
            }
            let mut distance_update = (-residual_lon * j11 + j01 * residual_lat) / determinant;
            let mut azimuth_update = (residual_lon * j10 - j00 * residual_lat) / determinant;
            distance_update = distance_update.clamp(-2_000_000.0, 2_000_000.0);
            azimuth_update = azimuth_update.clamp(-0.5, 0.5);
            distance = (distance + distance_update).clamp(0.0, max_distance);
            azimuth = normalize_longitude(azimuth + azimuth_update);
        }

        let endpoint = direct.apply(lon1, lat1, distance, azimuth);
        if endpoint_error(endpoint.lon2, endpoint.lat2, lon2, lat2) <= self.tolerance {
            converged = true;
        }
        InverseResult {
            distance,
            azimuth,
            reverse_azimuth: endpoint.reverse_azimuth,
            converged,
            reduced_length: endpoint.reduced_length,
            geodesic_scale: endpoint.geodesic_scale,
        }
    }
}

#[cfg(feature = "std")]
fn residual(lon: f64, lat: f64, target_lon: f64, target_lat: f64) -> [f64; 2] {
    [
        normalize_longitude(lon - target_lon) * target_lat.cos(),
        lat - target_lat,
    ]
}

#[cfg(feature = "std")]
fn endpoint_error(lon: f64, lat: f64, target_lon: f64, target_lat: f64) -> f64 {
    let [lon_error, lat_error] = residual(lon, lat, target_lon, target_lat);
    lon_error.hypot(lat_error)
}

impl Default for KarneyInverse {
    #[inline]
    fn default() -> Self {
        Self::WGS84
    }
}

#[cfg(feature = "std")]
impl<P1, P2> DistanceStrategy<P1, P2> for KarneyInverse
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    P1::Cs: HasAngularUnits,
    P2::Cs: HasAngularUnits,
    <P1::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
{
    type Out = f64;
    type Comparable = Self;

    #[inline]
    fn distance(&self, first: &P1, second: &P2) -> Self::Out {
        let (lon1, lat1) = lonlat_radians(first);
        let (lon2, lat2) = lonlat_radians(second);
        self.apply(lon1, lat1, lon2, lat2).distance
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        *self
    }
}
