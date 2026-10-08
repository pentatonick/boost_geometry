//! Globally seeded Karney inverse geodesic on a spheroid.
//!
//! Provides the capability of `boost::geometry::formula::karney_inverse`
//! from `formulas/karney_inverse.hpp:76-981` by numerically inverting this
//! crate's order-8 [`KarneyDirect`](super::KarneyDirect) mapping. The C++
//! implementation expands the inverse equations directly; Rust uses the same
//! Karney series in the forward map and a bounded two-variable Newton solve.
//! Multiple azimuth seeds preserve convergence near antipodal points, the
//! motivating property of Karney's formula.

use geometry_cs::Spheroid;
#[cfg(feature = "std")]
use geometry_cs::{CoordinateSystem, GeographicFamily};
#[cfg(feature = "std")]
use geometry_tag::SameAs;
#[cfg(feature = "std")]
use geometry_trait::Point;

#[cfg(feature = "std")]
use crate::distance::DistanceStrategy;

#[cfg(feature = "std")]
use super::inverse::InverseResult;

#[cfg(feature = "std")]
use super::direct_karney::KarneyDirect;
#[cfg(feature = "std")]
use super::meridian::Meridian;
#[cfg(feature = "std")]
use crate::normalise::normalized_longitude;
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
        let delta_lon = normalized_longitude(lon2 - lon1);
        // Every longitude names the same point at a pole.
        let same_pole = lat1 == lat2 && lat1.abs() == core::f64::consts::FRAC_PI_2;
        if (delta_lon == 0.0 && lat1 == lat2) || same_pole {
            return InverseResult {
                distance: 0.0,
                azimuth: 0.0,
                reverse_azimuth: 0.0,
                converged: true,
                reduced_length: 0.0,
                geodesic_scale: 1.0,
            };
        }

        // Karney orders the endpoints so that `|lat1| >= |lat2|`
        // (`formulas/karney_inverse.hpp`, `swap_point`). Here that order
        // starts the Newton solve where every azimuth leaves along its own
        // course and aims it at the endpoint farther from a pole. Solve from
        // the endpoint nearer a pole and walk the same geodesic back.
        if lat2.abs() <= lat1.abs() {
            return self.solve(lon1, lat1, lon2, lat2);
        }
        let reversed = self.solve(lon2, lat2, lon1, lat1);
        let azimuth = normalized_longitude(reversed.reverse_azimuth + core::f64::consts::PI);
        let forward = KarneyDirect {
            spheroid: self.spheroid,
        }
        .apply(lon1, lat1, reversed.distance, azimuth);
        InverseResult {
            distance: reversed.distance,
            azimuth,
            reverse_azimuth: normalized_longitude(reversed.azimuth + core::f64::consts::PI),
            converged: reversed.converged,
            reduced_length: forward.reduced_length,
            geodesic_scale: forward.geodesic_scale,
        }
    }

    /// The seeded Newton search behind [`Self::apply`], for endpoints
    /// already ordered so that `|lat1| >= |lat2|` and not coincident.
    #[cfg(feature = "std")]
    #[allow(
        clippy::similar_names,
        reason = "symbols follow the inverse-geodesic equations"
    )]
    fn solve(&self, lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> InverseResult {
        let delta_lon = normalized_longitude(lon2 - lon1);
        let sin_delta_lon = delta_lon.sin();
        let sin_lat1 = lat1.sin();
        let cos_lat1 = lat1.cos();
        let cos_lat2 = lat2.cos();
        // The haversine form of the central angle: the cosine rule's `acos`
        // rounds the angle of endpoints centimetres apart to zero, and a
        // zero-length seed leaves the Newton solve no azimuth to turn.
        let sin_half_dlat = ((lat2 - lat1) / 2.0).sin();
        let sin_half_dlon = (delta_lon / 2.0).sin();
        let haversine =
            sin_half_dlat * sin_half_dlat + cos_lat1 * cos_lat2 * (sin_half_dlon * sin_half_dlon);
        let central_angle = 2.0 * haversine.sqrt().min(1.0).asin();
        // `cos φ1·sin φ2 − sin φ1·cos φ2·cos Δλ`, rearranged so endpoints
        // close together do not cancel it away.
        let spherical_azimuth = (sin_delta_lon * cos_lat2).atan2(
            (lat2 - lat1).sin() + 2.0 * sin_lat1 * cos_lat2 * (sin_half_dlon * sin_half_dlon),
        );
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
        let first_candidate = self.solve_seed(
            &direct,
            lon1,
            lat1,
            lon2,
            lat2,
            distance_seeds[0],
            azimuth_seeds[0],
        );
        let first_endpoint = direct.apply(
            lon1,
            lat1,
            first_candidate.distance,
            first_candidate.azimuth,
        );
        let first_error = endpoint_error(first_endpoint.lon2, first_endpoint.lat2, lon2, lat2);
        let mut best = (first_error, first_candidate);

        for (azimuth_index, &azimuth_seed) in azimuth_seeds.iter().enumerate() {
            let remaining_distances = if azimuth_index == 0 {
                &distance_seeds[1..]
            } else {
                &distance_seeds[..]
            };
            for &distance_seed in remaining_distances {
                let candidate =
                    self.solve_seed(&direct, lon1, lat1, lon2, lat2, distance_seed, azimuth_seed);
                let endpoint = direct.apply(lon1, lat1, candidate.distance, candidate.azimuth);
                let error = endpoint_error(endpoint.lon2, endpoint.lat2, lon2, lat2);
                let (best_error, best_result) = best;
                // Among converged candidates the length counts: a seed can
                // converge onto a longer geodesic through the same two
                // points, and its endpoint residual being the smaller says
                // nothing about which route is the shortest. Lengths within
                // the tolerance are one route, where the closer end wins.
                let replace = match (error <= self.tolerance, best_error <= self.tolerance) {
                    (true, true) => {
                        let margin = self.tolerance * self.spheroid.equatorial_radius;
                        if (candidate.distance - best_result.distance).abs() <= margin {
                            error < best_error
                        } else {
                            candidate.distance < best_result.distance
                        }
                    }
                    (true, false) => true,
                    (false, true) => false,
                    (false, false) => error < best_error,
                };
                if replace {
                    best = (error, candidate);
                }
            }
        }

        best.1
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
        for iteration in 0..self.max_iterations {
            let current = direct.apply(lon1, lat1, distance, azimuth);
            let [residual_lon, residual_lat, residual_up] =
                residual(current.lon2, current.lat2, lon2, lat2);
            let error = residual_lon.hypot(residual_lat).hypot(residual_up);
            // A seed already within tolerance of endpoints a hair apart still
            // takes one step: the tolerance bounds where the geodesic ends,
            // not which way it leaves.
            if error <= self.tolerance && iteration > 0 {
                converged = true;
                break;
            }

            // Steps that move the endpoint about ten metres either way: an
            // azimuth step fixed small would move the end of a short
            // geodesic by less than the rounding of its coordinates.
            let distance_step = 10.0;
            let azimuth_step = (distance_step / distance).clamp(1e-6, 1e-2);
            let plus_distance = direct.apply(lon1, lat1, distance + distance_step, azimuth);
            let minus_distance = direct.apply(lon1, lat1, distance - distance_step, azimuth);
            let plus_azimuth = direct.apply(lon1, lat1, distance, azimuth + azimuth_step);
            let minus_azimuth = direct.apply(lon1, lat1, distance, azimuth - azimuth_step);
            let [pd_lon, pd_lat, _] = residual(plus_distance.lon2, plus_distance.lat2, lon2, lat2);
            let [md_lon, md_lat, _] =
                residual(minus_distance.lon2, minus_distance.lat2, lon2, lat2);
            let [pa_lon, pa_lat, _] = residual(plus_azimuth.lon2, plus_azimuth.lat2, lon2, lat2);
            let [ma_lon, ma_lat, _] = residual(minus_azimuth.lon2, minus_azimuth.lat2, lon2, lat2);
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
            azimuth = normalized_longitude(azimuth + azimuth_update);
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

/// The offset of `(lon, lat)` from the target on the sphere of geodetic
/// directions, east and north in the target's tangent plane and then up
/// along the target: about `(Δlon·cos lat, Δlat, 0)` beside the target,
/// but smooth through a pole, where that form leaves the longitude error
/// no weight and a finite difference across the pole flips it. The Newton
/// step solves the first two; the third tells the target from its
/// antipode, where they vanish too.
#[cfg(feature = "std")]
fn residual(lon: f64, lat: f64, target_lon: f64, target_lat: f64) -> [f64; 3] {
    let dlon = normalized_longitude(lon - target_lon);
    let sin_half_dlon = (dlon / 2.0).sin();
    let sin_half_dlat = ((lat - target_lat) / 2.0).sin();
    let cos_lat = lat.cos();
    [
        cos_lat * dlon.sin(),
        (lat - target_lat).sin() + 2.0 * target_lat.sin() * cos_lat * sin_half_dlon * sin_half_dlon,
        -2.0 * (sin_half_dlat * sin_half_dlat
            + cos_lat * target_lat.cos() * sin_half_dlon * sin_half_dlon),
    ]
}

/// The chord from `(lon, lat)` to the target on the sphere of geodetic
/// directions.
#[cfg(feature = "std")]
fn endpoint_error(lon: f64, lat: f64, target_lon: f64, target_lat: f64) -> f64 {
    let [east, north, up] = residual(lon, lat, target_lon, target_lat);
    east.hypot(north).hypot(up)
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

    /// Mirrors `strategy::distance::geographic<karney>::apply`
    /// (`strategies/geographic/distance.hpp:91-112`): a meridian pair takes
    /// the meridian arc at Karney's order (eight, the full series;
    /// `strategies/geographic/parameters.hpp:201-204`), the rest the inverse.
    #[inline]
    fn distance(&self, first: &P1, second: &P2) -> Self::Out {
        let (lon1, lat1) = lonlat_radians(first);
        let (lon2, lat2) = lonlat_radians(second);
        let meridian = Meridian {
            spheroid: self.spheroid,
        }
        .inverse_to_order(lon1, lat1, lon2, lat2, 8);
        if meridian.meridian {
            return meridian.distance;
        }
        self.apply(lon1, lat1, lon2, lat2).distance
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        *self
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::KarneyInverse;
    use crate::distance::DistanceStrategy;
    use crate::geographic::Meridian;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Geographic};

    type GP = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;

    #[inline]
    fn deg(lon: f64, lat: f64) -> GP {
        WithCs::new(Adapt([lon, lat]))
    }

    /// A meridian pair takes the meridian arc, as
    /// `strategy::distance::geographic<karney>` does
    /// (`strategies/geographic/distance.hpp:91-112`).
    #[test]
    fn a_meridian_pair_takes_the_meridian_arc() {
        let d = KarneyInverse::WGS84.distance(&deg(10.0, 0.0), &deg(10.0, 50.0));
        let arc = Meridian::WGS84.arc_length(50_f64.to_radians());
        assert!((d - arc).abs() < 1e-3, "got {d} expected {arc}");
    }

    /// Without a Newton step only the seeds already within the tolerance
    /// converge, and the far-off seeds never displace them.
    #[test]
    fn an_unconverged_seed_never_displaces_a_converged_one() {
        let (lon2, lat2) = (1_f64.to_radians(), 1_f64.to_radians());
        let exact = KarneyInverse::WGS84.apply(0.0, 0.0, lon2, lat2);
        let coarse = KarneyInverse {
            max_iterations: 0,
            tolerance: 1e-3,
            ..KarneyInverse::WGS84
        }
        .apply(0.0, 0.0, lon2, lat2);
        assert!(coarse.converged);
        assert!((coarse.distance - exact.distance).abs() < 0.01 * exact.distance);
    }

    /// When no seed reaches the tolerance, the smallest endpoint residual
    /// wins, which is still the shortest geodesic.
    #[test]
    fn with_no_seed_converged_the_smallest_residual_wins() {
        let (lon2, lat2) = (1_f64.to_radians(), 1_f64.to_radians());
        let exact = KarneyInverse::WGS84.apply(0.0, 0.0, lon2, lat2);
        let strict = KarneyInverse {
            tolerance: 0.0,
            ..KarneyInverse::WGS84
        }
        .apply(0.0, 0.0, lon2, lat2);
        assert!((strict.distance - exact.distance).abs() < 1e-3);
    }

    /// Without a Newton step, a fixed seed can land nearer than the
    /// spherical one: 50 km from the equator at 45°, the spherical seed
    /// misses by about `3.2e-5` and the `−3π/4` seed by `1.8e-5`, so the
    /// first converged candidate displaces the unconverged best.
    #[test]
    fn a_converged_seed_displaces_an_unconverged_best() {
        let end = super::KarneyDirect {
            spheroid: KarneyInverse::WGS84.spheroid,
        }
        .apply(0.0, 0.0, 50_000.0, core::f64::consts::FRAC_PI_4);
        let result = KarneyInverse {
            max_iterations: 0,
            tolerance: 2.5e-5,
            ..KarneyInverse::WGS84
        }
        .apply(0.0, 0.0, end.lon2, end.lat2);
        assert!(result.converged);
        assert!(
            (result.distance - 50_000.0).abs() < 500.0,
            "got {}",
            result.distance
        );
    }
}
