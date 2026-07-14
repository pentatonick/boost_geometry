//! Intersection of two geodesics on a spheroid.
//!
//! The gnomonic solver ports `gnomonic_spheroid.hpp` and
//! `gnomonic_intersection.hpp` using this crate's Karney inverse/direct pair.
//! `Sjoberg` exposes the same globally useful intersection contract with
//! additional projection seeds for the cases Boost routes through
//! `sjoberg_intersection.hpp`; this avoids duplicating a second low-order
//! inverse series while retaining its difficult-case behavior.

use geometry_cs::Spheroid;

use super::{KarneyDirect, KarneyInverse};

/// Longitude/latitude intersection of two infinite geodesics, in radians.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeodesicIntersection {
    /// Normalized longitude in radians.
    pub longitude: f64,
    /// Latitude in radians.
    pub latitude: f64,
}

/// Karney spheroidal-gnomonic geodesic intersection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gnomonic {
    /// Reference ellipsoid.
    pub spheroid: Spheroid,
}

impl Gnomonic {
    /// WGS84 gnomonic intersection.
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };

    /// Project a geographic point onto the spheroidal gnomonic plane.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn forward(&self, center: [f64; 2], point: [f64; 2]) -> Option<[f64; 2]> {
        let inverse = KarneyInverse {
            spheroid: self.spheroid,
            ..KarneyInverse::WGS84
        }
        .apply(center[0], center[1], point[0], point[1]);
        if inverse.geodesic_scale <= 0.0 {
            return None;
        }
        let rho = inverse.reduced_length / inverse.geodesic_scale;
        Some([inverse.azimuth.sin() * rho, inverse.azimuth.cos() * rho])
    }

    /// Invert a spheroidal gnomonic plane coordinate.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn reverse(&self, center: [f64; 2], point: [f64; 2]) -> Option<[f64; 2]> {
        let azimuth = point[0].atan2(point[1]);
        let rho = point[0].hypot(point[1]);
        let radius = self.spheroid.equatorial_radius;
        let mut distance = radius * (rho / radius).atan();
        let direct = KarneyDirect {
            spheroid: self.spheroid,
        };
        let mut found = false;
        for _ in 0..10 {
            let result = direct.apply(center[0], center[1], distance, azimuth);
            if result.geodesic_scale <= 0.0 {
                return None;
            }
            let rho_error = result.reduced_length / result.geodesic_scale - rho;
            let distance_update = rho_error * result.geodesic_scale * result.geodesic_scale;
            distance -= distance_update;
            if distance_update.abs() <= radius * f64::EPSILON {
                found = true;
                break;
            }
        }
        found.then(|| {
            let result = direct.apply(center[0], center[1], distance, azimuth);
            [result.lon2, result.lat2]
        })
    }

    /// Intersect two infinite geodesics, each defined by two points.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn intersection(
        &self,
        first1: [f64; 2],
        first2: [f64; 2],
        second1: [f64; 2],
        second2: [f64; 2],
    ) -> Option<GeodesicIntersection> {
        let seed = [
            (first1[0] + first2[0] + second1[0] + second2[0]) / 4.0,
            (first1[1] + first2[1] + second1[1] + second2[1]) / 4.0,
        ];
        self.intersection_from_seed(first1, first2, second1, second2, seed)
    }

    #[cfg(feature = "std")]
    fn intersection_from_seed(
        &self,
        first1: [f64; 2],
        first2: [f64; 2],
        second1: [f64; 2],
        second2: [f64; 2],
        mut center: [f64; 2],
    ) -> Option<GeodesicIntersection> {
        for _ in 0..10 {
            let projected_first1 = self.forward(center, first1)?;
            let projected_first2 = self.forward(center, first2)?;
            let projected_second1 = self.forward(center, second1)?;
            let projected_second2 = self.forward(center, second2)?;
            let projected = intersect_lines(
                projected_first1,
                projected_first2,
                projected_second1,
                projected_second2,
            )?;
            let next = self.reverse(center, projected)?;
            if (next[0] - center[0]).abs() <= 1e-13 && (next[1] - center[1]).abs() <= 1e-13 {
                center = next;
                break;
            }
            center = next;
        }
        Some(GeodesicIntersection {
            longitude: normalize_longitude(center[0]),
            latitude: center[1],
        })
    }
}

impl Default for Gnomonic {
    fn default() -> Self {
        Self::WGS84
    }
}

/// Robust geodesic intersection for the cases represented by Boost's
/// Sjöberg formula.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sjoberg {
    /// Reference ellipsoid.
    pub spheroid: Spheroid,
}

impl Sjoberg {
    /// WGS84 Sjöberg-compatible intersection.
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };

    /// Intersect two infinite geodesics using a globally seeded gnomonic
    /// solve. Multiple centers cover the broad and nearly equatorial cases
    /// for which Boost selects `sjoberg_intersection`.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn intersection(
        &self,
        first1: [f64; 2],
        first2: [f64; 2],
        second1: [f64; 2],
        second2: [f64; 2],
    ) -> Option<GeodesicIntersection> {
        let gnomonic = Gnomonic {
            spheroid: self.spheroid,
        };
        let mean = [
            (first1[0] + first2[0] + second1[0] + second2[0]) / 4.0,
            (first1[1] + first2[1] + second1[1] + second2[1]) / 4.0,
        ];
        let seeds = [
            mean,
            midpoint(first1, first2),
            midpoint(second1, second2),
            first1,
            first2,
            second1,
            second2,
        ];
        seeds.into_iter().find_map(|seed| {
            gnomonic.intersection_from_seed(first1, first2, second1, second2, seed)
        })
    }
}

impl Default for Sjoberg {
    fn default() -> Self {
        Self::WGS84
    }
}

#[cfg(feature = "std")]
fn midpoint(first: [f64; 2], second: [f64; 2]) -> [f64; 2] {
    [
        first[0] + normalize_longitude(second[0] - first[0]) / 2.0,
        f64::midpoint(first[1], second[1]),
    ]
}

#[cfg(feature = "std")]
fn intersect_lines(
    first1: [f64; 2],
    first2: [f64; 2],
    second1: [f64; 2],
    second2: [f64; 2],
) -> Option<[f64; 2]> {
    let line1 = cross3([first1[0], first1[1], 1.0], [first2[0], first2[1], 1.0]);
    let line2 = cross3([second1[0], second1[1], 1.0], [second2[0], second2[1], 1.0]);
    let point = cross3(line1, line2);
    if point[2].abs() <= f64::EPSILON {
        None
    } else {
        Some([point[0] / point[2], point[1] / point[2]])
    }
}

fn cross3(first: [f64; 3], second: [f64; 3]) -> [f64; 3] {
    [
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    ]
}

#[cfg(feature = "std")]
fn normalize_longitude(longitude: f64) -> f64 {
    let pi = core::f64::consts::PI;
    (longitude + pi).rem_euclid(core::f64::consts::TAU) - pi
}
