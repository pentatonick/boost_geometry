//! Meridian-arc formulas on a reference spheroid.
//!
//! Ports the coherent aggregate formed by Boost.Geometry's
//! `meridian_direct.hpp`, `meridian_inverse.hpp`, `meridian_segment.hpp`, and
//! `quarter_meridian.hpp`. Angles are radians and distances use the spheroid's
//! radius unit.

use geometry_cs::Spheroid;

use super::direct::DirectResult;

/// Classification of a spheroidal segment relative to a meridian.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeridianSegmentKind {
    /// The endpoints do not define a meridian segment.
    NonMeridian,
    /// Both endpoints use the same meridian and the path avoids a pole.
    NotCrossingPole,
    /// The longitudes differ by π and the meridian path crosses a pole.
    CrossingPole,
}

/// Distance result from recognizing a meridian endpoint pair.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeridianInverseResult {
    /// Meridian distance, or zero when the pair is not meridional.
    pub distance: f64,
    /// Whether the endpoint pair was recognized as meridional.
    pub meridian: bool,
}

/// Meridian formulas bound to a reference spheroid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Meridian {
    /// Reference ellipsoid.
    pub spheroid: Spheroid,
}

impl Meridian {
    /// WGS84 meridian formulas.
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };

    /// Length from the equator to a pole.
    ///
    /// Mirrors the order-eight generalized series in
    /// `formulas/quarter_meridian.hpp:54-76`.
    #[must_use]
    pub fn quarter_length(&self) -> f64 {
        const COEFFICIENTS: [f64; 9] = [
            1_073_741_824.0,
            268_435_456.0,
            16_777_216.0,
            4_194_304.0,
            1_638_400.0,
            802_816.0,
            451_584.0,
            278_784.0,
            184_041.0,
        ];
        let f = self.spheroid.flattening;
        let n = f / (2.0 - f);
        let ab4 = (self.spheroid.equatorial_radius + self.spheroid.polar_radius()) / 4.0;
        let series = COEFFICIENTS[..8]
            .iter()
            .rev()
            .fold(0.0, |value, coefficient| value * n * n + coefficient);
        core::f64::consts::PI * ab4 * series / COEFFICIENTS[0]
    }

    /// Signed meridian arc from the equator to `latitude`.
    ///
    /// Uses Boost's maximum order-five expansion from
    /// `formulas/meridian_inverse.hpp:116-178`.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn arc_length(&self, latitude: f64) -> f64 {
        let f = self.spheroid.flattening;
        let n = f / (2.0 - f);
        let n2 = n * n;
        let n3 = n2 * n;
        let n4 = n2 * n2;
        let n5 = n4 * n;
        let scale = self.spheroid.equatorial_radius / (1.0 + n);
        let c0 = 1.0 + 0.25 * n2;
        let c2 = -1.5 * n + 0.1875 * n3;
        let c4 = 0.9375 * n2 - 0.234_375 * n4;
        let c6 = -0.729_166_667 * n3 + 0.227_864_583 * n5;
        let c8 = 0.615_234_375 * n4;
        let c10 = -0.541_406_25 * n5;
        scale
            * (c0 * latitude
                + c2 * (2.0 * latitude).sin()
                + c4 * (4.0 * latitude).sin()
                + c6 * (6.0 * latitude).sin()
                + c8 * (8.0 * latitude).sin()
                + c10 * (10.0 * latitude).sin())
    }

    /// Latitude whose signed meridian arc is `distance`.
    ///
    /// Mirrors the order-four inverse series in
    /// `formulas/meridian_direct.hpp:123-171`.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn latitude_at_arc(&self, distance: f64) -> f64 {
        let f = self.spheroid.flattening;
        let n = f / (2.0 - f);
        let n2 = n * n;
        let n3 = n2 * n;
        let n4 = n2 * n2;
        let mu = core::f64::consts::FRAC_PI_2 * distance / self.quarter_length();
        let h2 = 1.5 * n - 0.843_75 * n3;
        let h4 = 1.3125 * n2 - 1.718_75 * n4;
        let h6 = 1.572_916_667 * n3;
        let h8 = 2.142_578_125 * n4;
        mu + h2 * (2.0 * mu).sin()
            + h4 * (4.0 * mu).sin()
            + h6 * (6.0 * mu).sin()
            + h8 * (8.0 * mu).sin()
    }

    /// Classify the endpoint pair as a meridian segment.
    #[cfg(feature = "std")]
    #[must_use]
    #[allow(
        clippy::unused_self,
        reason = "the instance method keeps the formula strategy API uniform with the spheroid-dependent methods"
    )]
    pub fn classify_segment(
        &self,
        longitude1: f64,
        latitude1: f64,
        longitude2: f64,
        latitude2: f64,
    ) -> MeridianSegmentKind {
        let difference = normalize_longitude(longitude2 - longitude1);
        if nearly_zero(difference)
            || (nearly_equal(latitude2, core::f64::consts::FRAC_PI_2)
                && nearly_equal(latitude1, -core::f64::consts::FRAC_PI_2))
            || (nearly_equal(latitude1, core::f64::consts::FRAC_PI_2)
                && nearly_equal(latitude2, -core::f64::consts::FRAC_PI_2))
        {
            MeridianSegmentKind::NotCrossingPole
        } else if nearly_equal(difference.abs(), core::f64::consts::PI) {
            MeridianSegmentKind::CrossingPole
        } else {
            MeridianSegmentKind::NonMeridian
        }
    }

    /// Solve the inverse problem when the endpoints form a meridian.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn inverse(
        &self,
        longitude1: f64,
        mut latitude1: f64,
        longitude2: f64,
        mut latitude2: f64,
    ) -> MeridianInverseResult {
        let kind = self.classify_segment(longitude1, latitude1, longitude2, latitude2);
        if latitude1 > latitude2 {
            core::mem::swap(&mut latitude1, &mut latitude2);
        }
        let distance = match kind {
            MeridianSegmentKind::NonMeridian => 0.0,
            MeridianSegmentKind::NotCrossingPole => {
                (self.arc_length(latitude2) - self.arc_length(latitude1)).abs()
            }
            MeridianSegmentKind::CrossingPole => {
                let latitude_sign = if latitude1 + latitude2 < 0.0 {
                    -1.0
                } else {
                    1.0
                };
                (latitude_sign * 2.0 * self.arc_length(core::f64::consts::FRAC_PI_2)
                    - self.arc_length(latitude1)
                    - self.arc_length(latitude2))
                .abs()
            }
        };
        MeridianInverseResult {
            distance,
            meridian: kind != MeridianSegmentKind::NonMeridian,
        }
    }

    /// Solve the direct geodesic problem along a meridian.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn direct(
        &self,
        longitude1: f64,
        latitude1: f64,
        distance: f64,
        north: bool,
    ) -> DirectResult {
        let initial_arc = self.arc_length(latitude1);
        let signed_distance = if north { distance } else { -distance };
        let raw_latitude = self.latitude_at_arc(initial_arc + signed_distance);
        let (lon2, lat2, reflected) = normalize_coordinates(longitude1, raw_latitude);
        let final_north = north ^ reflected;
        DirectResult::solved(
            longitude1,
            latitude1,
            if north { 0.0 } else { core::f64::consts::PI },
            self.spheroid,
            lon2,
            lat2,
            if final_north {
                0.0
            } else {
                core::f64::consts::PI
            },
        )
    }
}

impl Default for Meridian {
    fn default() -> Self {
        Self::WGS84
    }
}

#[cfg(feature = "std")]
fn normalize_coordinates(longitude: f64, latitude: f64) -> (f64, f64, bool) {
    let pi = core::f64::consts::PI;
    let mut lat = (latitude + pi).rem_euclid(core::f64::consts::TAU) - pi;
    let mut lon = longitude;
    let reflected = if lat > core::f64::consts::FRAC_PI_2 {
        lat = pi - lat;
        lon += pi;
        true
    } else if lat < -core::f64::consts::FRAC_PI_2 {
        lat = -pi - lat;
        lon += pi;
        true
    } else {
        false
    };
    (normalize_longitude(lon), lat, reflected)
}

#[cfg(feature = "std")]
fn normalize_longitude(longitude: f64) -> f64 {
    let pi = core::f64::consts::PI;
    (longitude + pi).rem_euclid(core::f64::consts::TAU) - pi
}

#[cfg(feature = "std")]
fn nearly_zero(value: f64) -> bool {
    value.abs() <= 1e-12
}

#[cfg(feature = "std")]
fn nearly_equal(first: f64, second: f64) -> bool {
    (first - second).abs() <= 1e-12
}
