//! Meridian-arc formulas on a reference spheroid.
//!
//! Ports the coherent aggregate formed by Boost.Geometry's
//! `meridian_direct.hpp`, `meridian_inverse.hpp`, `meridian_segment.hpp`, and
//! `quarter_meridian.hpp`. Angles are radians and distances use the spheroid's
//! radius unit.

#[cfg(feature = "std")]
use geometry_coords::CoordinateScalar;
use geometry_cs::Spheroid;

#[cfg(feature = "std")]
use crate::normalise::{longitude_distance_signed, normalized_longitude};

#[cfg(feature = "std")]
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
        self.arc_length_to_order(latitude, 5)
    }

    /// Signed meridian arc from the equator to `latitude`, the series cut
    /// at `order` (five and above give the full order-five expansion).
    ///
    /// Mirrors `meridian_inverse<CT, Order>::apply(lat, spheroid)`
    /// (`formulas/meridian_inverse.hpp:116-178`) term by term, so each
    /// order rounds as Boost's does.
    #[cfg(feature = "std")]
    pub(crate) fn arc_length_to_order(&self, latitude: f64, order: u32) -> f64 {
        let f = self.spheroid.flattening;
        let n = f / (2.0 - f);
        let scale = self.spheroid.equatorial_radius / (1.0 + n);
        let mut c0 = 1.0;
        if order == 0 {
            return scale * c0 * latitude;
        }
        let mut c2 = -1.5 * n;
        if order == 1 {
            return scale * (c0 * latitude + c2 * (2.0 * latitude).sin());
        }
        let n2 = n * n;
        c0 += 0.25 * n2;
        let mut c4 = 0.9375 * n2;
        if order == 2 {
            return scale
                * (c0 * latitude + c2 * (2.0 * latitude).sin() + c4 * (4.0 * latitude).sin());
        }
        let n3 = n2 * n;
        c2 += 0.1875 * n3;
        let mut c6 = -0.729_166_667 * n3;
        if order == 3 {
            return scale
                * (c0 * latitude
                    + c2 * (2.0 * latitude).sin()
                    + c4 * (4.0 * latitude).sin()
                    + c6 * (6.0 * latitude).sin());
        }
        let n4 = n2 * n2;
        c4 -= 0.234_375 * n4;
        let c8 = 0.615_234_375 * n4;
        if order == 4 {
            return scale
                * (c0 * latitude
                    + c2 * (2.0 * latitude).sin()
                    + c4 * (4.0 * latitude).sin()
                    + c6 * (6.0 * latitude).sin()
                    + c8 * (8.0 * latitude).sin());
        }
        let n5 = n4 * n;
        c6 += 0.227_864_583 * n5;
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
    ///
    /// Mirrors `meridian_inverse::meridian_not_crossing_pole` and
    /// `meridian_crossing_pole` (`formulas/meridian_inverse.hpp:50-62`):
    /// the longitude difference folded into `(−π, π]` must equal `0` or
    /// `±π` within `math::equals`, unless the endpoints are opposite poles.
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
        let half_pi = core::f64::consts::FRAC_PI_2;
        let difference = longitude_distance_signed(longitude1, longitude2);
        let (south, north) = if latitude1 > latitude2 {
            (latitude2, latitude1)
        } else {
            (latitude1, latitude2)
        };
        if difference.tolerant_eq(0.0)
            || (north.tolerant_eq(half_pi) && south.tolerant_eq(-half_pi))
        {
            MeridianSegmentKind::NotCrossingPole
        } else if difference.abs().tolerant_eq(core::f64::consts::PI) {
            MeridianSegmentKind::CrossingPole
        } else {
            MeridianSegmentKind::NonMeridian
        }
    }

    /// Solve the inverse problem when the endpoints form a meridian.
    ///
    /// Uses the order-five arc of [`Meridian::arc_length`].
    #[cfg(feature = "std")]
    #[must_use]
    pub fn inverse(
        &self,
        longitude1: f64,
        latitude1: f64,
        longitude2: f64,
        latitude2: f64,
    ) -> MeridianInverseResult {
        self.inverse_to_order(longitude1, latitude1, longitude2, latitude2, 5)
    }

    /// [`Meridian::inverse`] with the arc series cut at `order`.
    ///
    /// Mirrors `meridian_inverse<CT, Order>::apply`
    /// (`formulas/meridian_inverse.hpp:72-112`). Boost's geographic distance
    /// strategy runs it at the order its formula policy names
    /// (`strategies/geographic/parameters.hpp:186-204`).
    #[cfg(feature = "std")]
    pub(crate) fn inverse_to_order(
        &self,
        longitude1: f64,
        mut latitude1: f64,
        longitude2: f64,
        mut latitude2: f64,
        order: u32,
    ) -> MeridianInverseResult {
        let kind = self.classify_segment(longitude1, latitude1, longitude2, latitude2);
        if latitude1 > latitude2 {
            core::mem::swap(&mut latitude1, &mut latitude2);
        }
        let arc = |latitude| self.arc_length_to_order(latitude, order);
        let distance = match kind {
            MeridianSegmentKind::NonMeridian => 0.0,
            MeridianSegmentKind::NotCrossingPole => (arc(latitude2) - arc(latitude1)).abs(),
            MeridianSegmentKind::CrossingPole => {
                let latitude_sign = if latitude1 + latitude2 < 0.0 {
                    -1.0
                } else {
                    1.0
                };
                (latitude_sign * 2.0 * arc(core::f64::consts::FRAC_PI_2)
                    - arc(latitude1)
                    - arc(latitude2))
                .abs()
            }
        };
        MeridianInverseResult {
            distance,
            meridian: kind != MeridianSegmentKind::NonMeridian,
        }
    }

    /// Solve the direct geodesic problem along a meridian.
    ///
    /// Mirrors `formulas/meridian_direct.hpp:54-118`, except that the
    /// distance is used as given, where Boost truncates it to whole metres
    /// (`int signed_distance`), and an arc carried past a pole comes back
    /// down the opposite meridian, where Boost leaves the latitude beyond
    /// a right angle on the starting meridian.
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
        DirectResult::solved::<3>(
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
    (normalized_longitude(lon), lat, reflected)
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::Meridian;

    /// Each order of the series adds a term in the next power of `n`, so
    /// each closes on the full order-eight arc more tightly than the last;
    /// order zero is the rectifying-radius arc alone.
    #[test]
    fn each_series_order_closes_on_the_full_arc() {
        let meridian = Meridian::WGS84;
        let latitude = 1.0;
        let full = meridian.arc_length_to_order(latitude, 8);
        let mut previous = f64::INFINITY;
        for order in 0..=4 {
            let error = (meridian.arc_length_to_order(latitude, order) - full).abs();
            assert!(error < previous, "order {order}: {error} >= {previous}");
            previous = error;
        }
        let f = meridian.spheroid.flattening;
        let n = f / (2.0 - f);
        let rectifying = meridian.spheroid.equatorial_radius / (1.0 + n) * latitude;
        assert!((meridian.arc_length_to_order(latitude, 0) - rectifying).abs() < 1e-6);
    }
}
