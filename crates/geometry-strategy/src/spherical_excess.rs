//! Spherical excess of a ring's edges, the kernel the spherical and
//! geographic area strategies share.
//!
//! Mirrors the parts of `formula::area_formulas`
//! (`formulas/area_formulas.hpp`) that both `strategy::area::spherical`
//! and `strategy::area::geographic` use: the trapezoidal excess of one
//! edge, and the prime-meridian crossings both count to tell a ring that
//! winds around a pole, whose summed excess their states then correct
//! (`strategy/spherical/area.hpp:82-110`,
//! `strategy/geographic/area.hpp:142-179`).

use core::f64::consts::PI;

use crate::normalise::longitude_distance_signed;

/// Spherical excess of the edge between latitudes `lat1` and `lat2`
/// across the longitude difference `lon12`, all in radians.
///
/// Mirrors `area_formulas::trapezoidal_formula`
/// (`formulas/area_formulas.hpp:347-355`).
pub(crate) fn trapezoidal_excess(lat1: f64, lat2: f64, lon12: f64) -> f64 {
    let tan_lat1 = (lat1 / 2.0).tan();
    let tan_lat2 = (lat2 / 2.0).tan();
    2.0 * (((tan_lat1 + tan_lat2) / (1.0 + tan_lat1 * tan_lat2)) * (lon12 / 2.0).tan()).atan()
}

/// Spherical excess of the edge from `(lon1, lat1)` to `(lon2, lat2)`,
/// in radians: an edge spanning half a turn of longitude counts `π`.
///
/// Mirrors `area_formulas::spherical<false>`
/// (`formulas/area_formulas.hpp:358-416`).
#[allow(
    clippy::float_cmp,
    clippy::similar_names,
    reason = "Boost compares the normalised difference with π exactly; the names are its formula's"
)]
pub(crate) fn edge_excess(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    let lon12 = longitude_distance_signed(lon1, lon2);
    if lon12 == PI || lon12 == -PI {
        return PI;
    }
    trapezoidal_excess(lat1, lat2, lon12)
}

/// Whether the edge between longitudes `lon1` and `lon2` (radians)
/// crosses the prime meridian; half a turn of longitude always does.
///
/// Mirrors `area_formulas::crosses_prime_meridian`
/// (`formulas/area_formulas.hpp:588-616`).
#[allow(
    clippy::float_cmp,
    clippy::similar_names,
    reason = "Boost compares the normalised difference with π exactly; the names are its formula's"
)]
pub(crate) fn crosses_prime_meridian(lon1: f64, lon2: f64) -> bool {
    let lon12 = longitude_distance_signed(lon1, lon2);
    if lon12 == PI || lon12 == -PI {
        return true;
    }
    let two_pi = 2.0 * PI;
    let p1_lon = lon1 - (lon1 / two_pi).floor() * two_pi;
    let p2_lon = lon2 - (lon2 / two_pi).floor() * two_pi;
    let max_lon = p1_lon.max(p2_lon);
    let min_lon = p1_lon.min(p2_lon);
    max_lon > PI && min_lon < PI && max_lon - min_lon > PI
}

/// The area of a ring whose edges sum to `sum` and cross the prime
/// meridian `crossings` times, where `full_turn` is the area a full turn
/// around a pole sweeps (`2π` times the squared radius).
///
/// An odd crossing count means the ring winds around a pole, so the
/// summed edges measured the area on the far side of it: the strategies
/// return the encircled area instead, with the sum's sign reversed
/// (`strategy/spherical/area.hpp:82-110`,
/// `strategy/geographic/area.hpp:142-179`).
pub(crate) fn pole_corrected(sum: f64, crossings: u32, full_turn: f64) -> f64 {
    if crossings % 2 == 0 {
        return sum;
    }
    let result = full_turn * f64::from(1 + crossings / 2) - sum.abs();
    if sum > 0.0 { -result } else { result }
}

#[cfg(test)]
mod tests {
    use super::{PI, crosses_prime_meridian, edge_excess, trapezoidal_excess};

    /// An edge spanning half a turn of longitude counts `π`, whatever its
    /// latitudes, as `area_formulas::spherical<false>` returns it.
    #[test]
    fn a_half_turn_edge_counts_pi() {
        assert_eq!(edge_excess(0.0, 0.3, PI, 0.5), PI);
        assert_eq!(edge_excess(PI, -0.2, 0.0, 0.4), PI);
        let quarter = edge_excess(0.0, 0.3, PI / 2.0, 0.5);
        assert!((quarter - trapezoidal_excess(0.3, 0.5, PI / 2.0)).abs() < 1e-15);
    }

    /// Half a turn of longitude always crosses the prime meridian; a short
    /// edge crosses it only when it straddles longitude zero.
    #[test]
    fn a_half_turn_edge_crosses_the_prime_meridian() {
        assert!(crosses_prime_meridian(0.0, PI));
        assert!(crosses_prime_meridian(PI, 0.0));
        assert!(crosses_prime_meridian(-0.1, 0.1));
        assert!(!crosses_prime_meridian(0.1, 0.2));
    }
}
