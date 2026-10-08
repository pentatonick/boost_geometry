//! The direct problem on a sphere: where a great circle leads.
//!
//! Mirrors `boost::geometry::formula::spherical_direct` from
//! `formulas/spherical.hpp:220-280`.

#[cfg(not(feature = "std"))]
use geometry_coords::math::Float;

use crate::normalise::normalize_angle_cond;

/// The point `sig12` radians along the great circle leaving `(lon1, lat1)`
/// at azimuth `alp1`, as `(lon2, lat2)` in radians.
///
/// Mirrors `formula::spherical_direct<true, false>`
/// (`formulas/spherical.hpp:220-280`) for the coordinates, its distance
/// already divided by the sphere's radius, the longitude brought back into
/// `(−π, π]` by one turn at most.
#[allow(
    clippy::similar_names,
    reason = "the names are those of `formula::spherical_direct`"
)]
pub(crate) fn spherical_direct(lon1: f64, lat1: f64, sig12: f64, alp1: f64) -> (f64, f64) {
    let sin_alp1 = alp1.sin();
    let sin_lat1 = lat1.sin();
    let cos_alp1 = alp1.cos();
    let cos_lat1 = lat1.cos();

    let norm = (cos_alp1 * cos_alp1 + sin_alp1 * sin_alp1 * sin_lat1 * sin_lat1).sqrt();
    let alp0 = (sin_alp1 * cos_lat1).atan2(norm);
    let sig1 = sin_lat1.atan2(cos_alp1 * cos_lat1);
    let sig2 = sig1 + sig12;

    let cos_sig2 = sig2.cos();
    let sin_alp0 = alp0.sin();
    let cos_alp0 = alp0.cos();
    let sin_sig2 = sig2.sin();
    let sin_sig1 = sig1.sin();
    let cos_sig1 = sig1.cos();

    let norm2 = (cos_alp0 * cos_alp0 * cos_sig2 * cos_sig2 + sin_alp0 * sin_alp0).sqrt();
    let lat2 = (cos_alp0 * sin_sig2).atan2(norm2);
    let omg1 = (sin_alp0 * sin_sig1).atan2(cos_sig1);
    let lon2 = (sin_alp0 * sin_sig2).atan2(cos_sig2);

    (normalize_angle_cond(lon1 + lon2 - omg1), lat2)
}
