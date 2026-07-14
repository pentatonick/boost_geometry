//! Latitude and longitude of a geodesic vertex.
//!
//! Ports Boost.Geometry's `vertex_latitude.hpp` and
//! `vertex_longitude.hpp`. A vertex is the point at which a geodesic reaches
//! its most northerly or southerly latitude.

use geometry_cs::Spheroid;

/// Vertex latitude for a great circle on a sphere.
#[cfg(feature = "std")]
#[must_use]
pub fn spherical_vertex_latitude(latitude1: f64, azimuth1: f64) -> f64 {
    (latitude1.cos() * azimuth1.sin())
        .abs()
        .clamp(0.0, 1.0)
        .acos()
}

/// Vertex latitude for a geodesic on a spheroid.
#[cfg(feature = "std")]
#[must_use]
pub fn geographic_vertex_latitude(latitude1: f64, azimuth1: f64, spheroid: Spheroid) -> f64 {
    let one_minus_f = 1.0 - spheroid.flattening;
    let reduced_latitude1 = (one_minus_f * latitude1.tan()).atan();
    let reduced_vertex = spherical_vertex_latitude(reduced_latitude1, azimuth1);
    (reduced_vertex.tan() / one_minus_f).atan()
}

/// Vertex longitude for the great circle through two ordered endpoints.
#[cfg(feature = "std")]
#[must_use]
pub fn spherical_vertex_longitude(
    longitude1: f64,
    latitude1: f64,
    longitude2: f64,
    latitude2: f64,
    vertex_latitude: f64,
) -> f64 {
    vertex_longitude(
        longitude1,
        latitude1,
        longitude2,
        latitude2,
        vertex_latitude,
        spherical_vertex_delta(
            latitude1,
            latitude2,
            vertex_latitude,
            (longitude1 - longitude2).sin(),
            (longitude1 - longitude2).cos(),
        ),
    )
}

/// Vertex longitude for a spheroidal geodesic through ordered endpoints.
///
/// `azimuth1` is the forward azimuth at `(longitude1, latitude1)`.
#[cfg(feature = "std")]
#[must_use]
#[allow(
    clippy::too_many_arguments,
    reason = "the public formula follows Boost's two endpoints, vertex, azimuth, and spheroid inputs"
)]
pub fn geographic_vertex_longitude(
    longitude1: f64,
    latitude1: f64,
    longitude2: f64,
    latitude2: f64,
    vertex_latitude: f64,
    azimuth1: f64,
    spheroid: Spheroid,
) -> f64 {
    let delta = geographic_vertex_delta(latitude1, latitude2, vertex_latitude, azimuth1, spheroid);
    vertex_longitude(
        longitude1,
        latitude1,
        longitude2,
        latitude2,
        vertex_latitude,
        delta,
    )
}

#[cfg(feature = "std")]
fn spherical_vertex_delta(
    latitude1: f64,
    latitude2: f64,
    vertex_latitude: f64,
    sin_longitude12: f64,
    cos_longitude12: f64,
) -> f64 {
    let a = latitude1.sin() * latitude2.cos() * vertex_latitude.cos() * sin_longitude12;
    let b = latitude1.sin() * latitude2.cos() * vertex_latitude.cos() * cos_longitude12
        - latitude1.cos() * latitude2.sin() * vertex_latitude.cos();
    b.atan2(a) + core::f64::consts::PI
}

#[cfg(feature = "std")]
#[allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    reason = "sigma/omega-indexed symbols follow vertex_longitude_on_spheroid in Boost"
)]
fn geographic_vertex_delta(
    latitude1: f64,
    latitude2: f64,
    vertex_latitude: f64,
    azimuth1: f64,
    spheroid: Spheroid,
) -> f64 {
    let half_pi = core::f64::consts::FRAC_PI_2;
    if (latitude1.abs() - half_pi).abs() <= 1e-12 || (latitude2.abs() - half_pi).abs() <= 1e-12 {
        return 0.0;
    }

    let pi = core::f64::consts::PI;
    let f = spheroid.flattening;
    let one_minus_f = 1.0 - f;
    let beta1 = (one_minus_f * latitude1.tan()).atan();
    let beta2 = (one_minus_f * latitude2.tan()).atan();
    let beta3 = (one_minus_f * vertex_latitude.tan()).atan();
    let mut cos_beta1 = beta1.cos();
    let mut cos_beta2 = beta2.cos();
    let sin_beta1 = beta1.sin();
    let sin_beta2 = beta2.sin();
    let sin_beta3 = beta3.sin();
    let mut omega12 = 0.0;
    if beta1 < 0.0 {
        cos_beta1 = -cos_beta1;
        omega12 += pi;
    }
    if beta2 < 0.0 {
        cos_beta2 = -cos_beta2;
        omega12 += pi;
    }

    let sin_alpha1 = azimuth1.sin();
    let cos_alpha1 = (1.0 - sin_alpha1 * sin_alpha1).max(0.0).sqrt();
    let norm = (cos_alpha1 * cos_alpha1 + sin_alpha1 * sin_alpha1 * sin_beta1 * sin_beta1).sqrt();
    let sin_alpha0 = (sin_alpha1 * cos_beta1).atan2(norm).sin();
    let sin_alpha2 = (sin_alpha1 * cos_beta1 / cos_beta2).clamp(-1.0, 1.0);
    let cos_alpha0 = (1.0 - sin_alpha0 * sin_alpha0).max(0.0).sqrt();
    let cos_alpha2 = (1.0 - sin_alpha2 * sin_alpha2).max(0.0).sqrt();
    let sigma1 = sin_beta1.atan2(cos_alpha1 * cos_beta1);
    let sigma2 = sin_beta2.atan2(-cos_alpha2 * cos_beta2);
    let cos_sigma1 = sigma1.cos();
    let sin_sigma1 = (1.0 - cos_sigma1 * cos_sigma1).max(0.0).sqrt();
    let cos_sigma2 = sigma2.cos();
    let sin_sigma2 = (1.0 - cos_sigma2 * cos_sigma2).max(0.0).sqrt();
    let omega1 = (sin_alpha0 * sin_sigma1).atan2(cos_sigma1);
    let omega2 = (sin_alpha0 * sin_sigma2).atan2(cos_sigma2);
    omega12 += omega1 - omega2;

    let mut omega13 = spherical_vertex_delta(beta1, beta2, beta3, omega12.sin(), omega12.cos());
    if latitude1 * latitude2 < 0.0 && (latitude2 - latitude1) * vertex_latitude > 0.0 {
        omega13 = pi - omega13;
    }

    let e2 = f * (2.0 - f);
    let ep = (e2 / (1.0 - e2)).sqrt();
    let k2 = (ep * cos_alpha0) * (ep * cos_alpha0);
    let root = (1.0 + k2).sqrt();
    let epsilon = (root - 1.0) / (root + 1.0);
    let epsilon2 = epsilon * epsilon;
    let n = f / (2.0 - f);
    let sigma3 = if sin_beta3 > 0.0 { half_pi } else { -half_pi };
    let mut sigma13 = sigma3 - sigma1;
    if sigma13 > pi {
        sigma13 -= 2.0 * pi;
    }
    let a3 = 1.0 - (0.5 - 0.5 * n) * epsilon - 0.25 * epsilon2;
    let c31 = (0.25 - 0.25 * n) * epsilon + 0.125 * epsilon2;
    let c32 = 0.0625 * epsilon2;
    let sin2_sigma1 = 2.0 * cos_sigma1 * sin_sigma1;
    let sin4_sigma1 = sin_sigma1 * (-4.0 * cos_sigma1 + 8.0 * cos_sigma1.powi(3));
    let i3 = a3 * (sigma13 - c31 * sin2_sigma1 - c32 * sin4_sigma1);
    let sign = if beta3 >= 0.0 { 1.0 } else { -1.0 };
    omega13 - sign * f * sin_alpha0 * i3
}

#[cfg(feature = "std")]
fn vertex_longitude(
    longitude1: f64,
    latitude1: f64,
    longitude2: f64,
    latitude2: f64,
    vertex_latitude: f64,
    delta: f64,
) -> f64 {
    if (vertex_latitude - latitude1).abs() <= 1e-12 {
        return normalize_longitude(longitude1);
    }
    if (vertex_latitude - latitude2).abs() <= 1e-12 {
        return normalize_longitude(longitude2);
    }
    if (longitude1 - longitude2).abs() <= 1e-12 {
        return normalize_longitude(longitude1);
    }
    let mut longitude = (longitude1 + delta).rem_euclid(core::f64::consts::TAU);
    if vertex_latitude < 0.0 {
        longitude -= core::f64::consts::PI;
    }
    if (longitude1 - longitude2).abs() > core::f64::consts::PI {
        longitude -= core::f64::consts::PI;
    }
    normalize_longitude(longitude)
}

#[cfg(feature = "std")]
fn normalize_longitude(longitude: f64) -> f64 {
    let pi = core::f64::consts::PI;
    (longitude + pi).rem_euclid(core::f64::consts::TAU) - pi
}
