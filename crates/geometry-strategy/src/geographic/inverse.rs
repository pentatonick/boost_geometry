//! Shared result shape for inverse geodesic formulas.
//!
//! Mirrors `boost::geometry::formula::result_inverse<CT>` from
//! `formulas/result_inverse.hpp:25-57`.

/// Distance and endpoint azimuths produced by an inverse geodesic solution.
///
/// Angular values are radians; distance uses the spheroid radius unit.
/// Mirrors `formula::result_inverse` from `formulas/result_inverse.hpp:31-48`,
/// with an additional convergence flag for iterative Rust solvers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InverseResult {
    /// Shortest geodesic distance.
    pub distance: f64,
    /// Forward azimuth at the first point.
    pub azimuth: f64,
    /// Final/reverse azimuth at the second point.
    pub reverse_azimuth: f64,
    /// Whether the inverse iteration met its angular convergence threshold.
    pub converged: bool,
    /// Reduced geodesic length in the spheroid's radius unit.
    pub reduced_length: f64,
    /// Dimensionless forward geodesic scale.
    pub geodesic_scale: f64,
}

impl Default for InverseResult {
    fn default() -> Self {
        Self {
            distance: 0.0,
            azimuth: 0.0,
            reverse_azimuth: 0.0,
            converged: false,
            reduced_length: 0.0,
            geodesic_scale: 1.0,
        }
    }
}
