//! Derived ellipsoid quantities cached for the geographic distance
//! strategies.
//!
//! Boost's geodesic formulas
//! (`boost/geometry/formulas/andoyer_inverse.hpp`,
//! `boost/geometry/formulas/vincenty_inverse.hpp`,
//! `boost/geometry/formulas/thomas_inverse.hpp`) each pull the same
//! handful of values off a `srs::spheroid<T>` at every call site —
//! semi-major axis `a`, semi-minor axis `b`, flattening `f`, and the
//! first eccentricity squared `e²`. We precompute them once when a
//! geographic strategy is built so the inner loops just read fields.

use geometry_cs::Spheroid;

/// Cached / derived spheroid quantities used across geographic
/// distance algorithms.
///
/// Mirrors the per-call expansions in
/// `boost/geometry/formulas/{andoyer_inverse, vincenty_inverse,
/// thomas_inverse}.hpp` (each invokes `formula::flattening<CT>(sph)`
/// and reads `get_radius<0>` / `get_radius<2>` directly).
#[derive(Debug, Clone, Copy)]
pub(crate) struct SpheroidCalc {
    /// Equatorial (semi-major) radius `a`, in metres.
    pub(crate) a: f64,
    /// Polar (semi-minor) radius `b = a · (1 − f)`, in metres.
    #[allow(dead_code, reason = "consumed by Vincenty / Thomas in T44+")]
    pub(crate) b: f64,
    /// Flattening `f = (a − b) / a`, dimensionless.
    pub(crate) f: f64,
    /// First eccentricity squared `e² = 2f − f²`.
    #[allow(dead_code, reason = "consumed by Vincenty / Thomas in T44+")]
    pub(crate) e2: f64,
}

impl SpheroidCalc {
    /// Precompute the derived quantities for `s`.
    ///
    /// Counterpart to the bag of `CT const f = ...; CT const a = ...;`
    /// expansions Boost performs at the top of every geodesic inverse
    /// formula.
    pub(crate) fn from(s: Spheroid) -> Self {
        Self {
            a: s.equatorial_radius,
            b: s.polar_radius(),
            f: s.flattening,
            e2: s.eccentricity_squared(),
        }
    }

    /// Second eccentricity squared `e'² = e² / (1 − e²)`.
    ///
    /// Used by Andoyer / Thomas series expansions; corresponds to
    /// `ep2` in `boost/geometry/formulas/thomas_inverse.hpp`.
    #[allow(dead_code, reason = "consumed by distance strategies landing in T43+")]
    pub(crate) fn second_eccentricity_squared(&self) -> f64 {
        self.e2 / (1.0 - self.e2)
    }
}

#[cfg(test)]
mod tests {
    use super::SpheroidCalc;
    use geometry_cs::Spheroid;

    #[test]
    fn wgs84_derived_constants() {
        let c = SpheroidCalc::from(Spheroid::WGS84);
        assert!((c.b - 6_356_752.314_245).abs() < 1e-3);
        // Reference e² for WGS84: 0.006_694_379_990_141_316
        assert!((c.e2 - 0.006_694_379_990_141_316).abs() < 1e-15);
    }
}
