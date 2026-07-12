//! Reference ellipsoid for geographic coordinate systems.
//!
//! Mirrors `boost::geometry::srs::spheroid<RadiusType>` from
//! `boost/geometry/srs/spheroid.hpp`. Boost stores the spheroid as a
//! pair of radii (`m_a`, `m_b`) and exposes them via a
//! `get_radius<I>()` template; we store the *flattening* alongside the
//! equatorial radius instead, because every geodesic formula
//! (Andoyer, Vincenty, Thomas) needs `f` directly — keeping it as the
//! primary stored field avoids the round-trip through `(a − b) / a`
//! that Boost's strategies do at every call site.
//!
//! `WGS84` is the only reference we ship in this task; other ellipsoids
//! (GRS80, IUGG67, …) land alongside the geographic strategies in T42.

/// Reference ellipsoid for geographic coordinate systems.
///
/// Field semantics match Boost's `srs::spheroid<T>`
/// (`boost/geometry/srs/spheroid.hpp:49-112`), except the second
/// stored value is the *flattening* instead of the polar radius — the
/// two forms are interconvertible via [`polar_radius`] /
/// [`flattening`] but `f` is what the geodesic formulas actually take
/// as input.
///
/// [`polar_radius`]: Spheroid::polar_radius
/// [`flattening`]: Spheroid::flattening
///
/// # Examples
///
/// ```
/// use geometry_cs::Spheroid;
/// let s = Spheroid::WGS84;
/// assert_eq!(s.equatorial_radius, 6_378_137.0);
/// // The WGS84 polar radius is ~6 356 752.3 m.
/// assert!((s.polar_radius() - 6_356_752.314_245).abs() < 0.01);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spheroid {
    /// Equatorial (semi-major) radius `a`, in metres.
    pub equatorial_radius: f64,
    /// Flattening `f = (a − b) / a`, dimensionless.
    pub flattening: f64,
}

impl Spheroid {
    /// The WGS84 reference ellipsoid.
    ///
    /// Equatorial radius and flattening per the WGS84 defining
    /// parameters; matches the default-constructed
    /// `srs::spheroid<RadiusType>` in
    /// `boost/geometry/srs/spheroid.hpp:62-69`, which seeds
    /// `m_a = 6_378_137.0` and `m_b = 6_356_752.314_245_179_3`.
    pub const WGS84: Self = Self {
        equatorial_radius: 6_378_137.0,
        flattening: 1.0 / 298.257_223_563,
    };

    /// Semi-minor axis `b = a · (1 − f)`, in metres.
    ///
    /// Counterpart to `srs::spheroid<T>::get_radius<2>()` in
    /// `boost/geometry/srs/spheroid.hpp:78-92`.
    #[inline]
    #[must_use]
    pub fn polar_radius(&self) -> f64 {
        self.equatorial_radius * (1.0 - self.flattening)
    }

    /// First eccentricity squared `e² = 2f − f²`.
    ///
    /// Used by the geodesic strategies (Andoyer / Vincenty / Thomas)
    /// in `boost/geometry/strategies/geographic/*.hpp`. The identity
    /// `e² = 2f − f²` follows from `b = a(1 − f)` and
    /// `e² = (a² − b²) / a²`.
    #[inline]
    #[must_use]
    pub fn eccentricity_squared(&self) -> f64 {
        2.0 * self.flattening - self.flattening * self.flattening
    }
}
