//! Geographic (spheroidal) surface area via the authalic-sphere
//! approximation.
//!
//! Mirrors the *intent* of
//! `boost::geometry::strategy::area::geographic` from
//! `boost/geometry/strategies/geographic/area.hpp`, but **not** its full
//! numerics. Boost's geographic area is a Gauss–Legendre / Clenshaw
//! series over the spheroid
//! (`boost/geometry/formulas/area_formulas.hpp`, ~400 lines of
//! coefficient tables plus a per-segment geodesic-inverse call). That
//! series is out of scope here.
//!
//! Instead this strategy maps the ellipsoid onto its **authalic sphere**
//! — the sphere of radius `R_A` that has the *same total surface area*
//! as the ellipsoid — and applies the exact spherical trapezoidal
//! excess rule on it (the same kernel as
//! [`SphericalArea`](crate::spherical::SphericalArea)):
//!
//! ```text
//! R_A = sqrt( a²/2 · (1 + (1 − e²)/e · atanh(e)) )
//! A   = R_A² · Σ trapezoidal_excess(edgeᵢ)
//! ```
//!
//! # Precision
//!
//! The authalic-sphere area is a standard, *correct* approximation (it
//! preserves the ellipsoid's total surface area exactly), but it is not
//! Boost's Vincenty-grade series. For a 1° × 1° box near the equator on
//! WGS84 it returns ≈ `12_364` km² against Boost's ≈ `12_309` km² — a
//! ~0.45 % divergence. Callers needing exact spheroidal area should wait
//! for the series port; this strategy is the honest, low-error default
//! so the family dispatch (`area(&geographic_polygon)`) has a working
//! implementation rather than a `todo!`.

use geometry_cs::{CoordinateSystem, GeographicFamily, Spheroid};
use geometry_tag::SameAs;
use geometry_trait::{Point, PointOrder, Polygon, Ring};

use crate::area::AreaStrategy;

#[cfg(feature = "std")]
use crate::normalise::HasAngularUnits;
#[cfg(feature = "std")]
use crate::spherical::area::excess_accumulator;

/// Geographic surface area via the authalic-sphere approximation.
///
/// Carries the reference [`Spheroid`]; `Default::default()` produces
/// [`GeographicArea::WGS84`]. See the module docs for the precision
/// caveat.
#[derive(Debug, Clone, Copy)]
pub struct GeographicArea {
    /// Reference ellipsoid the area is measured on.
    pub spheroid: Spheroid,
}

impl GeographicArea {
    /// Area on the WGS84 reference ellipsoid — the default for nearly
    /// every real geographic dataset (matches `Andoyer::WGS84`).
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };

    /// The authalic radius `R_A` of this spheroid — the radius of the
    /// sphere with the same total surface area as the ellipsoid.
    #[cfg(feature = "std")]
    #[inline]
    #[must_use]
    fn authalic_radius(&self) -> f64 {
        let a = self.spheroid.equatorial_radius;
        let e2 = self.spheroid.eccentricity_squared();
        let e = e2.sqrt();
        // R_A = sqrt( a²/2 · (1 + (1 − e²)/e · atanh(e)) ).
        (a * a / 2.0 * (1.0 + (1.0 - e2) / e * e.atanh())).sqrt()
    }
}

impl Default for GeographicArea {
    #[inline]
    fn default() -> Self {
        Self::WGS84
    }
}

/// Geographic surface area for a [`Polygon`] — the authalic-sphere
/// approximation applied per ring (outer positive, holes negative).
///
/// Separate from [`GeographicArea`] for the same Rust-coherence reason
/// [`SphericalPolygonArea`](crate::spherical::SphericalPolygonArea) is
/// separate from [`SphericalArea`](crate::spherical::SphericalArea).
#[derive(Debug, Clone, Copy)]
pub struct GeographicPolygonArea {
    /// Reference ellipsoid, forwarded to the per-ring [`GeographicArea`].
    pub spheroid: Spheroid,
}

impl GeographicPolygonArea {
    /// Area on the WGS84 reference ellipsoid. See [`GeographicArea::WGS84`].
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };
}

impl Default for GeographicPolygonArea {
    #[inline]
    fn default() -> Self {
        Self::WGS84
    }
}

// ---- Ring ------------------------------------------------------------

#[cfg(feature = "std")]
impl<R> AreaStrategy<R> for GeographicArea
where
    R: Ring,
    <R::Point as Point>::Cs: CoordinateSystem,
    <<R::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
    R::Point: Point<Scalar = f64>,
    <R::Point as Point>::Cs: HasAngularUnits,
{
    type Out = f64;

    #[inline]
    fn area(&self, r: &R) -> f64 {
        // Exact spherical trapezoidal excess evaluated on the authalic
        // sphere. The excess kernel is family-agnostic (it only reads
        // radian lon/lat), so it feeds a geographic ring identically;
        // the `SameAs<GeographicFamily>` fence above is what enforces
        // the geographic-only rule on this public strategy.
        let radius = self.authalic_radius();
        let signed = excess_accumulator::<R>(r) * radius * radius;
        match r.point_order() {
            PointOrder::Clockwise => signed,
            PointOrder::CounterClockwise => -signed,
        }
    }
}

// ---- Polygon ---------------------------------------------------------

#[cfg(feature = "std")]
impl<P> AreaStrategy<P> for GeographicPolygonArea
where
    P: Polygon,
    GeographicArea: AreaStrategy<P::Ring, Out = f64>,
{
    type Out = f64;

    #[inline]
    fn area(&self, p: &P) -> f64 {
        let ring = GeographicArea {
            spheroid: self.spheroid,
        };
        let mut total = ring.area(p.exterior());
        for inner in p.interiors() {
            total += ring.area(inner);
        }
        total
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/area/area_geo.cpp` — a 1° × 1°
    //! box near the equator on WGS84 ≈ `12_309` km². The authalic-sphere
    //! approximation lands within ~0.5 %.
    #![allow(
        clippy::float_cmp,
        reason = "areas are compared with an explicit relative tolerance, not `==`"
    )]

    use super::{GeographicArea, GeographicPolygonArea};
    use crate::area::AreaStrategy;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Geographic};
    use geometry_model::{Polygon, Ring};

    type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;

    #[inline]
    fn gg(lon: f64, lat: f64) -> Gg {
        WithCs::new(Adapt([lon, lat]))
    }

    /// `area_geo.cpp` — 1° × 1° box near the equator ≈ `12_309` km²
    /// (12.309e9 m²). Approximation tolerance 2 %.
    #[test]
    fn one_degree_box_near_equator_wgs84() {
        let r: Ring<Gg> = Ring::from_vec(vec![
            gg(0., 0.),
            gg(1., 0.),
            gg(1., 1.),
            gg(0., 1.),
            gg(0., 0.),
        ]);
        // The ring above is wound so that the trapezoidal excess is
        // negative on a default (clockwise) ring; take the magnitude.
        let got = GeographicArea::WGS84.area(&r).abs();
        let expected = 12_309e6;
        assert!(
            (got - expected).abs() / expected < 0.02,
            "got {} km² expected ~12309 km²",
            got / 1e6
        );
    }

    /// The polygon path matches the ring path (no holes).
    #[test]
    fn polygon_box_matches_ring() {
        let pg: Polygon<Gg> = Polygon::new(Ring::from_vec(vec![
            gg(0., 0.),
            gg(1., 0.),
            gg(1., 1.),
            gg(0., 1.),
            gg(0., 0.),
        ]));
        let got = GeographicPolygonArea::WGS84.area(&pg).abs();
        let expected = 12_309e6;
        assert!(
            (got - expected).abs() / expected < 0.02,
            "got {}",
            got / 1e6
        );
    }
}
