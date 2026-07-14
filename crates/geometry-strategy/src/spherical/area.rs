//! Spherical surface area via the trapezoidal (spherical-excess) rule.
//!
//! Mirrors `boost::geometry::strategy::area::spherical` from
//! `boost/geometry/strategies/spherical/area.hpp` together with the
//! per-segment kernel `formula::area_formulas::spherical<false>` in
//! `boost/geometry/formulas/area_formulas.hpp:365-416`.
//!
//! For each polygon edge `(p1, p2)` whose endpoints differ in
//! longitude, Boost accumulates the segment's spherical excess via the
//! trapezoidal formula
//! (`area_formulas.hpp:347-356`):
//!
//! ```text
//! e = 2·atan( ((tan(lat1/2) + tan(lat2/2)) / (1 + tan(lat1/2)·tan(lat2/2)))
//!             · tan(Δlon/2) )
//! ```
//!
//! where `Δlon` is normalised to `(-π, π]`. The running sum of excesses
//! is multiplied by `R²` to give the surface area
//! (`strategies/spherical/area.hpp:80-107`). On a unit sphere
//! (`radius = 1`) the result is the *solid angle* the polygon subtends;
//! a polygon covering `1/8` of the sphere returns `4π/8 = π/2`
//! (`test/algorithms/area/area_sph_geo.cpp:93-106`).
//!
//! # Sign convention
//!
//! Follows the Cartesian [`ShoelaceArea`](crate::area::ShoelaceArea):
//! a ring traversed in its declared [`PointOrder`] yields a positive
//! area, the opposite traversal a negative one — Boost's
//! `closed_clockwise_view` wrap, expressed here as a sign flip for a
//! [`PointOrder::CounterClockwise`] ring.
//!
//! # Caveats
//!
//! This implements Boost's `LongSegment = false` branch — the default.
//! The pole-encircling correction (`m_crosses_prime_meridian`) is *not*
//! reproduced: polygons that wind around a pole or cross the antimeridian
//! an odd number of times are out of scope, matching the "convex
//! spherical polygon" clamp documented on the spherical centroid. Holes
//! contribute negatively, same as the Cartesian shoelace.

use geometry_cs::{CoordinateSystem, SphericalFamily};
use geometry_tag::SameAs;
use geometry_trait::{Closure, Point, PointOrder, Polygon, Ring};

use crate::area::AreaStrategy;

// Rust coherence cannot prove a single type is not both a `Ring` and a
// `Polygon`, so — exactly as the Cartesian `ShoelaceArea` /
// `ShoelacePolygonArea` split (see `crate::area` module docs) — the
// spherical area is two sibling types, one per geometry kind.

#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Spherical surface area via the trapezoidal spherical-excess rule.
///
/// Carries the sphere `radius`; the result of
/// [`AreaStrategy::area`] is in *squared radius units* (m² for
/// [`SphericalArea::EARTH`], steradians for [`SphericalArea::UNIT`]).
/// `Default::default()` produces [`SphericalArea::EARTH`].
///
/// Mirrors `boost::geometry::strategy::area::spherical<>` from
/// `strategies/spherical/area.hpp`.
#[derive(Debug, Clone, Copy)]
pub struct SphericalArea {
    /// Sphere radius. The area comes back in these units squared.
    pub radius: f64,
}

impl SphericalArea {
    /// Mean Earth radius in metres, matching Boost's spherical Earth
    /// convention (`test/algorithms/area/area_sph_geo.cpp` uses
    /// per-test radii; `6_371_000` m is the WGS84 mean radius).
    pub const EARTH: Self = Self {
        radius: 6_371_000.0,
    };

    /// Unit sphere (`radius = 1`): the area is then the solid angle
    /// (steradians) the polygon subtends
    /// (`area_sph_geo.cpp:93-106`).
    pub const UNIT: Self = Self { radius: 1.0 };
}

impl Default for SphericalArea {
    #[inline]
    fn default() -> Self {
        Self::EARTH
    }
}

/// Spherical surface area for a [`Polygon`] — outer ring area plus the
/// sum of (oppositely-wound, hence negatively-signed) interior-ring
/// areas.
///
/// Separate from [`SphericalArea`] for the same coherence reason
/// [`ShoelacePolygonArea`](crate::area::ShoelacePolygonArea) is
/// separate from [`ShoelaceArea`](crate::area::ShoelaceArea).
#[derive(Debug, Clone, Copy)]
pub struct SphericalPolygonArea {
    /// Sphere radius, forwarded to the per-ring [`SphericalArea`].
    pub radius: f64,
}

impl SphericalPolygonArea {
    /// Mean Earth radius in metres. See [`SphericalArea::EARTH`].
    pub const EARTH: Self = Self {
        radius: 6_371_000.0,
    };

    /// Unit sphere. See [`SphericalArea::UNIT`].
    pub const UNIT: Self = Self { radius: 1.0 };
}

impl Default for SphericalPolygonArea {
    #[inline]
    fn default() -> Self {
        Self::EARTH
    }
}

// ---- Ring ------------------------------------------------------------

// `std`-gated: the excess kernel it calls (`excess_accumulator` /
// `segment_excess`) needs `f64::tan`/`atan`, which `geometry-coords`
// does not shim under `libm`. Mirrors the identical gate on the
// geographic sibling (`geographic::area::GeographicArea`'s impl).
#[cfg(feature = "std")]
impl<R> AreaStrategy<R> for SphericalArea
where
    R: Ring,
    <R::Point as Point>::Cs: CoordinateSystem,
    <<R::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
    R::Point: Point<Scalar = f64>,
    <R::Point as Point>::Cs: HasAngularUnits,
{
    type Out = f64;

    #[inline]
    fn area(&self, r: &R) -> f64 {
        let excess = excess_accumulator::<R>(r);
        let signed = excess * self.radius * self.radius;
        match r.point_order() {
            PointOrder::Clockwise => signed,
            PointOrder::CounterClockwise => -signed,
        }
    }
}

// ---- Polygon ---------------------------------------------------------
//
// Mirrors Boost's polygon area recursion: outer ring positive, each
// inner ring subtracted. Interior rings are conventionally wound
// opposite the exterior, so `ShoelaceArea`'s "plain sum of signed ring
// areas" trick applies here too — the excess of an oppositely-wound
// hole already carries the opposite sign.

#[cfg(feature = "std")]
impl<P> AreaStrategy<P> for SphericalPolygonArea
where
    P: Polygon,
    SphericalArea: AreaStrategy<P::Ring, Out = f64>,
{
    type Out = f64;

    #[inline]
    fn area(&self, p: &P) -> f64 {
        let ring = SphericalArea {
            radius: self.radius,
        };
        let mut total = ring.area(p.exterior());
        for inner in p.interiors() {
            total += ring.area(inner);
        }
        total
    }
}

/// Sum the per-segment spherical excess over the consecutive vertex
/// pairs of `r`, mirroring the `apply` accumulation in
/// `strategies/spherical/area.hpp:127-150` and the `spherical<false>`
/// kernel in `area_formulas.hpp:365-416`.
///
/// For an open ring the implicit `last -> first` closing pair is added
/// explicitly, mirroring the way [`crate::area`] closes an open ring.
///
/// The kernel only reads `(lon, lat)` in radians, so it is *family-
/// agnostic* — the geographic authalic-sphere area
/// ([`crate::geographic::GeographicArea`]) reuses it on the authalic
/// sphere. The family fences live on the public `AreaStrategy` impls,
/// not here.
#[cfg(feature = "std")]
#[inline]
pub(crate) fn excess_accumulator<R>(r: &R) -> f64
where
    R: Ring,
    R::Point: Point<Scalar = f64>,
    <R::Point as Point>::Cs: HasAngularUnits,
{
    let mut acc = 0.0;
    let it = r.points();
    let next = it.clone().skip(1);
    for (a, b) in it.zip(next) {
        acc += segment_excess::<R::Point>(a, b);
    }
    if matches!(r.closure(), Closure::Open) {
        let mut first_it = r.points();
        if let Some(first) = first_it.next() {
            if let Some(last) = r.points().last() {
                acc += segment_excess::<R::Point>(last, first);
            }
        }
    }
    acc
}

/// One segment's spherical excess via Boost's trapezoidal formula
/// (`area_formulas.hpp:347-356`, `:395-403`). Returns `0` for a
/// meridional edge (`lon1 == lon2`), mirroring the `! math::equals(
/// get<0>(p1), get<0>(p2))` guard at
/// `strategies/spherical/area.hpp:129`.
// The `lon1 == lon2` check mirrors Boost's `! math::equals(get<0>(p1),
// get<0>(p2))` guard at `strategies/spherical/area.hpp:129`
// letter-for-letter — an intentional exact float comparison, the same
// stance the Andoyer distance kernel takes.
#[allow(clippy::float_cmp)]
#[cfg(feature = "std")]
#[inline]
fn segment_excess<P>(a: &P, b: &P) -> f64
where
    P: Point<Scalar = f64>,
    P::Cs: HasAngularUnits,
{
    let (lon1, lat1) = lonlat_radians(a);
    let (lon2, lat2) = lonlat_radians(b);

    // Meridional segments contribute no excess — mirrors the
    // `! equals(get<0>(p1), get<0>(p2))` guard in area.hpp:129.
    if lon1 == lon2 {
        return 0.0;
    }

    // Δlon normalised to (-π, π], mirroring
    // `math::normalize_longitude` in area_formulas.hpp:378.
    let mut dlon = lon2 - lon1;
    let pi = core::f64::consts::PI;
    let two_pi = 2.0 * pi;
    while dlon > pi {
        dlon -= two_pi;
    }
    while dlon <= -pi {
        dlon += two_pi;
    }

    let tan_lat1 = (lat1 / 2.0).tan();
    let tan_lat2 = (lat2 / 2.0).tan();
    2.0 * (((tan_lat1 + tan_lat2) / (1.0 + tan_lat1 * tan_lat2)) * (dlon / 2.0).tan()).atan()
}

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/area/area_sph_geo.cpp:93-116`.
    #![allow(
        clippy::float_cmp,
        reason = "areas are compared with an explicit relative tolerance, not `==`"
    )]

    use super::{SphericalArea, SphericalPolygonArea};
    use crate::area::AreaStrategy;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Spherical};
    use geometry_model::{Polygon, Ring};

    type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;

    #[inline]
    fn sp(lon: f64, lat: f64) -> Sp {
        WithCs::new(Adapt([lon, lat]))
    }

    /// `area_sph_geo.cpp:93-106` — `POLYGON((0 0,0 90,90 0,0 0))` on a
    /// unit sphere covers `1/8` of it: `4π/8 = π/2 ≈ 1.5708`.
    #[test]
    fn unit_sphere_octant_is_pi_over_2() {
        let r: Ring<Sp> = Ring::from_vec(vec![sp(0., 0.), sp(0., 90.), sp(90., 0.), sp(0., 0.)]);
        let got = SphericalArea::UNIT.area(&r);
        let expected = core::f64::consts::FRAC_PI_2;
        assert!(
            (got - expected).abs() / expected < 1e-6,
            "got {got} expected {expected}"
        );
    }

    /// `area_sph_geo.cpp:109-116` — the same octant on a radius-2
    /// sphere scales by `2² = 4`.
    #[test]
    fn radius_2_sphere_octant_scales_by_4() {
        let r: Ring<Sp> = Ring::from_vec(vec![sp(0., 0.), sp(0., 90.), sp(90., 0.), sp(0., 0.)]);
        let got = SphericalArea { radius: 2.0 }.area(&r);
        let expected = 4.0 * core::f64::consts::FRAC_PI_2;
        assert!(
            (got - expected).abs() / expected < 1e-6,
            "got {got} expected {expected}"
        );
    }

    /// The same octant traversed in the opposite direction on a
    /// default (clockwise) ring yields a negated area — mirrors the
    /// Cartesian `ShoelaceArea` sign convention.
    #[test]
    fn reversed_octant_is_negative() {
        let r: Ring<Sp> = Ring::from_vec(vec![sp(0., 0.), sp(90., 0.), sp(0., 90.), sp(0., 0.)]);
        let got = SphericalArea::UNIT.area(&r);
        let expected = -core::f64::consts::FRAC_PI_2;
        assert!((got - expected).abs() / expected.abs() < 1e-6, "got {got}");
    }

    /// Polygon path: octant with no holes equals the ring area.
    #[test]
    fn polygon_octant_matches_ring() {
        let pg: Polygon<Sp> = Polygon::new(Ring::from_vec(vec![
            sp(0., 0.),
            sp(0., 90.),
            sp(90., 0.),
            sp(0., 0.),
        ]));
        let got = SphericalPolygonArea::UNIT.area(&pg);
        let expected = core::f64::consts::FRAC_PI_2;
        assert!((got - expected).abs() / expected < 1e-6, "got {got}");
    }

    /// Both strategies default to the mean-Earth radius.
    #[test]
    fn defaults_are_mean_earth_radius() {
        assert_eq!(SphericalArea::default().radius, 6_371_000.0);
        assert_eq!(SphericalPolygonArea::default().radius, 6_371_000.0);
    }

}
