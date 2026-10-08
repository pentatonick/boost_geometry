//! Spherical surface area via the trapezoidal (spherical-excess) rule.
//!
//! Mirrors `boost::geometry::strategy::area::spherical` from
//! `boost/geometry/strategy/spherical/area.hpp` together with the
//! per-segment kernel `formula::area_formulas::spherical<false>` in
//! `boost/geometry/formulas/area_formulas.hpp:358-416`.
//!
//! For each polygon edge `(p1, p2)` whose endpoints differ in
//! longitude, Boost accumulates the segment's spherical excess via the
//! trapezoidal formula
//! (`area_formulas.hpp:347-355`):
//!
//! ```text
//! e = 2·atan( ((tan(lat1/2) + tan(lat2/2)) / (1 + tan(lat1/2)·tan(lat2/2)))
//!             · tan(Δlon/2) )
//! ```
//!
//! where `Δlon` is normalised to `(-π, π]`, and an edge spanning exactly
//! half a turn of longitude counts `π`. The running sum of excesses is
//! multiplied by `R²` to give the surface area
//! (`strategy/spherical/area.hpp:82-110`). On a unit sphere
//! (`radius = 1`) the result is the *solid angle* the polygon subtends;
//! a polygon covering `1/8` of the sphere returns `4π/8 = π/2`
//! (`test/algorithms/area/area_sph_geo.cpp:93-106`).
//!
//! A ring whose edges cross the prime meridian an odd number of times
//! winds around a pole; as in Boost, its area is then the part of the
//! sphere the ring encircles (`strategy/spherical/area.hpp:85-100`).
//!
//! # Sign convention
//!
//! Follows Boost: a ring traversed in its declared [`PointOrder`]
//! yields a positive area, the opposite traversal a negative one, and a
//! ring below its minimum size (four points closed, three open) has none
//! (`algorithms/area.hpp:82-118`). Holes contribute negatively, same as
//! the Cartesian shoelace.
//!
//! [`PointOrder`]: geometry_trait::PointOrder

#[cfg(feature = "std")]
use geometry_cs::{CoordinateSystem, SphericalFamily};
#[cfg(feature = "std")]
use geometry_tag::SameAs;
#[cfg(feature = "std")]
use geometry_trait::{Point, Polygon, Ring};

#[cfg(feature = "std")]
use crate::area::AreaStrategy;

use super::Haversine;

// Rust coherence cannot prove a single type is not both a `Ring` and a
// `Polygon`, so — exactly as the Cartesian `ShoelaceArea` /
// `ShoelacePolygonArea` split (see `crate::area` module docs) — the
// spherical area is two sibling types, one per geometry kind.

#[cfg(feature = "std")]
use geometry_coords::CoordinateScalar;

#[cfg(feature = "std")]
use crate::clockwise_view::clockwise_points;
#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};
#[cfg(feature = "std")]
use crate::spherical_excess::{crosses_prime_meridian, edge_excess, pole_corrected};

/// Spherical surface area via the trapezoidal spherical-excess rule.
///
/// Carries the sphere `radius`; the result of
/// [`AreaStrategy::area`] is in *squared radius units* (m² for
/// [`SphericalArea::EARTH`], steradians for [`SphericalArea::UNIT`]).
/// `Default::default()` produces [`SphericalArea::EARTH`].
///
/// Mirrors `boost::geometry::strategy::area::spherical<>` from
/// `strategy/spherical/area.hpp`.
#[derive(Debug, Clone, Copy)]
pub struct SphericalArea {
    /// Sphere radius. The area comes back in these units squared.
    pub radius: f64,
}

impl SphericalArea {
    /// The Earth sphere of [`Haversine::EARTH`], in metres, so a
    /// spherical polygon's default area and perimeter share one sphere.
    pub const EARTH: Self = Self {
        radius: Haversine::EARTH.radius,
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
    /// The Earth sphere, in metres. See [`SphericalArea::EARTH`].
    pub const EARTH: Self = Self {
        radius: SphericalArea::EARTH.radius,
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

// `std`-gated: the excess kernel needs `f64::tan`/`atan`, which
// `geometry-coords` does not shim under `libm`. Mirrors the identical
// gate on the geographic sibling (`geographic::area::GeographicArea`'s
// impl).
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

    /// Mirrors `strategy::area::spherical::apply` and `result`
    /// (`strategy/spherical/area.hpp:82-110`, `:134-160`): edges along a
    /// meridian add
    /// nothing, the rest add their excess and count their prime-meridian
    /// crossings.
    #[inline]
    fn area(&self, r: &R) -> f64 {
        let mut sum = 0.0;
        let mut crossings = 0;
        for edge in clockwise_points(r).windows(2) {
            let (first, second) = (edge[0], edge[1]);
            if first.get::<0>().tolerant_eq(second.get::<0>()) {
                continue;
            }
            let (lon1, lat1) = lonlat_radians(first);
            let (lon2, lat2) = lonlat_radians(second);
            sum += edge_excess(lon1, lat1, lon2, lat2);
            if crosses_prime_meridian(lon1, lon2) {
                crossings += 1;
            }
        }
        pole_corrected(sum, crossings, 2.0 * core::f64::consts::PI) * (self.radius * self.radius)
    }
}

// ---- Polygon ---------------------------------------------------------

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
        // The interiors summed from zero, then added to the exterior:
        // `calculate_polygon_sum` (`algorithms/detail/calculate_sum.hpp:36-55`).
        let interiors = p.interiors().fold(0.0, |sum, inner| sum + ring.area(inner));
        ring.area(p.exterior()) + interiors
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/area/area_sph_geo.cpp:93-116`.
    #![allow(
        clippy::float_cmp,
        reason = "areas are compared with an explicit relative tolerance, not `==`"
    )]
    #![allow(
        clippy::excessive_precision,
        reason = "reference values are copied verbatim from Boost's output"
    )]

    use super::{SphericalArea, SphericalPolygonArea};
    use crate::area::AreaStrategy;
    use crate::spherical::Haversine;
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

    /// Both strategies default to the Earth sphere the spherical distance
    /// and perimeter use.
    #[test]
    fn defaults_are_the_haversine_earth() {
        assert_eq!(SphericalArea::default().radius, Haversine::EARTH.radius);
        assert_eq!(
            SphericalPolygonArea::default().radius,
            Haversine::EARTH.radius
        );
    }

    /// A ring around the north pole crosses the prime meridian once, so
    /// its area is the cap it encloses: positive walked westward
    /// (clockwise seen from outside the sphere), negative eastward. Boost
    /// (`aed7bc3`) gives `±0.061232934148970131` on the unit sphere.
    #[test]
    fn a_ring_around_a_pole_has_the_area_it_encircles() {
        let west: Ring<Sp> = Ring::from_vec(vec![
            sp(0., 80.),
            sp(-90., 80.),
            sp(-180., 80.),
            sp(90., 80.),
            sp(0., 80.),
        ]);
        let mut east = west.clone();
        east.0.reverse();
        let expected = 0.061_232_934_148_970_131;
        let got = SphericalArea::UNIT.area(&west);
        assert!((got - expected).abs() < 1e-15, "got {got}");
        let got = SphericalArea::UNIT.area(&east);
        assert!((got + expected).abs() < 1e-15, "got {got}");
    }

    /// A closed ring of fewer than four points encloses nothing, as in
    /// Boost's `ring_area`.
    #[test]
    fn a_ring_too_short_has_no_area() {
        let short: Ring<Sp> = Ring::from_vec(vec![sp(0., 0.), sp(1., 1.), sp(0., 0.)]);
        assert_eq!(SphericalArea::UNIT.area(&short), 0.0);
        let two: Ring<Sp> = Ring::from_vec(vec![sp(0., 0.), sp(1., 1.)]);
        assert_eq!(SphericalArea::UNIT.area(&two), 0.0);
    }

    /// The interiors are summed from zero and then added to the exterior,
    /// as `calculate_polygon_sum` adds them
    /// (`algorithms/detail/calculate_sum.hpp:36-55`). The order is pinned
    /// bit-exactly against this platform's ring areas, since the last ulp of
    /// the trig differs between libms (glibc rounds to `…855`). Boost
    /// (`aed7bc3`, macOS), on the unit sphere: `0.030048013763217845`.
    #[test]
    fn interiors_are_summed_before_the_exterior() {
        let ring = |points: &[(f64, f64)]| -> Ring<Sp> {
            Ring::from_vec(points.iter().map(|&(lon, lat)| sp(lon, lat)).collect())
        };
        let outer = ring(&[(0., 0.), (0., 10.), (10., 10.), (10., 0.), (0., 0.)]);
        let first = ring(&[
            (2.298, 3.689),
            (2.59, 2.906),
            (3.999, 3.705),
            (2.298, 3.689),
        ]);
        let second = ring(&[
            (7.952, 6.907),
            (6.976, 7.459),
            (6.958, 6.582),
            (7.952, 6.907),
        ]);
        let ra = SphericalArea::UNIT;
        let expected = ra.area(&outer) + (ra.area(&first) + ra.area(&second));
        let pg: Polygon<Sp> = Polygon::with_inners(outer, vec![first, second]);
        let got = SphericalPolygonArea::UNIT.area(&pg);
        assert_eq!(got, expected);
        assert!((got - 0.030_048_013_763_217_845).abs() < 1e-16, "got {got}");
    }

    /// Holes contribute negatively: a polygon's area is its outer ring's
    /// less its interior rings'.
    #[test]
    fn polygon_area_subtracts_holes() {
        let outer = Ring::from_vec(vec![sp(0., 0.), sp(0., 90.), sp(90., 0.), sp(0., 0.)]);
        let hole: Ring<Sp> =
            Ring::from_vec(vec![sp(20., 20.), sp(40., 20.), sp(30., 40.), sp(20., 20.)]);
        let hole_area = SphericalArea::UNIT.area(&hole);
        assert!(
            hole_area < 0.0,
            "an oppositely wound hole has negative signed area"
        );
        let mut pg: Polygon<Sp> = Polygon::new(outer);
        pg.inners.push(hole);
        let got = SphericalPolygonArea::UNIT.area(&pg);
        let expected = core::f64::consts::FRAC_PI_2 - hole_area.abs();
        assert!(
            (got - expected).abs() < 1e-9,
            "got {got} expected {expected}"
        );
        assert!(got < core::f64::consts::FRAC_PI_2 - 0.01);
    }
}
