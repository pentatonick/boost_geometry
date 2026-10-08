//! Geographic (spheroidal) surface area.
//!
//! Mirrors `boost::geometry::strategy::area::geographic` from
//! `boost/geometry/strategy/geographic/area.hpp` at its defaults: Andoyer
//! azimuths and Andoyer's order-one series
//! (`strategies/geographic/parameters.hpp:186-189`). Each edge adds the
//! spherical excess of its geodesic on the authalic sphere and an
//! ellipsoidal correction, Danielsen's integral of the area under the
//! geodesic (`formulas/area_formulas.hpp:438-586`):
//!
//! ```text
//! A = c²·Σ excess + e²·a²·Σ correction
//! ```
//!
//! where `c` is the authalic radius, the radius of the sphere with the
//! spheroid's surface area. A short edge off the meridians takes the
//! trapezoidal excess, any other the difference of its Andoyer azimuths;
//! the correction is dropped when it exceeds a hundredth of the
//! spherical term, which Boost reads as an azimuth inaccuracy
//! (`strategy/geographic/area.hpp:142-179`). As on the sphere, a ring
//! whose edges cross the prime meridian an odd number of times winds
//! around a pole and measures the area it encircles.
//!
//! # Sign convention
//!
//! As [`SphericalArea`](crate::spherical::SphericalArea): positive for a
//! ring traversed in its declared order, negative for the opposite
//! traversal, and none for a ring below its minimum size.

use geometry_cs::Spheroid;
#[cfg(feature = "std")]
use geometry_cs::{CoordinateSystem, GeographicFamily};
#[cfg(feature = "std")]
use geometry_tag::SameAs;
#[cfg(feature = "std")]
use geometry_trait::{Point, Polygon, Ring};

#[cfg(feature = "std")]
use crate::area::AreaStrategy;

#[cfg(feature = "std")]
use geometry_coords::CoordinateScalar;

#[cfg(feature = "std")]
use crate::clockwise_view::clockwise_points;
#[cfg(feature = "std")]
use crate::geographic::Andoyer;
#[cfg(feature = "std")]
use crate::geographic::spheroid_calc::SpheroidCalc;
#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, longitude_distance_signed, lonlat_radians};
#[cfg(feature = "std")]
use crate::spherical_excess::{crosses_prime_meridian, pole_corrected, trapezoidal_excess};

/// Geographic surface area on a reference spheroid.
///
/// Carries the reference [`Spheroid`]; `Default::default()` produces
/// [`GeographicArea::WGS84`]. The area comes back in the squared unit
/// of the spheroid's radii (m² for WGS84).
///
/// Mirrors `boost::geometry::strategy::area::geographic<>`
/// (`strategy/geographic/area.hpp`), whose defaults are the Andoyer
/// formula and its order-one series.
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
}

impl Default for GeographicArea {
    #[inline]
    fn default() -> Self {
        Self::WGS84
    }
}

/// Geographic surface area for a [`Polygon`] — outer ring area plus the
/// sum of (oppositely-wound, hence negatively-signed) interior-ring
/// areas.
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

/// The constants Boost's geographic area strategy derives from the
/// spheroid once (`strategy/geographic/area.hpp:83-125`).
#[cfg(feature = "std")]
struct SpheroidConstants {
    /// The Andoyer formula the azimuths come from.
    andoyer: Andoyer,
    /// Squared equatorial radius.
    a2: f64,
    /// Squared eccentricity.
    e2: f64,
    /// Second eccentricity.
    ep: f64,
    /// Squared authalic radius.
    c2: f64,
    /// Flattening.
    f: f64,
    /// Mean radius `(2a + b) / 3`, against which an edge counts as short.
    mean_radius: f64,
    /// The order-one series coefficients, polynomials in the third
    /// flattening `n` (`area_formulas::evaluate_coeffs_n`,
    /// `formulas/area_formulas.hpp:182-186`).
    coefficients: [f64; 3],
}

#[cfg(feature = "std")]
impl SpheroidConstants {
    fn of(spheroid: Spheroid) -> Self {
        let calc = SpheroidCalc::from(spheroid);
        let a2 = calc.a * calc.a;
        let e2 = calc.e2;
        // `formula_dispatch::authalic_radius_sqr` (`formulas/authalic_radius_sqr.hpp`).
        let c2 = if e2.tolerant_eq(0.0) {
            a2
        } else {
            let e = e2.sqrt();
            (a2 / 2.0) * (1.0 + (1.0 - e2) * e.atanh() / e)
        };
        let n = calc.f / (2.0 - calc.f);
        Self {
            andoyer: Andoyer { spheroid },
            a2,
            e2,
            ep: calc.second_eccentricity_squared().sqrt(),
            c2,
            f: calc.f,
            mean_radius: (2.0 * calc.a + calc.b) / 3.0,
            coefficients: [(10.0 - 4.0 * n) / 15.0, -1.0 / 5.0, 1.0 / 45.0],
        }
    }

    /// The spherical excess and the ellipsoidal correction of the edge
    /// from `(lon1, lat1)` to `(lon2, lat2)`, in radians.
    ///
    /// Mirrors `area_formulas::ellipsoidal`
    /// (`formulas/area_formulas.hpp:438-586`) with `ExpandEpsN`.
    #[allow(
        clippy::similar_names,
        reason = "the names are those of `area_formulas::ellipsoidal`"
    )]
    fn edge(&self, lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> (f64, f64) {
        let pi = core::f64::consts::PI;
        let half_pi = core::f64::consts::FRAC_PI_2;
        let inverse = self.andoyer.inverse::<true>(lon1, lat1, lon2, lat2);
        let alp1 = inverse.azimuth;
        let alp2 = inverse.reverse_azimuth;

        // Reduced latitudes.
        let one_minus_f = 1.0 - self.f;
        let tan_bet1 = lat1.tan() * one_minus_f;
        let tan_bet2 = lat2.tan() * one_minus_f;
        let cos_bet1 = tan_bet1.atan().cos();
        let cos_bet2 = tan_bet2.atan().cos();
        let sin_bet1 = tan_bet1 * cos_bet1;
        let sin_bet2 = tan_bet2 * cos_bet2;

        let sin_alp1 = alp1.sin();
        let cos_alp1 = alp1.cos();
        let cos_alp2 = alp2.cos();
        let sin_alp0 = sin_alp1 * cos_bet1;

        // Boost compares with `==` here, noting it suffices for its tests.
        #[allow(
            clippy::float_cmp,
            reason = "Boost tests the folded longitude and the latitudes for exact equality"
        )]
        let excess = {
            let lon12 = longitude_distance_signed(lon1, lon2);
            if lon12 == pi || lon12 == -pi {
                pi
            } else {
                let meridian = lon12 == 0.0
                    || lat1 == half_pi
                    || lat1 == -half_pi
                    || lat2 == half_pi
                    || lat2 == -half_pi;
                if !meridian && inverse.distance < self.mean_radius / 638.0 {
                    trapezoidal_excess(lat1, lat2, lon12)
                } else {
                    alp2 - alp1
                }
            }
        };

        // The integral, expanded in `eps` and `n` and summed by Clenshaw.
        let cos_alp0 = (1.0 - sin_alp0 * sin_alp0).sqrt();
        let cos_sig1 = unit_cosine(sin_bet1, cos_alp1 * cos_bet1);
        let cos_sig2 = unit_cosine(sin_bet2, cos_alp2 * cos_bet2);
        let k2 = (self.ep * cos_alp0) * (self.ep * cos_alp0);
        let sqrt_k2_plus_one = (1.0 + k2).sqrt();
        let eps = (sqrt_k2_plus_one - 1.0) / (sqrt_k2_plus_one + 1.0);
        // `evaluate_coeffs_var2` (`formulas/area_formulas.hpp:332-345`):
        // Horner in `eps` over each row, the row scaled by `eps^i`.
        let [c0, c1, c2] = self.coefficients;
        let series = [c1 * eps + c0, eps * c2];
        let i12 = clenshaw_sum(cos_sig2, &series) - clenshaw_sum(cos_sig1, &series);
        (excess, cos_alp0 * sin_alp0 * i12)
    }
}

/// The cosine of the angle whose sine and cosine are proportional to
/// `sine` and `cosine` — `area_formulas::normalize`
/// (`formulas/area_formulas.hpp:94-100`) keeping the half it uses, with
/// the hypotenuse taken as `boost::math::hypot` takes it
/// (`boost/math/special_functions/hypot.hpp`).
#[cfg(feature = "std")]
fn unit_cosine(sine: f64, cosine: f64) -> f64 {
    let (mut x, mut y) = (sine.abs(), cosine.abs());
    if y > x {
        core::mem::swap(&mut x, &mut y);
    }
    let h = if x * f64::EPSILON >= y {
        x
    } else {
        let rat = y / x;
        x * (1.0 + rat * rat).sqrt()
    };
    cosine / h
}

/// `Σ coefficients[l]·cos((2l + 1)·x)` plus `coefficients[0]`, from
/// `cosx = cos(x)`.
///
/// Mirrors `area_formulas::clenshaw_sum`
/// (`formulas/area_formulas.hpp:73-92`), whose constant term cancels in
/// the difference the strategy takes.
#[cfg(feature = "std")]
fn clenshaw_sum(cosx: f64, coefficients: &[f64]) -> f64 {
    let mut index = coefficients.len();
    let mut odd = true;
    let mut b_k1 = 0.0;
    let mut b_k2 = 0.0;
    loop {
        let c_k = if odd {
            index -= 1;
            coefficients[index]
        } else {
            0.0
        };
        let b_k = c_k + 2.0 * cosx * b_k1 - b_k2;
        b_k2 = b_k1;
        b_k1 = b_k;
        odd = !odd;
        if index == 0 {
            break;
        }
    }
    coefficients[0] + b_k1 * cosx - b_k2
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

    /// Mirrors `strategy::area::geographic::apply` and `result`
    /// (`strategy/geographic/area.hpp:142-179`, `:194-235`): edges along a
    /// meridian add nothing, edges along the equator only count their
    /// prime-meridian crossings.
    #[inline]
    fn area(&self, r: &R) -> f64 {
        let constants = SpheroidConstants::of(self.spheroid);
        let mut excess_sum = 0.0;
        let mut correction_sum = 0.0;
        let mut crossings = 0;
        for edge in clockwise_points(r).windows(2) {
            let (first, second) = (edge[0], edge[1]);
            if first.get::<0>().tolerant_eq(second.get::<0>()) {
                continue;
            }
            let (lon1, lat1) = lonlat_radians(first);
            let (lon2, lat2) = lonlat_radians(second);
            if crosses_prime_meridian(lon1, lon2) {
                crossings += 1;
            }
            if !(first.get::<1>().tolerant_eq(0.0) && second.get::<1>().tolerant_eq(0.0)) {
                let (excess, correction) = constants.edge(lon1, lat1, lon2, lat2);
                excess_sum += excess;
                correction_sum += correction;
            }
        }
        let spherical_term = constants.c2 * excess_sum;
        let ellipsoidal_term = constants.e2 * constants.a2 * correction_sum;
        let sum = if (ellipsoidal_term / spherical_term).abs() > 0.01 {
            spherical_term
        } else {
            spherical_term + ellipsoidal_term
        };
        pole_corrected(sum, crossings, 2.0 * core::f64::consts::PI * constants.c2)
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
        // The interiors summed from zero, then added to the exterior:
        // `calculate_polygon_sum` (`algorithms/detail/calculate_sum.hpp:36-55`).
        let interiors = p.interiors().fold(0.0, |sum, inner| sum + ring.area(inner));
        ring.area(p.exterior()) + interiors
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference values from Boost (`aed7bc3`), `bg::area` on
    //! `bg::model::polygon<bg::model::point<double, 2,
    //! bg::cs::geographic<bg::degree>>>` with the default WGS84 spheroid.
    #![allow(
        clippy::float_cmp,
        reason = "areas are compared with an explicit relative tolerance, not `==`"
    )]
    #![allow(
        clippy::excessive_precision,
        reason = "reference values are copied verbatim from Boost's output"
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

    fn assert_close(got: f64, expected: f64) {
        assert!(
            (got - expected).abs() <= 1e-12 * expected.abs(),
            "got {got} expected {expected}"
        );
    }

    /// A 1° × 1° box at the equator, clockwise: `12_308_778_368.75034` m²
    /// in Boost, against `≈ 12_308_778_000` m² by the closed form of the
    /// area between two parallels and the geodesic's bulge.
    #[test]
    fn one_degree_box_near_equator_wgs84() {
        let r: Ring<Gg> = Ring::from_vec(vec![
            gg(0., 0.),
            gg(0., 1.),
            gg(1., 1.),
            gg(1., 0.),
            gg(0., 0.),
        ]);
        assert_close(GeographicArea::WGS84.area(&r), 12_308_778_368.750_34);
        let mut reversed = r.clone();
        reversed.0.reverse();
        assert_close(
            GeographicArea::WGS84.area(&reversed),
            -12_308_778_368.750_34,
        );
    }

    /// A ring around the north pole measures the cap it encloses:
    /// `2_507_270_792_087.1562` m² walked westward in Boost.
    #[test]
    fn a_ring_around_a_pole_has_the_area_it_encircles() {
        let r: Ring<Gg> = Ring::from_vec(vec![
            gg(0., 80.),
            gg(-90., 80.),
            gg(-180., 80.),
            gg(90., 80.),
            gg(0., 80.),
        ]);
        assert_close(GeographicArea::WGS84.area(&r), 2_507_270_792_087.156_2);
    }

    /// An octant `POLYGON((0 0,0 90,90 0,0 0))`: `63_758_202_715_511.047`
    /// m² in Boost, an eighth of the spheroid's surface.
    #[test]
    fn polygon_octant_is_an_eighth_of_the_spheroid() {
        let pg: Polygon<Gg> = Polygon::new(Ring::from_vec(vec![
            gg(0., 0.),
            gg(0., 90.),
            gg(90., 0.),
            gg(0., 0.),
        ]));
        assert_close(
            GeographicPolygonArea::WGS84.area(&pg),
            63_758_202_715_511.047,
        );
    }

    /// `Default` for both strategies is WGS84.
    #[test]
    fn default_is_wgs84() {
        assert_eq!(
            GeographicArea::default().spheroid.equatorial_radius,
            GeographicArea::WGS84.spheroid.equatorial_radius
        );
        assert_eq!(
            GeographicPolygonArea::default().spheroid.equatorial_radius,
            GeographicPolygonArea::WGS84.spheroid.equatorial_radius
        );
    }
}
