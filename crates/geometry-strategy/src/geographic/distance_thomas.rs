//! Thomas geographic distance on a reference spheroid.
//!
//! Mirrors `boost::geometry::strategy::distance::thomas<Spheroid, T>`
//! from `strategies/geographic/distance_thomas.hpp`. The underlying
//! arithmetic comes from `formulas/thomas_inverse.hpp` — a
//! Forsyth–Andoyer–Lambert type approximation with second-order
//! terms (Paul D. Thomas, 1965 / 1970).
//!
//! Thomas sits between Andoyer (first-order) and Vincenty (iterative
//! exact) on the speed / accuracy curve: faster than Vincenty,
//! noticeably more precise than Andoyer on long lines.
//!
//! # Calculation-type policy
//!
//! Following Andoyer (T43) and Haversine (T40), we hardcode
//! `Scalar = f64` on both inputs so the kernel reaches for `f64::sin`
//! / `cos` / `acos` / `tan` / `atan` directly without growing the
//! [`CoordinateScalar`](geometry_coords::CoordinateScalar) trait
//! surface.
//!
//! `#[cfg(feature = "std")]` gates the impl: the standard library
//! provides the trig functions as inherent methods on `f64`. A
//! `no_std` build of `geometry-strategy` (default-features off) does
//! not get Thomas; that mirrors the same gate Andoyer / Haversine
//! use.
//!
//! # Comparable form
//!
//! Thomas has no useful "skip the sqrt" form — like Andoyer, the
//! corrections involve `acos` and additive flattening terms that
//! cannot be shed while preserving ordering. We follow Boost
//! (`strategies/geographic/distance_thomas.hpp:83-87`) and set
//! `type Comparable = Self;`.
//!
//! # Default-strategy registration
//!
//! Andoyer is the Geographic × Geographic default; Thomas is
//! explicitly opt-in. We deliberately do **not** implement
//! [`DefaultDistance`](crate::distance::DefaultDistance) here.

use geometry_cs::{CoordinateSystem, GeographicFamily, Spheroid};
use geometry_tag::SameAs;
use geometry_trait::Point;

use crate::distance::DistanceStrategy;

#[cfg(feature = "std")]
use crate::geographic::spheroid_calc::SpheroidCalc;
#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Thomas geographic distance on a reference spheroid.
///
/// Inputs follow the [`Geographic<U>`](geometry_cs::Geographic)
/// equatorial convention — see its rustdoc.
///
/// Mirrors `boost::geometry::strategy::distance::thomas<Spheroid, T>`
/// from `strategies/geographic/distance_thomas.hpp:46-64`. The
/// spheroid is supplied at construction and the output is in metres
/// (or whatever units the spheroid's equatorial radius is expressed
/// in).
///
/// The underlying arithmetic mirrors
/// `boost::geometry::formula::thomas_inverse::apply` from
/// `formulas/thomas_inverse.hpp:59-215` — the distance-only branch
/// (`EnableDistance = true`, all azimuth / reduced-length / scale
/// flags false).
///
/// # Antipodal / pole-to-pole limitation
///
/// Like Boost's raw `thomas_inverse`, this strategy returns **`0.0`**
/// for antipodal or pole-to-pole endpoints: at a central angle of `π`
/// the series' `sin(d)` term vanishes and the formula degenerates
/// (`formulas/thomas_inverse.hpp:120-125` returns a zero distance).
/// Boost's higher-level geographic strategy masks this with a
/// `meridian_inverse` fallback ladder; that fallback is deferred here,
/// so a caller measuring a near-antipodal line with `Thomas` gets `0`
/// silently. Use [`Andoyer`](super::Andoyer) (no such degeneracy) or
/// [`Haversine`](crate::spherical::Haversine) for antipodal inputs.
#[derive(Debug, Clone, Copy)]
pub struct Thomas {
    /// Reference ellipsoid the distance is measured on.
    pub spheroid: Spheroid,
}

impl Thomas {
    /// Thomas parameterised by the WGS84 reference ellipsoid — the
    /// default for nearly every real geographic dataset. Matches the
    /// default-constructed `srs::spheroid<RadiusType>` Boost uses when
    /// `thomas<>` is built without arguments
    /// (`strategies/geographic/distance_thomas.hpp:57-59`).
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };
}

impl Default for Thomas {
    #[inline]
    fn default() -> Self {
        Self::WGS84
    }
}

// ---- DistanceStrategy impl ------------------------------------------
//
// The `SameAs<GeographicFamily>` bounds on both points enforce the
// geographic-only rule. A caller wiring a Cartesian or Spherical point
// through here by mistake gets the `#[diagnostic::on_unimplemented]`
// plate on `geometry_tag::SameAs` pointing them at
// `WithCs<_, Geographic<…>>` or at the Cartesian / Spherical
// strategies; that is the same redirect plate Andoyer / Haversine
// rely on.

/// Thomas on `f64` geographic points.
///
/// Mirrors `formula::thomas_inverse<CT, true, false>::apply` at
/// `formulas/thomas_inverse.hpp:59-215` — the distance-only branch
/// (`EnableDistance = true`, all other flags false). The full
/// derivation lives in Paul D. Thomas, *Mathematical Models for
/// Navigation Systems*, 1965, and *Spheroidal Geodesics, Reference
/// Systems, and Local Geometry*, 1970.
///
/// # Diagnostics on mis-paired CS
///
/// A caller who pairs a Cartesian or Spherical point with [`Thomas`]
/// hits the `<P::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>`
/// bound below and gets the redirect plate on
/// [`geometry_tag::SameAs`] pointing them at
/// `WithCs<_, Geographic<…>>` or at the Cartesian / Spherical
/// strategies. See T31 and proposal §3.7.
#[cfg(feature = "std")]
impl<P1, P2> DistanceStrategy<P1, P2> for Thomas
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    P1::Cs: HasAngularUnits,
    P2::Cs: HasAngularUnits,
    <P1::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
{
    type Out = f64;
    type Comparable = Self;

    // `many_single_char_names`, `float_cmp`: the single-letter names
    // `H, L, U, V, X, Y, T, D, E, A, B, C, F, G, M, Q` and the exact
    // `== 0.0` / `== same-lonlat` checks mirror
    // `formula::thomas_inverse::apply` in
    // `formulas/thomas_inverse.hpp:69-153` letter-for-letter; the
    // exact-equality short-circuit is the intentional analogue of
    // Boost's `math::equals` against zero on the same line numbers.
    #[allow(
        clippy::many_single_char_names,
        clippy::similar_names,
        clippy::float_cmp
    )]
    #[inline]
    fn distance(&self, a: &P1, b: &P2) -> Self::Out {
        let calc = SpheroidCalc::from(self.spheroid);
        let (lon1, lat1) = lonlat_radians(a);
        let (lon2, lat2) = lonlat_radians(b);

        // Mirrors the `math::equals(lon1, lon2) && math::equals(lat1, lat2)`
        // short-circuit at `formulas/thomas_inverse.hpp:70-73`.
        if lon1 == lon2 && lat1 == lat2 {
            return 0.0;
        }

        let f = calc.f;
        let one_minus_f = 1.0 - f;

        let pi_half = core::f64::consts::FRAC_PI_2;

        // Reduced latitudes θ = atan((1 − f) · tan(lat)), with the
        // pole short-circuit from `formulas/thomas_inverse.hpp:89-94`.
        let theta1 = if lat1 == pi_half || lat1 == -pi_half {
            lat1
        } else {
            (one_minus_f * lat1.tan()).atan()
        };
        let theta2 = if lat2 == pi_half || lat2 == -pi_half {
            lat2
        } else {
            (one_minus_f * lat2.tan()).atan()
        };

        let theta_m = f64::midpoint(theta1, theta2);
        let d_theta_m = (theta2 - theta1) / 2.0;
        let d_lambda = lon2 - lon1;
        let d_lambda_m = d_lambda / 2.0;

        let sin_theta_m = theta_m.sin();
        let cos_theta_m = theta_m.cos();
        let sin_d_theta_m = d_theta_m.sin();
        let cos_d_theta_m = d_theta_m.cos();
        let sin2_theta_m = sin_theta_m * sin_theta_m;
        let cos2_theta_m = cos_theta_m * cos_theta_m;
        let sin2_d_theta_m = sin_d_theta_m * sin_d_theta_m;
        let cos2_d_theta_m = cos_d_theta_m * cos_d_theta_m;
        let sin_d_lambda_m = d_lambda_m.sin();
        let sin2_d_lambda_m = sin_d_lambda_m * sin_d_lambda_m;

        // Auxiliary quantities. Mirrors
        // `formulas/thomas_inverse.hpp:112-116`.
        let upper_h = cos2_theta_m - sin2_d_theta_m;
        let l_term = sin2_d_theta_m + upper_h * sin2_d_lambda_m;
        let cos_d = (1.0 - 2.0 * l_term).clamp(-1.0, 1.0);
        let d = cos_d.acos();
        let sin_d = d.sin();

        let one_minus_l = 1.0 - l_term;

        // Degenerate guards from `formulas/thomas_inverse.hpp:120-125`:
        // `sin_d == 0` ⇒ coincident / antipodal where d=0 or π;
        // `L == 0` or `1 − L == 0` would divide by zero below.
        if sin_d == 0.0 || l_term == 0.0 || one_minus_l == 0.0 {
            return 0.0;
        }

        let upper_u = 2.0 * sin2_theta_m * cos2_d_theta_m / one_minus_l;
        let upper_v = 2.0 * sin2_d_theta_m * cos2_theta_m / l_term;
        let upper_x = upper_u + upper_v;
        let upper_y = upper_u - upper_v;
        let upper_t = d / sin_d;
        let upper_d = 4.0 * upper_t * upper_t;
        let upper_e = 2.0 * cos_d;
        let upper_a = upper_d * upper_e;
        let upper_b = 2.0 * upper_d;
        let upper_c = upper_t - (upper_a - upper_e) / 2.0;

        let f_sqr = f * f;
        let f_sqr_per_64 = f_sqr / 64.0;

        // Distance branch. Mirrors
        // `formulas/thomas_inverse.hpp:141-154`.
        let n1 = upper_x * (upper_a + upper_c * upper_x);
        let n2 = upper_y * (upper_b + upper_e * upper_y);
        let n3 = upper_d * upper_x * upper_y;

        let delta1d = f * (upper_t * upper_x - upper_y) / 4.0;
        let delta2d = f_sqr_per_64 * (n1 - n2 + n3);

        calc.a * sin_d * (upper_t - delta1d + delta2d)
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        *self
    }
}

// ---- Tests ----------------------------------------------------------

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference values come from
    //! `geometry/test/strategies/thomas.cpp:107-112`. The Boost test
    //! uses `BOOST_CHECK_CLOSE(..., 0.001)` (0.001%) — we match the
    //! same relative tolerance here, which works out to a few metres
    //! on a 1000 km baseline.
    //!
    //! We also cross-check against Andoyer on the normal mid-latitude
    //! case to confirm the second-order Thomas correction stays
    //! within a sensible band of Andoyer's first-order result.

    use super::Thomas;
    use crate::distance::DistanceStrategy;
    use crate::geographic::Andoyer;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Geographic};

    type GP = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;

    #[inline]
    fn deg(lon: f64, lat: f64) -> GP {
        WithCs::new(Adapt([lon, lat]))
    }

    /// `test/strategies/thomas.cpp:109` — polar case
    /// `(0, 90) → (1, 80) ≈ 1116.825795 km`.
    #[test]
    fn polar_north_1deg_lon_10deg_lat() {
        let d = Thomas::WGS84.distance(&deg(0.0, 90.0), &deg(1.0, 80.0));
        // Boost's BOOST_CHECK_CLOSE(_, 0.001) ≈ 0.001 % → ~11 m here.
        assert!(
            (d / 1000.0 - 1_116.825_795).abs() < 0.012,
            "{} km expected ~ 1116.825795 km",
            d / 1000.0
        );
    }

    /// `test/strategies/thomas.cpp:110` — southern polar mirror
    /// `(0, -90) → (1, -80) ≈ 1116.825795 km`.
    #[test]
    fn polar_south_1deg_lon_10deg_lat() {
        let d = Thomas::WGS84.distance(&deg(0.0, -90.0), &deg(1.0, -80.0));
        assert!(
            (d / 1000.0 - 1_116.825_795).abs() < 0.012,
            "{} km expected ~ 1116.825795 km",
            d / 1000.0
        );
    }

    /// `test/strategies/thomas.cpp:111` — zero distance on equal
    /// points, mirroring the
    /// `math::equals(lon1, lon2) && math::equals(lat1, lat2)`
    /// short-circuit at `formulas/thomas_inverse.hpp:70-73`.
    #[test]
    fn zero_distance_on_equal_points() {
        let p = deg(4.0, 52.0);
        let d = Thomas::WGS84.distance(&p, &p);
        assert!(d.abs() < 1e-6, "got {d}");
    }

    /// `test/strategies/thomas.cpp:112` — normal case:
    /// `(4, 52) → (3, 40) ≈ 1336.025365 km`.
    #[test]
    fn lon_4_lat_52_to_lon_3_lat_40() {
        let d = Thomas::WGS84.distance(&deg(4.0, 52.0), &deg(3.0, 40.0));
        // Boost's BOOST_CHECK_CLOSE(_, 0.001) ≈ 0.001 % → ~13 m here.
        assert!(
            (d / 1000.0 - 1_336.025_365).abs() < 0.014,
            "{} km expected ~ 1336.025365 km",
            d / 1000.0
        );
    }

    /// Cross-check: Thomas (second-order) and Andoyer (first-order)
    /// should disagree by only tens of metres on a normal mid-latitude
    /// 1336 km line. Bigger gaps would flag a coding error in the
    /// second-order correction translated from
    /// `formulas/thomas_inverse.hpp:141-153`.
    #[test]
    fn thomas_within_50m_of_andoyer_amsterdam_to_madrid() {
        let a = deg(4.0, 52.0);
        let b = deg(3.0, 40.0);
        let t = Thomas::WGS84.distance(&a, &b);
        let an = Andoyer::WGS84.distance(&a, &b);
        assert!(
            (t - an).abs() < 50.0,
            "Thomas {t} m vs Andoyer {an} m differs by {} m",
            (t - an).abs()
        );
    }

    /// Thomas's default constructor selects WGS84 — mirrors Boost's
    /// `thomas()` no-arg constructor at
    /// `strategies/geographic/distance_thomas.hpp:57-59`.
    #[test]
    fn default_is_wgs84() {
        let a = Thomas::default();
        let w = Thomas::WGS84;
        assert_eq!(a.spheroid, w.spheroid);
    }

    // KC1.T2 witness: proves this strategy accepts a read-only `Point`
    // (one that need not implement `PointMut`). If it compiles, the
    // read-only bound is locked.
    fn _accepts_readonly_point<P, S>(s: &S, a: &P, b: &P) -> S::Out
    where
        P: geometry_trait::Point,
        S: DistanceStrategy<P, P>,
    {
        s.distance(a, b)
    }



    /// `comparable()` returns a strategy producing the same value — the
    /// geodesic formulas have no sqrt to skip, so the comparable form is
    /// the strategy itself.
    #[test]
    fn comparable_produces_the_same_distance() {
        let a = deg(4.0, 52.0);
        let b = deg(3.0, 40.0);
        let real = Thomas::WGS84.distance(&a, &b);
        let cmp = DistanceStrategy::<GP, GP>::comparable(&Thomas::WGS84).distance(&a, &b);
        assert!((real - cmp).abs() < 1e-9);
    }

    /// The read-only-point witness computes a distance when invoked.
    #[test]
    #[allow(clippy::used_underscore_items, reason = "the test exists to run the compile-time witness's body")]
    fn readonly_witness_computes_distance() {
        let d = _accepts_readonly_point(&Thomas::WGS84, &deg(4.0, 52.0), &deg(3.0, 40.0));
        assert!(d > 1_000_000.0, "≈1336 km, got {d}");
    }
}
