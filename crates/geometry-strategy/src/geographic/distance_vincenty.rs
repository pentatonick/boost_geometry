//! Vincenty inverse geodesic distance on a reference spheroid.
//!
//! Mirrors `boost::geometry::strategy::distance::vincenty<Spheroid, T>`
//! from `strategies/geographic/distance_vincenty.hpp`. The underlying
//! iterative inverse formula lives in
//! `boost/geometry/formulas/vincenty_inverse.hpp` — see Vincenty (1975)
//! and the references collected in Boost's doc-block at
//! `formulas/vincenty_inverse.hpp:38-47`. Vincenty is slower than
//! Andoyer (it iterates λ until convergence) but is the gold-standard
//! ellipsoidal geodesic on non-antipodal inputs.
//!
//! # Calculation-type policy
//!
//! Like [`Andoyer`](super::Andoyer), this implementation hardcodes
//! `Scalar = f64` on both inputs (the T40 / T43 convention). The
//! kernel reaches for `f64::sin` / `cos` / `atan2` / `sqrt` directly
//! without growing the [`CoordinateScalar`](geometry_coords::CoordinateScalar)
//! trait surface; mixed-scalar support folds in alongside the
//! `Promote` lattice once a real caller appears.
//!
//! `#[cfg(feature = "std")]` gates the impl: the standard library
//! provides the trig and `sqrt` functions as inherent methods on `f64`.
//! A `no_std` build of `geometry-strategy` (default-features off) does
//! not get Vincenty; that mirrors the same gate Andoyer / Haversine
//! use.
//!
//! # Comparable form
//!
//! There is no useful "skip the sqrt" form of Vincenty — the kernel
//! is dominated by trig calls and an iterative refinement rather than
//! a trailing `sqrt`. We follow Boost
//! (`strategies/geographic/distance_vincenty.hpp:85-89`) and set
//! `type Comparable = Self;`.
//!
//! # Default strategy slot
//!
//! Vincenty deliberately does *not* implement
//! `DefaultDistance<GeographicFamily>` — Andoyer holds that slot
//! (matches Boost's `services::default_strategy` for the geographic
//! tag). Callers opt into Vincenty via
//! `geometry_algorithm::distance_with(a, b, Vincenty::WGS84)`.

use geometry_cs::{CoordinateSystem, GeographicFamily, Spheroid};
use geometry_tag::SameAs;
use geometry_trait::Point;

use crate::distance::DistanceStrategy;

#[cfg(feature = "std")]
use crate::geographic::spheroid_calc::SpheroidCalc;
#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Vincenty inverse geodesic distance on a reference spheroid.
///
/// Inputs follow the [`Geographic<U>`](geometry_cs::Geographic)
/// equatorial convention — see its rustdoc.
///
/// Mirrors `boost::geometry::strategy::distance::vincenty<Spheroid, T>`
/// from `strategies/geographic/distance_vincenty.hpp:42-66`. The
/// spheroid is supplied at construction and the output is in metres
/// (or whatever units the spheroid's equatorial radius is expressed
/// in).
///
/// The iterative arithmetic mirrors
/// `boost::geometry::formula::vincenty_inverse::apply` from
/// `formulas/vincenty_inverse.hpp:67-188` — the distance-only branch
/// (`EnableDistance = true`, all other flags false).
#[derive(Debug, Clone, Copy)]
pub struct Vincenty {
    /// Reference ellipsoid the distance is measured on.
    pub spheroid: Spheroid,
    /// Maximum iterations before bailing out near antipodal points.
    /// Boost's default is 1000 — see
    /// `BOOST_GEOMETRY_DETAIL_VINCENTY_MAX_STEPS` at
    /// `formulas/vincenty_inverse.hpp:30-32`.
    pub max_iterations: u32,
    /// Convergence threshold on λ in radians. Boost hardcodes
    /// `c_e_12 = 1e-12` at `formulas/vincenty_inverse.hpp:87`.
    pub tolerance: f64,
}

impl Vincenty {
    /// Vincenty parameterised by the WGS84 reference ellipsoid — the
    /// default ellipsoid for nearly every real geographic dataset.
    /// Matches the default-constructed `srs::spheroid<RadiusType>`
    /// Boost uses when `vincenty<>` is built without arguments
    /// (`strategies/geographic/distance_vincenty.hpp:59-61`).
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
        max_iterations: 1000,
        tolerance: 1e-12,
    };
}

impl Default for Vincenty {
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

/// Vincenty on `f64` geographic points.
///
/// Mirrors `formula::vincenty_inverse<CT, true, false>::apply` at
/// `formulas/vincenty_inverse.hpp:67-188` — the distance-only branch.
/// Iterates λ on the auxiliary sphere until either successive λ
/// differ by less than `self.tolerance`, `|λ|` exits the (−π, π)
/// interval, or `self.max_iterations` is reached, then evaluates the
/// arc length on the ellipsoid via the standard A / B / Δσ series.
///
/// # Diagnostics on mis-paired CS
///
/// A caller who pairs a Cartesian or Spherical point with [`Vincenty`]
/// hits the `<P::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>`
/// bound below and gets the redirect plate on
/// [`geometry_tag::SameAs`] pointing them at
/// `WithCs<_, Geographic<…>>` or at the Cartesian / Spherical
/// strategies. See T31 and proposal §3.7.
#[cfg(feature = "std")]
impl<P1, P2> DistanceStrategy<P1, P2> for Vincenty
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

    // `many_single_char_names`, `float_cmp`, `similar_names`: the
    // single-letter names `A, B, C, L, u_sq`, the `sin_sigma` /
    // `sin2_sigma` / `cos_2sigma_m` / `cos2_2sigma_m` family, and the
    // exact `== same-lonlat` short-circuit mirror
    // `formula::vincenty_inverse::apply` in
    // `formulas/vincenty_inverse.hpp:76-187` letter-for-letter; the
    // exact-equality early-out is the intentional analogue of Boost's
    // `math::equals` against the same inputs at lines 76-79.
    #[allow(
        clippy::many_single_char_names,
        clippy::float_cmp,
        clippy::similar_names
    )]
    #[inline]
    fn distance(&self, a: &P1, b: &P2) -> Self::Out {
        let calc = SpheroidCalc::from(self.spheroid);
        let (lon1, lat1) = lonlat_radians(a);
        let (lon2, lat2) = lonlat_radians(b);

        // Mirrors the `math::equals(lat1, lat2) && math::equals(lon1, lon2)`
        // short-circuit at `formulas/vincenty_inverse.hpp:76-79`.
        if lon1 == lon2 && lat1 == lat2 {
            return 0.0;
        }

        let pi = core::f64::consts::PI;
        let two_pi = 2.0 * pi;

        // λ: difference in longitude on an auxiliary sphere. Mirrors
        // `formulas/vincenty_inverse.hpp:92-97`.
        let mut big_l = lon2 - lon1;
        if big_l < -pi {
            big_l += two_pi;
        }
        if big_l > pi {
            big_l -= two_pi;
        }
        let mut lambda = big_l;

        let f = calc.f;
        let a_radius = calc.a;
        let b_radius = calc.b;

        // U: reduced latitude, tan U = (1 − f) tan φ. Mirrors
        // `formulas/vincenty_inverse.hpp:103-117`.
        let one_min_f = 1.0 - f;
        let tan_u1 = one_min_f * lat1.tan();
        let tan_u2 = one_min_f * lat2.tan();

        let cos_u1 = 1.0 / (1.0 + tan_u1 * tan_u1).sqrt();
        let cos_u2 = 1.0 / (1.0 + tan_u2 * tan_u2).sqrt();
        let sin_u1 = tan_u1 * cos_u1;
        let sin_u2 = tan_u2 * cos_u2;

        // Iteration state carried out of the do-while loop. Mirrors
        // the declarations at `formulas/vincenty_inverse.hpp:127-137`.
        // The loop runs at least once (do-while shape), so these are
        // always written before they are read after the break.
        let mut sin_sigma;
        let mut cos_sigma;
        let mut sigma;
        let mut sin_alpha;
        let mut cos2_alpha;
        let mut cos_2sigma_m;
        let mut cos2_2sigma_m;

        // do-while loop from `formulas/vincenty_inverse.hpp:139-160`.
        // We translate the `do { … } while (cond)` directly into a
        // `loop { …; if !cond { break; } }` to keep the iteration
        // count and the break predicate aligned with the C++.
        let mut counter: u32 = 0;
        loop {
            let previous_lambda = lambda;
            let sin_lambda = lambda.sin();
            let cos_lambda = lambda.cos();

            // (14) sin σ
            let sx = cos_u2 * sin_lambda;
            let sy = cos_u1 * sin_u2 - sin_u1 * cos_u2 * cos_lambda;
            sin_sigma = (sx * sx + sy * sy).sqrt();

            // (15) cos σ
            cos_sigma = sin_u1 * sin_u2 + cos_u1 * cos_u2 * cos_lambda;

            // (17) sin α
            sin_alpha = if sin_sigma == 0.0 {
                0.0
            } else {
                cos_u1 * cos_u2 * sin_lambda / sin_sigma
            };
            cos2_alpha = 1.0 - sin_alpha * sin_alpha;

            // (18) cos 2σ_m — guard the equatorial line (cos²α == 0)
            // exactly as `formulas/vincenty_inverse.hpp:148`.
            cos_2sigma_m = if cos2_alpha == 0.0 {
                0.0
            } else {
                cos_sigma - 2.0 * sin_u1 * sin_u2 / cos2_alpha
            };
            cos2_2sigma_m = cos_2sigma_m * cos_2sigma_m;

            // (10) C
            let c = f / 16.0 * cos2_alpha * (4.0 + f * (4.0 - 3.0 * cos2_alpha));

            // (16) σ
            sigma = sin_sigma.atan2(cos_sigma);

            // (11) λ ← L + (1 − C) f sinα (σ + C sinσ (cos 2σ_m + C cosσ (−1 + 2 cos² 2σ_m)))
            lambda = big_l
                + (1.0 - c)
                    * f
                    * sin_alpha
                    * (sigma
                        + c * sin_sigma
                            * (cos_2sigma_m + c * cos_sigma * (-1.0 + 2.0 * cos2_2sigma_m)));

            counter += 1;

            // Termination matches `formulas/vincenty_inverse.hpp:158-160`:
            //   * converged on λ (Δλ ≤ tolerance), or
            //   * |λ| escaped the principal branch (anti-meridian
            //     wrap — near-antipodal stress case, handled by the
            //     meridian-fallback ladder in M5), or
            //   * iteration cap hit.
            let converged = (previous_lambda - lambda).abs() <= self.tolerance;
            if converged || lambda.abs() >= pi || counter >= self.max_iterations {
                break;
            }
        }

        // u² = cos²α · ((a/b)² − 1). Mirrors `formulas/vincenty_inverse.hpp:178`.
        let a_over_b = a_radius / b_radius;
        let u_sq = cos2_alpha * (a_over_b * a_over_b - 1.0);

        // (3) A
        let big_a =
            1.0 + u_sq / 16384.0 * (4096.0 + u_sq * (-768.0 + u_sq * (320.0 - 175.0 * u_sq)));
        // (4) B
        let big_b = u_sq / 1024.0 * (256.0 + u_sq * (-128.0 + u_sq * (74.0 - 47.0 * u_sq)));

        let cos_sigma_final = sigma.cos();
        let sin2_sigma = sin_sigma * sin_sigma;

        // (6) Δσ
        let delta_sigma = big_b
            * sin_sigma
            * (cos_2sigma_m
                + (big_b / 4.0)
                    * (cos_sigma_final * (-1.0 + 2.0 * cos_2sigma_m * cos_2sigma_m)
                        - (big_b / 6.0)
                            * cos_2sigma_m
                            * (-3.0 + 4.0 * sin2_sigma)
                            * (-3.0 + 4.0 * cos_2sigma_m * cos_2sigma_m)));

        // (19) s = b · A · (σ − Δσ)
        b_radius * big_a * (sigma - delta_sigma)
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        *self
    }
}

// ---- Tests ----------------------------------------------------------

#[cfg(all(test, feature = "std"))]
#[allow(
    clippy::doc_markdown,
    reason = "doc-comments quote `1_336_039.890` etc. — Rust numeric \
              literal syntax which clippy flags as missing backticks; \
              wrapping them in extra backticks hurts readability for \
              what is plainly a numeric value with its units."
)]
mod tests {
    //! Reference values come from `geometry/test/strategies/vincenty.cpp`
    //! — the cases below cite the exact lines in that file. The Boost
    //! tests use the GDA spheroid (`a = 6378.1370 km`,
    //! `f = 1 / 298.25722210`) at `vincenty.cpp:246-249`.

    use super::Vincenty;
    use crate::distance::DistanceStrategy;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Geographic, Spheroid};

    type GP = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;

    #[inline]
    fn deg(lon: f64, lat: f64) -> GP {
        WithCs::new(Adapt([lon, lat]))
    }

    /// GDA spheroid used by `vincenty.cpp:246-249`.
    const GDA: Spheroid = Spheroid {
        equatorial_radius: 6_378_137.0,
        flattening: 1.0 / 298.257_222_10,
    };

    fn gda_vincenty() -> Vincenty {
        Vincenty {
            spheroid: GDA,
            ..Vincenty::WGS84
        }
    }

    /// `vincenty.cpp:280` — N: `(0, 0) → (0, 50)` = 5_540_847.042 m.
    #[test]
    fn meridian_north_50_degrees() {
        let d = gda_vincenty().distance(&deg(0.0, 0.0), &deg(0.0, 50.0));
        assert!(
            (d - 5_540_847.042).abs() < 1e-2,
            "got {d} m, expected ~ 5_540_847.042 m"
        );
    }

    /// `vincenty.cpp:282` — E: `(0, 0) → (50, 0)` = 5_565_974.540 m.
    #[test]
    fn equator_east_50_degrees() {
        let d = gda_vincenty().distance(&deg(0.0, 0.0), &deg(50.0, 0.0));
        assert!(
            (d - 5_565_974.540).abs() < 1e-2,
            "got {d} m, expected ~ 5_565_974.540 m"
        );
    }

    /// `vincenty.cpp:285` — NE: `(0, 0) → (50, 50)` = 7_284_879.297 m.
    #[test]
    fn northeast_50_50() {
        let d = gda_vincenty().distance(&deg(0.0, 0.0), &deg(50.0, 50.0));
        assert!(
            (d - 7_284_879.297).abs() < 1e-2,
            "got {d} m, expected ~ 7_284_879.297 m"
        );
    }

    /// `vincenty.cpp:287-289` — sub-polar: `(0, 89) → (1, 80)` =
    /// 1_005_153.576_9 m. The `test_vincenty` invocations at
    /// `vincenty.cpp:289-291` deliberately omit the GDA spheroid
    /// argument — the in-source comment at line 287 reads
    /// *"Using default spheroid units (meters)"*, so these three
    /// reference values are on Boost's default `srs::spheroid<double>`,
    /// which is WGS84.
    ///
    /// Boost compares with `BOOST_CHECK_CLOSE(..., tolerance)` where
    /// `tolerance = 0.001` is a *percent* (not absolute) bound
    /// (`vincenty.cpp:114-115, 138`). 0.001% of 1_005_153 m ≈ 10 m,
    /// which is the bound we apply here.
    #[test]
    fn sub_polar() {
        let d = Vincenty::WGS84.distance(&deg(0.0, 89.0), &deg(1.0, 80.0));
        let tolerance = 1_005_153.576_9 * 0.001 / 100.0;
        assert!(
            (d - 1_005_153.576_9).abs() < tolerance,
            "got {d} m, expected ~ 1_005_153.576_9 m (tol {tolerance} m)"
        );
    }

    /// `vincenty.cpp:290` — identity (no distance) doesn't blow up.
    #[test]
    fn identity_zero() {
        let p = deg(4.0, 52.0);
        let d = gda_vincenty().distance(&p, &p);
        assert!(d.abs() < 1e-6, "got {d}");
    }

    /// `vincenty.cpp:291` — normal: `(4, 52) → (3, 40)` =
    /// 1_336_039.890 m. Same default-spheroid (WGS84) caveat as
    /// [`sub_polar`] above — see `vincenty.cpp:287` comment.
    ///
    /// The 1_336_039.890 m reference matches Andoyer to the metre
    /// (see `andoyer.cpp:230-231`); Vincenty for this pair on WGS84
    /// works out to ~1_336_027 m. Boost's actual assertion is
    /// `BOOST_CHECK_CLOSE` with `tolerance = 0.001%` of the reference
    /// (`vincenty.cpp:114-115, 138`) — 0.001% of 1_336_040 m ≈ 13 m,
    /// which is the bound we apply here.
    #[test]
    fn lon_4_lat_52_to_lon_3_lat_40() {
        let d = Vincenty::WGS84.distance(&deg(4.0, 52.0), &deg(3.0, 40.0));
        let tolerance = 1_336_039.890 * 0.001 / 100.0;
        assert!(
            (d - 1_336_039.890).abs() < tolerance,
            "got {d} m, expected ~ 1_336_039.890 m (tol {tolerance} m)"
        );
    }

    /// `vincenty.cpp:264-267` — Lodz → Trondheim ≈ 1_399_032.724 m
    /// (Boost reports 1399.032724 km).
    #[test]
    fn lodz_to_trondheim() {
        let lodz = deg(19.0 + 28.0 / 60.0, 51.0 + 47.0 / 60.0);
        let trondheim = deg(10.0 + 21.0 / 60.0, 63.0 + 23.0 / 60.0);
        let d = gda_vincenty().distance(&lodz, &trondheim);
        assert!(
            (d - 1_399_032.724).abs() < 1.0,
            "got {d} m, expected ~ 1_399_032.724 m"
        );
    }

    /// `vincenty.cpp:269-272` — London → New York ≈ 5_602_044.851 m.
    #[test]
    fn london_to_new_york() {
        let london = deg(
            0.0 + 7.0 / 60.0 + 39.0 / 3600.0,
            51.0 + 30.0 / 60.0 + 26.0 / 3600.0,
        );
        let nyc = deg(
            -(74.0 + 0.0 / 60.0 + 21.0 / 3600.0),
            40.0 + 42.0 / 60.0 + 46.0 / 3600.0,
        );
        let d = gda_vincenty().distance(&london, &nyc);
        assert!(
            (d - 5_602_044.851).abs() < 1.0,
            "got {d} m, expected ~ 5_602_044.851 m"
        );
    }

    /// Vincenty's default constructor selects WGS84 — mirrors Boost's
    /// `vincenty()` no-arg constructor at
    /// `strategies/geographic/distance_vincenty.hpp:59-61`.
    #[test]
    fn default_is_wgs84() {
        let v = Vincenty::default();
        let w = Vincenty::WGS84;
        assert_eq!(v.spheroid, w.spheroid);
        assert_eq!(v.max_iterations, w.max_iterations);
        assert!((v.tolerance - w.tolerance).abs() < 1e-30);
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

    /// `comparable()` returns a strategy producing the same distance.
    #[test]
    fn comparable_produces_the_same_distance() {
        let a = deg(4.0, 52.0);
        let b = deg(3.0, 40.0);
        let real = Vincenty::WGS84.distance(&a, &b);
        let cmp = DistanceStrategy::<GP, GP>::comparable(&Vincenty::WGS84).distance(&a, &b);
        assert!((real - cmp).abs() < 1e-9);
    }

    /// The read-only-point witness computes a distance when invoked.
    #[test]
    #[allow(
        clippy::used_underscore_items,
        reason = "the test exists to run the compile-time witness's body"
    )]
    fn readonly_witness_computes_distance() {
        let d = _accepts_readonly_point(&Vincenty::WGS84, &deg(4.0, 52.0), &deg(3.0, 40.0));
        assert!(d > 1_000_000.0, "≈1336 km, got {d}");
    }
}
