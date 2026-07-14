//! Andoyer–Lambert geographic distance on a reference spheroid.
//!
//! Mirrors `boost::geometry::strategy::distance::andoyer<Spheroid, T>`
//! from `strategies/geographic/distance_andoyer.hpp`. The underlying
//! arithmetic comes from `formulas/andoyer_inverse.hpp` — a
//! Forsyth–Andoyer–Lambert first-order spheroidal correction to the
//! spherical great-circle distance. Boost's commentary on the header
//! notes that the approximation is accurate to within a few metres of
//! Vincenty in all tested cases.
//!
//! # Calculation-type policy
//!
//! Boost runs the inputs through
//! `util::calculation_type::geographic::binary` and picks a working
//! scalar; the v1 Rust port follows Haversine's approach (T40 spec) and
//! hardcodes `Scalar = f64` on both inputs. This lets the kernel reach
//! for `f64::sin` / `cos` / `acos` / `sqrt` directly without growing
//! the [`CoordinateScalar`](geometry_coords::CoordinateScalar) trait
//! surface. Mixed-scalar support folds in alongside the `Promote`
//! lattice when a real caller appears.
//!
//! `#[cfg(feature = "std")]` gates the impl: the standard library
//! provides the trig and `sqrt` functions as inherent methods on `f64`.
//! A `no_std` build of `geometry-strategy` (default-features off) does
//! not get Andoyer; that mirrors the same gate Haversine uses.
//!
//! # Comparable form
//!
//! Andoyer has no useful "skip the sqrt" form — the corrections
//! involve `acos` and additive flattening terms that cannot be shed
//! while preserving ordering. We follow Boost
//! (`strategies/geographic/distance_andoyer.hpp:91-94`) and set
//! `type Comparable = Self;`.

use geometry_cs::{CoordinateSystem, GeographicFamily, Spheroid};
use geometry_tag::SameAs;
use geometry_trait::Point;

use crate::distance::{DefaultDistance, DistanceStrategy};

#[cfg(feature = "std")]
use crate::geographic::spheroid_calc::SpheroidCalc;
#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Andoyer–Lambert geographic distance on a reference spheroid.
///
/// Inputs follow the [`Geographic<U>`](geometry_cs::Geographic)
/// equatorial convention — see its rustdoc.
///
/// Mirrors `boost::geometry::strategy::distance::andoyer<Spheroid, T>`
/// from `strategies/geographic/distance_andoyer.hpp:46-70`. The
/// spheroid is supplied at construction and the output is in metres
/// (or whatever units the spheroid's equatorial radius is expressed
/// in).
///
/// The underlying arithmetic mirrors
/// `boost::geometry::formula::andoyer_inverse::apply` from
/// `formulas/andoyer_inverse.hpp:58-123`.
#[derive(Debug, Clone, Copy)]
pub struct Andoyer {
    /// Reference ellipsoid the distance is measured on.
    pub spheroid: Spheroid,
}

impl Andoyer {
    /// Andoyer parameterised by the WGS84 reference ellipsoid — the
    /// default for nearly every real geographic dataset. Matches the
    /// default-constructed `srs::spheroid<RadiusType>` Boost uses when
    /// `andoyer<>` is built without arguments
    /// (`strategies/geographic/distance_andoyer.hpp:63-65`).
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };
}

impl Default for Andoyer {
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
// strategies; that is the same redirect plate Haversine relies on.

/// Andoyer on `f64` geographic points.
///
/// Mirrors `formula::andoyer_inverse<CT, true, false>::apply` at
/// `formulas/andoyer_inverse.hpp:58-123` — the distance-only branch
/// (`EnableDistance = true`, all other flags false). Computes:
///
/// ```text
/// cos_d  = sin(lat1)·sin(lat2) + cos(lat1)·cos(lat2)·cos(Δlon)
/// d      = acos(cos_d)
/// K      = (sin(lat1) − sin(lat2))²
/// L      = (sin(lat1) + sin(lat2))²
/// H      = (d + 3·sin_d) / (1 − cos_d)
/// G      = (d − 3·sin_d) / (1 + cos_d)
/// dd     = −(f/4) · (H·K + G·L)
/// result = a · (d + dd)
/// ```
///
/// with the degenerate `1 ± cos_d == 0` branches falling back to
/// `H = 0` / `G = 0` as in
/// `formulas/andoyer_inverse.hpp:111-117`.
///
/// # Diagnostics on mis-paired CS
///
/// A caller who pairs a Cartesian or Spherical point with [`Andoyer`]
/// hits the `<P::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>`
/// bound below and gets the redirect plate on
/// [`geometry_tag::SameAs`] pointing them at
/// `WithCs<_, Geographic<…>>` or at the Cartesian / Spherical
/// strategies. See T31 and proposal §3.7.
#[cfg(feature = "std")]
impl<P1, P2> DistanceStrategy<P1, P2> for Andoyer
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
    // `d, dd, h, g, k, l` and the exact `== 0.0` / `== same-lonlat`
    // checks mirror `formula::andoyer_inverse::apply` in
    // `formulas/andoyer_inverse.hpp:74-122` letter-for-letter; the
    // exact-equality short-circuit is the intentional analogue of
    // Boost's `math::equals` against zero on the same line numbers.
    #[allow(clippy::many_single_char_names, clippy::float_cmp)]
    #[inline]
    fn distance(&self, a: &P1, b: &P2) -> Self::Out {
        let calc = SpheroidCalc::from(self.spheroid);
        let (lon1, lat1) = lonlat_radians(a);
        let (lon2, lat2) = lonlat_radians(b);

        // Mirrors the `math::equals(lon1, lon2) && math::equals(lat1, lat2)`
        // short-circuit at `formulas/andoyer_inverse.hpp:69-72`.
        if lon1 == lon2 && lat1 == lat2 {
            return 0.0;
        }

        let dlon = lon2 - lon1;
        let cos_dlon = dlon.cos();
        let sin_lat1 = lat1.sin();
        let cos_lat1 = lat1.cos();
        let sin_lat2 = lat2.sin();
        let cos_lat2 = lat2.cos();

        // Spherical great-circle term, clamped to [-1, 1] to defend
        // against rounding pushing `cos_d` slightly outside the
        // acos domain — same defence as
        // `formulas/andoyer_inverse.hpp:90-95`.
        let cos_d = (sin_lat1 * sin_lat2 + cos_lat1 * cos_lat2 * cos_dlon).clamp(-1.0, 1.0);

        let d = cos_d.acos();
        let sin_d = d.sin();

        // Andoyer–Lambert flattening correction. Mirrors
        // `formulas/andoyer_inverse.hpp:102-122`.
        let k = (sin_lat1 - sin_lat2) * (sin_lat1 - sin_lat2);
        let l = (sin_lat1 + sin_lat2) * (sin_lat1 + sin_lat2);
        let three_sin_d = 3.0 * sin_d;

        let one_minus_cos_d = 1.0 - cos_d;
        let one_plus_cos_d = 1.0 + cos_d;

        // `cos_d == 1` ⇒ near-coincident, `cos_d == -1` ⇒ antipodal.
        // Boost guards `H` / `G` against the singular denominators
        // with `math::equals(..., c0)` — we use exact `== 0.0`
        // because the trig of finite inputs that escaped the
        // earlier `lon1 == lon2 && lat1 == lat2` short-circuit can
        // only land on exactly 0.0 in pathological cases.
        let h = if one_minus_cos_d == 0.0 {
            0.0
        } else {
            (d + three_sin_d) / one_minus_cos_d
        };
        let g = if one_plus_cos_d == 0.0 {
            0.0
        } else {
            (d - three_sin_d) / one_plus_cos_d
        };

        let dd = -(calc.f / 4.0) * (h * k + g * l);

        calc.a * (d + dd)
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        *self
    }
}

// ---- Default Geographic × Geographic = Andoyer ----------------------

/// Geographic × Geographic defaults to Andoyer.
///
/// Mirrors the `services::default_strategy<point_tag, point_tag, P1,
/// P2, geographic_tag, geographic_tag>` specialisation in
/// `strategies/geographic/distance.hpp` — Boost picks
/// `strategy::distance::geographic<strategy::andoyer, Spheroid>` as
/// the geographic default, which is exactly
/// `strategy::distance::andoyer<Spheroid>`.
impl DefaultDistance<GeographicFamily> for GeographicFamily {
    type Strategy = Andoyer;
}

// ---- Tests ----------------------------------------------------------

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference values come from
    //! `geometry/test/strategies/andoyer.cpp` — the cases below cite
    //! the exact lines in that file.

    use super::Andoyer;
    use crate::distance::DistanceStrategy;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Geographic};

    type GP = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;

    #[inline]
    fn deg(lon: f64, lat: f64) -> GP {
        WithCs::new(Adapt([lon, lat]))
    }

    /// `test/strategies/andoyer.cpp:222-223` — polar case:
    /// `(0, 90) → (1, 80) ≈ 1116.814 km`.
    #[test]
    fn polar_1deg_lon_10deg_lat() {
        let d = Andoyer::WGS84.distance(&deg(0.0, 90.0), &deg(1.0, 80.0));
        assert!(
            (d / 1000.0 - 1_116.814_237).abs() < 0.01,
            "{} km expected ~ 1116.814 km",
            d / 1000.0
        );
    }

    /// `test/strategies/andoyer.cpp:226-227` — zero distance on equal
    /// points.
    #[test]
    fn zero_distance_on_equal_points() {
        let p = deg(4.0, 52.0);
        let d = Andoyer::WGS84.distance(&p, &p);
        assert!(d.abs() < 1e-3, "got {d}");
    }

    /// `test/strategies/andoyer.cpp:230-231` — normal case:
    /// `(4, 52) → (3, 40) ≈ 1336.040 km`.
    #[test]
    fn lon_4_lat_52_to_lon_3_lat_40() {
        let d = Andoyer::WGS84.distance(&deg(4.0, 52.0), &deg(3.0, 40.0));
        assert!(
            (d / 1000.0 - 1_336.039_890).abs() < 0.01,
            "{} km expected ~ 1336.040 km",
            d / 1000.0
        );
    }

    /// `test/strategies/andoyer.cpp:243-246` — four antipodal
    /// equatorial pairs.
    ///
    /// **Divergence from Boost's expected `20_003.9 km`:** the Boost
    /// test value is produced by the *strategy*
    /// `strategy::distance::geographic<andoyer>` at
    /// `strategies/geographic/distance.hpp:91-112`, which first runs
    /// `formula::meridian_inverse` and only falls back to
    /// `andoyer_inverse` for non-meridian pairs. For the four
    /// equatorial-antipodal cases here `|Δlon| == 180°` triggers the
    /// meridian shortcut. T43's scope is `andoyer_inverse` itself —
    /// the meridian / nearly-antipodal fallback ladder is M5 (T46) —
    /// so we feed the inputs directly through `andoyer_inverse`,
    /// which (correctly) reports the equatorial circumference / 2 ≈
    /// `20_037.5 km`. Tolerance is 1 km against that value.
    #[test]
    fn antipodal_equatorial() {
        // 2π · a / 2 for WGS84 ≈ 20_037.508 km.
        let expected_km = core::f64::consts::PI * 6_378_137.0 / 1000.0;
        for (a, b) in [
            (deg(0.0, 0.0), deg(180.0, 0.0)),
            (deg(0.0, 0.0), deg(-180.0, 0.0)),
            (deg(-90.0, 0.0), deg(90.0, 0.0)),
            (deg(90.0, 0.0), deg(-90.0, 0.0)),
        ] {
            let d = Andoyer::WGS84.distance(&a, &b);
            assert!(
                (d / 1000.0 - expected_km).abs() < 1.0,
                "got {} km, expected ~ {} km",
                d / 1000.0,
                expected_km,
            );
        }
    }

    /// Andoyer's default constructor selects WGS84 — mirrors Boost's
    /// `andoyer()` no-arg constructor at
    /// `strategies/geographic/distance_andoyer.hpp:63-65`.
    #[test]
    fn default_is_wgs84() {
        let a = Andoyer::default();
        let w = Andoyer::WGS84;
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


    /// `comparable()` returns a strategy producing the same distance —
    /// there is no sqrt to skip in the geodesic formula.
    #[test]
    fn comparable_produces_the_same_distance() {
        let a = deg(4.0, 52.0);
        let b = deg(3.0, 40.0);
        let real = Andoyer::WGS84.distance(&a, &b);
        let cmp = DistanceStrategy::<GP, GP>::comparable(&Andoyer::WGS84).distance(&a, &b);
        assert!((real - cmp).abs() < 1e-9);
    }

    /// The read-only-point witness computes a distance when invoked.
    #[test]
    #[allow(clippy::used_underscore_items, reason = "the test exists to run the compile-time witness's body")]
    fn readonly_witness_computes_distance() {
        let d = _accepts_readonly_point(&Andoyer::WGS84, &deg(4.0, 52.0), &deg(3.0, 40.0));
        assert!(d > 1_000_000.0, "≈1336 km, got {d}");
    }
}
