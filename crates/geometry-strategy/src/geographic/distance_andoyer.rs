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
//! the [`CoordinateScalar`] trait
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

#[cfg(feature = "std")]
use geometry_cs::CoordinateSystem;
use geometry_cs::{GeographicFamily, Spheroid};
#[cfg(feature = "std")]
use geometry_tag::SameAs;
#[cfg(feature = "std")]
use geometry_trait::Point;

use crate::distance::DefaultDistance;
#[cfg(feature = "std")]
use crate::distance::DistanceStrategy;

#[cfg(feature = "std")]
use geometry_coords::CoordinateScalar;

#[cfg(feature = "std")]
use crate::geographic::InverseResult;
#[cfg(feature = "std")]
use crate::geographic::Meridian;
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

    /// The inverse geodesic problem between two longitude/latitude pairs in
    /// radians: the distance always, both azimuths when `AZIMUTHS` is set.
    ///
    /// Mirrors `formula::andoyer_inverse::apply`
    /// (`formulas/andoyer_inverse.hpp:58-243`), whose `math::equals` guards
    /// are [`CoordinateScalar::tolerant_eq`] here. Coincident points keep
    /// the zero result; a coincident or antipodal pair the guards let
    /// through gets the fixed azimuths of
    /// `formulas/andoyer_inverse.hpp:127-163`.
    #[cfg(feature = "std")]
    #[allow(
        clippy::many_single_char_names,
        clippy::similar_names,
        reason = "the single-letter names mirror formula::andoyer_inverse letter for letter"
    )]
    pub(crate) fn inverse<const AZIMUTHS: bool>(
        &self,
        lon1: f64,
        lat1: f64,
        lon2: f64,
        lat2: f64,
    ) -> InverseResult {
        let mut result = InverseResult {
            converged: true,
            ..InverseResult::default()
        };
        if lon1.tolerant_eq(lon2) && lat1.tolerant_eq(lat2) {
            return result;
        }

        let calc = SpheroidCalc::from(self.spheroid);
        let f = calc.f;
        let pi = core::f64::consts::PI;
        let dlon = lon2 - lon1;
        let sin_dlon = dlon.sin();
        let cos_dlon = dlon.cos();
        let sin_lat1 = lat1.sin();
        let cos_lat1 = lat1.cos();
        let sin_lat2 = lat2.sin();
        let cos_lat2 = lat2.cos();

        // Rounding can carry `cos_d` past ±1 (`andoyer_inverse.hpp:90-95`).
        let cos_d = (sin_lat1 * sin_lat2 + cos_lat1 * cos_lat2 * cos_dlon).clamp(-1.0, 1.0);
        let d = cos_d.acos();
        let sin_d = d.sin();

        // `H` and `G` are infinite where `cos_d` is ±1: points very close
        // or antipodal (`andoyer_inverse.hpp:102-122`).
        let k = (sin_lat1 - sin_lat2) * (sin_lat1 - sin_lat2);
        let l = (sin_lat1 + sin_lat2) * (sin_lat1 + sin_lat2);
        let three_sin_d = 3.0 * sin_d;
        let one_minus_cos_d = 1.0 - cos_d;
        let one_plus_cos_d = 1.0 + cos_d;
        let h = if one_minus_cos_d.tolerant_eq(0.0) {
            0.0
        } else {
            (d + three_sin_d) / one_minus_cos_d
        };
        let g = if one_plus_cos_d.tolerant_eq(0.0) {
            0.0
        } else {
            (d - three_sin_d) / one_plus_cos_d
        };
        let dd = -(f / 4.0) * (h * k + g * l);
        result.distance = calc.a * (d + dd);
        if !AZIMUTHS {
            return result;
        }

        if sin_d.tolerant_eq(0.0) {
            // Very close points keep both azimuths at zero; antipodal ones
            // head north, or south from the north pole.
            if cos_d < 0.0 {
                if sin_lat1.tolerant_eq(1.0) {
                    result.azimuth = pi;
                } else {
                    result.reverse_azimuth = pi;
                }
            }
            return result;
        }

        let (a, u) = if cos_lat2.tolerant_eq(0.0) {
            (if sin_lat2 < 0.0 { pi } else { 0.0 }, 0.0)
        } else {
            let tan_lat2 = sin_lat2 / cos_lat2;
            let m = cos_lat1 * tan_lat2 - sin_lat1 * cos_dlon;
            let a = sin_dlon.atan2(m);
            (a, (f / 2.0) * (cos_lat1 * cos_lat1) * (2.0 * a).sin())
        };
        let (b, v) = if cos_lat1.tolerant_eq(0.0) {
            (if sin_lat1 < 0.0 { pi } else { 0.0 }, 0.0)
        } else {
            let tan_lat1 = sin_lat1 / cos_lat1;
            let n = cos_lat2 * tan_lat1 - sin_lat2 * cos_dlon;
            let b = sin_dlon.atan2(n);
            (b, (f / 2.0) * (cos_lat2 * cos_lat2) * (2.0 * b).sin())
        };
        let t = d / sin_d;

        let da = v * t - u;
        result.azimuth = a - da;
        normalize_azimuth(&mut result.azimuth, a, da);

        let db = -u * t + v;
        result.reverse_azimuth = if b >= 0.0 { pi - b - db } else { -pi - b - db };
        normalize_azimuth(&mut result.reverse_azimuth, b, db);
        result
    }
}

/// Keep a corrected azimuth from crossing the meridian its uncorrected
/// value `base` lies beside: the correction `delta` may carry it to the
/// meridian but not past.
///
/// Mirrors `andoyer_inverse::normalize_azimuth`
/// (`formulas/andoyer_inverse.hpp:246-286`).
#[cfg(feature = "std")]
fn normalize_azimuth(azimuth: &mut f64, base: f64, delta: f64) {
    let pi = core::f64::consts::PI;
    if base >= 0.0 {
        // Eastern hemisphere.
        if delta >= 0.0 {
            if *azimuth < 0.0 {
                *azimuth = 0.0;
            }
        } else if *azimuth > pi {
            *azimuth = pi;
        }
    } else if delta <= 0.0 {
        // Western hemisphere, corrected towards zero.
        if *azimuth > 0.0 {
            *azimuth = 0.0;
        }
    } else if *azimuth < -pi {
        *azimuth = -pi;
    }
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
/// Mirrors `strategy::distance::geographic<andoyer>::apply`
/// (`strategies/geographic/distance.hpp:91-112`): endpoints on one
/// meridian, or on opposite meridians with the route over a pole, take the
/// meridian arc at Andoyer's order-one series
/// (`strategies/geographic/parameters.hpp:186-189`); the rest take the
/// distance branch of `formula::andoyer_inverse`
/// (`formulas/andoyer_inverse.hpp:58-123`), see [`Andoyer`]'s inverse:
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

    #[inline]
    fn distance(&self, a: &P1, b: &P2) -> Self::Out {
        let (lon1, lat1) = lonlat_radians(a);
        let (lon2, lat2) = lonlat_radians(b);
        let meridian = Meridian {
            spheroid: self.spheroid,
        }
        .inverse_to_order(lon1, lat1, lon2, lat2, 1);
        if meridian.meridian {
            return meridian.distance;
        }
        self.inverse::<false>(lon1, lat1, lon2, lat2).distance
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
    use crate::geographic::Meridian;
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
        assert!((d / 1000.0 - 1_116.814_237).abs() < 0.01);
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
        assert!((d / 1000.0 - 1_336.039_890).abs() < 0.01);
    }

    /// `test/strategies/andoyer.cpp:243-246` — four antipodal
    /// equatorial pairs expect `20_003.9 km`: the strategy
    /// `strategy::distance::geographic<andoyer>`
    /// (`strategies/geographic/distance.hpp:91-112`) runs
    /// `formula::meridian_inverse` first, and `|Δlon| == 180°` routes
    /// these pairs over a pole instead of the raw formula's half
    /// equatorial circumference (`20_037.5 km`), which is longer than that
    /// known path. Andoyer measures the route with the order-one meridian
    /// series, `20_003_917.356955905` m in Boost (`aed7bc3`), 14 m short
    /// of twice the quarter meridian.
    #[test]
    fn antipodal_equatorial() {
        let expected = 20_003_917.356_955_905;
        assert!((2.0 * Meridian::WGS84.quarter_length() - expected - 14.1).abs() < 0.01);
        for (a, b) in [
            (deg(0.0, 0.0), deg(180.0, 0.0)),
            (deg(0.0, 0.0), deg(-180.0, 0.0)),
            (deg(-90.0, 0.0), deg(90.0, 0.0)),
            (deg(90.0, 0.0), deg(-90.0, 0.0)),
            (deg(10.0, 20.0), deg(-170.0, -20.0)),
        ] {
            let d = Andoyer::WGS84.distance(&a, &b);
            assert!((d - expected).abs() < 1e-6, "{d}");
        }
    }

    /// Points on one meridian take Andoyer's order-one meridian series
    /// (`strategies/geographic/parameters.hpp:186-189`), not the full arc:
    /// Boost (`aed7bc3`) gives `110_573.13812782228` m for a degree from
    /// the equator, 1.25 m short of the meridian, and `10_001_958.678477952`
    /// m to the pole.
    #[test]
    fn meridian_pairs_take_the_order_one_series() {
        let degree = Andoyer::WGS84.distance(&deg(0.0, 0.0), &deg(0.0, 1.0));
        assert!((degree - 110_573.138_127_822_28).abs() < 1e-8, "{degree}");
        let quarter = Andoyer::WGS84.distance(&deg(0.0, 0.0), &deg(0.0, 90.0));
        assert!((quarter - 10_001_958.678_477_952).abs() < 1e-6, "{quarter}");
        let full = Meridian::WGS84.arc_length(1.0_f64.to_radians());
        assert!((full - degree - 1.25).abs() < 0.01);
    }

    /// The clamp branches of `normalize_azimuth`
    /// (`andoyer_inverse.hpp:246-286`): the flattening correction must
    /// not push an azimuth past 0 / ±π on the side it started.
    #[test]
    fn normalize_azimuth_clamps_all_four_quadrants() {
        use super::normalize_azimuth;
        let pi = core::f64::consts::PI;

        // A ≥ 0, dA ≥ 0: an azimuth pushed below 0 clamps to 0.
        let mut az = -0.1;
        normalize_azimuth(&mut az, 0.05, 0.15);
        assert_eq!(az, 0.0);

        // A ≥ 0, dA < 0: an azimuth pushed above π clamps to π.
        let mut az = pi + 0.1;
        normalize_azimuth(&mut az, pi - 0.05, -0.15);
        assert_eq!(az, pi);

        // A < 0, dA ≤ 0: an azimuth pushed above 0 clamps to 0.
        let mut az = 0.1;
        normalize_azimuth(&mut az, -0.05, -0.15);
        assert_eq!(az, 0.0);

        // A < 0, dA > 0: an azimuth pushed below −π clamps to −π.
        let mut az = -pi - 0.1;
        normalize_azimuth(&mut az, -pi + 0.05, 0.15);
        assert_eq!(az, -pi);

        // In-range azimuths pass through untouched.
        let mut az = 0.5;
        normalize_azimuth(&mut az, 0.4, -0.1);
        assert_eq!(az, 0.5);

        let mut az = -0.5;
        normalize_azimuth(&mut az, -0.4, -0.1);
        assert_eq!(az, -0.5);
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
    #[allow(
        clippy::used_underscore_items,
        reason = "the test exists to run the compile-time witness's body"
    )]
    fn readonly_witness_computes_distance() {
        let d = _accepts_readonly_point(&Andoyer::WGS84, &deg(4.0, 52.0), &deg(3.0, 40.0));
        assert!(d > 1_000_000.0, "≈1336 km, got {d}");
    }
}
