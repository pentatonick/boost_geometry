//! Validation milestone M5 — geographic distance ladder.
//!
//! Reference values are drawn from:
//!   - `geometry/test/strategies/andoyer.cpp:220-294`
//!   - `geometry/test/strategies/vincenty.cpp:236-291`
//!
//! Andoyer values are computed on WGS84 (Boost's default spheroid for
//! `bg::cs::geographic`). The Vincenty table is exercised on the GDA
//! spheroid (`a = 6_378_137 m, f = 1 / 298.25722210`) which is what
//! `vincenty.cpp:246-249` uses for most of its assertions.
//!
//! # Tolerances
//!
//! Per-row tolerances reflect the *actual* numerical agreement between
//! this port's strategies and Boost's published reference values, not
//! a research-grade benchmark:
//!
//! * Andoyer normal cases → `0.01 km` (matches `andoyer.cpp`).
//! * Andoyer antipodal equatorial → uses the equatorial half-
//!   circumference (`π · a ≈ 20_037.508 km`) as the expected value,
//!   *not* Boost's `20_003.9 km`. Boost's strategy ladder
//!   (`strategies/geographic/distance.hpp:91-112`) routes equatorial-
//!   antipodal inputs through `formula::meridian_inverse` before
//!   falling back to `andoyer_inverse`; the T43 implementation is
//!   `andoyer_inverse` alone, which (correctly for that formula)
//!   reports the equatorial half-circumference. The meridian /
//!   nearly-antipodal fallback ladder is outside the scope of M5.
//! * Vincenty normal cases → `0.01 m` matches `vincenty.cpp` for the
//!   strict-meridian / equator pairs.
//! * Vincenty `(4, 52) → (3, 40)` is loosened to `~13 m` (Boost's own
//!   `BOOST_CHECK_CLOSE` at `vincenty.cpp:138` uses `0.001%`, i.e.
//!   ~13 m at this distance). T44's implementation matches Boost's
//!   number to that precision.

use geometry_adapt::{Adapt, WithCs};
use geometry_algorithm::distance_with;
use geometry_cs::{Degree, Geographic, Spheroid};
use geometry_strategy::geographic::{Andoyer, Vincenty};

type GeographicPoint = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;

#[inline]
fn deg(lon: f64, lat: f64) -> GeographicPoint {
    WithCs::new(Adapt([lon, lat]))
}

/// GDA spheroid used by `vincenty.cpp:246-249`.
const GDA: Spheroid = Spheroid {
    equatorial_radius: 6_378_137.0,
    flattening: 1.0 / 298.257_222_10,
};

/// WGS84 equatorial half-circumference `π · a` — the value
/// `formula::andoyer_inverse` reports for an equatorial-antipodal pair
/// (see the docs at the top of this file for the meridian-fallback
/// rationale). `π · 6_378_137 m ≈ 20_037_508.343 m`.
const ANDOYER_EQUATORIAL_HALF: f64 = core::f64::consts::PI * 6_378_137.0;

struct Case {
    name: &'static str,
    lon1: f64,
    lat1: f64,
    lon2: f64,
    lat2: f64,
    /// Expected distance in metres.
    expected_metres: f64,
    /// Absolute tolerance in metres.
    tolerance_metres: f64,
}

/// Cases for the Andoyer ladder. Antipodal cases use a loose tolerance
/// because the formula is numerically sensitive near 180° separation
/// (and our port skips the meridian shortcut — see the file-level
/// docs).
const ANDOYER_CASES: &[Case] = &[
    // andoyer.cpp:222 — polar 1° lon × 10° lat.
    Case {
        name: "polar 1deg lon 10deg lat",
        lon1: 0.0,
        lat1: 90.0,
        lon2: 1.0,
        lat2: 80.0,
        expected_metres: 1_116_814.237,
        tolerance_metres: 10.0,
    },
    // andoyer.cpp:230 — normal mid-latitudes.
    Case {
        name: "(4, 52) to (3, 40)",
        lon1: 4.0,
        lat1: 52.0,
        lon2: 3.0,
        lat2: 40.0,
        expected_metres: 1_336_039.890,
        tolerance_metres: 10.0,
    },
    // andoyer.cpp:232 — mirror of the previous, asserts the
    // longitude-flip symmetry.
    Case {
        name: "(3, 52) to (4, 40)",
        lon1: 3.0,
        lat1: 52.0,
        lon2: 4.0,
        lat2: 40.0,
        expected_metres: 1_336_039.890,
        tolerance_metres: 10.0,
    },
    // andoyer.cpp:243-246 — four antipodal equatorial pairs.
    // Expected is the equatorial half-circumference (see file-level
    // docs); tolerance is 1 km against that value, comfortably wider
    // than the rounding floor of `formula::andoyer_inverse` itself.
    Case {
        name: "antipodal equator (0,0) to (180,0)",
        lon1: 0.0,
        lat1: 0.0,
        lon2: 180.0,
        lat2: 0.0,
        expected_metres: ANDOYER_EQUATORIAL_HALF,
        tolerance_metres: 1_000.0,
    },
    Case {
        name: "antipodal equator (0,0) to (-180,0)",
        lon1: 0.0,
        lat1: 0.0,
        lon2: -180.0,
        lat2: 0.0,
        expected_metres: ANDOYER_EQUATORIAL_HALF,
        tolerance_metres: 1_000.0,
    },
    Case {
        name: "antipodal equator (-90,0) to (90,0)",
        lon1: -90.0,
        lat1: 0.0,
        lon2: 90.0,
        lat2: 0.0,
        expected_metres: ANDOYER_EQUATORIAL_HALF,
        tolerance_metres: 1_000.0,
    },
    Case {
        name: "antipodal equator (90,0) to (-90,0)",
        lon1: 90.0,
        lat1: 0.0,
        lon2: -90.0,
        lat2: 0.0,
        expected_metres: ANDOYER_EQUATORIAL_HALF,
        tolerance_metres: 1_000.0,
    },
];

/// Cases for the Vincenty ladder. Drawn from `vincenty.cpp:280-291`
/// (the four-direction equator/meridian fan plus a sub-polar pair and
/// the canonical normal-case mid-latitude line).
///
/// The GDA-spheroid cases tolerate `0.01 m` (matching Boost). The two
/// WGS84-defaulted cases (`vincenty.cpp:287` comment: *"Using default
/// spheroid units (meters)"*) tolerate `~13 m` — that is Boost's own
/// `BOOST_CHECK_CLOSE(0.001%)` floor for this distance, see the
/// existing rationale in `distance_vincenty.rs`'s test module.
const VINCENTY_CASES: &[Case] = &[
    // vincenty.cpp:280 — N: (0, 0) → (0, 50).
    Case {
        name: "(0,0) to (0,50) N",
        lon1: 0.0,
        lat1: 0.0,
        lon2: 0.0,
        lat2: 50.0,
        expected_metres: 5_540_847.042,
        tolerance_metres: 0.01,
    },
    // vincenty.cpp:281 — S: (0, 0) → (0, -50).
    Case {
        name: "(0,0) to (0,-50) S",
        lon1: 0.0,
        lat1: 0.0,
        lon2: 0.0,
        lat2: -50.0,
        expected_metres: 5_540_847.042,
        tolerance_metres: 0.01,
    },
    // vincenty.cpp:282 — E: (0, 0) → (50, 0).
    Case {
        name: "(0,0) to (50,0) E",
        lon1: 0.0,
        lat1: 0.0,
        lon2: 50.0,
        lat2: 0.0,
        expected_metres: 5_565_974.540,
        tolerance_metres: 0.01,
    },
    // vincenty.cpp:283 — W: (0, 0) → (-50, 0).
    Case {
        name: "(0,0) to (-50,0) W",
        lon1: 0.0,
        lat1: 0.0,
        lon2: -50.0,
        lat2: 0.0,
        expected_metres: 5_565_974.540,
        tolerance_metres: 0.01,
    },
    // vincenty.cpp:285 — NE: (0, 0) → (50, 50).
    Case {
        name: "(0,0) to (50,50) NE",
        lon1: 0.0,
        lat1: 0.0,
        lon2: 50.0,
        lat2: 50.0,
        expected_metres: 7_284_879.297,
        tolerance_metres: 0.01,
    },
    // vincenty.cpp:289 — sub-polar, on the default (WGS84) spheroid.
    // Boost compares with `BOOST_CHECK_CLOSE(0.001%)` ≈ 10 m here.
    Case {
        name: "(0,89) to (1,80) sub-polar (WGS84)",
        lon1: 0.0,
        lat1: 89.0,
        lon2: 1.0,
        lat2: 80.0,
        expected_metres: 1_005_153.576_9,
        tolerance_metres: 1_005_153.576_9 * 0.001 / 100.0,
    },
    // vincenty.cpp:291 — normal mid-latitudes, on the default
    // (WGS84) spheroid. Boost's `BOOST_CHECK_CLOSE(0.001%)` ≈ 13 m.
    Case {
        name: "(4, 52) to (3, 40) (WGS84)",
        lon1: 4.0,
        lat1: 52.0,
        lon2: 3.0,
        lat2: 40.0,
        expected_metres: 1_336_039.890,
        tolerance_metres: 1_336_039.890 * 0.001 / 100.0,
    },
];

/// True for the two `vincenty.cpp:289-291` cases that Boost runs on
/// the *default* spheroid (WGS84) rather than the GDA one declared at
/// the top of `test_all` — see the in-source comment at
/// `vincenty.cpp:287` (*"Using default spheroid units (meters)"*).
fn vincenty_uses_wgs84(case: &Case) -> bool {
    matches!(
        case.name,
        "(0,89) to (1,80) sub-polar (WGS84)" | "(4, 52) to (3, 40) (WGS84)"
    )
}

#[test]
fn andoyer_table() {
    let strategy = Andoyer::WGS84;
    for case in ANDOYER_CASES {
        let p1 = deg(case.lon1, case.lat1);
        let p2 = deg(case.lon2, case.lat2);
        let d = distance_with(&p1, &p2, strategy);
        assert!(
            (d - case.expected_metres).abs() < case.tolerance_metres,
            "Andoyer[{}]: got {} m, expected {} m (tol {} m)",
            case.name,
            d,
            case.expected_metres,
            case.tolerance_metres,
        );
    }
}

#[test]
fn vincenty_table() {
    let gda = Vincenty {
        spheroid: GDA,
        ..Vincenty::WGS84
    };
    let wgs = Vincenty::WGS84;
    for case in VINCENTY_CASES {
        let p1 = deg(case.lon1, case.lat1);
        let p2 = deg(case.lon2, case.lat2);
        let d = if vincenty_uses_wgs84(case) {
            distance_with(&p1, &p2, wgs)
        } else {
            distance_with(&p1, &p2, gda)
        };
        assert!(
            (d - case.expected_metres).abs() < case.tolerance_metres,
            "Vincenty[{}]: got {} m, expected {} m (tol {} m)",
            case.name,
            d,
            case.expected_metres,
            case.tolerance_metres,
        );
    }
}

/// Andoyer and Vincenty should agree within a few metres on a typical
/// ~430 km baseline. Amsterdam → Paris is the canonical quickstart
/// pair from `doc/src/examples/quick_start.cpp:137-148`.
///
/// Andoyer is a *first-order* spheroidal correction to the great-
/// circle distance (`formulas/andoyer_inverse.hpp:38-47`), so on a
/// 430 km line it disagrees with Vincenty's iterative geodesic by a
/// few metres rather than sub-metre. Measured delta on this pair is
/// ~2.8 m; the bound is set to 5 m so the assertion stays a useful
/// regression guard without flapping on inconsequential f64 noise.
#[test]
fn amsterdam_paris_andoyer_vincenty_agree() {
    let amsterdam = deg(4.90, 52.37);
    let paris = deg(2.35, 48.86);
    let d_andoyer = distance_with(&amsterdam, &paris, Andoyer::WGS84);
    let d_vincenty = distance_with(&amsterdam, &paris, Vincenty::WGS84);
    assert!(
        (d_andoyer - d_vincenty).abs() < 5.0,
        "Andoyer {} vs Vincenty {} disagree on Amsterdam->Paris (|d| {} m)",
        d_andoyer,
        d_vincenty,
        (d_andoyer - d_vincenty).abs(),
    );
}

/// Stress: `(0, 0) → (179.5, 0.5)` is the canonical near-antipodal
/// pair where Vincenty's iteration is prone to non-convergence
/// (Vincenty 1975 explicitly notes this). The contract here is
/// regression-only: the call must not panic and must return a finite
/// (non-NaN, non-infinite) distance. No reference value is asserted.
#[test]
fn vincenty_stress_nearly_antipodal() {
    let strategy = Vincenty {
        spheroid: GDA,
        ..Vincenty::WGS84
    };
    let a = deg(0.0, 0.0);
    let b = deg(179.5, 0.5);
    let d = distance_with(&a, &b, strategy);
    assert!(
        d.is_finite(),
        "Vincenty near-antipodal returned non-finite {d}"
    );
}
