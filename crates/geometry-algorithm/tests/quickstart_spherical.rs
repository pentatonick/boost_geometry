//! Validation milestone M4 — Amsterdam–Paris reproduces.
//!
//! Mirrors `doc/src/examples/quick_start.cpp:137-148` (the spherical
//! quickstart that prints "Distance in miles: 267.02") and the
//! reference values in `test/strategies/haversine.cpp:29, 60-76`.
//!
//! The four tests exercise:
//!  1. The unit-sphere shortcut times a miles radius (the literal
//!     quickstart computation), driven through an `Adapt<[f64; 2]>`
//!     re-tagged into `Spherical<Degree>` via `WithCs`.
//!  2. The strategy-less `distance(&a, &b)` entry point — defaults
//!     for `SphericalFamily × SphericalFamily` should pick
//!     `Haversine::EARTH` (metres), and the value should agree with
//!     the miles result after a unit conversion.
//!  3. The explicit `Haversine::EARTH` strategy on `(4, 52) → (2, 48)`
//!     — Boost's headline `467_270.4 m` reference.
//!  4. The explicit `Haversine::EARTH` strategy on `(4, 52) → (2, 41)`
//!     — Amsterdam → Barcelona, `1_232_906.5 m`.

use geometry_adapt::{Adapt, WithCs};
use geometry_algorithm::{distance, distance_with};
use geometry_cs::{Degree, Spherical};
use geometry_strategy::spherical::Haversine;

type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;

#[inline]
fn deg(lon: f64, lat: f64) -> Sp {
    WithCs::new(Adapt([lon, lat]))
}

/// `doc/src/examples/quick_start.cpp:137-148` — Amsterdam and Paris,
/// computed on a unit sphere, then scaled by an earth radius in miles
/// (3959). The qbk advertises `Distance in miles: 267.02`.
#[test]
fn quickstart_amsterdam_paris_in_miles() {
    let amsterdam = deg(4.90, 52.37);
    let paris = deg(2.35, 48.86);
    let angle = distance_with(&amsterdam, &paris, Haversine::UNIT);
    let miles = angle * 3959.0;
    assert!(
        (miles - 267.02).abs() < 0.05,
        "got {miles} expected ~ 267.02",
    );
}

/// `DefaultDistance<SphericalFamily, SphericalFamily> = Haversine` (on
/// `Haversine::EARTH`), so `distance(&a, &b)` returns metres. Cross-
/// checks the miles answer above by converting 267.02 mi × 1609.344
/// m/mi ≈ `429_675` m. Tolerance is ±200 m to absorb the
/// quickstart-radius vs. average-earth-radius mismatch.
#[test]
fn default_haversine_amsterdam_paris_metres() {
    let amsterdam = deg(4.90, 52.37);
    let paris = deg(2.35, 48.86);
    let metres = distance(&amsterdam, &paris);
    assert!(
        (metres - 429_675.0).abs() < 200.0,
        "got {metres} metres expected ~ 429_675",
    );
}

/// `test/strategies/haversine.cpp:66-67` — Amsterdam → Paris on
/// `(4, 52) → (2, 48)` with the Boost average earth radius. The Boost
/// test asserts `467_270.4 m` within `1 m`.
#[test]
fn haversine_4_52_to_2_48_is_467_270_4_m() {
    let p1 = deg(4.0, 52.0);
    let p2 = deg(2.0, 48.0);
    let d = distance_with(&p1, &p2, Haversine::EARTH);
    assert!((d - 467_270.4).abs() < 1.0, "got {d} expected ~ 467_270.4");
}

/// `test/strategies/haversine.cpp:72-75` — Amsterdam → Barcelona on
/// `(4, 52) → (2, 41)`. The Boost test asserts `1_232_906.5 m` within
/// `1 m`.
#[test]
fn haversine_4_52_to_2_41_is_1_232_906_5_m() {
    let p1 = deg(4.0, 52.0);
    let p2 = deg(2.0, 41.0);
    let d = distance_with(&p1, &p2, Haversine::EARTH);
    assert!(
        (d - 1_232_906.5).abs() < 1.0,
        "got {d} expected ~ 1_232_906.5",
    );
}
