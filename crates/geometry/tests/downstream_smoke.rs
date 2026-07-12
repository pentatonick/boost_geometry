//! Integration smoke test that uses ONLY the `geometry` crate's
//! public surface — no direct imports from inner crates. If anything
//! below refers to an inner crate path (`geometry_strategy::…` etc.)
//! the facade is incomplete.
//!
//! Mirrors the four quickstart examples (Cartesian, comparable,
//! spherical, geographic) plus a `#[derive(Point)]` round-trip, all
//! exercised via `boost_geometry::*` only.

// The 3-4-5 and squared 3-4-5 results are exact in IEEE 754, so we
// compare against them with `==` instead of an epsilon — clippy's
// blanket float-cmp warning does not apply here.
#![allow(clippy::float_cmp)]

use boost_geometry::adapt::{Adapt, WithCs};
use boost_geometry::cs::{Cartesian, Degree, Geographic, Spherical};
use boost_geometry::model::Point2D;
use boost_geometry::prelude::*;
use boost_geometry::strategy::geographic::{Andoyer, Vincenty};
use boost_geometry::strategy::spherical::Haversine;

#[test]
fn cartesian_3_4_5_via_facade() {
    let a = Point2D::<f64, Cartesian>::new(0.0, 0.0);
    let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
    assert_eq!(distance(&a, &b), 5.0);
}

#[test]
fn comparable_distance_via_facade() {
    let a = Point2D::<f64, Cartesian>::new(0.0, 0.0);
    let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
    assert_eq!(comparable_distance(&a, &b), 25.0);
}

/// Amsterdam → Paris on the unit sphere via Haversine. Multiplying
/// the returned angle by a 3959-mile earth radius reproduces the
/// `Distance in miles: 267.02` quickstart figure
/// (`doc/src/examples/quick_start.cpp:137-148`).
#[test]
fn spherical_amsterdam_paris_via_facade() {
    let ams = WithCs::<_, Spherical<Degree>>::new(Adapt([4.90_f64, 52.37]));
    let par = WithCs::<_, Spherical<Degree>>::new(Adapt([2.35_f64, 48.86]));
    let angle = distance_with(&ams, &par, Haversine::UNIT);
    let miles = angle * 3959.0;
    assert!(
        (miles - 267.02).abs() < 0.05,
        "haversine miles {miles} outside tolerance (got angle {angle} rad)"
    );
}

/// Amsterdam → Paris on WGS84 via the default Andoyer strategy.
/// Boost's reference value is ~430 km; verify within 1 km.
#[test]
fn geographic_amsterdam_paris_via_facade() {
    let ams = WithCs::<_, Geographic<Degree>>::new(Adapt([4.90_f64, 52.37]));
    let par = WithCs::<_, Geographic<Degree>>::new(Adapt([2.35_f64, 48.86]));
    let metres = distance_with(&ams, &par, Andoyer::WGS84);
    let km = metres / 1_000.0;
    assert!(
        (km - 430.0).abs() < 1.0,
        "Andoyer Amsterdam->Paris {km} km outside 1 km tolerance"
    );

    // Vincenty should agree with Andoyer to within a few metres on
    // this short pair — sanity-check that path resolves too.
    let m_v = distance_with(&ams, &par, Vincenty::WGS84);
    assert!(
        (metres - m_v).abs() < 50.0,
        "Andoyer {metres} m disagrees with Vincenty {m_v} m"
    );
}

/// `#[derive(Point)]` is re-exported at the crate root so users
/// only need a single `geometry` dependency. Verify a round-trip
/// through `distance`.
#[test]
fn derive_macro_re_exported() {
    use boost_geometry::Point as DerivePoint;

    #[derive(Default, DerivePoint)]
    #[geometry(cs = "Cartesian", scalar = "f64")]
    struct MyXy {
        x: f64,
        y: f64,
    }

    let d = distance(&MyXy { x: 0.0, y: 0.0 }, &MyXy { x: 3.0, y: 4.0 });
    assert_eq!(d, 5.0);
}
