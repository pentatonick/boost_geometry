//! Runs the proposal §5 end-user walkthrough as a cargo test.
//!
//! Each test mirrors a fenced code block from the `geometry` crate's
//! top-level rustdoc (`crates/geometry/src/lib.rs`). The doctests in
//! lib.rs already compile-and-run these examples, but `cargo test
//! --workspace` (without `--doc`) does not pick them up; this file is
//! the "the rustdoc examples also run as unit tests" requirement from
//! the T48 spec.

// The 3-4-5 and squared 3-4-5 results are exact in IEEE 754; comparing
// them with `==` is correct, so opt out of the blanket float-cmp lint.
#![allow(clippy::float_cmp)]

use boost_geometry::Point as DerivePoint;
use boost_geometry::adapt::{Adapt, WithCs};
use boost_geometry::model::Polygon as ModelPolygon;
use boost_geometry::prelude::*;
use boost_geometry::strategy::geographic::Vincenty;
use boost_geometry::strategy::spherical::Haversine;
use geometry_model::polygon;

#[test]
fn cartesian_distance_pythagoras() {
    let a = Point2D::<f64, Cartesian>::new(0.0, 0.0);
    let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
    assert_eq!(distance(&a, &b), 5.0);
    // Compare cheaply — no `sqrt`.
    assert_eq!(comparable_distance(&a, &b), 25.0);
}

#[test]
fn spherical_amsterdam_paris_haversine() {
    let ams = WithCs::<_, Spherical<Degree>>::new(Adapt([4.90_f64, 52.37]));
    let par = WithCs::<_, Spherical<Degree>>::new(Adapt([2.35_f64, 48.86]));
    let angle = distance_with(&ams, &par, Haversine::UNIT);
    let miles = angle * 3959.0;
    assert!(
        (miles - 267.02).abs() < 0.05,
        "haversine miles {miles} outside tolerance"
    );
}

#[test]
fn geographic_amsterdam_paris_andoyer_vs_vincenty() {
    let ams = WithCs::<_, Geographic<Degree>>::new(Adapt([4.90_f64, 52.37]));
    let par = WithCs::<_, Geographic<Degree>>::new(Adapt([2.35_f64, 48.86]));

    // No strategy argument — picks Andoyer for the geographic family.
    let m_a = distance(&ams, &par);
    let m_v = distance_with(&ams, &par, Vincenty::WGS84);

    assert!((m_a - m_v).abs() < 50.0);
    assert!(((m_a / 1_000.0) - 430.0).abs() < 1.0);
}

#[test]
fn derive_point_macro() {
    #[derive(Default, DerivePoint)]
    #[geometry(cs = "Cartesian", scalar = "f64")]
    struct MyXy {
        x: f64,
        y: f64,
    }

    let d = distance(&MyXy { x: 0.0, y: 0.0 }, &MyXy { x: 3.0, y: 4.0 });
    assert_eq!(d, 5.0);
}

#[test]
fn adapt_array_cartesian() {
    let a = Adapt([0.0_f64, 0.0]);
    let b = Adapt([3.0_f64, 4.0]);
    assert_eq!(distance(&a, &b), 5.0);
}

#[test]
fn polygon_area_and_within() {
    // Ring traversed clockwise (the default `PointOrder` for `Ring`).
    let p: ModelPolygon<Point2D<f64, Cartesian>> =
        polygon![[(0.0, 0.0), (0.0, 3.0), (4.0, 3.0), (4.0, 0.0), (0.0, 0.0)]];
    assert_eq!(area(&p), 12.0);
    assert!(within(&Point2D::<f64, Cartesian>::new(2.0, 1.0), &p));
    assert!(!within(&Point2D::<f64, Cartesian>::new(5.0, 5.0), &p));
}
