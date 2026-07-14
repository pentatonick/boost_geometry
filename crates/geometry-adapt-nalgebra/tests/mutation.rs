//! Exercise the read-write surface of the `nalgebra` wrappers that the
//! distance-only `interop.rs` milestone does not reach: `PointMut::set`,
//! `into_inner`, and per-ordinate `get` round-trips for every wrapper.

#![allow(
    clippy::float_cmp,
    reason = "each assertion reads back an exact value just written, with no arithmetic between"
)]

use geometry_adapt_nalgebra::{NaPoint2, NaPoint3, NaVector2, NaVector3};
use geometry_trait::{Point as _, PointMut as _};
use nalgebra::{Point2, Point3, Vector2, Vector3};

/// `NaPoint2::set` writes each ordinate through to the wrapped
/// `nalgebra::Point2`, visible via `get` and after `into_inner`.
#[test]
fn na_point2_set_get_and_unwrap() {
    let mut p = NaPoint2::new(Point2::new(0.0_f64, 0.0));
    p.set::<0>(3.0);
    p.set::<1>(4.0);
    assert_eq!(p.get::<0>(), 3.0);
    assert_eq!(p.get::<1>(), 4.0);
    let inner = p.into_inner();
    assert_eq!(inner, Point2::new(3.0, 4.0));
}

/// `NaPoint3::set` covers all three ordinates and `into_inner` recovers
/// the mutated point.
#[test]
fn na_point3_set_get_and_unwrap() {
    let mut p = NaPoint3::new(Point3::new(0.0_f64, 0.0, 0.0));
    p.set::<0>(1.0);
    p.set::<1>(2.0);
    p.set::<2>(3.0);
    assert_eq!((p.get::<0>(), p.get::<1>(), p.get::<2>()), (1.0, 2.0, 3.0));
    assert_eq!(p.into_inner(), Point3::new(1.0, 2.0, 3.0));
}

/// `NaVector2::set` writes through to the wrapped `nalgebra::Vector2`.
#[test]
fn na_vector2_set_get_and_unwrap() {
    let mut v = NaVector2::new(Vector2::new(0.0_f64, 0.0));
    v.set::<0>(5.0);
    v.set::<1>(6.0);
    assert_eq!((v.get::<0>(), v.get::<1>()), (5.0, 6.0));
    assert_eq!(v.into_inner(), Vector2::new(5.0, 6.0));
}

/// `NaVector3::set` covers all three ordinates and `into_inner` recovers
/// the mutated vector.
#[test]
fn na_vector3_set_get_and_unwrap() {
    let mut v = NaVector3::new(Vector3::new(0.0_f64, 0.0, 0.0));
    v.set::<0>(7.0);
    v.set::<1>(8.0);
    v.set::<2>(9.0);
    assert_eq!((v.get::<0>(), v.get::<1>(), v.get::<2>()), (7.0, 8.0, 9.0));
    assert_eq!(v.into_inner(), Vector3::new(7.0, 8.0, 9.0));
}
