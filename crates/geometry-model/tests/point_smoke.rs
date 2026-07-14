//! Smoke tests for `model::Point<T, D, Cs>`.
//!
//! Mirrors the C++ tests in `boost/geometry/test/core/`:
//!
//! - `access.cpp` — `bg::set<D>` / `bg::get<D>` round-trip
//! - `tag.cpp` — `traits::tag<P>::type` resolves to `point_tag`
//! - `coordinate_dimension.cpp` — `traits::dimension<P>::value` matches
//! - `coordinate_type.cpp` — `traits::coordinate_type<P>::type` matches
//!
//! applied to the default `model::Point` (the type Boost calls
//! `model::point`, declared in `geometries/point.hpp`).

use geometry_cs::Cartesian;
use geometry_model::{Point2D, Point3D};
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point as PointTrait, PointMut};

/// Tight float comparison helper — the same `f64::EPSILON` idiom used
/// by `crates/geometry-trait/tests/point_smoke.rs`. All values written
/// and read here are integer-valued (`1.0`, `2.0`, …) and survive the
/// round-trip exactly, so the epsilon is purely defensive.
fn approx_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < f64::EPSILON
}

// test/core/access.cpp:46-51 — set then get round-trips in 2D.
#[test]
fn point2d_round_trip() {
    let mut p = Point2D::<f64, Cartesian>::default();
    p.set::<0>(1.0);
    p.set::<1>(2.0);
    assert!(approx_eq(p.get::<0>(), 1.0));
    assert!(approx_eq(p.get::<1>(), 2.0));
}

// test/core/access.cpp same pattern in 3D, also exercises `new`.
#[test]
fn point3d_round_trip() {
    let p = Point3D::<f64, Cartesian>::new(1.0, 2.0, 3.0);
    assert!(approx_eq(p.get::<0>(), 1.0));
    assert!(approx_eq(p.get::<1>(), 2.0));
    assert!(approx_eq(p.get::<2>(), 3.0));
}

// test/core/access.cpp — 2D constructor + get.
#[test]
fn point2d_new_get() {
    let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
    assert!(approx_eq(p.get::<0>(), 1.0));
    assert!(approx_eq(p.get::<1>(), 2.0));
}

// point.hpp:133-149 — the single-argument 1-D constructor.
#[test]
fn point1d_new_get() {
    let p = geometry_model::Point::<f64, 1, Cartesian>::new(7.5);
    assert!(approx_eq(p.get::<0>(), 7.5));
}

// test/core/tag.cpp — `tag<G>::type` resolves to `point_tag`.
#[test]
fn tag_is_point_tag() {
    fn assert_kind<P: Geometry<Kind = PointTag>>() {}
    assert_kind::<Point2D<f64, Cartesian>>();
    assert_kind::<Point3D<f32, Cartesian>>();
}

// test/core/coordinate_dimension.cpp — `dimension<P>::value` matches.
#[test]
fn dim_matches() {
    assert_eq!(<Point2D<f64, Cartesian> as PointTrait>::DIM, 2);
    assert_eq!(<Point3D<f64, Cartesian> as PointTrait>::DIM, 3);
}

// test/core/coordinate_type.cpp — `coordinate_type<P>::type` matches.
#[test]
fn scalar_matches() {
    fn accept_f64<P: PointTrait<Scalar = f64>>() {}
    fn accept_f32<P: PointTrait<Scalar = f32>>() {}
    accept_f64::<Point2D<f64, Cartesian>>();
    accept_f64::<Point3D<f64, Cartesian>>();
    accept_f32::<Point2D<f32, Cartesian>>();
}

// `Geometry::Point` projects to `Self` for any point (the
// "a point is its own point type" invariant from
// `core/point_type.hpp`).
#[test]
fn point_projects_to_self() {
    fn assert_self<P: Geometry<Point = P> + PointTrait>() {}
    assert_self::<Point2D<f64, Cartesian>>();
    assert_self::<Point3D<f64, Cartesian>>();
}

// `Default` produces a zero point — mirrors Boost's `m_values{}` in
// `geometries/point.hpp:113-119`.
#[test]
fn default_is_zero() {
    let p2 = Point2D::<f64, Cartesian>::default();
    assert!(approx_eq(p2.get::<0>(), 0.0));
    assert!(approx_eq(p2.get::<1>(), 0.0));
    let p3 = Point3D::<f64, Cartesian>::default();
    assert!(approx_eq(p3.get::<0>(), 0.0));
    assert!(approx_eq(p3.get::<1>(), 0.0));
    assert!(approx_eq(p3.get::<2>(), 0.0));
}
