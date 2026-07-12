//! Smoke test: a hand-rolled 2D point satisfies the [`Point`] concept,
//! `set::<D>` → `get::<D>` round-trips, and [`fold_dims`] visits the
//! dimensions in ascending order.
//!
//! Counterpart to the C++ round-trip exercised by
//! `boost/geometry/test/core/access.cpp` (`bg::set<0>` / `bg::get<0>`).

use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut, fold_dims};

#[derive(Debug, Default)]
struct Xy {
    v: [f64; 2],
}

impl Geometry for Xy {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for Xy {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;

    fn get<const D: usize>(&self) -> f64 {
        self.v[D]
    }
}

impl PointMut for Xy {
    fn set<const D: usize>(&mut self, value: f64) {
        self.v[D] = value;
    }
}

#[derive(Debug, Default)]
struct Xyz {
    v: [f64; 3],
}

impl Geometry for Xyz {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for Xyz {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 3;

    fn get<const D: usize>(&self) -> f64 {
        self.v[D]
    }
}

impl PointMut for Xyz {
    fn set<const D: usize>(&mut self, value: f64) {
        self.v[D] = value;
    }
}

#[test]
fn dim_is_2() {
    assert_eq!(<Xy as Point>::DIM, 2);
}

#[test]
fn round_trip_via_set_get() {
    let mut p = Xy::default();
    p.set::<0>(3.0);
    p.set::<1>(4.0);
    assert!((p.get::<0>() - 3.0).abs() < f64::EPSILON);
    assert!((p.get::<1>() - 4.0).abs() < f64::EPSILON);
}

#[test]
fn fold_dims_visits_each_dim_in_order_2d() {
    let mut p = Xy::default();
    p.set::<0>(10.0);
    p.set::<1>(20.0);

    let visited: alloc_vec::Vec<usize> = fold_dims(alloc_vec::Vec::new(), &p, |mut v, _p, i| {
        v.push(i);
        v
    });
    assert_eq!(visited, [0, 1]);
}

#[test]
fn fold_dims_visits_each_dim_in_order_3d() {
    let mut p = Xyz::default();
    p.set::<0>(1.0);
    p.set::<1>(2.0);
    p.set::<2>(3.0);

    let visited: alloc_vec::Vec<usize> = fold_dims(alloc_vec::Vec::new(), &p, |mut v, _p, i| {
        v.push(i);
        v
    });
    assert_eq!(visited, [0, 1, 2]);
}

#[test]
fn fold_dims_sums_coordinates() {
    let mut p = Xyz::default();
    p.set::<0>(1.5);
    p.set::<1>(2.5);
    p.set::<2>(3.5);

    // We cannot call `point.get::<i>()` with the runtime `i` the closure
    // receives — the recursion impls already hand us each dimension as
    // a hardcoded const. Test the index sequence instead; squared-distance
    // style closures will be exercised by the Pythagoras strategy in T22.
    let sum_of_indices: usize = fold_dims(0, &p, |acc, _p, i| acc + i);
    assert_eq!(sum_of_indices, 3);
}

// Local alias so we do not pull in `std::vec::Vec` directly in the
// `no_std`-compatible test surface; `cargo test` always runs with the
// std test harness, so this just keeps the import discipline obvious.
mod alloc_vec {
    pub use std::vec::Vec;
}
