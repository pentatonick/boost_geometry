//! [`Point`] impl for `Adapt<[T; N]>`.
//!
//! Mirrors `boost/geometry/geometries/adapted/c_array.hpp` (raw C
//! arrays) and `boost/geometry/geometries/adapted/std_array.hpp`
//! (`std::array<T, N>`) collapsed into one impl — Rust's `[T; N]`
//! covers both shapes.
//!
//! # Silent-Cartesian warning
//!
//! This impl pins `type Cs = Cartesian`. A raw `Adapt::<[f64; 2]>(
//! [lon, lat])` therefore satisfies *every* Cartesian-only strategy
//! bound and will silently compute Pythagorean nonsense over
//! lat/lon. Wrap geographic / spherical arrays with
//! [`WithCs`](crate::WithCs) — see [`Adapt`](crate::Adapt)'s
//! rustdoc for the worked example.

use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut};

use crate::Adapt;

impl<T: CoordinateScalar, const N: usize> Geometry for Adapt<[T; N]> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar, const N: usize> Point for Adapt<[T; N]> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = N;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        self.0[D]
    }
}

impl<T: CoordinateScalar, const N: usize> PointMut for Adapt<[T; N]> {
    #[inline]
    fn set<const D: usize>(&mut self, value: T) {
        self.0[D] = value;
    }
}
