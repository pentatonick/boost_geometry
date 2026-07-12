//! [`Point`] impls for `Adapt<(T, T)>` and `Adapt<(T, T, T)>`.
//!
//! Mirrors `boost/geometry/geometries/adapted/boost_tuple.hpp`,
//! which registers `boost::tuple<T, T>` and `boost::tuple<T, T, T>`
//! as 2D and 3D Cartesian points. Rust's native tuples replace the
//! Boost ones; we adapt the two arities the C++ header covers.
//!
//! # Silent-Cartesian warning
//!
//! Both impls pin `type Cs = Cartesian`. A raw `Adapt((lon, lat))`
//! therefore satisfies *every* Cartesian-only strategy bound and
//! will silently compute Pythagorean nonsense over lat/lon. Wrap
//! geographic / spherical tuples with [`WithCs`](crate::WithCs) —
//! see [`Adapt`](crate::Adapt)'s rustdoc for the worked example.
//!
//! The `_ => unreachable!()` arms below are dead at runtime because
//! `D` is const-evaluated at the call site: a `get::<2>()` on
//! `Adapt<(T, T)>` fails to typecheck against `Point::DIM = 2` in
//! the bounds the algorithm layer enforces. A future `where D < N`
//! clause via `generic_const_exprs` would let us delete them.

use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut};

use crate::Adapt;

impl<T: CoordinateScalar> Geometry for Adapt<(T, T)> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar> Point for Adapt<(T, T)> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = 2;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        match D {
            0 => self.0.0,
            1 => self.0.1,
            _ => unreachable!(),
        }
    }
}

impl<T: CoordinateScalar> PointMut for Adapt<(T, T)> {
    #[inline]
    fn set<const D: usize>(&mut self, value: T) {
        match D {
            0 => self.0.0 = value,
            1 => self.0.1 = value,
            _ => unreachable!(),
        }
    }
}

impl<T: CoordinateScalar> Geometry for Adapt<(T, T, T)> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar> Point for Adapt<(T, T, T)> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = 3;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        match D {
            0 => self.0.0,
            1 => self.0.1,
            2 => self.0.2,
            _ => unreachable!(),
        }
    }
}

impl<T: CoordinateScalar> PointMut for Adapt<(T, T, T)> {
    #[inline]
    fn set<const D: usize>(&mut self, value: T) {
        match D {
            0 => self.0.0 = value,
            1 => self.0.1 = value,
            2 => self.0.2 = value,
            _ => unreachable!(),
        }
    }
}
