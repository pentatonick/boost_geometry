//! [`Point`] impl for `Adapt<&[T; N]>` — a borrowed array.
//!
//! Mirrors `boost/geometry/geometries/adapted/c_array.hpp` (raw C
//! arrays) and `boost/geometry/geometries/adapted/std_array.hpp`
//! (`std::array<T, N>`); the C++ side adapts both via the same
//! read-only `traits::access` specialisation because writing through
//! a const pointer is forbidden anyway. The Rust port encodes that
//! read-only nature by implementing [`Point`] (read) but *not*
//! `PointMut` (write) — the split added in KC1.T1.
//!
//! # Silent-Cartesian warning
//!
//! Same as for [`Adapt<[T; N]>`](crate::Adapt): this impl pins
//! `type Cs = Cartesian`. A raw `Adapt::<&[f64; 2]>(&[lon, lat])`
//! therefore satisfies every Cartesian-only strategy bound and will
//! silently compute Pythagorean nonsense over lat/lon. Wrap
//! geographic / spherical borrowed arrays with [`WithCs`](crate::WithCs).

use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point};

use crate::Adapt;

impl<T: CoordinateScalar, const N: usize> Geometry for Adapt<&[T; N]> {
    type Kind = PointTag;
    type Point = Self;
}

impl<T: CoordinateScalar, const N: usize> Point for Adapt<&[T; N]> {
    type Scalar = T;
    type Cs = Cartesian;
    const DIM: usize = N;

    #[inline]
    fn get<const D: usize>(&self) -> T {
        self.0[D]
    }
    // No `set::<D>` — the borrow is read-only by construction.
    // No `impl PointMut` either, *deliberately*: callers who need
    // mutation must own their array.
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Coordinates are read back unchanged from a literal array."
)]
mod tests {
    //! Mirrors the access round-trip checks in
    //! `boost/geometry/test/core/access.cpp` (the C-array case) but
    //! for a borrow.

    use super::*;
    use geometry_trait::check_point;

    #[test]
    fn read_a_borrowed_array() {
        let storage = [3.0_f64, 4.0];
        let p = Adapt(&storage);
        assert_eq!(p.get::<0>(), 3.0);
        assert_eq!(p.get::<1>(), 4.0);
        assert_eq!(<Adapt<&[f64; 2]> as Point>::DIM, 2);
    }

    #[test]
    fn concept_check_passes_for_borrowed_adapter() {
        check_point::<Adapt<&[f64; 2]>>();
        check_point::<Adapt<&[f64; 3]>>();
    }

    /// Doctest-style demonstration: a `Vec` of borrows into a fixed
    /// backing store. The borrows live as long as the backing slice.
    #[test]
    fn vec_of_borrows_iterates() {
        let backing = [[1.0_f64, 2.0], [3.0, 4.0], [5.0, 6.0]];
        let points: Vec<Adapt<&[f64; 2]>> = backing.iter().map(Adapt).collect();
        assert_eq!(points.len(), 3);
        assert_eq!(points[2].get::<0>(), 5.0);
    }
}
