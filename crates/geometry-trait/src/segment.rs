//! The [`Segment`] concept and the [`segment_start`] / [`segment_end`]
//! materialiser helpers.
//!
//! Mirrors `doc/concept/segment.qbk` and the model in
//! `boost/geometry/geometries/segment.hpp`. The two-axis access
//! pattern (endpoint index, then dimension) is provided by
//! [`crate::IndexedAccess`]; this module layers the concept marker on
//! top and exposes the two endpoints as full [`crate::Point`] values
//! reconstructed on demand from the indexed accessors.
//!
//! Design notes:
//! * Boost spells the two endpoints as `s.first` / `s.second` on
//!   `model::segment` (see `boost/geometry/geometries/segment.hpp:55-71`)
//!   and reads/writes them through
//!   `traits::indexed_access<segment, 0, D>` /
//!   `traits::indexed_access<segment, 1, D>`
//!   (`boost/geometry/geometries/segment.hpp:136-169`).
//! * The Rust port keeps only the indexed view as the *required* shape
//!   on the concept — implementers do not need to surface
//!   `&Self::Point` accessors directly. The free helpers below
//!   reconstruct a [`Point`] value via `Default::default()` followed
//!   by per-dimension `set::<D>` writes driven by a private
//!   const-recursive iterator. This matches the Boost behaviour where
//!   `indexed_access::set` writes the dimension on the underlying
//!   endpoint point.

use crate::geometry::Geometry;
use crate::indexed_access::IndexedAccess;
use crate::point::{Point, PointMut};
use geometry_tag::SegmentTag;

/// A two-point segment.
///
/// Models `doc/concept/segment.qbk` — equivalent in C++ to the trait
/// specialisations on `model::segment` in
/// `boost/geometry/geometries/segment.hpp:119-183`. Implementers
/// provide [`IndexedAccess`] for `I` ∈ {`0`, `1`}, where `0` is the
/// start endpoint and `1` is the end endpoint. The endpoints can but
/// need not be materialised as `Point` values; [`segment_start`] /
/// [`segment_end`] reconstruct them from the indexed accessors on
/// demand.
///
/// # Examples
///
/// ```
/// use geometry_trait::Segment;
/// fn _accepts<S: Segment>(_s: &S) {}
/// ```
pub trait Segment: Geometry<Kind = SegmentTag> + IndexedAccess {}

/// Materialise the start endpoint of a segment as a [`Point`] value.
///
/// Mirrors reading `s.first` on `model::segment`
/// (`boost/geometry/geometries/segment.hpp:142-150`). The point is
/// built via [`Default::default`] and one `set::<D>` call per
/// dimension, with each value sourced from
/// `s.get_indexed::<0, D>()`.
///
/// # Examples
///
/// ```
/// use geometry_trait::{segment_start, PointMut, Segment};
/// fn _ex<S: Segment>(s: &S) where S::Point: Default + PointMut { let _ = segment_start(s); }
/// ```
pub fn segment_start<S: Segment>(s: &S) -> S::Point
where
    S::Point: Default + PointMut,
{
    materialise::<S, 0>(s)
}

/// Materialise the end endpoint of a segment as a [`Point`] value.
///
/// Mirrors reading `s.second` on `model::segment`
/// (`boost/geometry/geometries/segment.hpp:160-168`). The point is
/// built via [`Default::default`] and one `set::<D>` call per
/// dimension, with each value sourced from
/// `s.get_indexed::<1, D>()`.
///
/// # Examples
///
/// ```
/// use geometry_trait::{segment_end, PointMut, Segment};
/// fn _ex<S: Segment>(s: &S) where S::Point: Default + PointMut { let _ = segment_end(s); }
/// ```
pub fn segment_end<S: Segment>(s: &S) -> S::Point
where
    S::Point: Default + PointMut,
{
    materialise::<S, 1>(s)
}

/// Reconstruct a [`Point`] from the indexed accessors of a geometry.
///
/// `I` is the corner / endpoint index (`0` / `1` for a segment;
/// [`crate::corner::MIN`] / [`crate::corner::MAX`] for a box). The
/// returned point has `set::<D>(get_indexed::<I, D>())` written for
/// each `D` in `0..Point::DIM`.
///
/// Shared by [`segment_start`] / [`segment_end`] in this module and
/// by `box_min` / `box_max` in [`crate::boxg`]: both concepts use the
/// same `(I, D)` two-axis access from
/// `boost/geometry/core/access.hpp:79-84`, so the materialiser is
/// also shared.
pub(crate) fn materialise<G, const I: usize>(g: &G) -> G::Point
where
    G: IndexedAccess,
    G::Point: Default + PointMut,
{
    let mut p: G::Point = Default::default();
    match <G::Point as Point>::DIM {
        0 => {}
        1 => <Step<0, 1> as WriteDims<0, 1>>::run::<G, I>(g, &mut p),
        2 => <Step<0, 2> as WriteDims<0, 2>>::run::<G, I>(g, &mut p),
        3 => <Step<0, 3> as WriteDims<0, 3>>::run::<G, I>(g, &mut p),
        4 => <Step<0, 4> as WriteDims<0, 4>>::run::<G, I>(g, &mut p),
        _ => panic!("materialise: Point::DIM exceeds MAX_DIM (4)"),
    }
    p
}

/// Marker carrying a `(cursor, end)` pair of dimensions for the
/// indexed-access materialiser. Private; reachable only via
/// [`materialise`].
struct Step<const D: usize, const N: usize>;

/// Sealed const-recursive iterator that writes each dimension of a
/// point by copying from `get_indexed::<I, D>()`. Mirrors the shape
/// of the [`crate::point::fold_dims`] recursion, but with `set::<D>`
/// as the operation rather than a user-supplied closure.
trait WriteDims<const D: usize, const N: usize>: sealed::Sealed<D, N> {
    fn run<G, const I: usize>(g: &G, p: &mut G::Point)
    where
        G: IndexedAccess,
        G::Point: PointMut;
}

mod sealed {
    pub trait Sealed<const D: usize, const N: usize> {}
}

/// Base case: `D == N`, nothing left to write.
impl<const N: usize> sealed::Sealed<N, N> for Step<N, N> {}
impl<const N: usize> WriteDims<N, N> for Step<N, N> {
    #[inline]
    fn run<G, const I: usize>(_g: &G, _p: &mut G::Point)
    where
        G: IndexedAccess,
        G::Point: PointMut,
    {
    }
}

/// Inductive step for one fixed `(D, N)` pair. Writes dimension `D`
/// of the destination point from `get_indexed::<I, D>()`, then
/// descends to `Step<D + 1, N>`. We cannot write `D + 1` in a generic
/// bound on stable, so the macro unrolls one impl per pair.
macro_rules! impl_write_dims {
    ($d:expr, $n:expr) => {
        impl sealed::Sealed<$d, $n> for Step<$d, $n> {}
        impl WriteDims<$d, $n> for Step<$d, $n> {
            #[inline]
            fn run<G, const I: usize>(g: &G, p: &mut G::Point)
            where
                G: IndexedAccess,
                G::Point: PointMut,
            {
                let v = g.get_indexed::<I, $d>();
                <G::Point as PointMut>::set::<$d>(p, v);
                <Step<{ $d + 1 }, $n> as WriteDims<{ $d + 1 }, $n>>::run::<G, I>(g, p);
            }
        }
    };
}

// All (D, N) pairs with 0 <= D < N <= 4. Keep in sync with the match
// arms in `materialise` and with `crate::point::MAX_DIM`.
impl_write_dims!(0, 1);
impl_write_dims!(0, 2);
impl_write_dims!(1, 2);
impl_write_dims!(0, 3);
impl_write_dims!(1, 3);
impl_write_dims!(2, 3);
impl_write_dims!(0, 4);
impl_write_dims!(1, 4);
impl_write_dims!(2, 4);
impl_write_dims!(3, 4);

#[cfg(test)]
mod tests {
    //! Synthetic Segment round-trip — mirrors
    //! `boost/geometry/test/core/access.cpp:55-86` for the segment
    //! case. The `MySegment` here is *not* `geometry-model::segment`;
    //! that lands in T17.

    use super::*;
    use geometry_cs::Cartesian;
    use geometry_tag::PointTag;

    #[derive(Default)]
    struct Xy {
        x: f64,
        y: f64,
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
            if D == 0 { self.x } else { self.y }
        }
    }

    impl PointMut for Xy {
        fn set<const D: usize>(&mut self, v: f64) {
            if D == 0 {
                self.x = v;
            } else {
                self.y = v;
            }
        }
    }

    struct MySegment {
        e: [[f64; 2]; 2],
    }

    impl Geometry for MySegment {
        type Kind = SegmentTag;
        type Point = Xy;
    }

    impl IndexedAccess for MySegment {
        fn get_indexed<const I: usize, const D: usize>(&self) -> f64 {
            self.e[I][D]
        }
        fn set_indexed<const I: usize, const D: usize>(&mut self, v: f64) {
            self.e[I][D] = v;
        }
    }

    impl Segment for MySegment {}

    #[test]
    fn segment_indexed_round_trip() {
        let mut s = MySegment { e: [[0.0; 2]; 2] };
        s.set_indexed::<0, 0>(7.0);
        s.set_indexed::<1, 1>(8.0);
        assert_eq!(s.get_indexed::<0, 0>().to_bits(), 7.0_f64.to_bits());
        assert_eq!(s.get_indexed::<1, 1>().to_bits(), 8.0_f64.to_bits());
    }

    #[test]
    fn segment_endpoints_materialise() {
        let mut s = MySegment { e: [[0.0; 2]; 2] };
        s.set_indexed::<0, 0>(1.0);
        s.set_indexed::<0, 1>(2.0);
        s.set_indexed::<1, 0>(3.0);
        s.set_indexed::<1, 1>(4.0);

        let a = segment_start(&s);
        let b = segment_end(&s);
        assert_eq!(a.get::<0>().to_bits(), 1.0_f64.to_bits());
        assert_eq!(a.get::<1>().to_bits(), 2.0_f64.to_bits());
        assert_eq!(b.get::<0>().to_bits(), 3.0_f64.to_bits());
        assert_eq!(b.get::<1>().to_bits(), 4.0_f64.to_bits());
    }
}
