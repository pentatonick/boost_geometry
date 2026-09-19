//! The const-recursive dimension dispatch, at every arity it supports.
//!
//! [`fold_dims`] and the indexed-access materialiser behind
//! [`segment_start`] / [`segment_end`] both turn a runtime `DIM` into a
//! const-generic recursion through a `match` over `0..=MAX_DIM`, with a
//! `_` arm that panics. Mirrors the dimension-agnostic access in
//! `boost/geometry/core/access.hpp`, where C++ recovers the arity from
//! `dimension<Geometry>::value` at compile time and needs no such
//! dispatch.
//!
//! The rest of the suite only ever builds 2D and 3D geometries, so
//! every other arm — including both documented panics — went unrun. A
//! transposed or off-by-one row in either `match` would have compiled
//! and shipped.

use geometry_cs::Cartesian;
use geometry_tag::{PointTag, SegmentTag};
use geometry_trait::{
    Geometry, IndexedAccess, Point, PointMut, Segment, fold_dims, segment_end, segment_start,
};

/// A point of any arity, so one fixture covers the whole dispatch table
/// (and one arity past it).
#[derive(Debug)]
struct NDim<const N: usize> {
    v: [f64; N],
}

// `[f64; N]: Default` only holds for `N <= 32` via the std macro impls,
// and `N` here is generic, so the array is built explicitly.
impl<const N: usize> Default for NDim<N> {
    fn default() -> Self {
        Self { v: [0.0; N] }
    }
}

impl<const N: usize> Geometry for NDim<N> {
    type Kind = PointTag;
    type Point = Self;
}

impl<const N: usize> Point for NDim<N> {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = N;

    fn get<const D: usize>(&self) -> f64 {
        self.v[D]
    }
}

impl<const N: usize> PointMut for NDim<N> {
    fn set<const D: usize>(&mut self, value: f64) {
        self.v[D] = value;
    }
}

/// The dimensions `fold_dims` visits, in the order it visits them.
fn visited<const N: usize>() -> Vec<usize> {
    let p = NDim::<N>::default();
    fold_dims(Vec::new(), &p, |mut acc, _p, i| {
        acc.push(i);
        acc
    })
}

#[test]
fn fold_dims_walks_ascending_at_every_supported_arity() {
    assert_eq!(visited::<0>(), Vec::<usize>::new());
    assert_eq!(visited::<1>(), vec![0]);
    assert_eq!(visited::<2>(), vec![0, 1]);
    assert_eq!(visited::<3>(), vec![0, 1, 2]);
    assert_eq!(visited::<4>(), vec![0, 1, 2, 3]);
}

/// A zero-dimensional point yields the accumulator untouched — the base
/// case of the recursion, and the one arm that does no work at all.
#[test]
fn fold_dims_on_a_zero_dim_point_returns_the_seed() {
    let p = NDim::<0>::default();
    let acc = fold_dims(7usize, &p, |acc, _p, i| acc + i + 100);
    assert_eq!(acc, 7);
}

/// The accumulator threads through every step rather than being
/// rebuilt, so a fold that depends on order gives the ordered answer.
#[test]
fn fold_dims_threads_the_accumulator_through_each_step() {
    let p = NDim::<4>::default();
    let folded = fold_dims(String::new(), &p, |mut acc, _p, i| {
        acc.push_str(&i.to_string());
        acc
    });
    assert_eq!(folded, "0123");
}

/// `fold_dims` documents a `# Panics` clause for an arity past
/// `MAX_DIM`. It had never been run.
#[test]
#[should_panic(expected = "fold_dims: DIM exceeds MAX_DIM")]
fn fold_dims_panics_past_max_dim() {
    let p = NDim::<5>::default();
    let _ = fold_dims(0usize, &p, |acc, _p, i| acc + i);
}

/// A segment of any arity, carrying its two endpoints as raw ordinates
/// so the materialiser is the only thing that turns them into points.
struct NSegment<const N: usize> {
    e: [[f64; N]; 2],
}

impl<const N: usize> Geometry for NSegment<N> {
    type Kind = SegmentTag;
    type Point = NDim<N>;
}

impl<const N: usize> IndexedAccess for NSegment<N> {
    fn get_indexed<const I: usize, const D: usize>(&self) -> f64 {
        self.e[I][D]
    }
    fn set_indexed<const I: usize, const D: usize>(&mut self, v: f64) {
        self.e[I][D] = v;
    }
}

impl<const N: usize> Segment for NSegment<N> {}

/// Endpoints whose ordinates are all distinct, so a materialiser that
/// transposed the endpoint index with the dimension index, or repeated
/// a row, would produce the wrong numbers rather than the right ones by
/// luck.
fn distinct<const N: usize>() -> NSegment<N> {
    let mut s = NSegment::<N> { e: [[0.0; N]; 2] };
    for i in 0..2 {
        for d in 0..N {
            s.e[i][d] = f64::from(u16::try_from(i * 100 + d + 1).unwrap());
        }
    }
    s
}

fn ordinates<const N: usize>(p: &NDim<N>) -> Vec<f64> {
    p.v.to_vec()
}

#[test]
fn segment_endpoints_materialise_at_every_supported_arity() {
    assert_eq!(
        ordinates(&segment_start(&distinct::<0>())),
        Vec::<f64>::new()
    );
    assert_eq!(ordinates(&segment_end(&distinct::<0>())), Vec::<f64>::new());

    assert_eq!(ordinates(&segment_start(&distinct::<1>())), vec![1.0]);
    assert_eq!(ordinates(&segment_end(&distinct::<1>())), vec![101.0]);

    assert_eq!(ordinates(&segment_start(&distinct::<2>())), vec![1.0, 2.0]);
    assert_eq!(
        ordinates(&segment_end(&distinct::<2>())),
        vec![101.0, 102.0]
    );

    assert_eq!(
        ordinates(&segment_start(&distinct::<3>())),
        vec![1.0, 2.0, 3.0]
    );
    assert_eq!(
        ordinates(&segment_end(&distinct::<3>())),
        vec![101.0, 102.0, 103.0]
    );

    assert_eq!(
        ordinates(&segment_start(&distinct::<4>())),
        vec![1.0, 2.0, 3.0, 4.0]
    );
    assert_eq!(
        ordinates(&segment_end(&distinct::<4>())),
        vec![101.0, 102.0, 103.0, 104.0]
    );
}

/// A zero-dimensional segment materialises the `Default` point and
/// writes nothing into it — the `0 => {}` arm, which is the only arm
/// whose correctness is that it does nothing.
#[test]
fn a_zero_dim_segment_materialises_the_default_point() {
    let s = NSegment::<0> { e: [[]; 2] };
    assert_eq!(ordinates(&segment_start(&s)), Vec::<f64>::new());
}

/// The materialiser panics past `MAX_DIM`, the counterpart to
/// `fold_dims`'s clause. Also never run before.
#[test]
#[should_panic(expected = "materialise: Point::DIM exceeds MAX_DIM")]
fn segment_start_panics_past_max_dim() {
    let _ = segment_start(&distinct::<5>());
}

#[test]
#[should_panic(expected = "materialise: Point::DIM exceeds MAX_DIM")]
fn segment_end_panics_past_max_dim() {
    let _ = segment_end(&distinct::<5>());
}
