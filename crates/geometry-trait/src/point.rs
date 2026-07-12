//! The [`Point`] concept and the [`fold_dims`] dimension iterator.
//!
//! Mirrors `boost/geometry/geometries/concepts/point_concept.hpp`
//! together with the per-dimension `traits::access<P, D>::{get,set}`
//! free-function pair declared in
//! `boost/geometry/core/access.hpp:270-313`.
//!
//! Design notes (`specs/rust-port-proposal.md` §3.2):
//! * The five C++ trait specialisations collapse to four associated
//!   items on one trait plus the super-bound on [`Geometry`].
//! * `get` and `set` take the dimension as a *const generic*, the
//!   Rust analogue of Boost's `template <std::size_t Dim>` parameter
//!   on `traits::access`. Out-of-range access becomes a compile
//!   error (cf. `tests/ui/get_out_of_range.rs`).
//! * Dimension-by-dimension recursion (the C++ idiom in
//!   `strategies/cartesian/distance_pythagoras.hpp:44-66`) is
//!   expressed by [`fold_dims`], which descends through a private
//!   sealed [`Recurse`] trait.
//!
//! Phase 01 split (KC1.T1): the v1 `Point` trait fused `Point` and
//! `ConstPoint` from the C++ side. It now carries only the *read*
//! surface; the *write* surface (`set::<D>`) lives on a
//! [`PointMut`]`: Point` subtrait. Mirrors the `Point` / `ConstPoint`
//! concept pair in
//! `boost/geometry/geometries/concepts/point_concept.hpp:82-133`.

use geometry_coords::CoordinateScalar;
use geometry_cs::CoordinateSystem;
use geometry_tag::PointTag;

use crate::geometry::Geometry;

/// The Point concept.
///
/// Models `doc/concept/point.qbk` — equivalent in C++ to the five
/// trait specialisations spelled out in
/// `boost/geometry/geometries/concepts/point_concept.hpp` and the
/// per-dimension accessor declared in
/// `boost/geometry/core/access.hpp:270-313`. The five C++ pieces
/// collapse here to four associated items plus the super-bound:
///
/// | C++ trait | Rust counterpart |
/// |---|---|
/// | `traits::tag<P>` (`core/tag.hpp`) | `Geometry<Kind = PointTag>` |
/// | `traits::dimension<P>` (`core/coordinate_dimension.hpp`) | `const DIM: usize` |
/// | `traits::coordinate_type<P>` (`core/coordinate_type.hpp`) | `type Scalar` |
/// | `traits::coordinate_system<P>` (`core/coordinate_system.hpp`) | `type Cs` |
/// | `traits::access<P, D>::get` (`core/access.hpp:270-313`) | `fn get::<D>` |
///
/// The read-write half — `traits::access<P, D>::set` — lives on the
/// [`PointMut`] subtrait, mirroring the way Boost's mutating `Point`
/// concept sits above the read-only `ConstPoint`.
///
/// The `Point = Self` projection on the [`Geometry`] super-trait is
/// the Rust spelling of Boost's "a Point is its own point type"
/// invariant — `point_type<P>::type = P` whenever
/// `tag<P>::type = point_tag`.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_tag::PointTag;
/// use geometry_trait::{Geometry, Point};
///
/// #[derive(Default)]
/// struct Xy { x: f64, y: f64 }
///
/// impl Geometry for Xy { type Kind = PointTag; type Point = Self; }
///
/// impl Point for Xy {
///     type Scalar = f64;
///     type Cs = Cartesian;
///     const DIM: usize = 2;
///     fn get<const D: usize>(&self) -> f64 { if D == 0 { self.x } else { self.y } }
/// }
///
/// let p = Xy { x: 3.0, y: 4.0 };
/// assert_eq!(p.get::<0>(), 3.0);
/// ```
#[diagnostic::on_unimplemented(
    message = "`{Self}` does not implement the Point concept",
    label = "implement `Point` for this type, or wrap it in `Adapt` / `WithCs`",
    note = "see `geometry_trait::Point` — four items are required \
            (Kind = PointTag, Point = Self, Scalar, Cs, DIM, get::<D>). \
            If you also need to *write* coordinates, additionally \
            implement `PointMut`."
)]
pub trait Point: Geometry<Kind = PointTag, Point = Self> + Sized {
    /// The scalar coordinate type.
    ///
    /// Mirrors `boost::geometry::traits::coordinate_type<P>::type`
    /// (`boost/geometry/core/coordinate_type.hpp`).
    type Scalar: CoordinateScalar;

    /// The coordinate system this point lives in.
    ///
    /// Mirrors `boost::geometry::traits::coordinate_system<P>::type`
    /// (`boost/geometry/core/coordinate_system.hpp`).
    type Cs: CoordinateSystem;

    /// The number of dimensions.
    ///
    /// Mirrors `boost::geometry::traits::dimension<P>::value`
    /// (`boost/geometry/core/coordinate_dimension.hpp`).
    const DIM: usize;

    /// Read coordinate `D`.
    ///
    /// Mirrors `boost::geometry::traits::access<P, D>::get(p)`
    /// (`boost/geometry/core/access.hpp:270-313`). `D` is a const
    /// generic: passing an out-of-range index is rejected at the
    /// call site rather than at runtime.
    fn get<const D: usize>(&self) -> Self::Scalar;
    // The read-write half — `set::<D>` — lives on [`PointMut`] below.
}

/// The *mutating* half of the Point concept.
///
/// Mirrors Boost's read-write `Point` concept sitting above the
/// read-only `ConstPoint` in
/// `boost/geometry/geometries/concepts/point_concept.hpp:82-133`.
/// Algorithms that need to *construct* a Point — e.g. the
/// [`segment_start`](crate::segment_start) /
/// [`segment_end`](crate::segment_end) materialisers, the
/// [`box_min`](crate::box_min) / [`box_max`](crate::box_max) helpers,
/// the envelope output builder in `geometry-strategy::envelope` —
/// bound on `PointMut`; everything
/// else (`distance`, `length`, `area`, `within`, `intersects`,
/// `equals`, `comparable_distance`) bounds on the read-only
/// [`Point`] only.
///
/// Implementers that already provide [`Point`] get `PointMut` for
/// almost free: add `impl PointMut for MyPoint { fn set::<D>(…) }`
/// next to the existing `impl Point`. Every kernel concrete `Point`
/// impl (model, derive, `Adapt` array/tuple, `WithCs`) ships the
/// matching `PointMut` impl.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_tag::PointTag;
/// use geometry_trait::{Geometry, Point, PointMut};
///
/// #[derive(Default)]
/// struct Xy { x: f64, y: f64 }
/// impl Geometry for Xy { type Kind = PointTag; type Point = Self; }
/// impl Point for Xy {
///     type Scalar = f64;
///     type Cs = Cartesian;
///     const DIM: usize = 2;
///     fn get<const D: usize>(&self) -> f64 { if D == 0 { self.x } else { self.y } }
/// }
/// impl PointMut for Xy {
///     fn set<const D: usize>(&mut self, v: f64) { if D == 0 { self.x = v } else { self.y = v } }
/// }
///
/// let mut p = Xy::default();
/// p.set::<0>(3.0);
/// assert_eq!(p.get::<0>(), 3.0);
/// ```
#[diagnostic::on_unimplemented(
    message = "`{Self}` implements `Point` but not `PointMut`, and this algorithm needs to construct a point",
    label = "implement `PointMut` for this type, or feed the algorithm a mutable point type instead",
    note = "see `geometry_trait::PointMut` — adds `set::<D>` next to the inherited `get::<D>`. \
            Algorithms that only read coordinates (distance, length, area, within, intersects, \
            equals, comparable_distance) require `Point` only — this bound is reached when the \
            algorithm needs to *output* a Point (envelope, segment_start, box_min, …)."
)]
pub trait PointMut: Point {
    /// Write coordinate `D`.
    ///
    /// Mirrors `boost::geometry::traits::access<P, D>::set(p, v)`
    /// (`boost/geometry/core/access.hpp:270-313`) — the read-write
    /// half of `traits::access` only available on `Point` (not
    /// `ConstPoint`) in
    /// `boost/geometry/geometries/concepts/point_concept.hpp:82-133`.
    fn set<const D: usize>(&mut self, value: Self::Scalar);
}

/// Fold a closure over the dimensions `0, 1, …, P::DIM - 1` of a
/// [`Point`] at compile time.
///
/// Used by Pythagoras, equality, transforms — anything that needs
/// to iterate over coordinates without paying a runtime bounds
/// check. Mirrors the C++ template-recursion pattern in
/// `boost/geometry/strategies/cartesian/distance_pythagoras.hpp:44-66`
/// (the `detail::compute_pythagoras<I, T>` recursion).
///
/// The closure is invoked once per dimension, in ascending order.
/// It receives the running accumulator, the point, and the
/// dimension index (as a `usize`, only as a label — `get::<D>`
/// itself must still be called with a const-generic `D`, which is
/// what the `Recurse` sealed trait below provides for the
/// dimensions kernel code actually needs).
///
/// # Supported dimensions
///
/// Const-generic recursion via `I + 1` requires the unstable
/// `generic_const_exprs` feature. To stay on stable, the
/// recursive descent is unrolled by `Recurse` for the dimensions
/// the kernel needs (1‥=`MAX_DIM`). `Point` impls with
/// `DIM > MAX_DIM` are a compile-time error at the call site —
/// raise `MAX_DIM` and add the matching `impl_recurse!` row.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_tag::PointTag;
/// use geometry_trait::{fold_dims, Geometry, Point};
///
/// #[derive(Default)]
/// struct Xy { x: f64, y: f64 }
/// impl Geometry for Xy { type Kind = PointTag; type Point = Self; }
/// impl Point for Xy {
///     type Scalar = f64;
///     type Cs = Cartesian;
///     const DIM: usize = 2;
///     fn get<const D: usize>(&self) -> f64 { if D == 0 { self.x } else { self.y } }
/// }
///
/// let p = Xy { x: 3.0, y: 4.0 };
/// // Fold the closure over each dimension; the recursion supplies the
/// // dimension index, the closure does its own per-D dispatch.
/// let sum = fold_dims(0.0_f64, &p, |acc, p, i| {
///     let v = match i { 0 => p.get::<0>(), 1 => p.get::<1>(), _ => 0.0 };
///     acc + v
/// });
/// assert_eq!(sum, 7.0);
/// ```
///
/// In practice algorithm code uses [`fold_dims`] indirectly via
/// the `Recurse` machinery already wired below: the closure can
/// rely on the dimension index being supplied to it, but
/// `Point::get::<D>` calls inside the closure are issued with
/// hard-coded `D`s by the recursion impls themselves.
///
/// # Panics
///
/// Panics if `P::DIM` exceeds `MAX_DIM`. Raise `MAX_DIM` and
/// add the matching `impl_recurse!` row to support a wider point.
pub fn fold_dims<P, T, F>(init: T, point: &P, mut f: F) -> T
where
    P: Point,
    F: FnMut(T, &P, usize) -> T,
{
    // `P::DIM` is monomorphisation-time constant but cannot appear in
    // a const-generic position on stable Rust. We dispatch via a
    // `match` to the right `Cursor<0, N>` impl; the unreachable arms
    // collapse to dead code once the compiler sees the concrete `DIM`.
    match P::DIM {
        0 => <Cursor<0, 0> as Recurse<0, 0>>::step(init, point, &mut f),
        1 => <Cursor<0, 1> as Recurse<0, 1>>::step(init, point, &mut f),
        2 => <Cursor<0, 2> as Recurse<0, 2>>::step(init, point, &mut f),
        3 => <Cursor<0, 3> as Recurse<0, 3>>::step(init, point, &mut f),
        4 => <Cursor<0, 4> as Recurse<0, 4>>::step(init, point, &mut f),
        _ => panic!("fold_dims: DIM exceeds MAX_DIM ({MAX_DIM})"),
    }
}

/// Largest `DIM` the const-recursive [`fold_dims`] supports on
/// stable Rust. Raise this constant and add the corresponding
/// `impl_recurse!` row below when a strategy needs more.
///
/// Three is the working assumption (XYZ); four is included so
/// future homogeneous-coordinate kernels do not stall waiting for
/// `generic_const_exprs` to stabilise.
pub const MAX_DIM: usize = 4;

/// Marker carrying a `(cursor, end)` pair of dimensions to descend
/// over. Private; reachable only via [`fold_dims`].
struct Cursor<const I: usize, const N: usize>;

/// Sealed const-recursive iterator over the dimensions of a
/// [`Point`]. The two const parameters are the *current* dimension
/// `I` and the *end* dimension `N` (= `P::DIM`).
///
/// Mirrors the `template <size_t I, typename T> compute_pythagoras`
/// recursion in
/// `boost/geometry/strategies/cartesian/distance_pythagoras.hpp:44-66`,
/// except the C++ side counts *down* from `I = DIM` to `0` while
/// we count *up* — same shape, more natural Rust read order.
///
/// On nightly with `generic_const_exprs` this trait would collapse
/// to a single blanket impl
/// `impl<const I: usize, const N: usize> Recurse<I, N> for Cursor<I, N>
///   where Cursor<{I + 1}, N>: Recurse<{I + 1}, N>`. On stable we
/// unroll the recursion via the [`impl_recurse!`] macro below for
/// every dimension up to [`MAX_DIM`].
trait Recurse<const I: usize, const N: usize>: sealed::Sealed<I, N> {
    fn step<P, T, F>(acc: T, point: &P, f: &mut F) -> T
    where
        P: Point,
        F: FnMut(T, &P, usize) -> T;
}

mod sealed {
    pub trait Sealed<const I: usize, const N: usize> {}
}

/// Base case: `I == N`, nothing left to visit, return the
/// accumulator. Matches the partial specialisation
/// `compute_pythagoras<0, T>` in the C++ header.
impl<const N: usize> sealed::Sealed<N, N> for Cursor<N, N> {}
impl<const N: usize> Recurse<N, N> for Cursor<N, N> {
    #[inline]
    fn step<P, T, F>(acc: T, _point: &P, _f: &mut F) -> T
    where
        P: Point,
        F: FnMut(T, &P, usize) -> T,
    {
        acc
    }
}

/// Inductive step for one fixed `(I, N)` pair. Calls the closure
/// with dimension `I`, then descends to `Cursor<I + 1, N>`. We
/// cannot write `I + 1` in a generic bound on stable, so the
/// macro unrolls one impl per pair we want to support.
macro_rules! impl_recurse {
    ($i:expr, $n:expr) => {
        impl sealed::Sealed<$i, $n> for Cursor<$i, $n> {}
        impl Recurse<$i, $n> for Cursor<$i, $n> {
            #[inline]
            fn step<P, T, F>(acc: T, point: &P, f: &mut F) -> T
            where
                P: Point,
                F: FnMut(T, &P, usize) -> T,
            {
                let acc = f(acc, point, $i);
                <Cursor<{ $i + 1 }, $n> as Recurse<{ $i + 1 }, $n>>::step(acc, point, f)
            }
        }
    };
}

// All (I, N) pairs with 0 <= I < N <= MAX_DIM. Keep `MAX_DIM` and
// this block in sync.
impl_recurse!(0, 1);
impl_recurse!(0, 2);
impl_recurse!(1, 2);
impl_recurse!(0, 3);
impl_recurse!(1, 3);
impl_recurse!(2, 3);
impl_recurse!(0, 4);
impl_recurse!(1, 4);
impl_recurse!(2, 4);
impl_recurse!(3, 4);
