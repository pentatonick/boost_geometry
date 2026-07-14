//! Pythagorean point-to-point distance for the Cartesian family.
//!
//! Mirrors `boost/geometry/strategies/cartesian/distance_pythagoras.hpp`:
//!
//! * lines 44-66 — the `detail::compute_pythagoras<I, T>` template
//!   recursion that walks the coordinates of the two points, summing
//!   the squared per-dimension differences. The Rust port reproduces
//!   the recursion via a sealed `SumSquares` trait so that
//!   `Point::get::<D>` is always invoked with a *const-generic* `D`,
//!   matching the C++ `template <std::size_t Dim>` access requirement.
//! * lines 71-117 — `namespace comparable::pythagoras`: the
//!   squared-distance form ([`ComparablePythagoras`]) that callers
//!   compare without paying for a `sqrt`.
//! * lines 134-173 — `pythagoras`: the sqrt-paying companion
//!   ([`Pythagoras`]).
//! * lines 276-283 — the `services::default_strategy<…, cartesian_tag,
//!   cartesian_tag>` specialisation that picks `pythagoras<>` as the
//!   default Cartesian × Cartesian distance strategy — reproduced as
//!   `impl DefaultDistance<CartesianFamily> for CartesianFamily`.
//!
//! For the calculation-type policy we deliberately deviate from the
//! C++ side: Boost runs the two scalars through
//! `util::calculation_type::geometric::binary` (`util/calculation_type.hpp`)
//! which promotes the working type independently of the input pair.
//! The v1 Rust port follows the T22 spec's "for simplicity" branch and
//! requires `P2::Scalar = P1::Scalar`; the [`Promote`](geometry_coords::Promote) lattice is
//! ready to fold in once a mixed-scalar caller appears
//! (see `geometry-coords::Promote`).

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::SameAs;
use geometry_trait::Point;

use crate::distance::{DefaultDistance, DistanceStrategy};

/// Pythagorean (Euclidean) distance for Cartesian points of any
/// dimension supported by `MAX_DIM` (from `geometry-trait`).
///
/// Mirrors `boost::geometry::strategy::distance::pythagoras<CalcType>`
/// from `strategies/cartesian/distance_pythagoras.hpp:134-173`. The
/// associated [`DistanceStrategy::Comparable`] type is
/// [`ComparablePythagoras`] — the squared-distance form, mirroring
/// `boost::geometry::strategy::distance::comparable::pythagoras`.
#[derive(Debug, Default, Clone, Copy)]
pub struct Pythagoras;

/// Squared Pythagorean distance for Cartesian points.
///
/// Mirrors `boost::geometry::strategy::distance::comparable::pythagoras`
/// from `strategies/cartesian/distance_pythagoras.hpp:71-117`. The
/// `Comparable = Self` projection matches Boost's
/// `comparable_type<comparable::pythagoras<…>>` specialisation at
/// `:242-246`.
#[derive(Debug, Default, Clone, Copy)]
pub struct ComparablePythagoras;

// ---- DistanceStrategy impls ------------------------------------------
//
// The family-equality bound `SameAs<CartesianFamily>` is what enforces
// the Cartesian-only rule. When a caller wires a `Geographic` or
// `Spherical` point through here by mistake — the silent-Cartesian
// trap from proposal §8 — the unsatisfied bound surfaces the
// `#[diagnostic::on_unimplemented]` plate that lives on
// `geometry_tag::SameAs`, which points users at `WithCs<_,
// Geographic<Degree>>` / `WithCs<_, Spherical<Degree>>` and the
// CS-appropriate strategies (Haversine, Andoyer, Vincenty).

impl<P1, P2> DistanceStrategy<P1, P2> for Pythagoras
where
    P1: Point,
    P2: Point<Scalar = P1::Scalar>,
    <P1::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = P1::Scalar;
    type Comparable = ComparablePythagoras;

    #[inline]
    fn distance(&self, a: &P1, b: &P2) -> Self::Out {
        // Pay the `sqrt` on top of the comparable form, mirroring
        // `strategies/cartesian/distance_pythagoras.hpp:160-172`.
        sum_squared_diffs::<P1, P2>(a, b).sqrt()
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        ComparablePythagoras
    }
}

impl<P1, P2> DistanceStrategy<P1, P2> for ComparablePythagoras
where
    P1: Point,
    P2: Point<Scalar = P1::Scalar>,
    <P1::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = P1::Scalar;
    type Comparable = Self;

    #[inline]
    fn distance(&self, a: &P1, b: &P2) -> Self::Out {
        sum_squared_diffs::<P1, P2>(a, b)
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        ComparablePythagoras
    }
}

// ---- Default Cartesian × Cartesian = Pythagoras ----------------------

/// Cartesian × Cartesian defaults to Pythagoras.
///
/// Mirrors the `services::default_strategy<point_tag, point_tag, P1,
/// P2, cartesian_tag, cartesian_tag>` specialisation at
/// `strategies/cartesian/distance_pythagoras.hpp:276-283`.
impl DefaultDistance<CartesianFamily> for CartesianFamily {
    type Strategy = Pythagoras;
}

// ---- Const-recursive helper ------------------------------------------
//
// Mirrors `detail::compute_pythagoras<I, T>` from
// `strategies/cartesian/distance_pythagoras.hpp:44-66`. The C++ template
// recursion counts *down* from `I = dimension<P1>::value` to `0`; we
// count *up* via a sealed trait, the same shape as `geometry_trait`'s
// `Recurse` helper (see `point.rs::fold_dims`). Counting up reads more
// naturally in Rust without changing what gets computed.
//
// `Point::get::<D>` must be invoked with a const-generic `D`, so we
// cannot use the runtime-`usize` `fold_dims` from `geometry-trait` here
// — `fold_dims` was designed for closures that *label* the dimension
// for diagnostics, not for kernels that need to *index* into it. Hence
// the dedicated recursion below, unrolled up to [`MAX_DIM`].

/// Largest `DIM` the squared-difference walk supports on stable Rust.
/// Matches `geometry_trait::MAX_DIM` so any [`Point`] the kernel
/// accepts has a Pythagoras impl as well.
const MAX_DIM: usize = 4;

/// Entry point of the squared-difference recursion.
///
/// Dispatches on `P1::DIM` to the right `(0, N)` start of the recursion,
/// then descends one dimension at a time via [`SumSquares`].
#[inline]
fn sum_squared_diffs<P1, P2>(a: &P1, b: &P2) -> P1::Scalar
where
    P1: Point,
    P2: Point<Scalar = P1::Scalar>,
{
    // `P1::DIM` is a monomorphisation-time constant but cannot appear
    // in a const-generic position on stable Rust. Same shape as
    // `geometry_trait::fold_dims` — match to the right `(0, N)` start.
    match P1::DIM {
        1 => <Walk<0, 1> as SumSquares<0, 1>>::step(P1::Scalar::ZERO, a, b),
        2 => <Walk<0, 2> as SumSquares<0, 2>>::step(P1::Scalar::ZERO, a, b),
        3 => <Walk<0, 3> as SumSquares<0, 3>>::step(P1::Scalar::ZERO, a, b),
        4 => <Walk<0, 4> as SumSquares<0, 4>>::step(P1::Scalar::ZERO, a, b),
        _ => panic!("Pythagoras: P1::DIM exceeds MAX_DIM ({MAX_DIM})"),
    }
}

/// Cursor marker carrying `(current, end)` dimension indices.
/// Private — reachable only via [`sum_squared_diffs`].
struct Walk<const I: usize, const N: usize>;

/// Sealed const-recursive iterator: at step `(I, N)` adds
/// `(a.get::<I>() − b.get::<I>())²` to the accumulator and descends to
/// `(I + 1, N)`. Base case is `I == N` — return the accumulator.
///
/// Mirrors `detail::compute_pythagoras<I, T>` from
/// `strategies/cartesian/distance_pythagoras.hpp:44-66`.
trait SumSquares<const I: usize, const N: usize>: sealed::Sealed<I, N> {
    fn step<P1, P2>(acc: P1::Scalar, a: &P1, b: &P2) -> P1::Scalar
    where
        P1: Point,
        P2: Point<Scalar = P1::Scalar>;
}

mod sealed {
    pub trait Sealed<const I: usize, const N: usize> {}
}

// Base case: `I == N` — nothing left to visit. Counterpart to the
// `compute_pythagoras<0, T>` partial specialisation at
// `distance_pythagoras.hpp:57-65`.
impl<const N: usize> sealed::Sealed<N, N> for Walk<N, N> {}
impl<const N: usize> SumSquares<N, N> for Walk<N, N> {
    #[inline]
    fn step<P1, P2>(acc: P1::Scalar, _a: &P1, _b: &P2) -> P1::Scalar
    where
        P1: Point,
        P2: Point<Scalar = P1::Scalar>,
    {
        acc
    }
}

/// Inductive step macro: one impl per `(I, N)` pair with `I < N`.
///
/// We cannot write `I + 1` in a generic bound on stable Rust, so the
/// recursion is unrolled — same trick as `geometry_trait::fold_dims`.
/// Keep this in sync with [`MAX_DIM`].
macro_rules! impl_sum_squares {
    ($i:expr, $n:expr) => {
        impl sealed::Sealed<$i, $n> for Walk<$i, $n> {}
        impl SumSquares<$i, $n> for Walk<$i, $n> {
            #[inline]
            fn step<P1, P2>(acc: P1::Scalar, a: &P1, b: &P2) -> P1::Scalar
            where
                P1: Point,
                P2: Point<Scalar = P1::Scalar>,
            {
                let d = a.get::<$i>() - b.get::<$i>();
                let acc = acc + d * d;
                <Walk<{ $i + 1 }, $n> as SumSquares<{ $i + 1 }, $n>>::step(acc, a, b)
            }
        }
    };
}

// All `(I, N)` pairs with `0 <= I < N <= MAX_DIM`.
impl_sum_squares!(0, 1);
impl_sum_squares!(0, 2);
impl_sum_squares!(1, 2);
impl_sum_squares!(0, 3);
impl_sum_squares!(1, 3);
impl_sum_squares!(2, 3);
impl_sum_squares!(0, 4);
impl_sum_squares!(1, 4);
impl_sum_squares!(2, 4);
impl_sum_squares!(3, 4);

// ---- Tests -----------------------------------------------------------

#[cfg(test)]
mod tests {
    //! Reference values come from
    //! `geometry/test/strategies/pythagoras.cpp`; each test cites the
    //! line range it mirrors.

    use super::{ComparablePythagoras, Pythagoras};
    use crate::distance::DistanceStrategy;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Point3D};

    /// `pythagoras.cpp:50-66` — arbitrary 2D pair, classic 3-4-5
    /// triangle.
    #[test]
    fn three_four_five_2d() {
        let a = Point2D::<f64, Cartesian>::new(0.0, 0.0);
        let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
        assert!((Pythagoras.distance(&a, &b) - 5.0).abs() < 1e-12);
        assert!((ComparablePythagoras.distance(&a, &b) - 25.0).abs() < 1e-12);
    }

    /// `pythagoras.cpp:76-88` — unit axes in 3D.
    #[test]
    fn unit_axis_3d() {
        let o = Point3D::<f64, Cartesian>::new(0.0, 0.0, 0.0);
        let px = Point3D::<f64, Cartesian>::new(1.0, 0.0, 0.0);
        let py = Point3D::<f64, Cartesian>::new(0.0, 1.0, 0.0);
        let pz = Point3D::<f64, Cartesian>::new(0.0, 0.0, 1.0);
        assert!((Pythagoras.distance(&o, &px) - 1.0).abs() < 1e-12);
        assert!((Pythagoras.distance(&o, &py) - 1.0).abs() < 1e-12);
        assert!((Pythagoras.distance(&o, &pz) - 1.0).abs() < 1e-12);
    }

    /// `pythagoras.cpp:90-115` — arbitrary 3D pair, squared = 116,
    /// distance ≈ `10.770_329_614_27`.
    #[test]
    fn arbitrary_3d() {
        let a = Point3D::<f64, Cartesian>::new(1.0, 2.0, 3.0);
        let b = Point3D::<f64, Cartesian>::new(9.0, 8.0, 7.0);
        let d = Pythagoras.distance(&a, &b);
        assert!((d - 10.770_329_614_269_007).abs() < 1e-9);
        assert!((ComparablePythagoras.distance(&a, &b) - 116.0).abs() < 1e-12);
    }

    /// `pythagoras.cpp:136-187` (`test_services`) — the strategy must
    /// produce the same value when the arguments are swapped.
    #[test]
    fn symmetric_in_arguments() {
        let a = Point3D::<f64, Cartesian>::new(1.0, 2.0, 3.0);
        let b = Point3D::<f64, Cartesian>::new(4.0, 5.0, 6.0);
        let ab = Pythagoras.distance(&a, &b);
        let ba = Pythagoras.distance(&b, &a);
        assert!((ab - ba).abs() < 1e-12);
        // sqrt(3² + 3² + 3²) = sqrt(27).
        assert!((ab - 27.0_f64.sqrt()).abs() < 1e-12);
    }

    /// `pythagoras.cpp:162-186` (`comparable_type`) — the comparable
    /// form preserves order against the real distance form, which is
    /// the whole point of skipping the sqrt.
    #[test]
    fn comparable_orders_match_real_distance() {
        let o = Point2D::<f64, Cartesian>::new(0.0, 0.0);
        // Distance² = 25.
        let p_25 = Point2D::<f64, Cartesian>::new(3.0, 4.0);
        // Distance² = 50.
        let p_50 = Point2D::<f64, Cartesian>::new(5.0, 5.0);
        let c25 = ComparablePythagoras.distance(&o, &p_25);
        let c50 = ComparablePythagoras.distance(&o, &p_50);
        assert!((c25 - 25.0).abs() < 1e-12);
        assert!((c50 - 50.0).abs() < 1e-12);
        assert!(c25 < c50);
    }

    // KC1.T2 witness: proves this strategy accepts a read-only `Point`
    // (one that need not implement `PointMut`). If it compiles, the
    // read-only bound is locked.
    fn _accepts_readonly_point<P, S>(s: &S, a: &P, b: &P) -> S::Out
    where
        P: geometry_trait::Point,
        S: DistanceStrategy<P, P>,
    {
        s.distance(a, b)
    }

    /// `ComparablePythagoras::comparable()` returns itself — the
    /// comparable form is already sqrt-free.
    #[test]
    fn comparable_of_comparable_is_itself() {
        let o = Point2D::<f64, Cartesian>::new(0.0, 0.0);
        let p = Point2D::<f64, Cartesian>::new(3.0, 4.0);
        let cmp = DistanceStrategy::<Point2D<f64, Cartesian>, Point2D<f64, Cartesian>>::comparable(
            &ComparablePythagoras,
        );
        assert!((cmp.distance(&o, &p) - 25.0).abs() < 1e-12);
    }

    /// The read-only-point witness computes the same value when actually
    /// invoked with a concrete strategy and points.
    #[test]
    #[allow(
        clippy::used_underscore_items,
        reason = "the test exists to run the compile-time witness's body"
    )]
    fn readonly_witness_computes_distance() {
        let a = Point2D::<f64, Cartesian>::new(0.0, 0.0);
        let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
        assert!((_accepts_readonly_point(&Pythagoras, &a, &b) - 5.0).abs() < 1e-12);
    }
}

#[cfg(test)]
mod large_coordinate_tests {
    //! Mirrors `boost/geometry/test/strategies/pythagoras.cpp:188-235`
    //! (`test_big_2d_with`, `test_big_2d`, `test_big_2d_string`).
    //! Exercises FP precision on coordinates around 10⁶ m, where naive
    //! double-precision sums-of-squares lose digits to cancellation.
    //!
    //! Boost's reference value:
    //! `1_076_554.548_583_395_567_829_438_778_905_7` metres, tolerance
    //! `0.001 %` (Boost's `BOOST_CHECK_CLOSE` default for this test).

    use super::{ComparablePythagoras, Pythagoras};
    use crate::distance::DistanceStrategy;
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    /// Reference value from Boost's `test_big_2d_with` reference line
    /// (`pythagoras.cpp:213`). The Rust port reproduces it within
    /// 0.001%.
    const REF: f64 = 1_076_554.548_583_395_567_829_438_778_905_7;

    /// Tolerance matching Boost's `BOOST_CHECK_CLOSE(d, ref, 0.001)`
    /// — 0.001 percent, i.e. `1e-5 * |ref|`.
    fn close(actual: f64, expected: f64) -> bool {
        (actual - expected).abs() <= expected.abs() * 1e-5
    }

    /// `test_big_2d_with` lifted with the f64/f64 row from `test_big_2d`.
    #[test]
    fn big_2d_f64_x_f64() {
        let p1 = Point2D::<f64, Cartesian>::new(123_456.789_000_01, 234_567.891_000_01);
        let p2 = Point2D::<f64, Cartesian>::new(987_654.321_000_01, 876_543.219_000_01);
        let d = Pythagoras.distance(&p1, &p2);
        assert!(close(d, REF), "got {d} expected ≈ {REF} (within 0.001%)");
    }

    /// Same inputs, comparable form. The squared sum is `REF * REF`,
    /// so the comparable result has the same relative-tolerance
    /// behaviour.
    #[test]
    fn big_2d_comparable() {
        let p1 = Point2D::<f64, Cartesian>::new(123_456.789_000_01, 234_567.891_000_01);
        let p2 = Point2D::<f64, Cartesian>::new(987_654.321_000_01, 876_543.219_000_01);
        let cmp = ComparablePythagoras.distance(&p1, &p2);
        let expected = REF * REF;
        assert!(
            (cmp - expected).abs() <= expected.abs() * 1e-5,
            "got {cmp} expected ≈ {expected} (within 0.001%)",
        );
    }

    /// Boost's `test_big_2d_string` parses coordinates from string
    /// literals. Rust's `f64::from_str` goes through the same rounding
    /// as a literal, so a single sanity check suffices.
    #[test]
    fn big_2d_from_string_parse() {
        let p1 = Point2D::<f64, Cartesian>::new(
            "123456.78900001".parse::<f64>().unwrap(),
            "234567.89100001".parse::<f64>().unwrap(),
        );
        let p2 = Point2D::<f64, Cartesian>::new(
            "987654.32100001".parse::<f64>().unwrap(),
            "876543.21900001".parse::<f64>().unwrap(),
        );
        let d = Pythagoras.distance(&p1, &p2);
        assert!(close(d, REF));
    }
}
