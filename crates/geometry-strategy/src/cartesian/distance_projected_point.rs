//! Point-to-segment Cartesian distance via projection onto the segment.
//!
//! Mirrors `boost::geometry::strategy::distance::projected_point` from
//! `boost/geometry/strategies/cartesian/distance_projected_point.hpp`.
//! The C++ class delegates the final point-point measurement to an
//! inner strategy (defaulting to `pythagoras<>`) — the Rust port
//! mirrors that with the [`PointToSegment`] generic parameter `PP`.
//!
//! # Algorithm
//!
//! For point `P` and segment `A`-`B`:
//!
//! 1. Compute the projection parameter
//!    `t = ((P − A) · (B − A)) / ((B − A) · (B − A))`.
//! 2. Clamp `t` to `[0, 1]`. When `t ∈ [0, 1]` the foot of the
//!    perpendicular sits on the segment; otherwise the nearer endpoint
//!    is the closest point.
//! 3. Return the inner strategy's distance from `P` to the foot.
//!
//! When `A == B` (degenerate segment), `dot(B − A, B − A) == 0` and
//! the projection step is skipped — the foot is taken as `A`. This
//! matches the C++ implementation's behaviour through
//! `closest_points::detail::compute_closest_point_to_segment`
//! (`strategies/cartesian/closest_points_pt_seg.hpp`).

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut, Segment};

use crate::cartesian::Pythagoras;
use crate::distance::DistanceStrategy;

/// Distance from a point to a segment in Cartesian space.
///
/// Generic over the *point-point* sub-strategy `PP` so that comparable
/// and non-comparable forms compose: `PointToSegment<Pythagoras>`
/// returns the real distance; `PointToSegment<ComparablePythagoras>`
/// returns the squared distance.
///
/// Mirrors `boost::geometry::strategy::distance::projected_point` from
/// `boost/geometry/strategies/cartesian/distance_projected_point.hpp`.
#[derive(Debug, Default, Clone, Copy)]
pub struct PointToSegment<PP = Pythagoras>(pub PP);

impl<P, S, PP> DistanceStrategy<P, S> for PointToSegment<PP>
where
    P: PointMut + Default,
    S: Segment<Point = P>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    PP: DistanceStrategy<P, P, Out = P::Scalar>,
    PP::Comparable: DistanceStrategy<P, P, Out = P::Scalar>,
{
    type Out = P::Scalar;
    type Comparable = PointToSegment<PP::Comparable>;

    #[inline]
    fn distance(&self, p: &P, s: &S) -> Self::Out {
        let foot = closest_point_on_segment::<P, S>(p, s);
        self.0.distance(p, &foot)
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        PointToSegment(self.0.comparable())
    }
}

// ---- Closest-point kernel --------------------------------------------
//
// Mirrors `closest_points::detail::compute_closest_point_to_segment`
// from `strategies/cartesian/closest_points_pt_seg.hpp`. Split out from
// the strategy impl so the const-recursive walks below can be reused
// for each operation (two dot products and one foot-assembly).

/// Compute the closest point on segment `seg` to point `p`.
///
/// Builds the segment endpoints `start`, `end` from the indexed
/// accessors, computes the clamped projection parameter, then
/// assembles the foot of the perpendicular as a fresh [`Point`].
fn closest_point_on_segment<P, S>(p: &P, seg: &S) -> P
where
    P: PointMut + Default,
    S: Segment<Point = P>,
{
    // Endpoints, reconstructed via the Point trait's `set::<D>` writer.
    // We avoid `segment_start` / `segment_end` to keep this module
    // free of the `S::Point: Default` re-statement those helpers would
    // require — `P: Default` already covers it for the foot below.
    let start = endpoint::<P, S, 0>(seg);
    let end = endpoint::<P, S, 1>(seg);

    let (numerator, denominator) = dots::<P>(p, &start, &end);

    // Degenerate segment: `start == end`. The foot is the start
    // endpoint, and the distance becomes `|p − start|`. Mirrors the
    // early-out path in `closest_points_pt_seg.hpp` where division by
    // zero is avoided by checking `denominator <= 0`.
    if denominator <= P::Scalar::ZERO {
        return start;
    }

    let t = numerator / denominator;

    if t <= P::Scalar::ZERO {
        start
    } else if t >= P::Scalar::ONE {
        end
    } else {
        assemble_foot::<P>(&start, &end, t)
    }
}

/// Materialise endpoint `I` of segment `s` into a [`Point`] via the
/// indexed accessors. Same shape as
/// [`geometry_trait::segment_start`] / [`geometry_trait::segment_end`],
/// inlined here to keep the const-generic walk co-located with the
/// other recursions in this file.
#[inline]
fn endpoint<P, S, const I: usize>(s: &S) -> P
where
    P: PointMut + Default,
    S: Segment<Point = P>,
{
    let mut out: P = Default::default();
    match P::DIM {
        1 => <Walk<0, 1> as WriteEndpoint<0, 1>>::run::<P, S, I>(s, &mut out),
        2 => <Walk<0, 2> as WriteEndpoint<0, 2>>::run::<P, S, I>(s, &mut out),
        3 => <Walk<0, 3> as WriteEndpoint<0, 3>>::run::<P, S, I>(s, &mut out),
        4 => <Walk<0, 4> as WriteEndpoint<0, 4>>::run::<P, S, I>(s, &mut out),
        _ => panic!("PointToSegment: P::DIM exceeds MAX_DIM (4)"),
    }
    out
}

/// Compute `(dot(p − a, b − a), dot(b − a, b − a))` in one pass.
#[inline]
fn dots<P: Point>(p: &P, a: &P, b: &P) -> (P::Scalar, P::Scalar) {
    let init = (P::Scalar::ZERO, P::Scalar::ZERO);
    match P::DIM {
        1 => <Walk<0, 1> as Dots<0, 1>>::run::<P>(init, p, a, b),
        2 => <Walk<0, 2> as Dots<0, 2>>::run::<P>(init, p, a, b),
        3 => <Walk<0, 3> as Dots<0, 3>>::run::<P>(init, p, a, b),
        4 => <Walk<0, 4> as Dots<0, 4>>::run::<P>(init, p, a, b),
        _ => panic!("PointToSegment: P::DIM exceeds MAX_DIM (4)"),
    }
}

/// Assemble the foot `foot[D] = a[D] + t · (b[D] − a[D])` for each
/// dimension `D ∈ 0..P::DIM`.
#[inline]
fn assemble_foot<P: PointMut + Default>(a: &P, b: &P, t: P::Scalar) -> P {
    let mut out: P = Default::default();
    match P::DIM {
        1 => <Walk<0, 1> as AssembleFoot<0, 1>>::run::<P>(a, b, t, &mut out),
        2 => <Walk<0, 2> as AssembleFoot<0, 2>>::run::<P>(a, b, t, &mut out),
        3 => <Walk<0, 3> as AssembleFoot<0, 3>>::run::<P>(a, b, t, &mut out),
        4 => <Walk<0, 4> as AssembleFoot<0, 4>>::run::<P>(a, b, t, &mut out),
        _ => panic!("PointToSegment: P::DIM exceeds MAX_DIM (4)"),
    }
    out
}

// ---- Const-recursive walkers -----------------------------------------
//
// Same sealed-trait shape as `distance_pythagoras::SumSquares` and
// `geometry_trait::segment::WriteDims`. Three operations live on three
// sibling traits so the recursion bodies stay small.

/// Cursor carrying a `(current, end)` pair of dimensions.
struct Walk<const I: usize, const N: usize>;

mod sealed {
    pub trait Sealed<const I: usize, const N: usize> {}
}

/// Walk that writes endpoint `I_SEG` of a segment into `out` one
/// dimension at a time, sourcing each coordinate from
/// `s.get_indexed::<I_SEG, D>()`.
trait WriteEndpoint<const I: usize, const N: usize>: sealed::Sealed<I, N> {
    fn run<P, S, const I_SEG: usize>(s: &S, out: &mut P)
    where
        P: PointMut,
        S: Segment<Point = P>;
}

/// Walk that accumulates `(ap · ab, ab · ab)` one dimension at a time.
trait Dots<const I: usize, const N: usize>: sealed::Sealed<I, N> {
    fn run<P: Point>(acc: (P::Scalar, P::Scalar), p: &P, a: &P, b: &P) -> (P::Scalar, P::Scalar);
}

/// Walk that writes `out[D] = a[D] + t · (b[D] − a[D])` for each `D`.
trait AssembleFoot<const I: usize, const N: usize>: sealed::Sealed<I, N> {
    fn run<P: PointMut>(a: &P, b: &P, t: P::Scalar, out: &mut P);
}

// Base cases: `I == N` — nothing left to do.
impl<const N: usize> sealed::Sealed<N, N> for Walk<N, N> {}

impl<const N: usize> WriteEndpoint<N, N> for Walk<N, N> {
    #[inline]
    fn run<P, S, const I_SEG: usize>(_s: &S, _out: &mut P)
    where
        P: PointMut,
        S: Segment<Point = P>,
    {
    }
}

impl<const N: usize> Dots<N, N> for Walk<N, N> {
    #[inline]
    fn run<P: Point>(
        acc: (P::Scalar, P::Scalar),
        _p: &P,
        _a: &P,
        _b: &P,
    ) -> (P::Scalar, P::Scalar) {
        acc
    }
}

impl<const N: usize> AssembleFoot<N, N> for Walk<N, N> {
    #[inline]
    fn run<P: PointMut>(_a: &P, _b: &P, _t: P::Scalar, _out: &mut P) {}
}

/// Inductive step for one `(I, N)` pair. We cannot write `I + 1` in a
/// generic bound on stable Rust, so the macro unrolls one impl per
/// pair. Keep in sync with `MAX_DIM = 4` matched on above.
macro_rules! impl_walk {
    ($i:expr, $n:expr) => {
        impl sealed::Sealed<$i, $n> for Walk<$i, $n> {}

        impl WriteEndpoint<$i, $n> for Walk<$i, $n> {
            #[inline]
            fn run<P, S, const I_SEG: usize>(s: &S, out: &mut P)
            where
                P: PointMut,
                S: Segment<Point = P>,
            {
                let v = s.get_indexed::<I_SEG, $i>();
                <P as PointMut>::set::<$i>(out, v);
                <Walk<{ $i + 1 }, $n> as WriteEndpoint<{ $i + 1 }, $n>>::run::<P, S, I_SEG>(s, out);
            }
        }

        impl Dots<$i, $n> for Walk<$i, $n> {
            #[inline]
            fn run<P: Point>(
                acc: (P::Scalar, P::Scalar),
                p: &P,
                a: &P,
                b: &P,
            ) -> (P::Scalar, P::Scalar) {
                let ap = p.get::<$i>() - a.get::<$i>();
                let ab = b.get::<$i>() - a.get::<$i>();
                let acc = (acc.0 + ap * ab, acc.1 + ab * ab);
                <Walk<{ $i + 1 }, $n> as Dots<{ $i + 1 }, $n>>::run::<P>(acc, p, a, b)
            }
        }

        impl AssembleFoot<$i, $n> for Walk<$i, $n> {
            #[inline]
            fn run<P: PointMut>(a: &P, b: &P, t: P::Scalar, out: &mut P) {
                let ad = a.get::<$i>();
                let bd = b.get::<$i>();
                <P as PointMut>::set::<$i>(out, ad + t * (bd - ad));
                <Walk<{ $i + 1 }, $n> as AssembleFoot<{ $i + 1 }, $n>>::run::<P>(a, b, t, out);
            }
        }
    };
}

// All `(I, N)` pairs with `0 <= I < N <= 4`.
impl_walk!(0, 1);
impl_walk!(0, 2);
impl_walk!(1, 2);
impl_walk!(0, 3);
impl_walk!(1, 3);
impl_walk!(2, 3);
impl_walk!(0, 4);
impl_walk!(1, 4);
impl_walk!(2, 4);
impl_walk!(3, 4);

// ---- Tests -----------------------------------------------------------

#[cfg(test)]
mod tests {
    //! Reference values mirror
    //! `geometry/test/strategies/projected_point.cpp:28-31` — the four
    //! `test_all_2d` cases. The C++ test reads WKT via the
    //! `test_2d<P1, P2>(wkt_p, wkt_a, wkt_b, expected)` helper; here
    //! the WKT is translated to literal coordinates.

    use super::PointToSegment;
    use crate::cartesian::{ComparablePythagoras, Pythagoras};
    use crate::distance::DistanceStrategy;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Segment};

    fn pt(x: f64, y: f64) -> Point2D<f64, Cartesian> {
        Point2D::<f64, Cartesian>::new(x, y)
    }

    fn seg(ax: f64, ay: f64, bx: f64, by: f64) -> Segment<Point2D<f64, Cartesian>> {
        Segment::new(pt(ax, ay), pt(bx, by))
    }

    /// `projected_point.cpp:28` — `POINT(1 1)` to segment
    /// `(0 0)`-`(2 3)`. The C++ test asserts the literal
    /// `0.27735203958327` against a `BOOST_CHECK_CLOSE(..., 0.001)`
    /// (0.001 % relative tolerance, see
    /// `test/strategies/test_projected_point.hpp:85`) — that literal
    /// is a rounded value, not the exact answer. Closed form: the
    /// projection of `(1, 1)` onto the line `(0,0)-(2,3)` has
    /// parameter `t = 5/13`, foot `(10/13, 15/13)`, and distance
    /// `sqrt((3/13)² + (−2/13)²) = sqrt(13/169) = 1/sqrt(13)`. We
    /// assert that closed form to `1e-12`.
    #[test]
    fn proj_case_1() {
        let d = PointToSegment::<Pythagoras>::default()
            .distance(&pt(1.0, 1.0), &seg(0.0, 0.0, 2.0, 3.0));
        let expected = 1.0_f64 / 13.0_f64.sqrt();
        assert!((d - expected).abs() < 1e-12);
        // Loose check against the literal in the C++ test — passes
        // with the same 0.001 % slack the C++ side uses.
        assert!((d - 0.277_352_039_583_27).abs() < 1e-5);
    }

    /// `projected_point.cpp:29` — `POINT(2 2)` to segment
    /// `(1 4)`-`(4 1)` is `0.5 · sqrt(2)`.
    #[test]
    fn proj_case_2() {
        let d = PointToSegment::<Pythagoras>::default()
            .distance(&pt(2.0, 2.0), &seg(1.0, 4.0, 4.0, 1.0));
        assert!((d - 0.5 * 2.0_f64.sqrt()).abs() < 1e-12);
    }

    /// `projected_point.cpp:30` — `POINT(6 1)` projects beyond the
    /// `(4 1)` endpoint, so the answer is `|P − (4 1)| = 2`.
    #[test]
    fn proj_clamps_to_end() {
        let d = PointToSegment::<Pythagoras>::default()
            .distance(&pt(6.0, 1.0), &seg(1.0, 4.0, 4.0, 1.0));
        assert!((d - 2.0).abs() < 1e-12);
    }

    /// `projected_point.cpp:31` — `POINT(1 6)` projects before the
    /// `(1 4)` endpoint, so the answer is `|P − (1 4)| = 2`.
    #[test]
    fn proj_clamps_to_start() {
        let d = PointToSegment::<Pythagoras>::default()
            .distance(&pt(1.0, 6.0), &seg(1.0, 4.0, 4.0, 1.0));
        assert!((d - 2.0).abs() < 1e-12);
    }

    /// Smoke test — perpendicular drop to a horizontal segment with
    /// the foot strictly between the endpoints. The answer is the
    /// vertical offset (`y = 7`).
    #[test]
    fn perpendicular_distance() {
        let d = PointToSegment::<Pythagoras>::default()
            .distance(&pt(5.0, 7.0), &seg(0.0, 0.0, 10.0, 0.0));
        assert!((d - 7.0).abs() < 1e-12);
    }

    /// Comparable form keeps the squared distance — `7² = 49`.
    /// Mirrors the `comparable::pythagoras` swap on the inner strategy
    /// described in
    /// `distance_projected_point.hpp:64-65` ("If the Strategy is a
    /// `comparable::pythagoras`, this strategy automatically is a
    /// comparable `projected_point` strategy (so without sqrt)").
    #[test]
    fn comparable_form_returns_squared_distance() {
        let cmp = PointToSegment::<ComparablePythagoras>::default()
            .distance(&pt(5.0, 7.0), &seg(0.0, 0.0, 10.0, 0.0));
        assert!((cmp - 49.0).abs() < 1e-12);
    }

    /// `comparable()` on the sqrt-paying form returns a strategy that
    /// produces the squared distance — same value as constructing
    /// `PointToSegment<ComparablePythagoras>` directly. Disambiguate
    /// the trait method via the fully-qualified
    /// `<_ as DistanceStrategy<Point, Segment>>::comparable` syntax
    /// so the `<P, S>` projection on the trait impl resolves.
    #[test]
    fn comparable_method_swaps_inner_strategy() {
        type P = Point2D<f64, Cartesian>;
        type S = Segment<P>;
        let real = PointToSegment::<Pythagoras>::default();
        let cmp = <PointToSegment<Pythagoras> as DistanceStrategy<P, S>>::comparable(&real);
        let d = cmp.distance(&pt(5.0, 7.0), &seg(0.0, 0.0, 10.0, 0.0));
        assert!((d - 49.0).abs() < 1e-12);
    }

    /// Degenerate segment `a == b`: the closest point is `a`, and the
    /// distance reduces to `|p − a|`. Guards the division-by-zero
    /// branch in [`super::closest_point_on_segment`].
    #[test]
    fn degenerate_segment_falls_back_to_endpoint_distance() {
        let d = PointToSegment::<Pythagoras>::default()
            .distance(&pt(3.0, 4.0), &seg(0.0, 0.0, 0.0, 0.0));
        assert!((d - 5.0).abs() < 1e-12);
    }

}
