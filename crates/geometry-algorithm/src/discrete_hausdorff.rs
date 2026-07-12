//! `discrete_hausdorff_distance(&l1, &l2)` — sup-sup distance over
//! vertex sets.
//!
//! Mirrors `boost::geometry::discrete_hausdorff_distance` from
//! `boost/geometry/algorithms/discrete_hausdorff_distance.hpp`. The
//! Boost overload is symmetric — `max(directed(A,B), directed(B,A))`
//! where `directed(A,B) = max_{p ∈ A} min_{q ∈ B} dist(p, q)` — the
//! Rust port matches. `O(m × n)` time, `O(1)` space.

use alloc::vec::Vec;

use geometry_cs::CoordinateSystem;
use geometry_strategy::distance::DefaultDistance;
use geometry_strategy::{DefaultDistanceStrategy, DistanceStrategy};
use geometry_trait::{Geometry, Linestring, Point};

/// Shorthand for the coordinate-system family of a point type.
type Family<P> = <<P as Point>::Cs as CoordinateSystem>::Family;

/// The scalar output type of the default distance strategy between the
/// point types of two linestrings.
type DefaultDistOut<L1, L2> = <DefaultDistanceStrategy<
    <L1 as Geometry>::Point,
    <L2 as Geometry>::Point,
> as DistanceStrategy<<L1 as Geometry>::Point, <L2 as Geometry>::Point>>::Out;

/// Symmetric discrete Hausdorff distance between two linestrings, using
/// the default distance strategy for their coordinate systems.
///
/// Mirrors `boost::geometry::discrete_hausdorff_distance(l1, l2)` from
/// `boost/geometry/algorithms/discrete_hausdorff_distance.hpp`.
///
/// # Panics
///
/// Panics on an empty linestring — Boost treats empty input as an
/// error; the Rust port panics with a clear message.
#[inline]
#[must_use]
pub fn discrete_hausdorff_distance<L1, L2>(l1: &L1, l2: &L2) -> DefaultDistOut<L1, L2>
where
    L1: Linestring,
    L2: Linestring,
    Family<L1::Point>: DefaultDistance<Family<L2::Point>>,
    DefaultDistanceStrategy<L1::Point, L2::Point>: DistanceStrategy<L1::Point, L2::Point> + Default,
{
    let s = <DefaultDistanceStrategy<L1::Point, L2::Point>>::default();
    discrete_hausdorff_distance_with(l1, l2, s)
}

/// Symmetric discrete Hausdorff distance using an explicit distance
/// strategy `dist`.
///
/// Mirrors the strategy-taking `boost::geometry::discrete_hausdorff_distance`
/// overload from
/// `boost/geometry/algorithms/discrete_hausdorff_distance.hpp`. The
/// reverse direction reuses the same strategy with the arguments
/// swapped, so a single `DistanceStrategy<L1::Point, L2::Point>` serves
/// both directed suprema.
///
/// # Panics
///
/// Panics if either linestring is empty.
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "Distance strategies are zero-sized/Copy; taking by value matches `distance_with`."
)]
pub fn discrete_hausdorff_distance_with<L1, L2, S>(l1: &L1, l2: &L2, dist: S) -> S::Out
where
    L1: Linestring,
    L2: Linestring,
    S: DistanceStrategy<L1::Point, L2::Point>,
    S::Out: PartialOrd + Copy,
{
    let seq1: Vec<&L1::Point> = l1.points().collect();
    let seq2: Vec<&L2::Point> = l2.points().collect();
    assert!(
        !seq1.is_empty() && !seq2.is_empty(),
        "empty linestring in discrete_hausdorff"
    );

    // directed(A, B): outer over `seq1` (L1 points), inner over `seq2`.
    let sup_ab = directed_sup(&seq1, &seq2, |p, q| dist.distance(p, q));
    // directed(B, A): outer over `seq2`, inner over `seq1`. The distance
    // call keeps its `(L1::Point, L2::Point)` argument order.
    let sup_ba = directed_sup(&seq2, &seq1, |p, q| dist.distance(q, p));

    if sup_ab > sup_ba { sup_ab } else { sup_ba }
}

/// Directed supremum `max_{p ∈ a} min_{q ∈ b} f(p, q)`.
fn directed_sup<PA, PB, O, F>(a: &[&PA], b: &[&PB], mut f: F) -> O
where
    O: PartialOrd + Copy,
    F: FnMut(&PA, &PB) -> O,
{
    let mut sup: Option<O> = None;
    for p in a {
        let mut inf: Option<O> = None;
        for q in b {
            let d = f(p, q);
            inf = Some(match inf {
                None => d,
                Some(cur) => {
                    if d < cur {
                        d
                    } else {
                        cur
                    }
                }
            });
        }
        let inf = inf.expect("non-empty inner sequence");
        sup = Some(match sup {
            None => inf,
            Some(cur) => {
                if inf > cur {
                    inf
                } else {
                    cur
                }
            }
        });
    }
    sup.expect("non-empty outer sequence")
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Hausdorff reference values compared with an epsilon."
)]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/similarity/discrete_hausdorff_distance.cpp`.

    use super::discrete_hausdorff_distance;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn identical_linestrings_distance_zero() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (2., 0.)];
        assert!(discrete_hausdorff_distance(&ls, &ls) < 1e-12);
    }

    /// Subset case: dropping the last vertex — the Hausdorff distance is
    /// the distance from the dropped vertex `(2,0)` to its nearest kept
    /// neighbour `(1,0)`, i.e. 1.
    #[test]
    fn subset_drops_last_vertex() {
        let a: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (2., 0.)];
        let b: Linestring<Pt> = linestring![(0., 0.), (1., 0.)];
        assert!((discrete_hausdorff_distance(&a, &b) - 1.0).abs() < 1e-9);
    }

    /// Parallel tracks 1 unit apart with aligned endpoints → 1.
    #[test]
    fn parallel_lines() {
        let a: Linestring<Pt> = linestring![(0., 0.), (5., 0.)];
        let b: Linestring<Pt> = linestring![(0., 1.), (5., 1.)];
        assert!((discrete_hausdorff_distance(&a, &b) - 1.0).abs() < 1e-9);
    }

    /// The symmetric property: `H(a, b) == H(b, a)`.
    #[test]
    fn symmetric() {
        let a: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (2., 0.)];
        let b: Linestring<Pt> = linestring![(0., 0.), (1., 0.)];
        assert_eq!(
            discrete_hausdorff_distance(&a, &b).to_bits(),
            discrete_hausdorff_distance(&b, &a).to_bits(),
        );
    }
}
