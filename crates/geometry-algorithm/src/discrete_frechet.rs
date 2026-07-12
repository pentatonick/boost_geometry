//! `discrete_frechet_distance(&l1, &l2)` — DP table over the two
//! point sequences.
//!
//! Mirrors `boost::geometry::discrete_frechet_distance` from
//! `boost/geometry/algorithms/discrete_frechet_distance.hpp`. The
//! discrete variant operates on vertex pairs only (no continuous
//! re-parameterisation); the coupling-measure DP table is
//!
//! ```text
//! C(i, j) = max( min( C(i-1, j), C(i-1, j-1), C(i, j-1) ),
//!                dist(L1[i], L2[j]) )
//! result  = C(m-1, n-1)
//! ```
//!
//! with `C(0, 0) = dist(L1[0], L2[0])`.

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

/// Discrete Fréchet distance between two linestrings, using the
/// default distance strategy for their coordinate systems (Pythagoras
/// for Cartesian, Andoyer for Geographic, …).
///
/// Mirrors `boost::geometry::discrete_frechet_distance(l1, l2)` from
/// `boost/geometry/algorithms/discrete_frechet_distance.hpp`.
///
/// # Panics
///
/// Panics on an empty linestring — Boost throws; the Rust port panics
/// with a clear message.
#[inline]
#[must_use]
pub fn discrete_frechet_distance<L1, L2>(l1: &L1, l2: &L2) -> DefaultDistOut<L1, L2>
where
    L1: Linestring,
    L2: Linestring,
    Family<L1::Point>: DefaultDistance<Family<L2::Point>>,
    DefaultDistanceStrategy<L1::Point, L2::Point>: DistanceStrategy<L1::Point, L2::Point> + Default,
{
    let s = <DefaultDistanceStrategy<L1::Point, L2::Point>>::default();
    discrete_frechet_distance_with(l1, l2, s)
}

/// Discrete Fréchet distance between two linestrings using an explicit
/// distance strategy `dist`.
///
/// Mirrors the strategy-taking `boost::geometry::discrete_frechet_distance`
/// overload from `boost/geometry/algorithms/discrete_frechet_distance.hpp`.
///
/// # Panics
///
/// Panics if either linestring is empty.
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "Distance strategies are zero-sized/Copy; taking by value matches `distance_with`."
)]
pub fn discrete_frechet_distance_with<L1, L2, S>(l1: &L1, l2: &L2, dist: S) -> S::Out
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
        "empty linestring in discrete_frechet"
    );

    let len1 = seq1.len();
    let len2 = seq2.len();
    let mut table: Vec<Vec<S::Out>> = (0..len1).map(|_| Vec::with_capacity(len2)).collect();

    // Base case + first row + first column.
    table[0].push(dist.distance(seq1[0], seq2[0]));
    for j in 1..len2 {
        let here = dist.distance(seq1[0], seq2[j]);
        let prev = table[0][j - 1];
        table[0].push(if here > prev { here } else { prev });
    }
    for i in 1..len1 {
        let here = dist.distance(seq1[i], seq2[0]);
        let prev = table[i - 1][0];
        table[i].push(if here > prev { here } else { prev });
    }

    for i in 1..len1 {
        for j in 1..len2 {
            let here = dist.distance(seq1[i], seq2[j]);
            let mut best = table[i - 1][j];
            if table[i][j - 1] < best {
                best = table[i][j - 1];
            }
            if table[i - 1][j - 1] < best {
                best = table[i - 1][j - 1];
            }
            table[i].push(if here > best { here } else { best });
        }
    }

    table[len1 - 1][len2 - 1]
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Fréchet reference values compared with an epsilon."
)]
mod tests {
    //! Reference values from
    //! `boost/geometry/test/algorithms/similarity/discrete_frechet_distance.cpp:28-35`.

    use super::discrete_frechet_distance;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};

    type Pt = Point2D<f64, Cartesian>;

    /// `discrete_frechet_distance.cpp:28` — coupling distance 3.
    #[test]
    fn diagonal_distance_3() {
        let a: Linestring<Pt> = linestring![(3., 0.), (2., 1.), (3., 2.)];
        let b: Linestring<Pt> = linestring![(0., 0.), (3., 4.), (4., 3.)];
        assert!((discrete_frechet_distance(&a, &b) - 3.0).abs() < 1e-9);
    }

    /// `discrete_frechet_distance.cpp:30` — identical linestrings → 0.
    #[test]
    fn identical_linestrings_distance_zero() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (1., 1.), (0., 1.), (0., 0.)];
        assert!(discrete_frechet_distance(&ls, &ls) < 1e-12);
    }

    /// `discrete_frechet_distance.cpp:31` — reversed unit square = √2.
    #[test]
    fn reversed_unit_square() {
        let a: Linestring<Pt> = linestring![(0., 0.), (1., 0.), (1., 1.), (0., 1.), (0., 0.)];
        let b: Linestring<Pt> = linestring![(1., 1.), (0., 1.), (0., 0.), (1., 0.), (1., 1.)];
        assert!((discrete_frechet_distance(&a, &b) - 2.0_f64.sqrt()).abs() < 1e-9);
    }

    /// `discrete_frechet_distance.cpp:35` — opposite traversal, 3-4-5 → 5.
    #[test]
    fn opposite_traversal_3_4_5() {
        let a: Linestring<Pt> = linestring![(0., 0.), (3., 4.), (4., 3.)];
        let b: Linestring<Pt> = linestring![(4., 3.), (3., 4.), (0., 0.)];
        assert!((discrete_frechet_distance(&a, &b) - 5.0).abs() < 1e-9);
    }
}
