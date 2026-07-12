//! Per-CS strategy for the `disjoint` set-relation algorithm.
//!
//! Mirrors `boost::geometry::disjoint(g1, g2)` from
//! `boost/geometry/algorithms/disjoint.hpp`. Boost's interface header
//! defines `disjoint` as the primary kernel and resolves `intersects`
//! through its negation
//! (`algorithms/detail/intersects/interface.hpp:64-78`). The Rust
//! port flips that: [`crate::intersects::CartesianIntersects`] is the
//! primary kernel because every per-pair test is naturally an
//! "is there a shared point?" question, and `disjoint` is the
//! negation. The two trait surfaces stay parallel so callers can pick
//! either entry point without surprises.

use crate::intersects::{CartesianIntersects, IntersectsStrategy};

/// A strategy for "do these two geometries share **no** point?".
///
/// Mirrors `boost::geometry::disjoint(g1, g2)` from
/// `boost/geometry/algorithms/disjoint.hpp`. Symmetric like
/// [`crate::intersects::IntersectsStrategy`].
pub trait DisjointStrategy<A, B> {
    /// `true` iff `a` and `b` share **no** point.
    fn disjoint(&self, a: &A, b: &B) -> bool;
}

/// The Cartesian disjoint kernel — Boost's default for the Cartesian
/// coordinate system. Implemented as `!CartesianIntersects` for every
/// pair where the intersects kernel is defined; specialised faster
/// per-pair tests (e.g. box-box bounding-box compare) can be added
/// later without changing the public surface.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianDisjoint;

impl<A, B> DisjointStrategy<A, B> for CartesianDisjoint
where
    CartesianIntersects: IntersectsStrategy<A, B>,
{
    #[inline]
    fn disjoint(&self, a: &A, b: &B) -> bool {
        !CartesianIntersects.intersects(a, b)
    }
}

#[cfg(test)]
mod tests {
    use super::{CartesianDisjoint, DisjointStrategy};
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};

    type P = Point2D<f64, Cartesian>;
    type LS = Linestring<P>;

    #[test]
    fn linestrings_disjoint_when_apart() {
        let a: LS = linestring![(0.0, 0.0), (2.0, 0.0)];
        let b: LS = linestring![(10.0, 10.0), (11.0, 11.0)];
        assert!(CartesianDisjoint.disjoint(&a, &b));
    }

    #[test]
    fn linestrings_not_disjoint_when_crossing() {
        let a: LS = linestring![(0.0, 0.0), (2.0, 0.0)];
        let b: LS = linestring![(1.0, -1.0), (1.0, 1.0)];
        assert!(!CartesianDisjoint.disjoint(&a, &b));
    }

    // KC1.T2 witness: proves this strategy accepts read-only `Point`
    // operands (that need not implement `PointMut`). If it compiles,
    // the read-only bound is locked.
    fn _accepts_readonly_point<A, B, S>(s: &S, a: &A, b: &B) -> bool
    where
        A: geometry_trait::Point,
        B: geometry_trait::Point,
        S: DisjointStrategy<A, B>,
    {
        s.disjoint(a, b)
    }
}
