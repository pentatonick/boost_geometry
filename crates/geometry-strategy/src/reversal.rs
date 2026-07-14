//! Argument-order reversal for symmetric strategy concepts.
//!
//! Mirrors `boost::geometry::reverse_dispatch` from
//! `core/reverse_dispatch.hpp` and the per-algorithm partial
//! specialisations that consume it. Rust expresses the shared ordering
//! decision as one wrapper with one impl per strategy concept.

use geometry_trait::Geometry;

use crate::distance::DistanceStrategy;
use crate::intersects::IntersectsStrategy;

/// Lift a strategy for `(A, B)` into the same strategy concept for `(B, A)`.
///
/// Mirrors `boost::geometry::reverse_dispatch<Geometry1, Geometry2>` from
/// `core/reverse_dispatch.hpp:33-64`. Boost consults the ordering trait from
/// each algorithm's dispatch specialisation; Rust implements each strategy
/// concept on this single wrapper instead.
#[derive(Debug, Default, Clone, Copy)]
pub struct Reversed<S>(pub S);

impl<A, B, S> DistanceStrategy<B, A> for Reversed<S>
where
    A: Geometry,
    B: Geometry,
    S: DistanceStrategy<A, B>,
{
    type Out = S::Out;
    type Comparable = Reversed<S::Comparable>;

    #[inline]
    fn distance(&self, b: &B, a: &A) -> S::Out {
        self.0.distance(a, b)
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        Reversed(self.0.comparable())
    }
}

impl<A, B, S> IntersectsStrategy<B, A> for Reversed<S>
where
    S: IntersectsStrategy<A, B>,
{
    #[inline]
    fn intersects(&self, b: &B, a: &A) -> bool {
        self.0.intersects(a, b)
    }
}
