//! Spatial query predicates.
//!
//! Mirrors `boost/geometry/index/predicates.hpp`. A predicate is tested
//! against candidate values during a query walk; the tree prunes any
//! subtree whose bounds cannot satisfy the predicate.
//!
//! The built-in [`Predicate`] covers Boost's box-query relations. The
//! [`QueryPredicate`] trait also supports Boost's logical `and`, `not`,
//! and `satisfies` predicates without giving up subtree pruning.

use crate::bounds::Bounds;
use crate::indexable::Indexable;

/// A spatial query against the index, expressed on bounding boxes.
///
/// Mirrors the `index::detail::predicates` family
/// (`index/predicates.hpp`). Each variant carries the query box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Predicate {
    /// Values whose bounds intersect the query box. Boost's
    /// `index::intersects`.
    Intersects(Bounds),
    /// Values whose non-degenerate bounds are fully inside the query
    /// box. Boost's `index::within`.
    Within(Bounds),
    /// Values whose bounds contain a non-degenerate query box. Boost's
    /// `index::contains`.
    Contains(Bounds),
    /// Values whose bounds are inside the query box, including
    /// degenerate values and coincident boundaries. Boost's
    /// `index::covered_by`.
    CoveredBy(Bounds),
    /// Values whose bounds contain the query box, including a
    /// degenerate query. Boost's `index::covers`.
    Covers(Bounds),
    /// Values whose bounds share no point with the query box. Boost's
    /// `index::disjoint`.
    Disjoint(Bounds),
    /// Values whose bounds overlap the query box without either box
    /// covering the other. Boost's `index::overlaps`.
    Overlaps(Bounds),
}

impl Predicate {
    /// Whether a leaf value with box `value` satisfies this predicate.
    #[must_use]
    #[inline]
    pub fn matches(&self, value: &Bounds) -> bool {
        if let Predicate::Intersects(query) = self {
            return query.intersects(value);
        }
        match self {
            Predicate::Intersects(_) => unreachable!("intersects returned above"),
            Predicate::Within(q) => value.within(q),
            Predicate::Contains(q) => q.within(value),
            Predicate::CoveredBy(q) => value.covered_by(q),
            Predicate::Covers(q) => q.covered_by(value),
            Predicate::Disjoint(q) => q.disjoint(value),
            Predicate::Overlaps(q) => q.overlaps(value),
        }
    }

    /// Whether a subtree with box `node` could contain any matching
    /// value — the pruning test. A subtree is skipped when this is
    /// `false`.
    #[must_use]
    #[inline]
    pub fn could_match(&self, node: &Bounds) -> bool {
        if let Predicate::Intersects(query) = self {
            return query.intersects(node);
        }
        match self {
            Predicate::Intersects(_) => unreachable!("intersects returned above"),
            Predicate::Within(q) | Predicate::CoveredBy(q) | Predicate::Overlaps(q) => {
                q.intersects(node)
            }
            Predicate::Contains(q) | Predicate::Covers(q) => node.contains(q),
            // If the whole node is covered by q, every value below it
            // intersects q and none can be disjoint.
            Predicate::Disjoint(q) => !node.covered_by(q),
        }
    }

    /// Whether EVERY value in a subtree with box `node` is a guaranteed
    /// match — the containment fast path. A covered subtree is dumped
    /// without per-node or per-value tests.
    ///
    /// Sound because a value's box is contained in its node's box:
    /// `Intersects` and `CoveredBy` can dump a node covered by the
    /// query, while `Disjoint` can dump one wholly apart from it.
    /// `Within` cannot use the covered-node shortcut because a subtree
    /// may contain degenerate values.
    #[must_use]
    #[inline]
    pub fn covers_all(&self, node: &Bounds) -> bool {
        if let Predicate::Intersects(query) = self {
            return query.contains(node);
        }
        match self {
            Predicate::Intersects(_) => unreachable!("intersects returned above"),
            Predicate::CoveredBy(q) => q.contains(node),
            Predicate::Disjoint(q) => q.disjoint(node),
            Predicate::Within(_)
            | Predicate::Contains(_)
            | Predicate::Covers(_)
            | Predicate::Overlaps(_) => false,
        }
    }

    /// Combine this spatial predicate with another condition.
    #[must_use]
    #[inline]
    pub fn and<P>(self, other: P) -> AndPredicate<Self, P> {
        and(self, other)
    }
}

impl core::ops::Not for Predicate {
    type Output = NotPredicate<Self>;

    #[inline]
    fn not(self) -> Self::Output {
        not(self)
    }
}

/// A predicate accepted by [`Rtree::query_with`](crate::Rtree::query_with).
///
/// `matches` decides leaf values. `could_match` and `covers_all` are
/// conservative subtree tests: false from the former prunes a subtree;
/// true from the latter yields it without further predicate calls.
pub trait QueryPredicate<T: Indexable> {
    /// Whether one indexed value satisfies the predicate.
    fn matches(&self, value: &T) -> bool;

    /// Whether a subtree may contain a match.
    fn could_match(&self, node: &Bounds) -> bool;

    /// Whether every value in a subtree is guaranteed to match.
    fn covers_all(&self, node: &Bounds) -> bool;
}

impl<T: Indexable> QueryPredicate<T> for Predicate {
    #[inline]
    fn matches(&self, value: &T) -> bool {
        self.matches(&value.bounds())
    }

    #[inline]
    fn could_match(&self, node: &Bounds) -> bool {
        self.could_match(node)
    }

    #[inline]
    fn covers_all(&self, node: &Bounds) -> bool {
        self.covers_all(node)
    }
}

/// The conjunction of two query predicates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AndPredicate<Left, Right> {
    left: Left,
    right: Right,
}

/// Combine two query predicates with logical conjunction.
#[must_use]
#[inline]
pub fn and<Left, Right>(left: Left, right: Right) -> AndPredicate<Left, Right> {
    AndPredicate { left, right }
}

impl<T, Left, Right> QueryPredicate<T> for AndPredicate<Left, Right>
where
    T: Indexable,
    Left: QueryPredicate<T>,
    Right: QueryPredicate<T>,
{
    #[inline]
    fn matches(&self, value: &T) -> bool {
        self.left.matches(value) && self.right.matches(value)
    }

    #[inline]
    fn could_match(&self, node: &Bounds) -> bool {
        self.left.could_match(node) && self.right.could_match(node)
    }

    #[inline]
    fn covers_all(&self, node: &Bounds) -> bool {
        self.left.covers_all(node) && self.right.covers_all(node)
    }
}

/// The logical negation of a query predicate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotPredicate<Inner> {
    inner: Inner,
}

/// Negate a query predicate.
#[must_use]
#[inline]
pub fn not<Inner>(inner: Inner) -> NotPredicate<Inner> {
    NotPredicate { inner }
}

impl<T, Inner> QueryPredicate<T> for NotPredicate<Inner>
where
    T: Indexable,
    Inner: QueryPredicate<T>,
{
    #[inline]
    fn matches(&self, value: &T) -> bool {
        !self.inner.matches(value)
    }

    #[inline]
    fn could_match(&self, node: &Bounds) -> bool {
        !self.inner.covers_all(node)
    }

    #[inline]
    fn covers_all(&self, node: &Bounds) -> bool {
        !self.inner.could_match(node)
    }
}

/// A value-level query condition corresponding to Boost's
/// `index::satisfies` predicate.
pub struct Satisfies<F> {
    condition: F,
}

/// Build a value-level `satisfies` query predicate.
#[must_use]
#[inline]
pub fn satisfies<F>(condition: F) -> Satisfies<F> {
    Satisfies { condition }
}

impl<T, F> QueryPredicate<T> for Satisfies<F>
where
    T: Indexable,
    F: Fn(&T) -> bool,
{
    #[inline]
    fn matches(&self, value: &T) -> bool {
        (self.condition)(value)
    }

    #[inline]
    fn could_match(&self, _node: &Bounds) -> bool {
        true
    }

    #[inline]
    fn covers_all(&self, _node: &Bounds) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::Predicate;
    use crate::bounds::Bounds;

    #[test]
    fn intersects_matches_overlap() {
        let p = Predicate::Intersects(Bounds::new([0.0, 0.0], [2.0, 2.0]));
        assert!(p.matches(&Bounds::new([1.0, 1.0], [3.0, 3.0])));
        assert!(!p.matches(&Bounds::new([5.0, 5.0], [6.0, 6.0])));
    }

    #[test]
    fn within_matches_contained() {
        let p = Predicate::Within(Bounds::new([0.0, 0.0], [10.0, 10.0]));
        assert!(p.matches(&Bounds::new([2.0, 2.0], [3.0, 3.0])));
        assert!(!p.matches(&Bounds::new([2.0, 2.0], [12.0, 3.0])));
    }

    #[test]
    fn contains_matches_covering() {
        let p = Predicate::Contains(Bounds::new([4.0, 4.0], [5.0, 5.0]));
        assert!(p.matches(&Bounds::new([0.0, 0.0], [10.0, 10.0])));
        assert!(!p.matches(&Bounds::new([0.0, 0.0], [4.5, 4.5])));
    }

    #[test]
    fn pruning_skips_disjoint_subtrees() {
        let p = Predicate::Intersects(Bounds::new([0.0, 0.0], [1.0, 1.0]));
        assert!(p.could_match(&Bounds::new([0.5, 0.5], [9.0, 9.0])));
        assert!(!p.could_match(&Bounds::new([5.0, 5.0], [9.0, 9.0])));
    }

    const QUERY: Bounds = Bounds::new([0.0, 0.0], [10.0, 10.0]);
    const CONTAINED: Bounds = Bounds::new([2.0, 2.0], [3.0, 3.0]);
    const OVERLAPPING: Bounds = Bounds::new([5.0, 5.0], [15.0, 15.0]);
    const DISJOINT: Bounds = Bounds::new([20.0, 20.0], [30.0, 30.0]);

    #[test]
    fn covers_all_intersects() {
        let p = Predicate::Intersects(QUERY);
        assert!(p.covers_all(&CONTAINED));
        assert!(p.covers_all(&QUERY));
        assert!(!p.covers_all(&OVERLAPPING));
        assert!(!p.covers_all(&DISJOINT));
    }

    #[test]
    fn covers_all_within_never_assumes_value_dimension() {
        let p = Predicate::Within(QUERY);
        assert!(!p.covers_all(&CONTAINED));
        assert!(!p.covers_all(&QUERY));
        assert!(!p.covers_all(&OVERLAPPING));
        assert!(!p.covers_all(&DISJOINT));
    }

    #[test]
    fn covers_all_contains_never() {
        let p = Predicate::Contains(QUERY);
        assert!(!p.covers_all(&CONTAINED));
        assert!(!p.covers_all(&QUERY));
        assert!(!p.covers_all(&OVERLAPPING));
        assert!(!p.covers_all(&DISJOINT));
    }
}
