//! Spatial query predicates.
//!
//! Mirrors `boost/geometry/index/predicates.hpp`. A predicate is tested
//! against candidate values during a query walk; the tree prunes any
//! subtree whose bounds cannot satisfy the predicate.
//!
//! v1 ships the box-level predicates the pruning walk needs directly —
//! [`Predicate::Intersects`], [`Predicate::Within`],
//! [`Predicate::Contains`]. The interior/boundary DE-9IM predicates
//! (`covered_by`, `overlaps`) layer on top once the value type carries a
//! full geometry rather than only its bounds.

use crate::bounds::Bounds;

/// A spatial query against the index, expressed on bounding boxes.
///
/// Mirrors the `index::detail::predicates` family
/// (`index/predicates.hpp`). Each variant carries the query box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Predicate {
    /// Values whose bounds intersect the query box. Boost's
    /// `index::intersects`.
    Intersects(Bounds),
    /// Values whose bounds are fully inside the query box. Boost's
    /// `index::within`.
    Within(Bounds),
    /// Values whose bounds fully contain the query box. Boost's
    /// `index::contains`.
    Contains(Bounds),
}

impl Predicate {
    /// Whether a leaf value with box `value` satisfies this predicate.
    #[must_use]
    pub fn matches(&self, value: &Bounds) -> bool {
        match self {
            Predicate::Intersects(q) => q.intersects(value),
            Predicate::Within(q) => q.contains(value),
            Predicate::Contains(q) => value.contains(q),
        }
    }

    /// Whether a subtree with box `node` could contain any matching
    /// value — the pruning test. A subtree is skipped when this is
    /// `false`.
    #[must_use]
    pub fn could_match(&self, node: &Bounds) -> bool {
        match self {
            // Intersect / within candidates must overlap the query box.
            Predicate::Intersects(q) | Predicate::Within(q) => q.intersects(node),
            // A value containing the query box must itself have a box
            // overlapping the query, and the subtree must cover it.
            Predicate::Contains(q) => node.intersects(q),
        }
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
}
