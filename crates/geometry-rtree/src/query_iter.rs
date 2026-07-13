//! Lazy spatial-query iteration — the walk behind
//! [`Rtree::query_iter`](crate::rtree::Rtree::query_iter).
//!
//! The pruning walk of `visitors/spatial_query.hpp`, unrolled from
//! recursion onto an explicit node stack so it can pause between
//! yields: a consumer that stops early performs no traversal past the
//! value it stopped at. [`Rtree::query`](crate::rtree::Rtree::query)
//! collects this iterator, so the crate has exactly one query walk.

use alloc::vec::Vec;
use core::iter::FusedIterator;

use crate::indexable::Indexable;
use crate::node::Node;
use crate::predicate::Predicate;

/// How the cursor drains the current leaf: `Filtered` tests each value
/// with [`Predicate::matches`]; `DumpAll` yields every value of a leaf
/// whose subtree box the predicate [`covers_all`](Predicate::covers_all)
/// — each one is a guaranteed match.
enum LeafMode {
    Filtered,
    DumpAll,
}

/// A lazy iterator over the values whose bounds satisfy a
/// [`Predicate`], in depth-first tree order.
///
/// Created by [`Rtree::query_iter`](crate::rtree::Rtree::query_iter).
/// Holds the unvisited subtrees on an explicit stack plus a cursor into
/// the current leaf; each [`next`](Iterator::next) drains the cursor,
/// then pops the stack — a popped leaf installs a new cursor, a popped
/// branch pushes the children that pass
/// [`Predicate::could_match`] (reversed, so pops visit them
/// first-to-last, the recursive walk's order). Each stack entry carries
/// a `covered` flag — set once `Predicate::covers_all` holds for a
/// subtree's box — under which children are pushed and leaves dumped
/// without further predicate tests, still one value per `next` call.
/// One `Vec` allocation per iterator, none per element.
pub struct QueryIter<'a, T> {
    predicate: Predicate,
    stack: Vec<(&'a Node<T>, bool)>,
    leaf: core::slice::Iter<'a, T>,
    leaf_mode: LeafMode,
}

impl<'a, T> QueryIter<'a, T> {
    /// `max_fanout` is the split strategy's branch maximum
    /// (`Params::BRANCH_MAX`): the
    /// stack's worst case is every level's unvisited siblings pending
    /// at once — the root pop pushes up to `max_fanout` entries, each
    /// deeper branch pop nets up to `max_fanout − 1` more, peaking at
    /// `(height − 1)·(max_fanout − 1) + 1` — so sizing to `height`
    /// alone under-allocates and reallocates on descent (a
    /// tree-size-dependent number of times, since `height` itself grows
    /// with tree size); the capacity below over-provisions that peak by
    /// one level of slack and keeps the one allocation constant across
    /// tree sizes at a fixed fanout (lazy R4).
    pub(crate) fn new(
        root: &'a Node<T>,
        predicate: Predicate,
        height: usize,
        max_fanout: usize,
    ) -> Self {
        let capacity = height.saturating_sub(1) * max_fanout.saturating_sub(1) + max_fanout;
        let mut stack = Vec::with_capacity(capacity);
        stack.push((root, false));
        Self {
            predicate,
            stack,
            leaf: [].iter(),
            leaf_mode: LeafMode::Filtered,
        }
    }
}

impl<'a, T: Indexable> Iterator for QueryIter<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<&'a T> {
        loop {
            match self.leaf_mode {
                LeafMode::Filtered => {
                    for value in self.leaf.by_ref() {
                        if self.predicate.matches(&value.bounds()) {
                            return Some(value);
                        }
                    }
                }
                LeafMode::DumpAll => {
                    if let Some(value) = self.leaf.next() {
                        return Some(value);
                    }
                }
            }
            let (node, covered) = self.stack.pop()?;
            match node {
                Node::Leaf(values) => {
                    self.leaf = values.iter();
                    self.leaf_mode = if covered {
                        LeafMode::DumpAll
                    } else {
                        LeafMode::Filtered
                    };
                }
                Node::Branch(children) => {
                    if covered {
                        for (_, child) in children.iter().rev() {
                            self.stack.push((child, true));
                        }
                    } else {
                        for (bounds, child) in children.iter().rev() {
                            if self.predicate.could_match(bounds) {
                                self.stack.push((child, self.predicate.covers_all(bounds)));
                            }
                        }
                    }
                }
            }
        }
    }

    /// `(0, None)`: a predicate cannot bound its match count without
    /// doing the walk.
    fn size_hint(&self) -> (usize, Option<usize>) {
        (0, None)
    }
}

impl<T: Indexable> FusedIterator for QueryIter<'_, T> {}
