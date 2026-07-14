//! Lazy spatial-query iteration.
//!
//! [`QueryIter`] preserves the compact hot path for built-in
//! [`Predicate`] queries. [`QueryWithIter`] runs the same pruning walk
//! for logical and user-defined [`QueryPredicate`] values. Both mirror
//! Boost's `visitors/spatial_query.hpp` and pause without traversing
//! beyond the last yielded value.

use alloc::vec::Vec;
use core::iter::FusedIterator;

use crate::bounds::Bounds;
use crate::indexable::Indexable;
use crate::node::Node;
use crate::predicate::{Predicate, QueryPredicate};

enum LeafMode {
    Filtered,
    DumpAll,
}

fn stack_capacity(height: usize, max_fanout: usize) -> usize {
    height.saturating_sub(1) * max_fanout.saturating_sub(1) + max_fanout
}

/// A lazy iterator over values whose bounds satisfy a built-in
/// [`Predicate`].
///
/// Created by [`Rtree::query_iter`](crate::Rtree::query_iter). A
/// subtree that cannot match is skipped, while one wholly covered by
/// the predicate is drained without further relation tests.
pub struct QueryIter<'a, T> {
    inner: QueryIterInner<'a, T>,
}

enum QueryIterInner<'a, T> {
    Intersects(QueryWithIter<'a, T, IntersectsPredicate>),
    Other(QueryWithIter<'a, T, Predicate>),
}

struct IntersectsPredicate(Bounds);

impl<T: Indexable> QueryPredicate<T> for IntersectsPredicate {
    #[inline]
    fn matches(&self, value: &T) -> bool {
        self.0.intersects(&value.bounds())
    }

    #[inline]
    fn could_match(&self, node: &Bounds) -> bool {
        self.0.intersects(node)
    }

    #[inline]
    fn covers_all(&self, node: &Bounds) -> bool {
        self.0.contains(node)
    }
}

impl<'a, T> QueryIter<'a, T> {
    pub(crate) fn new(
        root: &'a Node<T>,
        predicate: Predicate,
        height: usize,
        max_fanout: usize,
    ) -> Self {
        let inner = match predicate {
            Predicate::Intersects(bounds) => QueryIterInner::Intersects(QueryWithIter::new(
                root,
                IntersectsPredicate(bounds),
                height,
                max_fanout,
            )),
            other => QueryIterInner::Other(QueryWithIter::new(root, other, height, max_fanout)),
        };
        Self { inner }
    }
}

impl<'a, T: Indexable> Iterator for QueryIter<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        match &mut self.inner {
            QueryIterInner::Intersects(iter) => iter.next(),
            QueryIterInner::Other(iter) => iter.next(),
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        match &self.inner {
            QueryIterInner::Intersects(iter) => iter.size_hint(),
            QueryIterInner::Other(iter) => iter.size_hint(),
        }
    }
}

impl<T: Indexable> FusedIterator for QueryIter<'_, T> {}

/// A lazy iterator over values accepted by a logical or user-defined
/// [`QueryPredicate`].
///
/// Created by [`Rtree::query_iter_with`](crate::Rtree::query_iter_with).
pub struct QueryWithIter<'a, T, P> {
    predicate: P,
    stack: Vec<(&'a Node<T>, bool)>,
    leaf: core::slice::Iter<'a, T>,
    leaf_mode: LeafMode,
}

impl<'a, T, P> QueryWithIter<'a, T, P> {
    pub(crate) fn new(root: &'a Node<T>, predicate: P, height: usize, max_fanout: usize) -> Self {
        let mut stack = Vec::with_capacity(stack_capacity(height, max_fanout));
        stack.push((root, false));
        Self {
            predicate,
            stack,
            leaf: [].iter(),
            leaf_mode: LeafMode::Filtered,
        }
    }
}

impl<'a, T, P> Iterator for QueryWithIter<'a, T, P>
where
    T: Indexable,
    P: QueryPredicate<T>,
{
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            match self.leaf_mode {
                LeafMode::Filtered => {
                    for value in self.leaf.by_ref() {
                        if self.predicate.matches(value) {
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

    fn size_hint(&self) -> (usize, Option<usize>) {
        (0, None)
    }
}

impl<T, P> FusedIterator for QueryWithIter<'_, T, P>
where
    T: Indexable,
    P: QueryPredicate<T>,
{
}
