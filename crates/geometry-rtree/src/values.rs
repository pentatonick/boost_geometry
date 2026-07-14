//! Iteration over all values stored in an R-tree.
//!
//! Mirrors the value walk behind Boost.Geometry's R-tree `begin()` and
//! `end()` iterators. The Rust API exposes it through
//! [`Rtree::iter`](crate::Rtree::iter) and `IntoIterator` for `&Rtree`.

use alloc::vec::Vec;
use core::iter::FusedIterator;

use crate::node::Node;

/// A depth-first iterator over every value in an R-tree.
pub struct Values<'a, T> {
    stack: Vec<&'a Node<T>>,
    leaf: core::slice::Iter<'a, T>,
    remaining: usize,
}

impl<'a, T> Values<'a, T> {
    pub(crate) fn new(root: &'a Node<T>, len: usize, height: usize, max_fanout: usize) -> Self {
        let capacity = height.saturating_sub(1) * max_fanout.saturating_sub(1) + max_fanout;
        let mut stack = Vec::with_capacity(capacity);
        stack.push(root);
        Self {
            stack,
            leaf: [].iter(),
            remaining: len,
        }
    }
}

impl<'a, T> Iterator for Values<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some(value) = self.leaf.next() {
                self.remaining -= 1;
                return Some(value);
            }
            match self.stack.pop()? {
                Node::Leaf(values) => self.leaf = values.iter(),
                Node::Branch(children) => {
                    for (_, child) in children.iter().rev() {
                        self.stack.push(child);
                    }
                }
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl<T> ExactSizeIterator for Values<'_, T> {}
impl<T> FusedIterator for Values<'_, T> {}
