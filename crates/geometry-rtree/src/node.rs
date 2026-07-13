//! Tree nodes: leaves hold values, branches hold child subtrees.
//!
//! Mirrors `index/detail/rtree/node/variant.hpp`. Boost dispatches over
//! its node variant with a runtime visitor; the port uses a plain enum
//! and lets `match` be the visitor — the same shape, no vtable.

use alloc::vec::Vec;

use crate::bounds::{Bounds, union_all};
use crate::indexable::Indexable;

/// One node of the tree.
///
/// A [`Node::Leaf`] stores the indexed values directly; a
/// [`Node::Branch`] stores `(child_bounds, child_node)` pairs. Mirrors
/// the leaf / internal split of Boost's rtree node variant.
#[derive(Debug)]
pub enum Node<T> {
    /// Values at the bottom of the tree.
    Leaf(Vec<T>),
    /// Child subtrees, each paired with its bounding box.
    Branch(Vec<(Bounds, Node<T>)>),
}

impl<T: Indexable> Node<T> {
    /// The bounding box covering everything in this node, or `None` for
    /// an empty node.
    #[must_use]
    pub fn bounds(&self) -> Option<Bounds> {
        match self {
            Node::Leaf(values) => {
                if values.is_empty() {
                    None
                } else {
                    let boxes: Vec<Bounds> = values.iter().map(Indexable::bounds).collect();
                    Some(union_all(&boxes))
                }
            }
            Node::Branch(children) => {
                if children.is_empty() {
                    None
                } else {
                    let boxes: Vec<Bounds> = children.iter().map(|(b, _)| *b).collect();
                    Some(union_all(&boxes))
                }
            }
        }
    }

    /// Number of immediate entries (values in a leaf, children in a
    /// branch).
    #[must_use]
    pub fn entry_count(&self) -> usize {
        match self {
            Node::Leaf(values) => values.len(),
            Node::Branch(children) => children.len(),
        }
    }

    /// Total number of leaf values in this subtree.
    #[must_use]
    pub fn value_count(&self) -> usize {
        match self {
            Node::Leaf(values) => values.len(),
            Node::Branch(children) => children.iter().map(|(_, child)| child.value_count()).sum(),
        }
    }

    /// Height of this subtree: a leaf is height 1.
    #[must_use]
    pub fn height(&self) -> usize {
        match self {
            Node::Leaf(_) => 1,
            Node::Branch(children) => {
                1 + children
                    .iter()
                    .map(|(_, child)| child.height())
                    .max()
                    .unwrap_or(0)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Node;
    use crate::bounds::Bounds;

    #[test]
    fn leaf_bounds_union_of_values() {
        let leaf: Node<Bounds> = Node::Leaf(alloc::vec![
            Bounds::point([0.0, 0.0]),
            Bounds::point([2.0, 3.0]),
        ]);
        assert_eq!(leaf.bounds(), Some(Bounds::new([0.0, 0.0], [2.0, 3.0])));
        assert_eq!(leaf.entry_count(), 2);
        assert_eq!(leaf.height(), 1);
    }

    #[test]
    fn branch_bounds_and_height() {
        let leaf_a: Node<Bounds> = Node::Leaf(alloc::vec![Bounds::point([0.0, 0.0])]);
        let leaf_b: Node<Bounds> = Node::Leaf(alloc::vec![Bounds::point([5.0, 5.0])]);
        let branch: Node<Bounds> = Node::Branch(alloc::vec![
            (leaf_a.bounds().unwrap(), leaf_a),
            (leaf_b.bounds().unwrap(), leaf_b),
        ]);
        assert_eq!(branch.bounds(), Some(Bounds::new([0.0, 0.0], [5.0, 5.0])));
        assert_eq!(branch.height(), 2);
        assert_eq!(branch.value_count(), 2);
    }

    #[test]
    fn empty_leaf_has_no_bounds() {
        let leaf: Node<Bounds> = Node::Leaf(alloc::vec![]);
        assert_eq!(leaf.bounds(), None);
    }
}
