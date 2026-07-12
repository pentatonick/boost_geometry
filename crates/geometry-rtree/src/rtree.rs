//! The [`Rtree`] itself — insert, spatial query, nearest-neighbour, and
//! Sort-Tile-Recursive bulk load.
//!
//! Mirrors `boost/geometry/index/rtree.hpp` and the visitor family under
//! `index/detail/rtree/visitors/`. Insert is the recursive
//! least-enlargement descent of `visitors/insert.hpp`; query is the
//! pruning walk of `visitors/spatial_query.hpp`; nearest is the
//! best-first search of `visitors/distance_query.hpp`.

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::marker::PhantomData;

use crate::bounds::{Bounds, union_all};
use crate::indexable::Indexable;
use crate::node::Node;
use crate::predicate::Predicate;
use crate::split::{Quadratic, SplitParameters};

/// A spatial index over `Indexable` values, parameterised by a split
/// strategy.
///
/// Mirrors `boost::geometry::index::rtree<Value, Parameters>`
/// (`index/rtree.hpp`). The default split is [`Quadratic`]; pass
/// [`Linear`](crate::split::Linear) as `Params` for cheaper inserts.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_rtree::Rtree;
///
/// type P = Point2D<f64, Cartesian>;
/// let mut tree: Rtree<P> = Rtree::new();
/// tree.insert(P::new(1.0, 1.0));
/// tree.insert(P::new(5.0, 5.0));
/// assert_eq!(tree.len(), 2);
/// ```
#[derive(Debug)]
pub struct Rtree<T: Indexable, Params: SplitParameters = Quadratic> {
    root: Node<T>,
    len: usize,
    _params: PhantomData<Params>,
}

impl<T: Indexable, Params: SplitParameters> Default for Rtree<T, Params> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Indexable, Params: SplitParameters> Rtree<T, Params> {
    /// An empty tree.
    #[must_use]
    pub fn new() -> Self {
        Self {
            root: Node::Leaf(Vec::new()),
            len: 0,
            _params: PhantomData,
        }
    }

    /// Number of values in the tree.
    #[must_use]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether the tree holds no values.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The height of the tree (a single-leaf tree is height 1).
    #[must_use]
    pub fn height(&self) -> usize {
        self.root.height()
    }

    /// Insert one value.
    ///
    /// Descends by least-enlargement to a leaf, inserts, and splits and
    /// propagates upward if a node overflows. Mirrors
    /// `visitors/insert.hpp`.
    pub fn insert(&mut self, value: T) {
        self.len += 1;
        if let Some((b1, n1, b2, n2)) = insert_into::<T, Params>(&mut self.root, value) {
            // The root split into two nodes n1/n2 (which already hold all
            // the old root's entries); grow a new root one level taller
            // over them.
            self.root = Node::Branch(Vec::from([(b1, n1), (b2, n2)]));
        }
    }

    /// Iterate every value whose bounds satisfy `predicate`.
    ///
    /// Prunes subtrees whose bounds cannot match. Mirrors
    /// `visitors/spatial_query.hpp`.
    #[must_use]
    pub fn query(&self, predicate: Predicate) -> Vec<&T> {
        let mut out = Vec::new();
        query_node(&self.root, &predicate, &mut out);
        out
    }

    /// The `k` values nearest to the query point, closest first.
    ///
    /// Best-first search over node bounding boxes by minimum distance.
    /// Mirrors `visitors/distance_query.hpp`.
    #[must_use]
    pub fn nearest(&self, query: [f64; 2], k: usize) -> Vec<&T> {
        if k == 0 {
            return Vec::new();
        }
        let mut candidates: Vec<(f64, &T)> = Vec::new();
        collect_with_distance(&self.root, query, &mut candidates);
        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(core::cmp::Ordering::Equal));
        candidates.into_iter().take(k).map(|(_, v)| v).collect()
    }
}

impl<T: Indexable, Params: SplitParameters> FromIterator<T> for Rtree<T, Params> {
    /// Bulk-load with Sort-Tile-Recursive packing: sort by x-centroid,
    /// slice into vertical strips, sort each strip by y-centroid, and
    /// pack leaves. Produces a balanced tree in one pass, the analogue of
    /// Boost's `pack_create` (`index/detail/rtree/pack_create.hpp`).
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let values: Vec<T> = iter.into_iter().collect();
        let len = values.len();
        if len <= Params::MAX {
            return Self {
                root: Node::Leaf(values),
                len,
                _params: PhantomData,
            };
        }
        let root = str_pack::<T, Params>(values);
        Self {
            root,
            len,
            _params: PhantomData,
        }
    }
}

/// Recursively insert `value` into `node`. Returns `Some((b1,n1,b2,n2))`
/// if `node` split, giving the caller the two replacement children.
type Split<T> = (Bounds, Box<Node<T>>, Bounds, Box<Node<T>>);

fn insert_into<T: Indexable, Params: SplitParameters>(
    node: &mut Node<T>,
    value: T,
) -> Option<Split<T>> {
    match node {
        Node::Leaf(values) => {
            values.push(value);
            if values.len() > Params::MAX {
                Some(split_leaf::<T, Params>(values))
            } else {
                None
            }
        }
        Node::Branch(children) => {
            // Choose the child that needs the least enlargement.
            let vb = value.bounds();
            let choice = choose_child(children, &vb);
            let (_, child) = &mut children[choice];
            let split = insert_into::<T, Params>(child, value);

            // Refresh the chosen child's bounds after the insert.
            children[choice].0 = children[choice]
                .1
                .bounds()
                .unwrap_or_else(|| children[choice].0);

            if let Some((b1, n1, b2, n2)) = split {
                children[choice] = (b1, n1);
                children.push((b2, n2));
                if children.len() > Params::MAX {
                    return Some(split_branch::<T, Params>(children));
                }
            }
            None
        }
    }
}

/// Index of the child whose box enlarges least to admit `vb` (ties
/// broken by smaller area).
#[allow(
    clippy::float_cmp,
    reason = "exact tie-break between equal enlargements, as Boost's choose_next_node does"
)]
fn choose_child<T>(children: &[(Bounds, Box<Node<T>>)], vb: &Bounds) -> usize {
    let mut best = 0;
    let mut best_enl = f64::INFINITY;
    let mut best_area = f64::INFINITY;
    for (i, (b, _)) in children.iter().enumerate() {
        let enl = b.enlargement(vb);
        let area = b.area();
        if enl < best_enl || (enl == best_enl && area < best_area) {
            best = i;
            best_enl = enl;
            best_area = area;
        }
    }
    best
}

/// Split an overflowing leaf's values into two leaves.
fn split_leaf<T: Indexable, Params: SplitParameters>(values: &mut Vec<T>) -> Split<T> {
    let taken = core::mem::take(values);
    let boxes: Vec<Bounds> = taken.iter().map(Indexable::bounds).collect();
    let (g1, g2) = Params::split(&boxes);

    // Partition `taken` into the two index groups. Walk once, routing by
    // membership in g1.
    let mut in_g1 = alloc::vec![false; taken.len()];
    for &i in &g1 {
        in_g1[i] = true;
    }
    let mut v1: Vec<T> = Vec::new();
    let mut v2: Vec<T> = Vec::new();
    for (i, v) in taken.into_iter().enumerate() {
        if in_g1[i] {
            v1.push(v);
        } else {
            v2.push(v);
        }
    }
    debug_assert_eq!(v1.len(), g1.len());
    debug_assert_eq!(v2.len(), g2.len());

    let b1 = union_all(&v1.iter().map(Indexable::bounds).collect::<Vec<_>>());
    let b2 = union_all(&v2.iter().map(Indexable::bounds).collect::<Vec<_>>());
    (b1, Box::new(Node::Leaf(v1)), b2, Box::new(Node::Leaf(v2)))
}

/// Split an overflowing branch's children into two branches.
fn split_branch<T: Indexable, Params: SplitParameters>(
    children: &mut Vec<(Bounds, Box<Node<T>>)>,
) -> Split<T> {
    let taken = core::mem::take(children);
    let boxes: Vec<Bounds> = taken.iter().map(|(b, _)| *b).collect();
    let (g1, _g2) = Params::split(&boxes);

    let mut in_g1 = alloc::vec![false; taken.len()];
    for &i in &g1 {
        in_g1[i] = true;
    }
    let mut c1: Vec<(Bounds, Box<Node<T>>)> = Vec::new();
    let mut c2: Vec<(Bounds, Box<Node<T>>)> = Vec::new();
    for (i, c) in taken.into_iter().enumerate() {
        if in_g1[i] {
            c1.push(c);
        } else {
            c2.push(c);
        }
    }

    let b1 = union_all(&c1.iter().map(|(b, _)| *b).collect::<Vec<_>>());
    let b2 = union_all(&c2.iter().map(|(b, _)| *b).collect::<Vec<_>>());
    (
        b1,
        Box::new(Node::Branch(c1)),
        b2,
        Box::new(Node::Branch(c2)),
    )
}

/// Recursive pruning query walk.
fn query_node<'a, T: Indexable>(node: &'a Node<T>, predicate: &Predicate, out: &mut Vec<&'a T>) {
    match node {
        Node::Leaf(values) => {
            for v in values {
                if predicate.matches(&v.bounds()) {
                    out.push(v);
                }
            }
        }
        Node::Branch(children) => {
            for (b, child) in children {
                if predicate.could_match(b) {
                    query_node(child, predicate, out);
                }
            }
        }
    }
}

/// Collect every value with its distance to the query point. (A simple
/// full walk; the `nearest` caller sorts and truncates. Adequate for the
/// v1 milestone — a priority-queue best-first refinement is a later
/// optimisation.)
fn collect_with_distance<'a, T: Indexable>(
    node: &'a Node<T>,
    query: [f64; 2],
    out: &mut Vec<(f64, &'a T)>,
) {
    match node {
        Node::Leaf(values) => {
            for v in values {
                let b = v.bounds();
                out.push((b.min_distance_to(query), v));
            }
        }
        Node::Branch(children) => {
            for (_, child) in children {
                collect_with_distance(child, query, out);
            }
        }
    }
}

/// Sort-Tile-Recursive packing of `values` into a balanced tree.
fn str_pack<T: Indexable, Params: SplitParameters>(mut values: Vec<T>) -> Node<T> {
    // Leaf-pack: sort by x-centroid, cut into √(n / MAX) vertical strips,
    // sort each strip by y-centroid, then chop into MAX-sized leaves.
    values.sort_by(|a, b| {
        a.bounds().center()[0]
            .partial_cmp(&b.bounds().center()[0])
            .unwrap_or(core::cmp::Ordering::Equal)
    });

    let leaf_count = values.len().div_ceil(Params::MAX);
    let strip_count = isqrt_ceil(leaf_count).max(1);
    let per_strip = values.len().div_ceil(strip_count);

    let mut leaves: Vec<(Bounds, Box<Node<T>>)> = Vec::new();
    let mut rest = values;
    while !rest.is_empty() {
        // Peel off the next vertical strip.
        let take = per_strip.min(rest.len());
        let tail = rest.split_off(take);
        let mut strip = core::mem::replace(&mut rest, tail);

        strip.sort_by(|a, b| {
            a.bounds().center()[1]
                .partial_cmp(&b.bounds().center()[1])
                .unwrap_or(core::cmp::Ordering::Equal)
        });

        // Chop the y-sorted strip into MAX-sized leaves.
        let mut strip_iter = strip.into_iter();
        loop {
            let leaf_vals: Vec<T> = (&mut strip_iter).take(Params::MAX).collect();
            if leaf_vals.is_empty() {
                break;
            }
            let boxes: Vec<Bounds> = leaf_vals.iter().map(Indexable::bounds).collect();
            let b = union_all(&boxes);
            leaves.push((b, Box::new(Node::Leaf(leaf_vals))));
        }
    }

    build_branches::<T, Params>(leaves)
}

/// Recursively group `children` into branch levels until one root
/// remains.
fn build_branches<T: Indexable, Params: SplitParameters>(
    children: Vec<(Bounds, Box<Node<T>>)>,
) -> Node<T> {
    if children.len() <= Params::MAX {
        return Node::Branch(children);
    }
    let mut level: Vec<(Bounds, Box<Node<T>>)> = Vec::new();
    let mut it = children.into_iter();
    loop {
        let group: Vec<(Bounds, Box<Node<T>>)> = (&mut it).take(Params::MAX).collect();
        if group.is_empty() {
            break;
        }
        let boxes: Vec<Bounds> = group.iter().map(|(b, _)| *b).collect();
        let b = union_all(&boxes);
        level.push((b, Box::new(Node::Branch(group))));
    }
    build_branches::<T, Params>(level)
}

/// Ceil of the integer square root, without floating point (MSRV
/// predates `usize::isqrt`). Integer Newton iteration.
fn isqrt_ceil(n: usize) -> usize {
    if n < 2 {
        return n;
    }
    let mut x = n;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = usize::midpoint(x, n / x);
    }
    // `x` is floor(sqrt(n)); round up if it is not exact.
    if x * x == n { x } else { x + 1 }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact integer-valued point coordinates")]
mod tests {
    use super::Rtree;
    use crate::bounds::Bounds;
    use crate::predicate::Predicate;
    use crate::split::Linear;
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;
    use geometry_trait::Point as _;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn empty_tree() {
        let t: Rtree<P> = Rtree::new();
        assert!(t.is_empty());
        assert_eq!(t.len(), 0);
    }

    #[test]
    fn insert_many_points_keeps_len() {
        let mut t: Rtree<P> = Rtree::new();
        for i in 0..1000 {
            let x = f64::from(i % 100);
            let y = f64::from(i / 100);
            t.insert(P::new(x, y));
        }
        assert_eq!(t.len(), 1000);
        assert!(
            t.height() >= 2,
            "1000 points should build a multi-level tree"
        );
    }

    #[test]
    fn query_intersects_finds_the_point() {
        let mut t: Rtree<P> = Rtree::new();
        for i in 0..200 {
            t.insert(P::new(f64::from(i), 0.0));
        }
        let hits = t.query(Predicate::Intersects(Bounds::new([9.5, -1.0], [10.5, 1.0])));
        assert_eq!(hits.len(), 1);
    }

    #[test]
    fn query_within_a_window() {
        let mut t: Rtree<P> = Rtree::new();
        for x in 0..10 {
            for y in 0..10 {
                t.insert(P::new(f64::from(x), f64::from(y)));
            }
        }
        // The window [2,5]×[2,5] contains a 4×4 block of points.
        let hits = t.query(Predicate::Within(Bounds::new([2.0, 2.0], [5.0, 5.0])));
        assert_eq!(hits.len(), 16);
    }

    #[test]
    fn nearest_returns_closest_first() {
        let mut t: Rtree<P> = Rtree::new();
        for i in 0..100 {
            t.insert(P::new(f64::from(i), 0.0));
        }
        let near = t.nearest([10.2, 0.0], 3);
        assert_eq!(near.len(), 3);
        // The three closest to x=10.2 are x=10, 11, 9 in some order.
        let mut xs: Vec<f64> = near.iter().map(|p| p.get::<0>()).collect();
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(xs, [9.0, 10.0, 11.0]);
    }

    #[test]
    fn linear_split_also_works() {
        let mut t: Rtree<P, Linear<8, 3>> = Rtree::new();
        for i in 0..500 {
            t.insert(P::new(f64::from(i % 25), f64::from(i / 25)));
        }
        assert_eq!(t.len(), 500);
        let hits = t.query(Predicate::Intersects(Bounds::new([0.0, 0.0], [3.0, 3.0])));
        assert!(!hits.is_empty());
    }

    #[test]
    fn bulk_load_balances() {
        let points: Vec<P> = (0..10_000)
            .map(|i| P::new(f64::from(i % 100), f64::from(i / 100)))
            .collect();
        let t: Rtree<P> = points.into_iter().collect();
        assert_eq!(t.len(), 10_000);
        // A balanced tree of 10k / 8-per-leaf is shallow.
        assert!(t.height() <= 6, "STR tree too tall: {}", t.height());
    }
}
