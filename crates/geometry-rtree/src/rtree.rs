//! The [`Rtree`] itself — insert, spatial query, nearest-neighbour, and
//! Sort-Tile-Recursive bulk load.
//!
//! Mirrors `boost/geometry/index/rtree.hpp` and the visitor family under
//! `index/detail/rtree/visitors/`. Insert is the recursive
//! least-enlargement descent of `visitors/insert.hpp`; query is the
//! pruning walk of `visitors/spatial_query.hpp`; nearest is the
//! best-first search of `visitors/distance_query.hpp`.

use alloc::vec::Vec;
use core::marker::PhantomData;

use crate::bounds::Bounds;
use crate::indexable::Indexable;
use crate::nearest_bound::NearestBound;
use crate::nearest_iter::NearestIter;
use crate::node::Node;
use crate::predicate::Predicate;
use crate::query_iter::QueryIter;
use crate::search_frontier::SearchFrontier;
use crate::split::{AsymmetricRStarSplit, SplitParameters};

/// A spatial index over `Indexable` values, parameterised by a split
/// strategy.
///
/// Mirrors `boost::geometry::index::rtree<Value, Parameters>`
/// (`index/rtree.hpp`). The default uses six-child branches and
/// 12-value leaves for insertion, with four-child branches and
/// four-value leaves for bulk packing, via [`AsymmetricRStarSplit`]; pass a symmetric
/// [`RStarSplit`](crate::split::RStarSplit),
/// [`Quadratic`](crate::split::Quadratic), or
/// [`Linear`](crate::split::Linear) as `Params` for a different
/// trade-off. Most users should retain the default; the [`split`](crate::split)
/// module explains the parameter order, validity constraints, tuning process,
/// and benchmark evidence.
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
pub struct Rtree<T: Indexable, Params: SplitParameters = AsymmetricRStarSplit<6, 2, 12, 4, 4, 4>> {
    root: Node<T>,
    len: usize,
    height: usize,
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
            height: 1,
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
    ///
    /// This is cached and returned in constant time.
    #[must_use]
    pub fn height(&self) -> usize {
        self.height
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
            self.height += 1;
        }
    }

    /// Every value whose bounds satisfy `predicate`.
    ///
    /// [`query_iter`](Self::query_iter) collected — the lazy walk is
    /// the crate's single query implementation. Mirrors
    /// `visitors/spatial_query.hpp`.
    #[must_use]
    pub fn query(&self, predicate: Predicate) -> Vec<&T> {
        self.query_iter(predicate).collect()
    }

    /// Lazily iterate the values whose bounds satisfy `predicate`.
    ///
    /// The pruning walk of `visitors/spatial_query.hpp` as a lazy
    /// iterator: stopping early performs no traversal past the value
    /// stopped at, and folding stores no output.
    /// [`query`](Self::query) is this walk collected.
    ///
    /// # Examples
    ///
    /// Fold without collecting:
    ///
    /// ```
    /// use geometry_rtree::{Bounds, Predicate, Rtree};
    ///
    /// let tree: Rtree<(Bounds, u32)> = (0..100u32)
    ///     .map(|i| (Bounds::point([f64::from(i), 0.0]), i))
    ///     .collect();
    /// let window = Predicate::Intersects(Bounds::new([10.0, -1.0], [19.0, 1.0]));
    /// let id_sum: u32 = tree.query_iter(window).map(|(_, id)| id).sum();
    /// assert_eq!(id_sum, (10..=19).sum());
    /// ```
    #[must_use]
    pub fn query_iter(&self, predicate: Predicate) -> QueryIter<'_, T> {
        QueryIter::new(&self.root, predicate, self.height(), Params::BRANCH_MAX)
    }

    /// Lazily iterate ALL values, nearest to `query` first — an
    /// unbounded ordered stream over the entire tree.
    ///
    /// The consumer supplies its own bound via
    /// [`take`](Iterator::take); with no `k` up front nothing can be
    /// pruned, so a caller who knows `k` and wants maximum pruning
    /// calls [`nearest`](Self::nearest) instead. Distances are compared
    /// SQUARED, the same ordering [`nearest`](Self::nearest) uses.
    ///
    /// # Examples
    ///
    /// Nearest-one:
    ///
    /// ```
    /// use geometry_rtree::{Bounds, Rtree};
    ///
    /// let tree: Rtree<(Bounds, u32)> = (0..100u32)
    ///     .map(|i| (Bounds::point([f64::from(i), 0.0]), i))
    ///     .collect();
    /// let (_, nearest_id) = tree.nearest_iter([41.7, 0.0]).next().unwrap();
    /// assert_eq!(*nearest_id, 42);
    /// ```
    ///
    /// Over-fetch and re-rank: take more than needed by box distance,
    /// re-rank by a finer key, keep the best:
    ///
    /// ```
    /// use geometry_rtree::{Bounds, Rtree};
    ///
    /// let tree: Rtree<(Bounds, u32)> = (0..100u32)
    ///     .map(|i| (Bounds::point([f64::from(i), 0.0]), i))
    ///     .collect();
    /// let mut candidates: Vec<&(Bounds, u32)> =
    ///     tree.nearest_iter([50.2, 0.0]).take(8).collect();
    /// candidates.sort_by_key(|(_, id)| *id);
    /// candidates.truncate(2);
    /// let ids: Vec<u32> = candidates.iter().map(|(_, id)| *id).collect();
    /// assert_eq!(ids, [47, 48]);
    /// ```
    #[must_use]
    pub fn nearest_iter(&self, query: [f64; 2]) -> NearestIter<'_, T> {
        NearestIter::new(&self.root, query)
    }

    /// Lazily iterate all values nearest-first with caller-selected
    /// inline capacities for the node and value frontiers.
    ///
    /// Entries beyond either capacity spill that frontier to an
    /// allocated binary heap. Smaller capacities reduce the iterator's
    /// stack footprint and initialization cost; larger capacities avoid
    /// spills on wider searches. Prefer [`nearest_iter`](Self::nearest_iter)
    /// unless measurements for the caller's tree and query distribution
    /// justify a different pair.
    #[must_use]
    pub fn nearest_iter_with_inline_capacities<
        const NODE_INLINE_CAPACITY: usize,
        const VALUE_INLINE_CAPACITY: usize,
    >(
        &self,
        query: [f64; 2],
    ) -> NearestIter<'_, T, NODE_INLINE_CAPACITY, VALUE_INLINE_CAPACITY> {
        NearestIter::new(&self.root, query)
    }

    /// The `k` values nearest to the query point, closest first.
    ///
    /// Best-first search over node bounding boxes by SQUARED minimum
    /// possible distance (same ordering as true distance, no square
    /// roots). A stack-first frontier (`SearchFrontier`) holds
    /// unexpanded NODES only, popped nearest-first; candidate values
    /// never enter it. Each candidate instead goes through a bounded
    /// max-heap (`NearestBound`) whose entries pair each distance with
    /// its value. Collecting the final values reuses that heap's
    /// allocation in place, so the whole search performs one
    /// `min(k, len)`-sized heap allocation unless the frontier spills.
    ///
    /// Termination: a child's box is contained in its parent's, so
    /// frontier pops ascend in minimum possible distance. When a popped
    /// node's distance reaches the k-th-best value distance, every
    /// value in every unvisited subtree is at least that far away and
    /// the held ranks are final; equality only ties distances already
    /// held. Mirrors `visitors/distance_query.hpp`.
    ///
    /// This bounded implementation stays dedicated: its best-k pruning
    /// needs `k` up front, which the unbounded
    /// [`nearest_iter`](Self::nearest_iter) stream cannot have.
    #[must_use]
    pub fn nearest(&self, query: [f64; 2], k: usize) -> Vec<&T> {
        if k == 0 || self.len == 0 {
            return Vec::new();
        }
        let capacity = k.min(self.len);
        let mut ranks = NearestBound::new(k, capacity);
        let mut frontier: SearchFrontier<FrontierNode<'_, T>> = SearchFrontier::new();
        frontier.push(FrontierNode {
            dist: 0.0,
            node: &self.root,
        });
        while let Some(FrontierNode { dist, node }) = frontier.pop() {
            if dist.total_cmp(&ranks.bound()).is_ge() {
                break;
            }
            match node {
                Node::Leaf(values) => admit_nearest_values(values.iter(), query, &mut ranks),
                Node::Branch(children) => {
                    let bound = ranks.bound();
                    for (b, child) in children {
                        let dist = b.comparable_min_distance_to(query);
                        if dist.total_cmp(&bound).is_lt() {
                            frontier.push(FrontierNode { dist, node: child });
                        }
                    }
                }
            }
        }
        ranks.into_values()
    }
}

fn admit_nearest_values<'a, T: Indexable>(
    values: impl Iterator<Item = &'a T>,
    query: [f64; 2],
    ranks: &mut NearestBound<&'a T>,
) {
    for value in values {
        let dist = value.bounds().comparable_min_distance_to(query);
        if dist.total_cmp(&ranks.bound()).is_lt() {
            ranks.admit_better(dist, value);
        }
    }
}

impl<T: Indexable, Params: SplitParameters> FromIterator<T> for Rtree<T, Params> {
    /// Bulk-load with top-down Sort-Tile-Recursive packing: recursively
    /// partition cached centroids into balanced x/y tiles until they fit
    /// the configured bulk leaf capacity. Produces a balanced tree, the
    /// analogue of Boost's `pack_create`
    /// (`index/detail/rtree/pack_create.hpp`).
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let values: Vec<T> = iter.into_iter().collect();
        let len = values.len();
        assert!(
            Params::BULK_LEAF_MAX > 0,
            "bulk leaf capacity must be non-zero"
        );
        assert!(
            Params::BULK_BRANCH_MAX >= 2,
            "bulk branch capacity must be at least two"
        );
        if len <= Params::BULK_LEAF_MAX {
            return Self {
                root: Node::Leaf(values),
                len,
                height: 1,
                _params: PhantomData,
            };
        }
        let (root, height) = str_pack::<T, Params>(values);
        Self {
            root,
            len,
            height,
            _params: PhantomData,
        }
    }
}

/// A frontier entry of the best-first nearest search: an unexpanded
/// node keyed by the minimum possible distance of anything inside it.
struct FrontierNode<'a, T> {
    dist: f64,
    node: &'a Node<T>,
}

impl<T> PartialEq for FrontierNode<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.dist.total_cmp(&other.dist).is_eq()
    }
}

impl<T> Eq for FrontierNode<'_, T> {}

impl<T> PartialOrd for FrontierNode<'_, T> {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T> Ord for FrontierNode<'_, T> {
    /// Reversed so the max-first [`SearchFrontier`] pops the SMALLEST
    /// distance first.
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        other.dist.total_cmp(&self.dist)
    }
}

/// Recursively insert `value` into `node`. Returns `Some((b1,n1,b2,n2))`
/// if `node` split, giving the caller the two replacement children.
type Split<T> = (Bounds, Node<T>, Bounds, Node<T>);

fn insert_into<T: Indexable, Params: SplitParameters>(
    node: &mut Node<T>,
    value: T,
) -> Option<Split<T>> {
    match node {
        Node::Leaf(leaf) => {
            leaf.push(value);
            if leaf.len() > Params::LEAF_MAX {
                Some(split_leaf::<T, Params>(leaf))
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

            if let Some((b1, n1, b2, n2)) = split {
                children[choice] = (b1, n1);
                children.push((b2, n2));
                if children.len() > Params::BRANCH_MAX {
                    return Some(split_branch::<T, Params>(children));
                }
            } else {
                // No split: the child now holds its old contents plus
                // `value`, so its box is the old box grown by `vb` —
                // O(1), no subtree walk.
                children[choice].0 = children[choice].0.union(&vb);
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
fn choose_child<T>(children: &[(Bounds, Node<T>)], vb: &Bounds) -> usize {
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
fn split_leaf<T: Indexable, Params: SplitParameters>(leaf: &mut Vec<T>) -> Split<T> {
    let taken = core::mem::take(leaf);
    let boxes: Vec<Bounds> = taken.iter().map(Indexable::bounds).collect();
    let (g1, g2) = Params::split_leaf(&boxes);

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

    let b1 = v1
        .iter()
        .map(Indexable::bounds)
        .reduce(|a, b| a.union(&b))
        .expect("split group is non-empty by MIN invariant");
    let b2 = v2
        .iter()
        .map(Indexable::bounds)
        .reduce(|a, b| a.union(&b))
        .expect("split group is non-empty by MIN invariant");
    (b1, Node::Leaf(v1), b2, Node::Leaf(v2))
}

/// Split an overflowing branch's children into two branches.
fn split_branch<T: Indexable, Params: SplitParameters>(
    children: &mut Vec<(Bounds, Node<T>)>,
) -> Split<T> {
    let taken = core::mem::take(children);
    let boxes: Vec<Bounds> = taken.iter().map(|(b, _)| *b).collect();
    let (g1, _g2) = Params::split_branch(&boxes);

    let mut in_g1 = alloc::vec![false; taken.len()];
    for &i in &g1 {
        in_g1[i] = true;
    }
    let mut c1: Vec<(Bounds, Node<T>)> = Vec::new();
    let mut c2: Vec<(Bounds, Node<T>)> = Vec::new();
    for (i, c) in taken.into_iter().enumerate() {
        if in_g1[i] {
            c1.push(c);
        } else {
            c2.push(c);
        }
    }

    let b1 = c1
        .iter()
        .map(|(b, _)| *b)
        .reduce(|a, b| a.union(&b))
        .expect("split group is non-empty by MIN invariant");
    let b2 = c2
        .iter()
        .map(|(b, _)| *b)
        .reduce(|a, b| a.union(&b))
        .expect("split group is non-empty by MIN invariant");
    (b1, Node::Branch(c1), b2, Node::Branch(c2))
}

/// Sort-Tile-Recursive packing of `values` into a balanced tree.
fn str_pack<T: Indexable, Params: SplitParameters>(values: Vec<T>) -> (Node<T>, usize) {
    // Cache sort keys once: recursive spatial partitioning otherwise
    // calls `bounds()` throughout every sort comparator.
    let mut values: Vec<Option<T>> = values.into_iter().map(Some).collect();
    let keyed: Vec<([f64; 2], usize)> = values
        .iter()
        .enumerate()
        .map(|(index, value)| {
            (
                value
                    .as_ref()
                    .expect("packed value is present")
                    .bounds()
                    .center(),
                index,
            )
        })
        .collect();
    let mut height = 1;
    let mut capacity = Params::BULK_LEAF_MAX;
    while capacity < keyed.len() {
        capacity = capacity.saturating_mul(Params::BULK_BRANCH_MAX);
        height += 1;
    }
    (
        str_pack_height::<T, Params>(keyed, height, &mut values).1,
        height,
    )
}

/// Top-down STR partitioning at one known tree height. Each child gets
/// a balanced spatial tile small enough for the remaining subtree
/// capacity, avoiding cross-strip grouping at upper levels.
fn str_pack_height<T: Indexable, Params: SplitParameters>(
    mut keyed: Vec<([f64; 2], usize)>,
    height: usize,
    values: &mut [Option<T>],
) -> (Bounds, Node<T>) {
    if height == 1 {
        debug_assert!(keyed.len() <= Params::BULK_LEAF_MAX);
        let leaf_values: Vec<T> = keyed
            .into_iter()
            .map(|(_, index)| values[index].take().expect("packed value is present"))
            .collect();
        let bounds = leaf_values
            .iter()
            .map(Indexable::bounds)
            .reduce(|a, b| a.union(&b))
            .expect("packed leaf is non-empty");
        return (bounds, Node::Leaf(leaf_values));
    }

    let child_capacity = packed_subtree_capacity::<Params>(height - 1);
    let child_count = keyed.len().div_ceil(child_capacity);
    debug_assert!(child_count <= Params::BULK_BRANCH_MAX);
    let column_count = isqrt_ceil(child_count).max(1);

    let mut children = Vec::with_capacity(child_count);
    let mut remaining_children = child_count;
    for column in 0..column_count {
        let children_in_column =
            child_count / column_count + usize::from(column < child_count % column_count);
        if children_in_column == 0 {
            continue;
        }
        let base = keyed.len() / remaining_children;
        let extra = keyed.len() % remaining_children;
        let take = base * children_in_column + extra.min(children_in_column);
        let mut strip = take_lowest_by_axis(&mut keyed, take, 0);

        let mut remaining_in_column = children_in_column;
        while remaining_in_column != 0 {
            let take = strip.len().div_ceil(remaining_in_column);
            let tile = take_lowest_by_axis(&mut strip, take, 1);
            children.push(str_pack_height::<T, Params>(tile, height - 1, values));
            remaining_in_column -= 1;
        }
        remaining_children -= children_in_column;
    }

    let bounds = children
        .iter()
        .map(|(bounds, _)| *bounds)
        .reduce(|a, b| a.union(&b))
        .expect("packed branch is non-empty");
    (bounds, Node::Branch(children))
}

fn take_lowest_by_axis(
    values: &mut Vec<([f64; 2], usize)>,
    take: usize,
    axis: usize,
) -> Vec<([f64; 2], usize)> {
    if take == values.len() {
        return core::mem::take(values);
    }
    values.select_nth_unstable_by(take, |(a, _), (b, _)| a[axis].total_cmp(&b[axis]));
    let tail = values.split_off(take);
    core::mem::replace(values, tail)
}

fn packed_subtree_capacity<Params: SplitParameters>(height: usize) -> usize {
    let mut capacity = Params::BULK_LEAF_MAX;
    for _ in 1..height {
        capacity = capacity.saturating_mul(Params::BULK_BRANCH_MAX);
    }
    capacity
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
    use super::{FrontierNode, Rtree, isqrt_ceil};
    use crate::bounds::{Bounds, union_all};
    use crate::indexable::Indexable;
    use crate::nearest_bound::{NearestBound, NearestBoundMetrics};
    use crate::node::Node;
    use crate::predicate::Predicate;
    use crate::search_frontier::SearchFrontier;
    use crate::split::{
        AsymmetricQuadratic, AsymmetricRStarSplit, Linear, Quadratic, SplitParameters,
    };
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;
    use geometry_trait::Point as _;

    type P = Point2D<f64, Cartesian>;
    type Leaf<T> = Vec<T>;

    trait LeafProbe<T> {
        fn values(&self) -> &[T];
        fn packed_group_bounds(&self) -> Option<&[Bounds]>;
        fn packed_group(&self, index: usize) -> &[T];
    }

    impl<T> LeafProbe<T> for Vec<T> {
        fn values(&self) -> &[T] {
            self
        }

        fn packed_group_bounds(&self) -> Option<&[Bounds]> {
            None
        }

        fn packed_group(&self, index: usize) -> &[T] {
            const GROUP_SIZE: usize = 8;
            let start = index * GROUP_SIZE;
            &self[start..(start + GROUP_SIZE).min(self.len())]
        }
    }

    struct Lcg {
        state: u64,
    }

    impl Lcg {
        fn new() -> Self {
            Self {
                state: 0x9E37_79B9_7F4A_7C15,
            }
        }

        #[allow(
            clippy::cast_precision_loss,
            reason = "state >> 11 keeps 53 bits, exact in f64"
        )]
        fn next_f64(&mut self) -> f64 {
            self.state = self
                .state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (self.state >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    #[derive(Debug, Default)]
    struct BoundedSearchMetrics {
        frontier_pushes: usize,
        frontier_pops: usize,
        frontier_high_water: usize,
        terminated_by_bound: usize,
        branch_expansions: usize,
        child_distance_evaluations: usize,
        child_order_comparisons: usize,
        child_pushes: usize,
        child_pruned: usize,
        leaf_expansions: usize,
        reversed_leaf_scans: usize,
        leaf_group_bound_evaluations: usize,
        leaf_group_order_comparisons: usize,
        leaf_groups_scanned: usize,
        leaf_groups_pruned: usize,
        value_distance_evaluations: usize,
        value_bound_passes: usize,
        value_bound_rejections: usize,
        rank: NearestBoundMetrics,
    }

    impl BoundedSearchMetrics {
        fn add(&mut self, other: &Self) {
            self.frontier_pushes += other.frontier_pushes;
            self.frontier_pops += other.frontier_pops;
            self.frontier_high_water = self.frontier_high_water.max(other.frontier_high_water);
            self.terminated_by_bound += other.terminated_by_bound;
            self.branch_expansions += other.branch_expansions;
            self.child_distance_evaluations += other.child_distance_evaluations;
            self.child_order_comparisons += other.child_order_comparisons;
            self.child_pushes += other.child_pushes;
            self.child_pruned += other.child_pruned;
            self.leaf_expansions += other.leaf_expansions;
            self.reversed_leaf_scans += other.reversed_leaf_scans;
            self.leaf_group_bound_evaluations += other.leaf_group_bound_evaluations;
            self.leaf_group_order_comparisons += other.leaf_group_order_comparisons;
            self.leaf_groups_scanned += other.leaf_groups_scanned;
            self.leaf_groups_pruned += other.leaf_groups_pruned;
            self.value_distance_evaluations += other.value_distance_evaluations;
            self.value_bound_passes += other.value_bound_passes;
            self.value_bound_rejections += other.value_bound_rejections;
            self.rank.calls += other.rank.calls;
            self.rank.partition_comparisons += other.rank.partition_comparisons;
            self.rank.admissions += other.rank.admissions;
            self.rank.replacements += other.rank.replacements;
            self.rank.shifted_ranks += other.rank.shifted_ranks;
        }
    }

    enum PackedFrontierItem<'a, T> {
        Node(&'a Node<T>),
        Group(&'a Leaf<T>, usize),
        Value(&'a T),
    }

    struct PackedFrontierEntry<'a, T> {
        dist: f64,
        item: PackedFrontierItem<'a, T>,
    }

    impl<T> PartialEq for PackedFrontierEntry<'_, T> {
        fn eq(&self, other: &Self) -> bool {
            self.dist.total_cmp(&other.dist).is_eq()
        }
    }

    impl<T> Eq for PackedFrontierEntry<'_, T> {}

    impl<T> PartialOrd for PackedFrontierEntry<'_, T> {
        fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }

    impl<T> Ord for PackedFrontierEntry<'_, T> {
        fn cmp(&self, other: &Self) -> core::cmp::Ordering {
            other.dist.total_cmp(&self.dist)
        }
    }

    #[derive(Debug, Default)]
    struct PackedFrontierMetrics {
        pushes: usize,
        pops: usize,
        high_water: usize,
        branch_expansions: usize,
        leaf_expansions: usize,
        group_pushes: usize,
        group_pops: usize,
        value_pushes: usize,
        value_pops: usize,
    }

    impl PackedFrontierMetrics {
        fn add(&mut self, other: &Self) {
            self.pushes += other.pushes;
            self.pops += other.pops;
            self.high_water = self.high_water.max(other.high_water);
            self.branch_expansions += other.branch_expansions;
            self.leaf_expansions += other.leaf_expansions;
            self.group_pushes += other.group_pushes;
            self.group_pops += other.group_pops;
            self.value_pushes += other.value_pushes;
            self.value_pops += other.value_pops;
        }
    }

    fn nearest_with_metrics<T: Indexable, Params: SplitParameters>(
        tree: &Rtree<T, Params>,
        query: [f64; 2],
        k: usize,
        nearer_y_end_first: bool,
        leaf_group_size: usize,
        center_out: bool,
        leaf_bvh_terminal_size: usize,
    ) -> (Vec<&T>, BoundedSearchMetrics) {
        if k == 0 || tree.len == 0 {
            return (Vec::new(), BoundedSearchMetrics::default());
        }
        let capacity = k.min(tree.len);
        let mut ranks = NearestBound::new(k, capacity);
        let mut frontier: SearchFrontier<FrontierNode<'_, T>> = SearchFrontier::new();
        let mut metrics = BoundedSearchMetrics::default();
        frontier.push(FrontierNode {
            dist: 0.0,
            node: &tree.root,
        });
        while let Some(FrontierNode { dist, node }) = frontier.pop() {
            if dist.total_cmp(&ranks.bound()).is_ge() {
                metrics.terminated_by_bound += 1;
                break;
            }
            match node {
                Node::Leaf(leaf) => {
                    let values = leaf.values();
                    metrics.leaf_expansions += 1;
                    let reverse = nearer_y_end_first
                        && values
                            .first()
                            .zip(values.last())
                            .is_some_and(|(first, last)| {
                                let first_y = first.bounds().center()[1];
                                let last_y = last.bounds().center()[1];
                                (last_y - query[1]).abs() < (first_y - query[1]).abs()
                            });
                    metrics.reversed_leaf_scans += usize::from(reverse);
                    if leaf_bvh_terminal_size != 0 {
                        record_leaf_bvh(
                            values,
                            leaf_bvh_terminal_size,
                            query,
                            &mut ranks,
                            &mut metrics,
                        );
                    } else if center_out && leaf_group_size != 0 {
                        record_center_out_leaf_groups(
                            values,
                            leaf_group_size,
                            query,
                            &mut ranks,
                            &mut metrics,
                        );
                    } else if center_out {
                        metrics.value_distance_evaluations += values.len();
                        record_center_out_leaf(values, query, &mut ranks, &mut metrics);
                    } else if leaf_group_size != 0 {
                        // Model precomputed bounds over contiguous STR-y groups.
                        // Computing the bounds here is test-only instrumentation;
                        // the counters describe query work if they were stored.
                        if reverse {
                            for group in values.chunks(leaf_group_size).rev() {
                                record_leaf_group(group, true, query, &mut ranks, &mut metrics);
                            }
                        } else {
                            for group in values.chunks(leaf_group_size) {
                                record_leaf_group(group, false, query, &mut ranks, &mut metrics);
                            }
                        }
                    } else if reverse {
                        metrics.value_distance_evaluations += values.len();
                        for value in values.iter().rev() {
                            record_value_candidate(value, query, &mut ranks, &mut metrics);
                        }
                    } else {
                        metrics.value_distance_evaluations += values.len();
                        for value in values {
                            record_value_candidate(value, query, &mut ranks, &mut metrics);
                        }
                    }
                }
                Node::Branch(children) => {
                    metrics.branch_expansions += 1;
                    metrics.child_distance_evaluations += children.len();
                    for (bounds, child) in children {
                        let dist = bounds.comparable_min_distance_to(query);
                        if dist.total_cmp(&ranks.bound()).is_lt() {
                            metrics.child_pushes += 1;
                            frontier.push(FrontierNode { dist, node: child });
                        } else {
                            metrics.child_pruned += 1;
                        }
                    }
                }
            }
        }
        let frontier_metrics = frontier.metrics();
        metrics.frontier_pushes = frontier_metrics.pushes;
        metrics.frontier_pops = frontier_metrics.pops;
        metrics.frontier_high_water = frontier_metrics.high_water;
        metrics.rank = ranks.metrics();
        (ranks.into_values(), metrics)
    }

    fn nearest_packed_frontier_with_metrics<T: Indexable, Params: SplitParameters>(
        tree: &Rtree<T, Params>,
        query: [f64; 2],
        k: usize,
    ) -> (Vec<&T>, PackedFrontierMetrics) {
        let mut values = Vec::with_capacity(k.min(tree.len));
        let mut frontier: SearchFrontier<PackedFrontierEntry<'_, T>> = SearchFrontier::new();
        let mut metrics = PackedFrontierMetrics::default();
        frontier.push(PackedFrontierEntry {
            dist: 0.0,
            item: PackedFrontierItem::Node(&tree.root),
        });
        while values.len() < k {
            let Some(entry) = frontier.pop() else {
                break;
            };
            match entry.item {
                PackedFrontierItem::Node(Node::Branch(children)) => {
                    metrics.branch_expansions += 1;
                    frontier.extend(children.iter().map(|(bounds, child)| PackedFrontierEntry {
                        dist: bounds.comparable_min_distance_to(query),
                        item: PackedFrontierItem::Node(child),
                    }));
                }
                PackedFrontierItem::Node(Node::Leaf(leaf)) => {
                    metrics.leaf_expansions += 1;
                    if let Some(group_bounds) = leaf.packed_group_bounds() {
                        metrics.group_pushes += group_bounds.len();
                        frontier.extend(group_bounds.iter().enumerate().map(|(index, bounds)| {
                            PackedFrontierEntry {
                                dist: bounds.comparable_min_distance_to(query),
                                item: PackedFrontierItem::Group(leaf, index),
                            }
                        }));
                    } else {
                        metrics.value_pushes += leaf.len();
                        frontier.extend(leaf.values().iter().map(|value| PackedFrontierEntry {
                            dist: value.bounds().comparable_min_distance_to(query),
                            item: PackedFrontierItem::Value(value),
                        }));
                    }
                }
                PackedFrontierItem::Group(leaf, index) => {
                    metrics.group_pops += 1;
                    let group = leaf.packed_group(index);
                    metrics.value_pushes += group.len();
                    frontier.extend(group.iter().map(|value| PackedFrontierEntry {
                        dist: value.bounds().comparable_min_distance_to(query),
                        item: PackedFrontierItem::Value(value),
                    }));
                }
                PackedFrontierItem::Value(value) => {
                    metrics.value_pops += 1;
                    values.push(value);
                }
            }
        }
        let frontier_metrics = frontier.metrics();
        metrics.pushes = frontier_metrics.pushes;
        metrics.pops = frontier_metrics.pops;
        metrics.high_water = frontier_metrics.high_water;
        (values, metrics)
    }

    fn nearest_bounded_group_frontier_with_metrics<T: Indexable, Params: SplitParameters>(
        tree: &Rtree<T, Params>,
        query: [f64; 2],
        k: usize,
    ) -> (Vec<&T>, BoundedSearchMetrics) {
        if k == 0 || tree.len == 0 {
            return (Vec::new(), BoundedSearchMetrics::default());
        }
        let mut ranks = NearestBound::new(k, k.min(tree.len));
        let mut frontier: SearchFrontier<PackedFrontierEntry<'_, T>> = SearchFrontier::new();
        let mut metrics = BoundedSearchMetrics::default();
        frontier.push(PackedFrontierEntry {
            dist: 0.0,
            item: PackedFrontierItem::Node(&tree.root),
        });
        while let Some(PackedFrontierEntry { dist, item }) = frontier.pop() {
            if dist.total_cmp(&ranks.bound()).is_ge() {
                metrics.terminated_by_bound += 1;
                break;
            }
            match item {
                PackedFrontierItem::Node(Node::Branch(children)) => {
                    metrics.branch_expansions += 1;
                    metrics.child_distance_evaluations += children.len();
                    for (bounds, child) in children {
                        let dist = bounds.comparable_min_distance_to(query);
                        if dist.total_cmp(&ranks.bound()).is_lt() {
                            metrics.child_pushes += 1;
                            frontier.push(PackedFrontierEntry {
                                dist,
                                item: PackedFrontierItem::Node(child),
                            });
                        } else {
                            metrics.child_pruned += 1;
                        }
                    }
                }
                PackedFrontierItem::Node(Node::Leaf(leaf)) => {
                    metrics.leaf_expansions += 1;
                    if let Some(group_bounds) = leaf.packed_group_bounds() {
                        metrics.leaf_group_bound_evaluations += group_bounds.len();
                        for (index, bounds) in group_bounds.iter().enumerate() {
                            let dist = bounds.comparable_min_distance_to(query);
                            if dist.total_cmp(&ranks.bound()).is_lt() {
                                frontier.push(PackedFrontierEntry {
                                    dist,
                                    item: PackedFrontierItem::Group(leaf, index),
                                });
                            } else {
                                metrics.leaf_groups_pruned += 1;
                            }
                        }
                    } else {
                        metrics.value_distance_evaluations += leaf.len();
                        for value in leaf.values() {
                            record_value_candidate(value, query, &mut ranks, &mut metrics);
                        }
                    }
                }
                PackedFrontierItem::Group(leaf, index) => {
                    metrics.leaf_groups_scanned += 1;
                    let group = leaf.packed_group(index);
                    metrics.value_distance_evaluations += group.len();
                    let reverse = group
                        .first()
                        .zip(group.last())
                        .is_some_and(|(first, last)| {
                            let first_y = first.bounds().center()[1];
                            let last_y = last.bounds().center()[1];
                            (last_y - query[1]).abs() < (first_y - query[1]).abs()
                        });
                    if reverse {
                        for value in group.iter().rev() {
                            record_value_candidate(value, query, &mut ranks, &mut metrics);
                        }
                    } else {
                        for value in group {
                            record_value_candidate(value, query, &mut ranks, &mut metrics);
                        }
                    }
                }
                PackedFrontierItem::Value(_) => unreachable!("values are ranked, not queued"),
            }
        }
        let frontier_metrics = frontier.metrics();
        metrics.frontier_pushes = frontier_metrics.pushes;
        metrics.frontier_pops = frontier_metrics.pops;
        metrics.frontier_high_water = frontier_metrics.high_water;
        metrics.rank = ranks.metrics();
        (ranks.into_values(), metrics)
    }

    fn nearest_distance_ordered_groups_with_metrics<T: Indexable, Params: SplitParameters>(
        tree: &Rtree<T, Params>,
        query: [f64; 2],
        k: usize,
        group_size: usize,
    ) -> (Vec<&T>, BoundedSearchMetrics) {
        if k == 0 || tree.len == 0 {
            return (Vec::new(), BoundedSearchMetrics::default());
        }
        let mut ranks = NearestBound::new(k, k.min(tree.len));
        let mut frontier: SearchFrontier<FrontierNode<'_, T>> = SearchFrontier::new();
        let mut metrics = BoundedSearchMetrics::default();
        frontier.push(FrontierNode {
            dist: 0.0,
            node: &tree.root,
        });
        while let Some(FrontierNode { dist, node }) = frontier.pop() {
            if dist.total_cmp(&ranks.bound()).is_ge() {
                metrics.terminated_by_bound += 1;
                break;
            }
            match node {
                Node::Leaf(leaf) => {
                    metrics.leaf_expansions += 1;
                    record_distance_ordered_leaf_groups(
                        leaf.values(),
                        group_size,
                        query,
                        &mut ranks,
                        &mut metrics,
                    );
                }
                Node::Branch(children) => {
                    metrics.branch_expansions += 1;
                    metrics.child_distance_evaluations += children.len();
                    for (bounds, child) in children {
                        let dist = bounds.comparable_min_distance_to(query);
                        if dist.total_cmp(&ranks.bound()).is_lt() {
                            metrics.child_pushes += 1;
                            frontier.push(FrontierNode { dist, node: child });
                        } else {
                            metrics.child_pruned += 1;
                        }
                    }
                }
            }
        }
        let frontier_metrics = frontier.metrics();
        metrics.frontier_pushes = frontier_metrics.pushes;
        metrics.frontier_pops = frontier_metrics.pops;
        metrics.frontier_high_water = frontier_metrics.high_water;
        metrics.rank = ranks.metrics();
        (ranks.into_values(), metrics)
    }

    fn record_distance_ordered_leaf_groups<'a, T: Indexable>(
        values: &'a [T],
        group_size: usize,
        query: [f64; 2],
        ranks: &mut NearestBound<&'a T>,
        metrics: &mut BoundedSearchMetrics,
    ) {
        let mut ordered: Vec<(f64, usize)> = values
            .chunks(group_size)
            .enumerate()
            .map(|(index, group)| {
                let bounds = group
                    .iter()
                    .map(Indexable::bounds)
                    .reduce(|a, b| a.union(&b))
                    .expect("chunks are non-empty");
                metrics.leaf_group_bound_evaluations += 1;
                (bounds.comparable_min_distance_to(query), index)
            })
            .collect();
        ordered.sort_unstable_by(|a, b| {
            metrics.leaf_group_order_comparisons += 1;
            a.0.total_cmp(&b.0)
        });
        for (group_dist, group_index) in ordered {
            if group_dist.total_cmp(&ranks.bound()).is_ge() {
                metrics.leaf_groups_pruned += 1;
                continue;
            }
            metrics.leaf_groups_scanned += 1;
            let start = group_index * group_size;
            let group = &values[start..(start + group_size).min(values.len())];
            metrics.value_distance_evaluations += group.len();
            let reverse = group
                .first()
                .zip(group.last())
                .is_some_and(|(first, last)| {
                    let first_y = first.bounds().center()[1];
                    let last_y = last.bounds().center()[1];
                    (last_y - query[1]).abs() < (first_y - query[1]).abs()
                });
            if reverse {
                for value in group.iter().rev() {
                    record_value_candidate(value, query, ranks, metrics);
                }
            } else {
                for value in group {
                    record_value_candidate(value, query, ranks, metrics);
                }
            }
        }
    }

    fn nearest_depth_first_with_metrics<T: Indexable, Params: SplitParameters>(
        tree: &Rtree<T, Params>,
        query: [f64; 2],
        k: usize,
    ) -> (Vec<&T>, BoundedSearchMetrics) {
        if k == 0 || tree.len == 0 {
            return (Vec::new(), BoundedSearchMetrics::default());
        }
        let mut ranks = NearestBound::new(k, k.min(tree.len));
        let mut metrics = BoundedSearchMetrics::default();
        record_depth_first_node(&tree.root, query, true, &mut ranks, &mut metrics);
        metrics.rank = ranks.metrics();
        (ranks.into_values(), metrics)
    }

    fn record_depth_first_node<'a, T: Indexable>(
        node: &'a Node<T>,
        query: [f64; 2],
        nearer_y_end_first: bool,
        ranks: &mut NearestBound<&'a T>,
        metrics: &mut BoundedSearchMetrics,
    ) {
        match node {
            Node::Leaf(leaf) => {
                let values = leaf.values();
                metrics.leaf_expansions += 1;
                metrics.value_distance_evaluations += values.len();
                let reverse = nearer_y_end_first
                    && values
                        .first()
                        .zip(values.last())
                        .is_some_and(|(first, last)| {
                            let first_y = first.bounds().center()[1];
                            let last_y = last.bounds().center()[1];
                            (last_y - query[1]).abs() < (first_y - query[1]).abs()
                        });
                metrics.reversed_leaf_scans += usize::from(reverse);
                if reverse {
                    for value in values.iter().rev() {
                        record_value_candidate(value, query, ranks, metrics);
                    }
                } else {
                    for value in values {
                        record_value_candidate(value, query, ranks, metrics);
                    }
                }
            }
            Node::Branch(children) => {
                metrics.branch_expansions += 1;
                metrics.child_distance_evaluations += children.len();
                let mut ordered: Vec<FrontierNode<'_, T>> = children
                    .iter()
                    .map(|(bounds, child)| FrontierNode {
                        dist: bounds.comparable_min_distance_to(query),
                        node: child,
                    })
                    .collect();
                ordered.sort_unstable_by(|a, b| {
                    metrics.child_order_comparisons += 1;
                    a.dist.total_cmp(&b.dist)
                });
                for (index, FrontierNode { dist, node }) in ordered.iter().enumerate() {
                    if dist.total_cmp(&ranks.bound()).is_ge() {
                        metrics.child_pruned += ordered.len() - index;
                        break;
                    }
                    metrics.child_pushes += 1;
                    record_depth_first_node(node, query, nearer_y_end_first, ranks, metrics);
                }
            }
        }
    }

    fn record_center_out_leaf_groups<'a, T: Indexable>(
        values: &'a [T],
        group_size: usize,
        query: [f64; 2],
        ranks: &mut NearestBound<&'a T>,
        metrics: &mut BoundedSearchMetrics,
    ) {
        let group_bounds: Vec<Bounds> = values
            .chunks(group_size)
            .map(|group| {
                group
                    .iter()
                    .map(Indexable::bounds)
                    .reduce(|a, b| a.union(&b))
                    .expect("chunks are non-empty")
            })
            .collect();
        let mut upper = group_bounds.partition_point(|bounds| bounds.center()[1] < query[1]);
        let mut lower = upper;
        while lower != 0 || upper != group_bounds.len() {
            let take_lower = if lower == 0 {
                false
            } else if upper == group_bounds.len() {
                true
            } else {
                let lower_y = group_bounds[lower - 1].center()[1];
                let upper_y = group_bounds[upper].center()[1];
                (query[1] - lower_y).abs() <= (upper_y - query[1]).abs()
            };
            let group_index = if take_lower {
                lower -= 1;
                lower
            } else {
                let group_index = upper;
                upper += 1;
                group_index
            };
            metrics.leaf_group_bound_evaluations += 1;
            let group_dist = group_bounds[group_index].comparable_min_distance_to(query);
            if group_dist.total_cmp(&ranks.bound()).is_ge() {
                metrics.leaf_groups_pruned += 1;
                continue;
            }
            metrics.leaf_groups_scanned += 1;
            let start = group_index * group_size;
            let end = (start + group_size).min(values.len());
            let group = &values[start..end];
            metrics.value_distance_evaluations += group.len();
            record_center_out_leaf(group, query, ranks, metrics);
        }
    }

    fn record_center_out_leaf<'a, T: Indexable>(
        values: &'a [T],
        query: [f64; 2],
        ranks: &mut NearestBound<&'a T>,
        metrics: &mut BoundedSearchMetrics,
    ) {
        let mut upper = values.partition_point(|value| value.bounds().center()[1] < query[1]);
        let mut lower = upper;
        while lower != 0 || upper != values.len() {
            let take_lower = if lower == 0 {
                false
            } else if upper == values.len() {
                true
            } else {
                let lower_y = values[lower - 1].bounds().center()[1];
                let upper_y = values[upper].bounds().center()[1];
                (query[1] - lower_y).abs() <= (upper_y - query[1]).abs()
            };
            let value = if take_lower {
                lower -= 1;
                &values[lower]
            } else {
                let value = &values[upper];
                upper += 1;
                value
            };
            record_value_candidate(value, query, ranks, metrics);
        }
    }

    fn record_leaf_bvh<'a, T: Indexable>(
        values: &'a [T],
        terminal_size: usize,
        query: [f64; 2],
        ranks: &mut NearestBound<&'a T>,
        metrics: &mut BoundedSearchMetrics,
    ) {
        if values.len() <= terminal_size {
            metrics.leaf_groups_scanned += 1;
            metrics.value_distance_evaluations += values.len();
            let reverse = values
                .first()
                .zip(values.last())
                .is_some_and(|(first, last)| {
                    let first_y = first.bounds().center()[1];
                    let last_y = last.bounds().center()[1];
                    (last_y - query[1]).abs() < (first_y - query[1]).abs()
                });
            if reverse {
                for value in values.iter().rev() {
                    record_value_candidate(value, query, ranks, metrics);
                }
            } else {
                for value in values {
                    record_value_candidate(value, query, ranks, metrics);
                }
            }
            return;
        }

        let middle = values.len() / 2;
        let (lower, upper) = values.split_at(middle);
        let child_bounds = [lower, upper].map(|child| {
            child
                .iter()
                .map(Indexable::bounds)
                .reduce(|a, b| a.union(&b))
                .expect("BVH children are non-empty")
        });
        metrics.leaf_group_bound_evaluations += 2;
        metrics.leaf_group_order_comparisons += 1;
        let distances = child_bounds.map(|bounds| bounds.comparable_min_distance_to(query));
        let order = if distances[0].total_cmp(&distances[1]).is_le() {
            [0, 1]
        } else {
            [1, 0]
        };
        let children = [lower, upper];
        for index in order {
            if distances[index].total_cmp(&ranks.bound()).is_lt() {
                record_leaf_bvh(children[index], terminal_size, query, ranks, metrics);
            } else {
                metrics.leaf_groups_pruned += 1;
            }
        }
    }

    fn record_leaf_group<'a, T: Indexable>(
        group: &'a [T],
        reverse: bool,
        query: [f64; 2],
        ranks: &mut NearestBound<&'a T>,
        metrics: &mut BoundedSearchMetrics,
    ) {
        metrics.leaf_group_bound_evaluations += 1;
        let bounds = group
            .iter()
            .map(Indexable::bounds)
            .reduce(|a, b| a.union(&b))
            .expect("chunks are non-empty");
        let group_dist = bounds.comparable_min_distance_to(query);
        if group_dist.total_cmp(&ranks.bound()).is_ge() {
            metrics.leaf_groups_pruned += 1;
            return;
        }
        metrics.leaf_groups_scanned += 1;
        metrics.value_distance_evaluations += group.len();
        if reverse {
            for value in group.iter().rev() {
                record_value_candidate(value, query, ranks, metrics);
            }
        } else {
            for value in group {
                record_value_candidate(value, query, ranks, metrics);
            }
        }
    }

    fn record_value_candidate<'a, T: Indexable>(
        value: &'a T,
        query: [f64; 2],
        ranks: &mut NearestBound<&'a T>,
        metrics: &mut BoundedSearchMetrics,
    ) {
        let dist = value.bounds().comparable_min_distance_to(query);
        if dist.total_cmp(&ranks.bound()).is_lt() {
            metrics.value_bound_passes += 1;
            ranks.admit_better(dist, value);
        } else {
            metrics.value_bound_rejections += 1;
        }
    }

    #[test]
    fn empty_tree() {
        let t: Rtree<P> = Rtree::new();
        assert!(t.is_empty());
        assert_eq!(t.len(), 0);
    }

    #[test]
    fn private_metric_helpers_handle_empty_queries_and_small_integer_roots() {
        assert_eq!(isqrt_ceil(0), 0);
        assert_eq!(isqrt_ceil(1), 1);
        assert_eq!(isqrt_ceil(2), 2);
        assert_eq!(isqrt_ceil(4), 2);

        let values = vec![P::new(0.0, 0.0), P::new(1.0, 1.0)];
        assert_eq!(LeafProbe::packed_group(&values, 0).len(), 2);

        let first = PackedFrontierEntry {
            dist: 1.0,
            item: PackedFrontierItem::Group(&values, 0),
        };
        let equal = PackedFrontierEntry {
            dist: 1.0,
            item: PackedFrontierItem::Value(&values[0]),
        };
        let farther = PackedFrontierEntry {
            dist: 2.0,
            item: PackedFrontierItem::Value(&values[1]),
        };
        assert!(first == equal);
        assert!(first.partial_cmp(&farther).is_some());

        let ordered_values: Vec<P> = (0..12).map(|x| P::new(f64::from(x), 0.0)).collect();
        let mut ranks = NearestBound::new(1, 1);
        let mut metrics = BoundedSearchMetrics::default();
        record_distance_ordered_leaf_groups(
            &ordered_values,
            2,
            [0.0, 0.0],
            &mut ranks,
            &mut metrics,
        );
        assert!(metrics.leaf_group_order_comparisons > 0);
        assert!(metrics.leaf_groups_pruned > 0);

        let tree = Rtree::<P>::new();
        assert!(
            nearest_with_metrics(&tree, [0.0, 0.0], 0, false, 8, false, 8)
                .0
                .is_empty()
        );
        assert!(
            nearest_packed_frontier_with_metrics(&tree, [0.0, 0.0], 1)
                .0
                .is_empty()
        );
        assert!(
            nearest_bounded_group_frontier_with_metrics(&tree, [0.0, 0.0], 0)
                .0
                .is_empty()
        );
        assert!(
            nearest_distance_ordered_groups_with_metrics(&tree, [0.0, 0.0], 0, 8)
                .0
                .is_empty()
        );
        assert!(
            nearest_depth_first_with_metrics(&tree, [0.0, 0.0], 0)
                .0
                .is_empty()
        );
    }

    /// `Default` builds the same empty tree as `new()`.
    #[test]
    fn default_tree_is_empty() {
        let t: Rtree<P> = Rtree::default();
        assert!(t.is_empty());
        assert_eq!(t.len(), 0);
    }

    /// `FrontierNode` equality is keyed on the distance (total order),
    /// not on the node identity.
    #[test]
    fn frontier_node_eq_compares_distance() {
        let mut t: Rtree<P> = Rtree::new();
        t.insert(P::new(0.0, 0.0));
        let a = FrontierNode {
            dist: 1.5,
            node: &t.root,
        };
        let b = FrontierNode {
            dist: 1.5,
            node: &t.root,
        };
        let c = FrontierNode {
            dist: 2.5,
            node: &t.root,
        };
        assert!(a == b);
        assert!(a != c);
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

    fn uniform_points(n: usize) -> Vec<P> {
        let mut lcg = Lcg::new();
        (0..n)
            .map(|_| {
                let x = lcg.next_f64() * 50_000.0;
                let y = lcg.next_f64() * 50_000.0;
                P::new(x, y)
            })
            .collect()
    }

    fn clustered_points(n: usize) -> Vec<P> {
        const CLUSTER_COUNT: usize = 16;
        const CLUSTER_RADIUS: f64 = 100.0;
        const FIELD: f64 = 50_000.0;

        let mut lcg = Lcg::new();
        let centers: Vec<[f64; 2]> = (0..CLUSTER_COUNT)
            .map(|_| [lcg.next_f64() * FIELD, lcg.next_f64() * FIELD])
            .collect();
        (0..n)
            .map(|i| {
                let center = centers[i % CLUSTER_COUNT];
                P::new(
                    center[0] + lcg.next_f64() * 2.0 * CLUSTER_RADIUS - CLUSTER_RADIUS,
                    center[1] + lcg.next_f64() * 2.0 * CLUSTER_RADIUS - CLUSTER_RADIUS,
                )
            })
            .collect()
    }

    fn profile_queries(q: usize) -> Vec<[f64; 2]> {
        let mut lcg = Lcg::new();
        (0..q)
            .map(|_| {
                let x = lcg.next_f64() * 50_000.0;
                lcg.next_f64();
                let y = lcg.next_f64() * 50_000.0;
                [x, y]
            })
            .collect()
    }

    fn report_bounded_metrics(
        construction: &str,
        distribution: &str,
        leaf_order: &str,
        expected_results: usize,
        total: &BoundedSearchMetrics,
    ) {
        eprintln!(
            "[rtree-bounded-shape] construction={construction} distribution={distribution} leaf_order={leaf_order} expected_results={expected_results} frontier_pushes={} frontier_pops={} frontier_high_water={} terminated_by_bound={} branch_expansions={} child_distance_evaluations={} child_order_comparisons={} child_pushes={} child_pruned={} leaf_expansions={} reversed_leaf_scans={} leaf_group_bound_evaluations={} leaf_group_order_comparisons={} leaf_groups_scanned={} leaf_groups_pruned={} value_distance_evaluations={} value_bound_passes={} value_bound_rejections={} rank_calls={} rank_partition_comparisons={} rank_admissions={} rank_replacements={} rank_shifted_ranks={}",
            total.frontier_pushes,
            total.frontier_pops,
            total.frontier_high_water,
            total.terminated_by_bound,
            total.branch_expansions,
            total.child_distance_evaluations,
            total.child_order_comparisons,
            total.child_pushes,
            total.child_pruned,
            total.leaf_expansions,
            total.reversed_leaf_scans,
            total.leaf_group_bound_evaluations,
            total.leaf_group_order_comparisons,
            total.leaf_groups_scanned,
            total.leaf_groups_pruned,
            total.value_distance_evaluations,
            total.value_bound_passes,
            total.value_bound_rejections,
            total.rank.calls,
            total.rank.partition_comparisons,
            total.rank.admissions,
            total.rank.replacements,
            total.rank.shifted_ranks,
        );
    }

    #[test]
    fn records_bounded_search_shape() {
        const N: usize = 50_000;
        const Q: usize = 100;
        const K: usize = 8;

        for (construction, distribution, points) in [
            ("bulk", "uniform", uniform_points(N)),
            ("bulk", "clustered", clustered_points(N)),
            ("inserted", "uniform", uniform_points(N)),
            ("inserted", "clustered", clustered_points(N)),
        ] {
            let tree: Rtree<P> = if construction == "bulk" {
                points.into_iter().collect()
            } else {
                let mut tree = Rtree::new();
                for point in points {
                    tree.insert(point);
                }
                tree
            };
            let mut forward_total = BoundedSearchMetrics::default();
            let mut nearer_y_total = BoundedSearchMetrics::default();
            for query in profile_queries(Q) {
                let expected = tree.nearest(query, K);
                let (forward, metrics) = nearest_with_metrics(&tree, query, K, false, 0, false, 0);
                assert_eq!(forward, expected);
                forward_total.add(&metrics);
                let (nearer_y, metrics) = nearest_with_metrics(&tree, query, K, true, 0, false, 0);
                assert_eq!(nearer_y, expected);
                nearer_y_total.add(&metrics);
            }
            report_bounded_metrics(construction, distribution, "forward", Q * K, &forward_total);
            report_bounded_metrics(
                construction,
                distribution,
                "nearer-y-end",
                Q * K,
                &nearer_y_total,
            );
        }
    }

    fn record_inserted_parameter_shape<Params: SplitParameters>(
        parameters: &str,
        distribution: &str,
        points: &[P],
    ) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree = insert_built::<Params>(points);
        let mut total = BoundedSearchMetrics::default();
        for query in profile_queries(Q) {
            let (_, metrics) = nearest_with_metrics(&tree, query, K, false, 0, false, 0);
            total.add(&metrics);
        }
        report_bounded_metrics(parameters, distribution, "forward", Q * K, &total);
    }

    fn record_bulk_parameter_shape<Params: SplitParameters>(
        parameters: &str,
        distribution: &str,
        points: &[P],
    ) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree: Rtree<P, Params> = points.iter().copied().collect();
        let mut total = BoundedSearchMetrics::default();
        for query in profile_queries(Q) {
            let expected = tree.nearest(query, K);
            let (observed, metrics) = nearest_with_metrics(&tree, query, K, true, 0, false, 0);
            assert_eq!(observed, expected);
            total.add(&metrics);
        }
        report_bounded_metrics(parameters, distribution, "nearer-y-end", Q * K, &total);
    }

    fn record_bulk_group_shape(group_size: usize, distribution: &str, points: &[P]) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree: Rtree<P> = points.iter().copied().collect();
        let mut total = BoundedSearchMetrics::default();
        for query in profile_queries(Q) {
            let expected = tree.nearest(query, K);
            let (observed, metrics) =
                nearest_with_metrics(&tree, query, K, true, group_size, false, 0);
            assert_eq!(observed, expected);
            total.add(&metrics);
        }
        report_bounded_metrics(
            &alloc::format!("bulk-group{group_size}"),
            distribution,
            "nearer-y-end",
            Q * K,
            &total,
        );
    }

    fn record_bulk_center_out_shape(distribution: &str, points: &[P]) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree: Rtree<P> = points.iter().copied().collect();
        let mut total = BoundedSearchMetrics::default();
        for query in profile_queries(Q) {
            let expected = tree.nearest(query, K);
            let (observed, metrics) = nearest_with_metrics(&tree, query, K, false, 0, true, 0);
            assert_eq!(observed, expected);
            total.add(&metrics);
        }
        report_bounded_metrics("bulk-center-out", distribution, "center-out", Q * K, &total);
    }

    fn record_bulk_center_out_group_shape(group_size: usize, distribution: &str, points: &[P]) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree: Rtree<P> = points.iter().copied().collect();
        let mut total = BoundedSearchMetrics::default();
        for query in profile_queries(Q) {
            let expected = tree.nearest(query, K);
            let (observed, metrics) =
                nearest_with_metrics(&tree, query, K, false, group_size, true, 0);
            assert_eq!(observed, expected);
            total.add(&metrics);
        }
        report_bounded_metrics(
            &alloc::format!("bulk-center-group{group_size}"),
            distribution,
            "center-out-groups",
            Q * K,
            &total,
        );
    }

    fn record_bulk_depth_first_shape(distribution: &str, points: &[P]) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree: Rtree<P> = points.iter().copied().collect();
        let mut total = BoundedSearchMetrics::default();
        for query in profile_queries(Q) {
            let expected = tree.nearest(query, K);
            let (observed, metrics) = nearest_depth_first_with_metrics(&tree, query, K);
            assert_eq!(observed, expected);
            total.add(&metrics);
        }
        report_bounded_metrics(
            "bulk-depth-first",
            distribution,
            "nearer-y-end",
            Q * K,
            &total,
        );
    }

    fn record_bulk_distance_group_shape(group_size: usize, distribution: &str, points: &[P]) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree: Rtree<P> = points.iter().copied().collect();
        let mut total = BoundedSearchMetrics::default();
        for query in profile_queries(Q) {
            let expected = tree.nearest(query, K);
            let (observed, metrics) =
                nearest_distance_ordered_groups_with_metrics(&tree, query, K, group_size);
            assert_eq!(observed, expected);
            total.add(&metrics);
        }
        report_bounded_metrics(
            &alloc::format!("bulk-distance-group{group_size}"),
            distribution,
            "distance-ordered-groups",
            Q * K,
            &total,
        );
    }

    fn record_bulk_packed_frontier_shape(distribution: &str, points: &[P]) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree: Rtree<P> = points.iter().copied().collect();
        let mut total = PackedFrontierMetrics::default();
        for query in profile_queries(Q) {
            let expected = tree.nearest(query, K);
            let (observed, metrics) = nearest_packed_frontier_with_metrics(&tree, query, K);
            assert_eq!(observed, expected);
            total.add(&metrics);
        }
        eprintln!(
            "[rtree-packed-frontier] distribution={distribution} expected_results={} pushes={} pops={} high_water={} branch_expansions={} leaf_expansions={} group_pushes={} group_pops={} value_pushes={} value_pops={}",
            Q * K,
            total.pushes,
            total.pops,
            total.high_water,
            total.branch_expansions,
            total.leaf_expansions,
            total.group_pushes,
            total.group_pops,
            total.value_pushes,
            total.value_pops,
        );
    }

    fn record_bulk_bounded_group_frontier_shape(distribution: &str, points: &[P]) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree: Rtree<P> = points.iter().copied().collect();
        let mut total = BoundedSearchMetrics::default();
        for query in profile_queries(Q) {
            let expected = tree.nearest(query, K);
            let (observed, metrics) = nearest_bounded_group_frontier_with_metrics(&tree, query, K);
            assert_eq!(observed, expected);
            total.add(&metrics);
        }
        report_bounded_metrics(
            "bulk-group-frontier",
            distribution,
            "bounded-group-frontier",
            Q * K,
            &total,
        );
    }

    fn record_bulk_leaf_bvh_shape(terminal_size: usize, distribution: &str, points: &[P]) {
        const Q: usize = 100;
        const K: usize = 8;

        let tree: Rtree<P> = points.iter().copied().collect();
        let mut total = BoundedSearchMetrics::default();
        for query in profile_queries(Q) {
            let expected = tree.nearest(query, K);
            let (observed, metrics) =
                nearest_with_metrics(&tree, query, K, false, 0, false, terminal_size);
            assert_eq!(observed, expected);
            total.add(&metrics);
        }
        report_bounded_metrics(
            &alloc::format!("bulk-leaf-bvh{terminal_size}"),
            distribution,
            "distance-ordered-bvh",
            Q * K,
            &total,
        );
    }

    #[test]
    fn records_bulk_leaf_bvh_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            for terminal_size in [2, 4, 8] {
                record_bulk_leaf_bvh_shape(terminal_size, distribution, &points);
            }
        }
    }

    #[test]
    fn records_bulk_packed_frontier_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            record_bulk_packed_frontier_shape(distribution, &points);
        }
    }

    #[test]
    fn records_bulk_bounded_group_frontier_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            record_bulk_bounded_group_frontier_shape(distribution, &points);
        }
    }

    #[test]
    fn records_bulk_bounded_distance_group_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            for group_size in [4, 8] {
                record_bulk_distance_group_shape(group_size, distribution, &points);
            }
        }
    }

    #[test]
    fn records_bulk_bounded_depth_first_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            record_bulk_depth_first_shape(distribution, &points);
        }
    }

    #[test]
    fn records_bulk_bounded_center_out_group_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            for group_size in [2, 4, 8] {
                record_bulk_center_out_group_shape(group_size, distribution, &points);
            }
        }
    }

    #[test]
    fn records_bulk_bounded_center_out_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            record_bulk_center_out_shape(distribution, &points);
        }
    }

    #[test]
    fn records_bulk_bounded_group_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            for group_size in [2, 4, 8, 16] {
                record_bulk_group_shape(group_size, distribution, &points);
            }
        }
    }

    #[test]
    fn records_bulk_bounded_parameter_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            record_bulk_parameter_shape::<AsymmetricRStarSplit<4, 2, 4, 2>>(
                "bulk-b4-l4",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<6, 2, 4, 2>>(
                "bulk-b6-l4",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<8, 3, 4, 2>>(
                "bulk-b8-l4",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<12, 4, 4, 2>>(
                "bulk-b12-l4",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<6, 2, 6, 2>>(
                "bulk-b6-l6",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<4, 2, 8, 3>>(
                "bulk-b4-l8",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<6, 2, 8, 3>>(
                "bulk-b6-l8",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<8, 3, 8, 3>>(
                "bulk-b8-l8",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<8, 3, 12, 4>>(
                "bulk-b8-l12",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<8, 3, 16, 4>>(
                "bulk-b8-l16",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<8, 3, 24, 7>>(
                "bulk-b8-l24",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<8, 3, 32, 9>>(
                "bulk-b8-l32",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<12, 4, 8, 3>>(
                "bulk-b12-l8",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<16, 4, 8, 3>>(
                "bulk-b16-l8",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<32, 9, 8, 3>>(
                "bulk-b32-l8",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<12, 4, 16, 4>>(
                "bulk-b12-l16",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<16, 4, 16, 4>>(
                "bulk-b16-l16",
                distribution,
                &points,
            );
            record_bulk_parameter_shape::<AsymmetricRStarSplit<32, 9, 16, 4>>(
                "bulk-b32-l16",
                distribution,
                &points,
            );
        }
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn records_inserted_bounded_parameter_shape() {
        const N: usize = 50_000;

        for (distribution, points) in [
            ("uniform", uniform_points(N)),
            ("clustered", clustered_points(N)),
        ] {
            record_inserted_parameter_shape::<AsymmetricRStarSplit<4, 2, 4, 2>>(
                "inserted-rstar-split-a4-4",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<4, 2, 8, 3>>(
                "inserted-rstar-split-a4-8",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<4, 2, 16, 4>>(
                "inserted-rstar-split-a4-16",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<4, 2, 32, 9>>(
                "inserted-rstar-split-a4-32",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<6, 2, 8, 3>>(
                "inserted-rstar-split-a6-8",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<6, 2, 10, 3>>(
                "inserted-rstar-split-a6-10",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<6, 2, 12, 4>>(
                "inserted-rstar-split-a6-12",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<6, 2, 14, 4>>(
                "inserted-rstar-split-a6-14",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<6, 2, 16, 4>>(
                "inserted-rstar-split-a6-16",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<6, 2, 32, 9>>(
                "inserted-rstar-split-a6-32",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<8, 3, 8, 3>>(
                "inserted-rstar-split-a8-8",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<8, 3, 10, 3>>(
                "inserted-rstar-split-a8-10",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<8, 3, 12, 4>>(
                "inserted-rstar-split-a8-12",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<8, 3, 16, 4>>(
                "inserted-rstar-split-a8-16",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<12, 4, 16, 4>>(
                "inserted-rstar-split-a12-16",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<12, 4, 32, 9>>(
                "inserted-rstar-split-a12-32",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<Quadratic<6, 2>>(
                "inserted-q6-6",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<Quadratic<8, 3>>(
                "inserted-q8-8",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<Quadratic<16, 4>>(
                "inserted-q16-16",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<Quadratic<32, 9>>(
                "inserted-q32-32",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricQuadratic<8, 3, 16, 4>>(
                "inserted-a8-16",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricQuadratic<8, 3, 32, 9>>(
                "inserted-a8-32",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricQuadratic<12, 4, 32, 9>>(
                "inserted-a12-32",
                distribution,
                &points,
            );
            record_inserted_parameter_shape::<AsymmetricRStarSplit<8, 3, 32, 9>>(
                "inserted-rstar-split-a8-32",
                distribution,
                &points,
            );
        }
    }

    fn insert_built<Params: SplitParameters>(points: &[P]) -> Rtree<P, Params> {
        let mut tree: Rtree<P, Params> = Rtree::new();
        for p in points {
            tree.insert(*p);
        }
        tree
    }

    fn checked_subtree_union(node: &Node<P>) -> Bounds {
        match node {
            Node::Leaf(leaf) => union_all(
                &leaf
                    .values()
                    .iter()
                    .map(Indexable::bounds)
                    .collect::<Vec<_>>(),
            ),
            Node::Branch(children) => {
                for (b, child) in children {
                    assert_eq!(*b, checked_subtree_union(child));
                }
                union_all(&children.iter().map(|(b, _)| *b).collect::<Vec<_>>())
            }
        }
    }

    fn assert_fill_and_depth<Params: SplitParameters>(
        tree: &Rtree<P, Params>,
        inserted_len: usize,
    ) {
        fn walk<Params: SplitParameters>(
            node: &Node<P>,
            depth: usize,
            leaf_depths: &mut Vec<usize>,
        ) {
            match node {
                Node::Leaf(leaf) => {
                    assert!(leaf.len() <= Params::LEAF_MAX);
                    leaf_depths.push(depth);
                }
                Node::Branch(children) => {
                    assert!(children.len() <= Params::BRANCH_MAX);
                    for (_, child) in children {
                        walk::<Params>(child, depth + 1, leaf_depths);
                    }
                }
            }
        }
        let mut leaf_depths = Vec::new();
        walk::<Params>(&tree.root, 1, &mut leaf_depths);
        assert!(leaf_depths.iter().all(|&d| d == leaf_depths[0]));
        assert_eq!(tree.height(), leaf_depths[0]);
        assert_eq!(tree.height(), tree.root.height());
        assert_eq!(tree.root.value_count(), tree.len());
        assert_eq!(tree.len(), inserted_len);
    }

    fn adversarial_bulk_inputs() -> [Vec<P>; 4] {
        let sorted_by_x: Vec<P> = (0..5_000i32)
            .map(|i| P::new(f64::from(i), f64::from(i % 71)))
            .collect();
        let reverse_sorted_by_x: Vec<P> = sorted_by_x.iter().copied().rev().collect();
        let one_point: Vec<P> = core::iter::repeat_n(P::new(123.0, 456.0), 5_000).collect();
        let vertical_line: Vec<P> = (0..5_000i32).map(|i| P::new(7.0, f64::from(i))).collect();
        [sorted_by_x, reverse_sorted_by_x, one_point, vertical_line]
    }

    fn adversarial_str_invariant_case<Params: SplitParameters>() {
        for points in adversarial_bulk_inputs() {
            let bulk: Rtree<P, Params> = points.clone().into_iter().collect();
            checked_subtree_union(&bulk.root);
            assert_fill_and_depth(&bulk, points.len());
        }
    }

    #[test]
    fn invariants_hold_on_adversarial_bulk_inputs_max6() {
        adversarial_str_invariant_case::<Quadratic<6, 2>>();
    }

    #[test]
    fn invariants_hold_on_adversarial_bulk_inputs_max8() {
        adversarial_str_invariant_case::<Quadratic<8, 3>>();
    }

    #[test]
    fn invariants_hold_on_adversarial_bulk_inputs_max16() {
        adversarial_str_invariant_case::<Quadratic<16, 4>>();
    }

    #[test]
    fn invariants_hold_on_adversarial_bulk_inputs_max32() {
        adversarial_str_invariant_case::<Quadratic<32, 9>>();
    }

    fn structural_invariant_case<Params: SplitParameters>() {
        let points = uniform_points(10_000);
        let tree = insert_built::<Params>(&points);
        checked_subtree_union(&tree.root);
        assert_fill_and_depth(&tree, points.len());
        let bulk: Rtree<P, Params> = points.clone().into_iter().collect();
        checked_subtree_union(&bulk.root);
        assert_fill_and_depth(&bulk, points.len());
    }

    #[test]
    fn invariant_bounds_fill_and_depth_max6() {
        structural_invariant_case::<Quadratic<6, 2>>();
    }

    #[test]
    fn invariant_bounds_fill_and_depth_max8() {
        structural_invariant_case::<Quadratic<8, 3>>();
    }

    #[test]
    fn invariant_bounds_fill_and_depth_max16() {
        structural_invariant_case::<Quadratic<16, 4>>();
    }

    #[test]
    fn invariant_bounds_fill_and_depth_max32() {
        structural_invariant_case::<Quadratic<32, 9>>();
    }

    #[test]
    fn invariant_bounds_fill_and_depth_asymmetric_8_32() {
        structural_invariant_case::<AsymmetricQuadratic<8, 3, 32, 9>>();
    }

    #[test]
    fn query_of_an_exact_leaf_box_matches_scan() {
        fn collect_leaf_boxes(node: &Node<P>, boxes: &mut Vec<Bounds>) {
            match node {
                Node::Leaf(values) => boxes.push(
                    values
                        .iter()
                        .map(Indexable::bounds)
                        .reduce(|left, right| left.union(&right))
                        .expect("bulk leaves are non-empty"),
                ),
                Node::Branch(children) => {
                    for (_, child) in children {
                        collect_leaf_boxes(child, boxes);
                    }
                }
            }
        }

        let points = uniform_points(40);
        let tree: Rtree<P> = points.clone().into_iter().collect();
        let mut leaf_boxes = Vec::new();
        collect_leaf_boxes(&tree.root, &mut leaf_boxes);
        for leaf_box in leaf_boxes {
            let mut expected: Vec<[f64; 2]> = points
                .iter()
                .filter(|p| {
                    p.get::<0>() >= leaf_box.min[0]
                        && p.get::<0>() <= leaf_box.max[0]
                        && p.get::<1>() >= leaf_box.min[1]
                        && p.get::<1>() <= leaf_box.max[1]
                })
                .map(|p| [p.get::<0>(), p.get::<1>()])
                .collect();
            expected.sort_by(coordinate_order);
            for predicate in [Predicate::Intersects(leaf_box), Predicate::Within(leaf_box)] {
                let mut got: Vec<[f64; 2]> = tree
                    .query(predicate)
                    .iter()
                    .map(|p| [p.get::<0>(), p.get::<1>()])
                    .collect();
                got.sort_by(coordinate_order);
                assert_eq!(
                    got, expected,
                    "query of an exact leaf box diverges from the scan for {predicate:?}"
                );
            }
        }
    }

    fn coordinate_order(a: &[f64; 2], b: &[f64; 2]) -> core::cmp::Ordering {
        a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1]))
    }

    #[test]
    fn bulk_load_balances() {
        let points: Vec<P> = (0..10_000)
            .map(|i| P::new(f64::from(i % 100), f64::from(i / 100)))
            .collect();
        let t: Rtree<P> = points.into_iter().collect();
        assert_eq!(t.len(), 10_000);
        // Four-way bulk packing needs seven levels to cover 10k values
        // (4^6 values beneath a height-7 root).
        assert_eq!(t.height(), 7);
    }
}
