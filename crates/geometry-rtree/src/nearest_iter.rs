//! Unbounded nearest-first streaming — the search behind
//! [`Rtree::nearest_iter`](crate::rtree::Rtree::nearest_iter).
//!
//! A best-first stream over this crate's
//! `SearchFrontier`: separate node and value frontiers keep each entry
//! to a distance plus one reference. Each [`next`](Iterator::next)
//! expands nodes until the closest pending value is no farther than the
//! closest unexpanded node, then yields that value. No best-k
//! pruning happens because no `k` exists; the bounded
//! [`Rtree::nearest`](crate::rtree::Rtree::nearest) keeps the pruned
//! path for the fixed-k case.

use core::iter::FusedIterator;

#[cfg(test)]
use alloc::vec::Vec;

use crate::indexable::Indexable;
use crate::node::Node;
use crate::search_frontier::SearchFrontier;

#[cfg(test)]
use crate::search_frontier::FrontierMetrics;

/// Default inline entry capacity of a nearest iterator's node frontier.
pub const DEFAULT_NODE_INLINE_CAPACITY: usize = 32;

/// Default inline entry capacity of a nearest iterator's value frontier.
pub const DEFAULT_VALUE_INLINE_CAPACITY: usize = 64;

/// A lazy iterator over ALL values in the tree in exact non-decreasing
/// distance order from a query point — an unbounded ordered stream.
///
/// Created by [`Rtree::nearest_iter`](crate::rtree::Rtree::nearest_iter).
/// The consumer supplies its own bound via
/// [`take`](Iterator::take); consuming to exhaustion drains the whole
/// tree in distance order. The default inline capacities can be
/// overridden through
/// [`Rtree::nearest_iter_with_inline_capacities`](crate::rtree::Rtree::nearest_iter_with_inline_capacities)
/// when caller-specific measurements justify the stack/spill trade-off.
pub struct NearestIter<
    'a,
    T,
    const NODE_INLINE_CAPACITY: usize = DEFAULT_NODE_INLINE_CAPACITY,
    const VALUE_INLINE_CAPACITY: usize = DEFAULT_VALUE_INLINE_CAPACITY,
> {
    query: [f64; 2],
    nodes: SearchFrontier<DistanceEntry<'a, Node<T>>, NODE_INLINE_CAPACITY>,
    values: SearchFrontier<DistanceEntry<'a, T>, VALUE_INLINE_CAPACITY>,
    #[cfg(test)]
    branch_expansions: usize,
    #[cfg(test)]
    leaf_expansions: usize,
    #[cfg(test)]
    node_pushes: usize,
    #[cfg(test)]
    value_pushes: usize,
    #[cfg(test)]
    pending_nodes: usize,
    #[cfg(test)]
    pending_values: usize,
    #[cfg(test)]
    node_high_water: usize,
    #[cfg(test)]
    value_high_water: usize,
    #[cfg(test)]
    combined_high_water: usize,
    #[cfg(test)]
    yielded_by_leaf: Vec<usize>,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct NearestMetrics {
    pub(crate) frontier: FrontierMetrics,
    pub(crate) branch_expansions: usize,
    pub(crate) leaf_expansions: usize,
    pub(crate) node_pushes: usize,
    pub(crate) value_pushes: usize,
    pub(crate) node_pops: usize,
    pub(crate) value_pops: usize,
    pub(crate) contributing_leaves: usize,
    pub(crate) max_yields_per_leaf: usize,
    pub(crate) node_high_water: usize,
    pub(crate) value_high_water: usize,
}

impl<'a, T, const NODE_INLINE_CAPACITY: usize, const VALUE_INLINE_CAPACITY: usize>
    NearestIter<'a, T, NODE_INLINE_CAPACITY, VALUE_INLINE_CAPACITY>
{
    pub(crate) fn new(root: &'a Node<T>, query: [f64; 2]) -> Self {
        let mut nodes = SearchFrontier::new();
        nodes.push(DistanceEntry {
            dist: 0.0,
            item: root,
            #[cfg(test)]
            source_leaf: None,
        });
        Self {
            query,
            nodes,
            values: SearchFrontier::new(),
            #[cfg(test)]
            branch_expansions: 0,
            #[cfg(test)]
            leaf_expansions: 0,
            #[cfg(test)]
            node_pushes: 1,
            #[cfg(test)]
            value_pushes: 0,
            #[cfg(test)]
            pending_nodes: 1,
            #[cfg(test)]
            pending_values: 0,
            #[cfg(test)]
            node_high_water: 1,
            #[cfg(test)]
            value_high_water: 0,
            #[cfg(test)]
            combined_high_water: 1,
            #[cfg(test)]
            yielded_by_leaf: Vec::new(),
        }
    }

    #[cfg(test)]
    pub(crate) fn metrics(&self) -> NearestMetrics {
        let nodes = self.nodes.metrics();
        let values = self.values.metrics();
        NearestMetrics {
            frontier: FrontierMetrics {
                pushes: nodes.pushes + values.pushes,
                pops: nodes.pops + values.pops,
                high_water: self.combined_high_water,
            },
            branch_expansions: self.branch_expansions,
            leaf_expansions: self.leaf_expansions,
            node_pushes: self.node_pushes,
            value_pushes: self.value_pushes,
            node_pops: nodes.pops,
            value_pops: values.pops,
            contributing_leaves: self
                .yielded_by_leaf
                .iter()
                .filter(|&&yielded| yielded != 0)
                .count(),
            max_yields_per_leaf: self.yielded_by_leaf.iter().copied().max().unwrap_or(0),
            node_high_water: self.node_high_water,
            value_high_water: self.value_high_water,
        }
    }

    #[cfg(test)]
    fn yielded_by_leaf(&self) -> &[usize] {
        &self.yielded_by_leaf
    }
}

impl<'a, T: Indexable, const NODE_INLINE_CAPACITY: usize, const VALUE_INLINE_CAPACITY: usize>
    Iterator for NearestIter<'a, T, NODE_INLINE_CAPACITY, VALUE_INLINE_CAPACITY>
{
    type Item = &'a T;

    /// Correctness of the yield order: a child's box is contained in
    /// its parent's, so every entry pushed by an expansion is at least
    /// as far as the node it came from — a value that pops is nearer
    /// than everything still unexpanded.
    fn next(&mut self) -> Option<&'a T> {
        loop {
            let node_dist = self.nodes.peek().map(DistanceEntry::dist);
            let value_dist = self.values.peek().map(DistanceEntry::dist);
            if value_dist
                .is_some_and(|value| node_dist.is_none_or(|node| value.total_cmp(&node).is_le()))
            {
                let value = self
                    .values
                    .pop()
                    .expect("a distance was just read from the value frontier");
                #[cfg(test)]
                {
                    self.pending_values -= 1;
                    self.yielded_by_leaf[value
                        .source_leaf
                        .expect("only value entries enter the value frontier")] += 1;
                }
                return Some(value.item);
            }

            let node = self.nodes.pop()?;
            match node.item {
                Node::Leaf(values) => {
                    #[cfg(test)]
                    let source_leaf = {
                        self.pending_nodes -= 1;
                        self.leaf_expansions += 1;
                        self.value_pushes += values.len();
                        self.pending_values += values.len();
                        self.value_high_water = self.value_high_water.max(self.pending_values);
                        self.combined_high_water = self
                            .combined_high_water
                            .max(self.pending_nodes + self.pending_values);
                        self.yielded_by_leaf.push(0);
                        self.yielded_by_leaf.len() - 1
                    };
                    let query = self.query;
                    self.values.extend(values.iter().map(|value| DistanceEntry {
                        dist: value.bounds().comparable_min_distance_to(query),
                        item: value,
                        #[cfg(test)]
                        source_leaf: Some(source_leaf),
                    }));
                }
                Node::Branch(children) => {
                    #[cfg(test)]
                    {
                        self.pending_nodes -= 1;
                        self.branch_expansions += 1;
                        self.node_pushes += children.len();
                        self.pending_nodes += children.len();
                        self.node_high_water = self.node_high_water.max(self.pending_nodes);
                        self.combined_high_water = self
                            .combined_high_water
                            .max(self.pending_nodes + self.pending_values);
                    }
                    let query = self.query;
                    self.nodes
                        .extend(children.iter().map(|(bounds, child)| DistanceEntry {
                            dist: bounds.comparable_min_distance_to(query),
                            item: child,
                            #[cfg(test)]
                            source_leaf: None,
                        }));
                }
            }
        }
    }
}

impl<T: Indexable, const NODE_INLINE_CAPACITY: usize, const VALUE_INLINE_CAPACITY: usize>
    FusedIterator for NearestIter<'_, T, NODE_INLINE_CAPACITY, VALUE_INLINE_CAPACITY>
{
}

/// A node or value keyed by its squared minimum distance to the query.
struct DistanceEntry<'a, T> {
    dist: f64,
    item: &'a T,
    #[cfg(test)]
    source_leaf: Option<usize>,
}

impl<T> DistanceEntry<'_, T> {
    fn dist(&self) -> f64 {
        self.dist
    }
}

impl<T> PartialEq for DistanceEntry<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.dist().total_cmp(&other.dist()).is_eq()
    }
}

impl<T> Eq for DistanceEntry<'_, T> {}

impl<T> PartialOrd for DistanceEntry<'_, T> {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T> Ord for DistanceEntry<'_, T> {
    /// Reversed so the max-first [`SearchFrontier`] pops the SMALLEST
    /// distance first.
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        other.dist().total_cmp(&self.dist())
    }
}

#[cfg(test)]
mod tests {
    use super::{DEFAULT_NODE_INLINE_CAPACITY, DEFAULT_VALUE_INLINE_CAPACITY, NearestMetrics};
    use crate::{AsymmetricQuadratic, AsymmetricRStarSplit, Bounds, Rtree, SplitParameters};
    use core::mem::size_of;
    use rstar::primitives::GeomWithData;
    use rstar::{ParentNode, PointDistance, RTree, RTreeNode};
    use std::collections::BinaryHeap;

    const FIELD: f64 = 50_000.0;
    const CLUSTER_COUNT: usize = 16;
    const CLUSTER_RADIUS: f64 = 100.0;
    const N: usize = 50_000;
    const Q: usize = 100;
    const K: usize = 8;

    type RstarValue = GeomWithData<[f64; 2], u32>;

    struct RstarEntry<'a> {
        node: &'a RTreeNode<RstarValue>,
        dist: f64,
    }

    impl PartialEq for RstarEntry<'_> {
        fn eq(&self, other: &Self) -> bool {
            self.dist.total_cmp(&other.dist).is_eq()
        }
    }

    impl Eq for RstarEntry<'_> {}

    impl PartialOrd for RstarEntry<'_> {
        fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }

    impl Ord for RstarEntry<'_> {
        fn cmp(&self, other: &Self) -> core::cmp::Ordering {
            other.dist.total_cmp(&self.dist)
        }
    }

    #[derive(Default)]
    struct RstarMetrics {
        pushes: usize,
        parent_pushes: usize,
        leaf_pushes: usize,
        pops: usize,
        parent_expansions: usize,
        leaf_parent_expansions: usize,
        leaf_yields: usize,
        high_water: usize,
    }

    fn extend_rstar<'a>(
        parent: &'a ParentNode<RstarValue>,
        query: &[f64; 2],
        heap: &mut BinaryHeap<RstarEntry<'a>>,
        metrics: &mut RstarMetrics,
    ) {
        for node in parent.children() {
            match node {
                RTreeNode::Parent(_) => metrics.parent_pushes += 1,
                RTreeNode::Leaf(_) => metrics.leaf_pushes += 1,
            }
        }
        heap.extend(parent.children().iter().map(|node| {
            let dist = match node {
                RTreeNode::Parent(parent) => parent.envelope().distance_2(query),
                RTreeNode::Leaf(value) => value.distance_2(query),
            };
            RstarEntry { node, dist }
        }));
        metrics.pushes += parent.children().len();
        metrics.high_water = metrics.high_water.max(heap.len());
    }

    fn measure_rstar(tree: &RTree<RstarValue>, query: [f64; 2]) -> RstarMetrics {
        let mut metrics = RstarMetrics::default();
        let mut heap = BinaryHeap::new();
        extend_rstar(tree.root(), &query, &mut heap, &mut metrics);
        while metrics.leaf_yields < K {
            let current = heap.pop().expect("fixture tree contains K values");
            metrics.pops += 1;
            match current.node {
                RTreeNode::Parent(parent) => {
                    metrics.parent_expansions += 1;
                    if parent
                        .children()
                        .first()
                        .is_some_and(|child| matches!(child, RTreeNode::Leaf(_)))
                    {
                        metrics.leaf_parent_expansions += 1;
                    }
                    extend_rstar(parent, &query, &mut heap, &mut metrics);
                }
                RTreeNode::Leaf(_) => metrics.leaf_yields += 1,
            }
        }
        metrics
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

    fn uniform(n: usize) -> Vec<[f64; 2]> {
        let mut lcg = Lcg::new();
        (0..n)
            .map(|_| [lcg.next_f64() * FIELD, lcg.next_f64() * FIELD])
            .collect()
    }

    fn clustered(n: usize) -> Vec<[f64; 2]> {
        let mut lcg = Lcg::new();
        let centers: Vec<[f64; 2]> = (0..CLUSTER_COUNT)
            .map(|_| [lcg.next_f64() * FIELD, lcg.next_f64() * FIELD])
            .collect();
        (0..n)
            .map(|i| {
                let center = centers[i % CLUSTER_COUNT];
                [
                    center[0] + lcg.next_f64() * 2.0 * CLUSTER_RADIUS - CLUSTER_RADIUS,
                    center[1] + lcg.next_f64() * 2.0 * CLUSTER_RADIUS - CLUSTER_RADIUS,
                ]
            })
            .collect()
    }

    fn queries(q: usize) -> Vec<[f64; 2]> {
        let mut lcg = Lcg::new();
        (0..q)
            .map(|_| {
                let x = lcg.next_f64() * FIELD;
                lcg.next_f64();
                let y = lcg.next_f64() * FIELD;
                [x, y]
            })
            .collect()
    }

    fn measure<Params: SplitParameters>() -> (usize, usize, usize, usize, usize) {
        let tree: Rtree<(Bounds, u32), Params> = uniform(N)
            .into_iter()
            .enumerate()
            .map(|(i, point)| (Bounds::point(point), u32::try_from(i).expect("N fits u32")))
            .collect();
        let mut total_pushes = 0;
        let mut total_pops = 0;
        let mut max_high_water = 0;
        let mut branch_expansions = 0;
        let mut leaf_expansions = 0;
        for query in queries(Q) {
            let mut nearest = tree.nearest_iter(query);
            assert_eq!(nearest.by_ref().take(K).count(), K);
            let NearestMetrics {
                frontier,
                branch_expansions: branches,
                leaf_expansions: leaves,
                ..
            } = nearest.metrics();
            total_pushes += frontier.pushes;
            total_pops += frontier.pops;
            max_high_water = max_high_water.max(frontier.high_water);
            branch_expansions += branches;
            leaf_expansions += leaves;
        }
        (
            total_pushes,
            total_pops,
            max_high_water,
            branch_expansions,
            leaf_expansions,
        )
    }

    #[test]
    fn asymmetric_fanout_reduces_knn_frontier_work() {
        let baseline = measure::<crate::Quadratic<32, 9>>();
        let default = measure::<AsymmetricRStarSplit<6, 2, 12, 4, 4, 4>>();
        assert!(
            default.0 < baseline.0 * 7 / 10,
            "default asymmetric fanout must cut frontier pushes by at least 30%: baseline={baseline:?}, default={default:?}"
        );
        assert!(default.2 < baseline.2);
        eprintln!(
            "[rtree-asymmetric-fanout] config=branch32_leaf32 expected_high_water=267 observed={baseline:?}"
        );
        for (name, observed) in [
            (
                "branch6_leaf32",
                measure::<AsymmetricQuadratic<6, 2, 32, 9>>(),
            ),
            ("default_insert6_leaf12_bulk4_4", default),
            (
                "branch12_leaf32",
                measure::<AsymmetricQuadratic<12, 4, 32, 9>>(),
            ),
            (
                "branch16_leaf32",
                measure::<AsymmetricQuadratic<16, 4, 32, 9>>(),
            ),
        ] {
            eprintln!(
                "[rtree-asymmetric-fanout] config={name} expected_pushes_below={} observed={observed:?}",
                baseline.0 * 7 / 10
            );
        }
        for (name, observed) in [
            (
                "branch8_leaf6",
                measure::<AsymmetricQuadratic<8, 3, 6, 2>>(),
            ),
            (
                "branch8_leaf8",
                measure::<AsymmetricQuadratic<8, 3, 8, 3>>(),
            ),
            (
                "branch8_leaf12",
                measure::<AsymmetricQuadratic<8, 3, 12, 4>>(),
            ),
            (
                "branch8_leaf16",
                measure::<AsymmetricQuadratic<8, 3, 16, 4>>(),
            ),
            (
                "branch8_leaf24",
                measure::<AsymmetricQuadratic<8, 3, 24, 7>>(),
            ),
            ("branch8_leaf32", default),
        ] {
            eprintln!("[rtree-leaf-fanout] config={name} observed={observed:?}");
        }
    }

    #[test]
    fn records_rstar_iterator_shape_comparison() {
        for (distribution, points) in [("uniform", uniform(N)), ("clustered", clustered(N))] {
            let boost: Rtree<(Bounds, u32), AsymmetricRStarSplit<8, 3, 32, 9>> = points
                .iter()
                .copied()
                .enumerate()
                .map(|(i, point)| (Bounds::point(point), u32::try_from(i).expect("N fits u32")))
                .collect();
            let rstar = RTree::bulk_load(
                points
                    .into_iter()
                    .enumerate()
                    .map(|(i, point)| {
                        GeomWithData::new(point, u32::try_from(i).expect("N fits u32"))
                    })
                    .collect(),
            );

            let mut boost_pushes = 0;
            let mut boost_pops = 0;
            let mut boost_branches = 0;
            let mut boost_leaves = 0;
            let mut boost_node_pushes = 0;
            let mut boost_value_pushes = 0;
            let mut boost_node_pops = 0;
            let mut boost_value_pops = 0;
            let mut boost_contributing_leaves = 0;
            let mut boost_max_yields_per_leaf = 0;
            let mut boost_leaf_yield_histogram = [0usize; K + 1];
            let mut boost_leaf_expansions_per_query = Vec::with_capacity(Q);
            let mut boost_node_high_water = 0;
            let mut boost_value_high_water = 0;
            let mut rstar_pushes = 0;
            let mut rstar_parent_pushes = 0;
            let mut rstar_leaf_pushes = 0;
            let mut rstar_pops = 0;
            let mut rstar_parents = 0;
            let mut rstar_leaf_parents = 0;
            let mut rstar_high_water = 0;

            for query in queries(Q) {
                let mut nearest = boost.nearest_iter(query);
                assert_eq!(nearest.by_ref().take(K).count(), K);
                let metrics = nearest.metrics();
                boost_pushes += metrics.frontier.pushes;
                boost_pops += metrics.frontier.pops;
                boost_branches += metrics.branch_expansions;
                boost_leaves += metrics.leaf_expansions;
                boost_node_pushes += metrics.node_pushes;
                boost_value_pushes += metrics.value_pushes;
                boost_node_pops += metrics.node_pops;
                boost_value_pops += metrics.value_pops;
                boost_contributing_leaves += metrics.contributing_leaves;
                boost_max_yields_per_leaf =
                    boost_max_yields_per_leaf.max(metrics.max_yields_per_leaf);
                boost_leaf_expansions_per_query.push(metrics.leaf_expansions);
                for &yielded in nearest.yielded_by_leaf() {
                    boost_leaf_yield_histogram[yielded] += 1;
                }
                boost_node_high_water = boost_node_high_water.max(metrics.node_high_water);
                boost_value_high_water = boost_value_high_water.max(metrics.value_high_water);

                let metrics = measure_rstar(&rstar, query);
                assert_eq!(metrics.leaf_yields, K);
                rstar_pushes += metrics.pushes;
                rstar_parent_pushes += metrics.parent_pushes;
                rstar_leaf_pushes += metrics.leaf_pushes;
                rstar_pops += metrics.pops;
                rstar_parents += metrics.parent_expansions;
                rstar_leaf_parents += metrics.leaf_parent_expansions;
                rstar_high_water = rstar_high_water.max(metrics.high_water);
            }

            boost_leaf_expansions_per_query.sort_unstable();
            eprintln!(
                "[rtree-leaf-release] distribution={distribution} expected_yields={} observed_value_pushes={boost_value_pushes} observed_value_pops={boost_value_pops} observed_leaf_expansions={boost_leaves} observed_leaf_expansions_p50={} observed_leaf_expansions_p95={} observed_leaf_expansions_max={} observed_contributing_leaves={boost_contributing_leaves} observed_max_yields_per_leaf={boost_max_yields_per_leaf} observed_leaf_yield_histogram={boost_leaf_yield_histogram:?}",
                Q * K,
                boost_leaf_expansions_per_query[Q / 2],
                boost_leaf_expansions_per_query[Q * 95 / 100],
                boost_leaf_expansions_per_query[Q - 1],
            );
            eprintln!(
                "[rtree-rstar-iterator-shape] distribution={distribution} boost_pushes={boost_pushes} boost_pops={boost_pops} boost_branches={boost_branches} boost_leaves={boost_leaves} boost_node_pushes={boost_node_pushes} boost_value_pushes={boost_value_pushes} boost_node_pops={boost_node_pops} boost_value_pops={boost_value_pops} boost_node_high_water={boost_node_high_water} boost_value_high_water={boost_value_high_water} rstar_pushes={rstar_pushes} rstar_parent_pushes={rstar_parent_pushes} rstar_leaf_pushes={rstar_leaf_pushes} rstar_pops={rstar_pops} rstar_parents={rstar_parents} rstar_leaf_parents={rstar_leaf_parents} rstar_high_water={rstar_high_water}"
            );
        }
    }

    #[test]
    fn records_split_frontier_capacity_distribution() {
        for (distribution, points) in [("uniform", uniform(N)), ("clustered", clustered(N))] {
            let tree: Rtree<(Bounds, u32), AsymmetricRStarSplit<8, 3, 32, 9>> = points
                .into_iter()
                .enumerate()
                .map(|(i, point)| (Bounds::point(point), u32::try_from(i).expect("N fits u32")))
                .collect();
            let mut node_high_waters = Vec::with_capacity(Q);
            let mut value_high_waters = Vec::with_capacity(Q);
            for query in queries(Q) {
                let mut nearest = tree.nearest_iter(query);
                assert_eq!(nearest.by_ref().take(K).count(), K);
                let metrics = nearest.metrics();
                node_high_waters.push(metrics.node_high_water);
                value_high_waters.push(metrics.value_high_water);
            }
            node_high_waters.sort_unstable();
            value_high_waters.sort_unstable();
            let node_spills = Q - node_high_waters
                .partition_point(|&water| water <= DEFAULT_NODE_INLINE_CAPACITY);
            let value_spills = Q - value_high_waters
                .partition_point(|&water| water <= DEFAULT_VALUE_INLINE_CAPACITY);
            let spills_at = |high_waters: &[usize], capacity| {
                Q - high_waters.partition_point(|&water| water <= capacity)
            };
            let release_entry_bytes = size_of::<f64>() + size_of::<&(Bounds, u32)>();
            eprintln!(
                "[rtree-split-frontier] distribution={distribution} expected_node_capacity={DEFAULT_NODE_INLINE_CAPACITY} observed_node_spills={node_spills}/{Q} observed_node_p50={} observed_node_p95={} observed_node_max={} observed_node_spills_64_96_128={}/{}/{} expected_value_capacity={DEFAULT_VALUE_INLINE_CAPACITY} observed_value_spills={value_spills}/{Q} observed_value_p50={} observed_value_p95={} observed_value_max={} observed_value_spills_128_160_192_256={}/{}/{}/{} observed_entry_bytes={} observed_total_inline_bytes={}",
                node_high_waters[Q / 2],
                node_high_waters[Q * 95 / 100],
                node_high_waters[Q - 1],
                spills_at(&node_high_waters, 64),
                spills_at(&node_high_waters, 96),
                spills_at(&node_high_waters, 128),
                value_high_waters[Q / 2],
                value_high_waters[Q * 95 / 100],
                value_high_waters[Q - 1],
                spills_at(&value_high_waters, 128),
                spills_at(&value_high_waters, 160),
                spills_at(&value_high_waters, 192),
                spills_at(&value_high_waters, 256),
                release_entry_bytes,
                (DEFAULT_NODE_INLINE_CAPACITY + DEFAULT_VALUE_INLINE_CAPACITY)
                    * release_entry_bytes,
            );
        }
    }

    #[test]
    fn caller_selected_inline_capacities_preserve_distance_order() {
        let tree: Rtree<(Bounds, u32)> = uniform(2_000)
            .into_iter()
            .enumerate()
            .map(|(i, point)| (Bounds::point(point), u32::try_from(i).expect("N fits u32")))
            .collect();
        let query = [12_345.0, 23_456.0];
        let mut expected: Vec<f64> = tree
            .nearest_iter(query)
            .take(64)
            .map(|(bounds, _)| bounds.comparable_min_distance_to(query))
            .collect();
        let actual: Vec<f64> = tree
            .nearest_iter_with_inline_capacities::<0, 0>(query)
            .take(64)
            .map(|(bounds, _)| bounds.comparable_min_distance_to(query))
            .collect();

        expected.sort_by(f64::total_cmp);
        assert_eq!(
            actual, expected,
            "the all-spill configuration must stay exact"
        );
    }
}
