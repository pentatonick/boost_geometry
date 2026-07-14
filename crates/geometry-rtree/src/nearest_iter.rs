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
    pending_nodes: usize,
    #[cfg(test)]
    pending_values: usize,
    #[cfg(test)]
    node_high_water: usize,
    #[cfg(test)]
    value_high_water: usize,
    #[cfg(test)]
    combined_high_water: usize,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct NearestMetrics {
    pub(crate) frontier: FrontierMetrics,
    pub(crate) branch_expansions: usize,
    pub(crate) leaf_expansions: usize,
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
            pending_nodes: 1,
            #[cfg(test)]
            pending_values: 0,
            #[cfg(test)]
            node_high_water: 1,
            #[cfg(test)]
            value_high_water: 0,
            #[cfg(test)]
            combined_high_water: 1,
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
            node_high_water: self.node_high_water,
            value_high_water: self.value_high_water,
        }
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
                }
                return Some(value.item);
            }

            let node = self.nodes.pop()?;
            match node.item {
                Node::Leaf(values) => {
                    #[cfg(test)]
                    {
                        self.pending_nodes -= 1;
                        self.leaf_expansions += 1;
                        self.pending_values += values.len();
                        self.value_high_water = self.value_high_water.max(self.pending_values);
                        self.combined_high_water = self
                            .combined_high_water
                            .max(self.pending_nodes + self.pending_values);
                    }
                    let query = self.query;
                    self.values.extend(values.iter().map(|value| DistanceEntry {
                        dist: value.bounds().comparable_min_distance_to(query),
                        item: value,
                    }));
                }
                Node::Branch(children) => {
                    #[cfg(test)]
                    {
                        self.pending_nodes -= 1;
                        self.branch_expansions += 1;
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
    use super::{
        DEFAULT_NODE_INLINE_CAPACITY, DEFAULT_VALUE_INLINE_CAPACITY, DistanceEntry, NearestMetrics,
    };
    use crate::{AsymmetricQuadratic, AsymmetricRStarSplit, Bounds, Rtree, SplitParameters};
    use core::mem::size_of;

    const FIELD: f64 = 50_000.0;
    const CLUSTER_COUNT: usize = 16;
    const CLUSTER_RADIUS: f64 = 100.0;
    const N: usize = 50_000;
    const Q: usize = 100;
    const K: usize = 8;

    #[test]
    fn distance_entry_equality_is_distance_only() {
        let first_value = 1_u8;
        let second_value = 2_u8;
        let first = DistanceEntry {
            dist: 3.0,
            item: &first_value,
        };
        let same_distance = DistanceEntry {
            dist: 3.0,
            item: &second_value,
        };
        let farther = DistanceEntry {
            dist: 4.0,
            item: &first_value,
        };
        assert!(first == same_distance);
        assert!(first != farther);
        assert_eq!(*first.item, first_value);
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
