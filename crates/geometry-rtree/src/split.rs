//! Node-split strategies — the type parameter that governs tree shape.
//!
//! Mirrors `boost/geometry/index/parameters.hpp` and the
//! `index/detail/rtree/{linear,quadratic,rstar}/redistribute_elements.hpp`
//! family. When a node overflows, the split strategy decides how to
//! partition its children into two nodes. Boost exposes this as a
//! template parameter; the port uses a [`SplitParameters`] trait
//! carried on `Rtree<T, Params>`.
//!
//! Four strategies ship: [`AsymmetricRStarSplit`] (the [`crate::Rtree`]
//! default), symmetric [`RStarSplit`], [`Quadratic`] (Boost's textbook
//! split), and [`Linear`] (cheaper inserts, looser trees).
//!
//! # Choosing parameters
//!
//! Start with [`Rtree<T>`](crate::Rtree). Its default was selected from
//! uniform and clustered workloads covering bulk loading, insertion, range
//! queries, and nearest-neighbour queries. A different data distribution,
//! payload size, construction method, or query mix can change the best tree
//! shape, so customize the parameters only after benchmarking a
//! representative workload.
//!
//! The default is
//! `AsymmetricRStarSplit<6, 2, 12, 4, 4, 4>`. The parameters are positional:
//!
//! | Parameter | Used by | General trade-off |
//! |-----------|---------|-------------------|
//! | `BRANCH_MAX` | [`Rtree::insert`](crate::Rtree::insert) | Smaller branches evaluate fewer sibling bounds; larger branches make a shallower tree. |
//! | `BRANCH_MIN` | Branch splitting after insertion | Higher values keep branches denser; lower values give the split more freedom. |
//! | `LEAF_MAX` | [`Rtree::insert`](crate::Rtree::insert) | Smaller leaves test fewer values when reached; larger leaves reduce tree structure. |
//! | `LEAF_MIN` | Leaf splitting after insertion | Controls the minimum fill of each new leaf. |
//! | `PACKED_BRANCH_MAX` | [`FromIterator`] | Controls branch fanout in the initial bulk-packed tree. |
//! | `PACKED_LEAF_MAX` | [`FromIterator`] | Controls how many values an initial packed leaf may contain. |
//!
//! The packed capacities affect only the layout initially produced by
//! [`FromIterator`]. Later insertions use the first
//! four insertion capacities; existing packed nodes are not rebuilt. A zero
//! packed capacity means "inherit the corresponding insertion maximum", so a
//! four-parameter `AsymmetricRStarSplit` retains the traditional behavior.
//!
//! ```
//! use geometry_rtree::{AsymmetricRStarSplit, Bounds, Rtree};
//!
//! // Spell out the measured default so each positional parameter is visible.
//! type Parameters = AsymmetricRStarSplit<
//!     6,  // insertion branch maximum
//!     2,  // insertion branch minimum
//!     12, // insertion leaf maximum
//!     4,  // insertion leaf minimum
//!     4,  // bulk-packed branch maximum
//!     4,  // bulk-packed leaf maximum
//! >;
//!
//! let tree: Rtree<Bounds, Parameters> = [
//!     Bounds::point([0.0, 0.0]),
//!     Bounds::point([1.0, 1.0]),
//! ]
//! .into_iter()
//! .collect();
//! assert_eq!(tree.len(), 2);
//! ```
//!
//! ## Tuning procedure
//!
//! - For a read-mostly tree built with `collect`, tune the two packed maxima
//!   first.
//! - For a frequently modified tree, tune the insertion maxima and minima
//!   first.
//! - Smaller maxima can reduce the work of nearest-neighbour searches, but
//!   create more nodes and may increase tree height.
//! - Larger maxima can reduce structural and traversal overhead, but make
//!   each visited branch or leaf more expensive to scan.
//! - Measure construction together with representative calls to
//!   [`query`](crate::Rtree::query), [`nearest`](crate::Rtree::nearest), or
//!   [`nearest_iter`](crate::Rtree::nearest_iter). Improving one workload can
//!   regress another.
//!
//! The built-in insertion strategies expect both minimums to be non-zero and
//! to leave room for two groups when a node of `MAX + 1` entries splits:
//! `2 * BRANCH_MIN <= BRANCH_MAX + 1` and
//! `2 * LEAF_MIN <= LEAF_MAX + 1`. Bulk leaf capacity must be non-zero and
//! bulk branch capacity must be at least two; bulk loading checks those two
//! constraints.
//!
//! ## Why these defaults
//!
//! The retained defaults were measured on 2026-07-13 with iai-callgrind,
//! 50,000 two-dimensional points, 100 queries, and `k = 8`. Candidate
//! configurations were evaluated across inserted and bulk-built trees,
//! uniform and clustered distributions, construction cost, bounded and
//! streaming kNN, and range queries. The insertion capacities `6/2/12/4`
//! and packed capacities `4/4` were retained as the best balanced measured
//! configuration across that matrix. This evidence supports the general
//! default; it is not a guarantee for every workload.

use alloc::vec::Vec;

use crate::bounds::{Bounds, union_all};

/// How a full node is partitioned when it overflows.
///
/// Mirrors `index::parameters` + the split policy
/// (`index/parameters.hpp`). Insertion uses the leaf/branch maxima and
/// minima; bulk loading may select smaller maxima independently. The
/// `split` method takes overflowing children (as bounds, so the strategy
/// is payload-agnostic) and returns the two groups by index. Bulk leaf
/// capacity must be non-zero and bulk branch capacity must be at least two.
pub trait SplitParameters {
    /// Maximum children per node before it must split.
    const MAX: usize;
    /// Minimum children each node must keep after a split.
    const MIN: usize;
    /// Maximum values in a leaf. Defaults to [`Self::MAX`].
    const LEAF_MAX: usize = Self::MAX;
    /// Minimum values in either half of a split leaf.
    const LEAF_MIN: usize = Self::MIN;
    /// Maximum children in a branch. Defaults to [`Self::MAX`].
    const BRANCH_MAX: usize = Self::MAX;
    /// Minimum children in either half of a split branch.
    const BRANCH_MIN: usize = Self::MIN;
    /// Maximum values per leaf created by bulk loading.
    const BULK_LEAF_MAX: usize = Self::LEAF_MAX;
    /// Maximum children per branch created by bulk loading.
    const BULK_BRANCH_MAX: usize = Self::BRANCH_MAX;

    /// Partition `entries` (a bounds-per-child list) into two groups of
    /// indices, each of size at least [`Self::MIN`].
    fn split(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>);

    /// Partition an overflowing leaf.
    #[must_use]
    fn split_leaf(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        Self::split(entries)
    }

    /// Partition an overflowing branch.
    #[must_use]
    fn split_branch(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        Self::split(entries)
    }
}

/// Quadratic split — Boost's textbook default.
///
/// `O(n²)`: pick the two children whose combined bounding box wastes the
/// most area as the seeds of the two groups, then assign each remaining
/// child to whichever group's box it enlarges least. Mirrors
/// `index/detail/rtree/quadratic/redistribute_elements.hpp`.
///
/// The type's own `<32, 9>` default is a symmetric alternative; it is not the
/// default strategy of [`crate::Rtree`]. Treat an explicit strategy choice as
/// a workload-specific opt-in and follow the [module tuning guide](self).
#[derive(Debug, Clone, Copy, Default)]
pub struct Quadratic<const MAX: usize = 32, const MIN: usize = 9>;

impl<const MAX: usize, const MIN: usize> SplitParameters for Quadratic<MAX, MIN> {
    const MAX: usize = MAX;
    const MIN: usize = MIN;

    fn split(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        quadratic_partition(entries, MIN)
    }
}

/// Quadratic split with independent branch and leaf capacities.
///
/// This keeps broad leaves for cheap bulk loading and subtree dumps,
/// while narrower branches reduce the child volume expanded by nearest
/// searches. The four const parameters are branch maximum/minimum,
/// followed by leaf maximum/minimum.
#[derive(Debug, Clone, Copy, Default)]
pub struct AsymmetricQuadratic<
    const BRANCH_MAX: usize,
    const BRANCH_MIN: usize,
    const LEAF_MAX: usize,
    const LEAF_MIN: usize,
>;

impl<const BRANCH_MAX: usize, const BRANCH_MIN: usize, const LEAF_MAX: usize, const LEAF_MIN: usize>
    SplitParameters for AsymmetricQuadratic<BRANCH_MAX, BRANCH_MIN, LEAF_MAX, LEAF_MIN>
{
    const MAX: usize = BRANCH_MAX;
    const MIN: usize = BRANCH_MIN;
    const LEAF_MAX: usize = LEAF_MAX;
    const LEAF_MIN: usize = LEAF_MIN;
    const BRANCH_MAX: usize = BRANCH_MAX;
    const BRANCH_MIN: usize = BRANCH_MIN;

    fn split(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        quadratic_partition(entries, BRANCH_MIN)
    }

    fn split_leaf(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        quadratic_partition(entries, LEAF_MIN)
    }
}

/// R\*-split selection with one capacity for branches and leaves.
///
/// Chooses the split axis by minimum summed margin, then the split index
/// by minimum overlap and combined area. Insertion descent still uses
/// least enlargement; this policy does not perform forced reinsertion. Its
/// `<32, 9>` type default is not the default of [`crate::Rtree`]; see the
/// [module tuning guide](self).
#[derive(Debug, Clone, Copy, Default)]
pub struct RStarSplit<const MAX: usize = 32, const MIN: usize = 9>;

impl<const MAX: usize, const MIN: usize> SplitParameters for RStarSplit<MAX, MIN> {
    const MAX: usize = MAX;
    const MIN: usize = MIN;

    fn split(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        rstar_partition(entries, MIN)
    }
}

/// R\*-split selection with independent insertion and bulk capacities.
///
/// The first four parameters are insertion branch maximum/minimum and
/// leaf maximum/minimum. Optional `PACKED_BRANCH_MAX` and
/// `PACKED_LEAF_MAX` parameters set the top-down bulk layout; zero (the
/// default) inherits the corresponding insertion maximum. See the
/// [module tuning guide](self) before selecting non-default values.
#[derive(Debug, Clone, Copy, Default)]
pub struct AsymmetricRStarSplit<
    const BRANCH_MAX: usize,
    const BRANCH_MIN: usize,
    const LEAF_MAX: usize,
    const LEAF_MIN: usize,
    const PACKED_BRANCH_MAX: usize = 0,
    const PACKED_LEAF_MAX: usize = 0,
>;

impl<
    const BRANCH_MAX: usize,
    const BRANCH_MIN: usize,
    const LEAF_MAX: usize,
    const LEAF_MIN: usize,
    const PACKED_BRANCH_MAX: usize,
    const PACKED_LEAF_MAX: usize,
> SplitParameters
    for AsymmetricRStarSplit<
        BRANCH_MAX,
        BRANCH_MIN,
        LEAF_MAX,
        LEAF_MIN,
        PACKED_BRANCH_MAX,
        PACKED_LEAF_MAX,
    >
{
    const MAX: usize = BRANCH_MAX;
    const MIN: usize = BRANCH_MIN;
    const LEAF_MAX: usize = LEAF_MAX;
    const LEAF_MIN: usize = LEAF_MIN;
    const BRANCH_MAX: usize = BRANCH_MAX;
    const BRANCH_MIN: usize = BRANCH_MIN;
    const BULK_LEAF_MAX: usize = if PACKED_LEAF_MAX == 0 {
        LEAF_MAX
    } else {
        PACKED_LEAF_MAX
    };
    const BULK_BRANCH_MAX: usize = if PACKED_BRANCH_MAX == 0 {
        BRANCH_MAX
    } else {
        PACKED_BRANCH_MAX
    };

    fn split(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        rstar_partition(entries, BRANCH_MIN)
    }

    fn split_leaf(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        rstar_partition(entries, LEAF_MIN)
    }
}

fn quadratic_partition(entries: &[Bounds], min: usize) -> (Vec<usize>, Vec<usize>) {
    let n = entries.len();
    let (s1, s2) = quadratic_seeds(entries);

    let mut g1 = Vec::from([s1]);
    let mut g2 = Vec::from([s2]);
    let mut b1 = entries[s1];
    let mut b2 = entries[s2];

    let mut remaining: Vec<usize> = (0..n).filter(|&i| i != s1 && i != s2).collect();

    while !remaining.is_empty() {
        // Force a group to fill if every remaining entry is needed to
        // reach MIN.
        if g1.len() + remaining.len() == min {
            for idx in remaining.drain(..) {
                g1.push(idx);
                b1 = b1.union(&entries[idx]);
            }
            break;
        }
        if g2.len() + remaining.len() == min {
            for idx in remaining.drain(..) {
                g2.push(idx);
                b2 = b2.union(&entries[idx]);
            }
            break;
        }

        // Guttman's PickNext: decide the entry with the strongest
        // preference first, rather than depending on input order.
        let mut next_pos = 0;
        let mut best_difference = f64::NEG_INFINITY;
        for (pos, &idx) in remaining.iter().enumerate() {
            let e1 = b1.enlargement(&entries[idx]);
            let e2 = b2.enlargement(&entries[idx]);
            let difference = (e1 - e2).abs();
            if difference > best_difference {
                next_pos = pos;
                best_difference = difference;
            }
        }
        let idx = remaining.swap_remove(next_pos);
        let e1 = b1.enlargement(&entries[idx]);
        let e2 = b2.enlargement(&entries[idx]);
        let assign_first = match e1.total_cmp(&e2) {
            core::cmp::Ordering::Less => true,
            core::cmp::Ordering::Greater => false,
            core::cmp::Ordering::Equal => match b1.area().total_cmp(&b2.area()) {
                core::cmp::Ordering::Less => true,
                core::cmp::Ordering::Greater => false,
                core::cmp::Ordering::Equal => g1.len() <= g2.len(),
            },
        };
        if assign_first {
            g1.push(idx);
            b1 = b1.union(&entries[idx]);
        } else {
            g2.push(idx);
            b2 = b2.union(&entries[idx]);
        }
    }

    (g1, g2)
}

#[allow(
    clippy::float_cmp,
    reason = "exact R*-split tie-break between equal overlap and area"
)]
fn rstar_partition(entries: &[Bounds], min: usize) -> (Vec<usize>, Vec<usize>) {
    let mut best_axis = 0;
    let mut best_margin = f64::INFINITY;
    for axis in 0..2 {
        let ordered = sorted_indices(entries, axis);
        let suffixes = suffix_bounds(entries, &ordered);
        let mut left = partition_bounds(entries, &ordered[..min]);
        let mut margin = left.half_perimeter() + suffixes[min].half_perimeter();
        for split in (min + 1)..=entries.len() - min {
            left = left.union(&entries[ordered[split - 1]]);
            margin += left.half_perimeter() + suffixes[split].half_perimeter();
        }
        if margin < best_margin {
            best_axis = axis;
            best_margin = margin;
        }
    }

    let ordered = sorted_indices(entries, best_axis);
    let suffixes = suffix_bounds(entries, &ordered);
    let mut best_split = min;
    let mut best_overlap = f64::INFINITY;
    let mut best_area = f64::INFINITY;
    let mut left = partition_bounds(entries, &ordered[..min]);
    for split in min..=entries.len() - min {
        if split > min {
            left = left.union(&entries[ordered[split - 1]]);
        }
        let right = suffixes[split];
        let overlap = intersection_area(&left, &right);
        let area = left.area() + right.area();
        if overlap < best_overlap || (overlap == best_overlap && area < best_area) {
            best_split = split;
            best_overlap = overlap;
            best_area = area;
        }
    }

    (
        ordered[..best_split].to_vec(),
        ordered[best_split..].to_vec(),
    )
}

fn sorted_indices(entries: &[Bounds], axis: usize) -> Vec<usize> {
    let mut indices: Vec<usize> = (0..entries.len()).collect();
    indices.sort_unstable_by(|&left, &right| {
        entries[left].min[axis].total_cmp(&entries[right].min[axis])
    });
    indices
}

fn partition_bounds(entries: &[Bounds], indices: &[usize]) -> Bounds {
    let mut bounds = entries[indices[0]];
    for &index in &indices[1..] {
        bounds = bounds.union(&entries[index]);
    }
    bounds
}

fn suffix_bounds(entries: &[Bounds], indices: &[usize]) -> Vec<Bounds> {
    let mut suffixes = Vec::with_capacity(indices.len());
    let mut bounds = entries[*indices.last().expect("a split has at least two entries")];
    suffixes.push(bounds);
    for &index in indices[..indices.len() - 1].iter().rev() {
        bounds = bounds.union(&entries[index]);
        suffixes.push(bounds);
    }
    suffixes.reverse();
    suffixes
}

fn intersection_area(left: &Bounds, right: &Bounds) -> f64 {
    let width = left.max[0].min(right.max[0]) - left.min[0].max(right.min[0]);
    let height = left.max[1].min(right.max[1]) - left.min[1].max(right.min[1]);
    width.max(0.0) * height.max(0.0)
}

/// Linear split — cheap inserts, looser trees.
///
/// `O(n)`: pick the two children furthest apart along the axis of
/// greatest spread as seeds, then distribute the rest greedily by
/// least enlargement. Mirrors
/// `index/detail/rtree/linear/redistribute_elements.hpp`.
///
/// Its `<32, 9>` type default is a symmetric alternative, not the default of
/// [`crate::Rtree`]. See the [module tuning guide](self).
#[derive(Debug, Clone, Copy, Default)]
pub struct Linear<const MAX: usize = 32, const MIN: usize = 9>;

impl<const MAX: usize, const MIN: usize> SplitParameters for Linear<MAX, MIN> {
    const MAX: usize = MAX;
    const MIN: usize = MIN;

    fn split(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        let n = entries.len();
        let (s1, s2) = linear_seeds(entries);

        let mut g1 = Vec::from([s1]);
        let mut g2 = Vec::from([s2]);
        let mut b1 = entries[s1];
        let mut b2 = entries[s2];

        let mut remaining: Vec<usize> = (0..n).filter(|&i| i != s1 && i != s2).collect();
        while let Some(idx) = remaining.pop() {
            if g1.len() + remaining.len() + 1 == MIN {
                g1.push(idx);
                b1 = b1.union(&entries[idx]);
                continue;
            }
            if g2.len() + remaining.len() + 1 == MIN {
                g2.push(idx);
                b2 = b2.union(&entries[idx]);
                continue;
            }
            let e1 = b1.enlargement(&entries[idx]);
            let e2 = b2.enlargement(&entries[idx]);
            if e1 <= e2 {
                g1.push(idx);
                b1 = b1.union(&entries[idx]);
            } else {
                g2.push(idx);
                b2 = b2.union(&entries[idx]);
            }
        }
        (g1, g2)
    }
}

/// The pair of children whose combined box wastes the most area — the
/// quadratic-split seeds (`PickSeeds` in Boost's terms).
fn quadratic_seeds(entries: &[Bounds]) -> (usize, usize) {
    let mut worst = (0, 1, f64::NEG_INFINITY);
    for i in 0..entries.len() {
        for j in (i + 1)..entries.len() {
            let combined = entries[i].union(&entries[j]).area();
            let waste = combined - entries[i].area() - entries[j].area();
            if waste > worst.2 {
                worst = (i, j, waste);
            }
        }
    }
    (worst.0, worst.1)
}

/// The two children furthest apart along the widest axis — the
/// linear-split seeds.
fn linear_seeds(entries: &[Bounds]) -> (usize, usize) {
    let all = union_all(entries);
    let width = [all.max[0] - all.min[0], all.max[1] - all.min[1]];
    let axis = usize::from(width[1] > width[0]);

    // Extreme children: lowest high-side and highest low-side along axis.
    let mut lo_idx = 0;
    let mut hi_idx = 0;
    let mut max_low = f64::NEG_INFINITY;
    let mut min_high = f64::INFINITY;
    for (i, b) in entries.iter().enumerate() {
        if b.min[axis] > max_low {
            max_low = b.min[axis];
            hi_idx = i;
        }
        if b.max[axis] < min_high {
            min_high = b.max[axis];
            lo_idx = i;
        }
    }
    if lo_idx == hi_idx {
        // Degenerate: fall back to the first two.
        (0, usize::from(entries.len() > 1))
    } else {
        (lo_idx, hi_idx)
    }
}

#[cfg(test)]
#[allow(
    clippy::cast_precision_loss,
    reason = "small test indices convert exactly to f64"
)]
mod tests {
    use super::{
        AsymmetricQuadratic, AsymmetricRStarSplit, Linear, Quadratic, RStarSplit, SplitParameters,
    };
    use crate::bounds::Bounds;

    fn line_of_boxes(n: usize) -> Vec<Bounds> {
        (0..n)
            .map(|i| Bounds::point([i as f64, 0.0]))
            .collect::<Vec<_>>()
    }

    #[test]
    fn quadratic_splits_into_two_min_sized_groups() {
        let entries = line_of_boxes(9);
        let (g1, g2) = <Quadratic<8, 3>>::split(&entries);
        assert!(g1.len() >= 3 && g2.len() >= 3);
        assert_eq!(g1.len() + g2.len(), 9);
    }

    #[test]
    fn linear_splits_into_two_min_sized_groups() {
        let entries = line_of_boxes(9);
        let (g1, g2) = <Linear<8, 3>>::split(&entries);
        assert!(g1.len() >= 3 && g2.len() >= 3);
        assert_eq!(g1.len() + g2.len(), 9);
    }

    #[test]
    fn rstar_splits_into_two_min_sized_groups() {
        let entries = line_of_boxes(9);
        let (g1, g2) = <RStarSplit<8, 3>>::split(&entries);
        assert!(g1.len() >= 3 && g2.len() >= 3);
        assert_eq!(g1.len() + g2.len(), 9);
    }

    #[test]
    fn every_index_assigned_exactly_once() {
        let entries = line_of_boxes(9);
        let (mut g1, g2) = <Quadratic<8, 3>>::split(&entries);
        g1.extend(g2);
        g1.sort_unstable();
        assert_eq!(g1, (0..9).collect::<Vec<_>>());
    }

    #[test]
    fn asymmetric_leaf_and_branch_minimums_are_independent() {
        type Params = AsymmetricQuadratic<8, 3, 32, 9>;
        let branch_entries = line_of_boxes(9);
        let (b1, b2) = Params::split_branch(&branch_entries);
        assert!(b1.len() >= Params::BRANCH_MIN && b2.len() >= Params::BRANCH_MIN);

        let leaf_entries = line_of_boxes(33);
        let (l1, l2) = Params::split_leaf(&leaf_entries);
        assert!(l1.len() >= Params::LEAF_MIN && l2.len() >= Params::LEAF_MIN);
    }

    #[test]
    fn asymmetric_rstar_leaf_and_branch_minimums_are_independent() {
        type Params = AsymmetricRStarSplit<8, 3, 32, 9>;
        let branch_entries = line_of_boxes(9);
        let (b1, b2) = Params::split_branch(&branch_entries);
        assert!(b1.len() >= Params::BRANCH_MIN && b2.len() >= Params::BRANCH_MIN);

        let leaf_entries = line_of_boxes(33);
        let (l1, l2) = Params::split_leaf(&leaf_entries);
        assert!(l1.len() >= Params::LEAF_MIN && l2.len() >= Params::LEAF_MIN);
    }

    #[test]
    fn asymmetric_rstar_bulk_capacities_are_independent_and_optional() {
        type Inherited = AsymmetricRStarSplit<6, 2, 12, 4>;
        type Tuned = AsymmetricRStarSplit<6, 2, 12, 4, 4, 4>;

        assert_eq!(Inherited::BULK_BRANCH_MAX, 6);
        assert_eq!(Inherited::BULK_LEAF_MAX, 12);

        assert_eq!(Tuned::BRANCH_MAX, 6);
        assert_eq!(Tuned::LEAF_MAX, 12);
        assert_eq!(Tuned::BULK_BRANCH_MAX, 4);
        assert_eq!(Tuned::BULK_LEAF_MAX, 4);
    }
}
