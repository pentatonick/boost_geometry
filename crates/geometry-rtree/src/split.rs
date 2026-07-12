//! Node-split strategies — the type parameter that governs tree shape.
//!
//! Mirrors `boost/geometry/index/parameters.hpp` and the
//! `index/detail/rtree/{linear,quadratic,rstar}/redistribute_elements.hpp`
//! family. When a node overflows, the split strategy decides how to
//! partition its children into two nodes. Boost exposes this as a
//! template parameter; the port uses a sealed [`SplitParameters`] trait
//! carried on `Rtree<T, Params>`.
//!
//! Two strategies ship: [`Quadratic`] (the default — Boost's textbook
//! split, good all-round trees) and [`Linear`] (cheaper inserts, looser
//! trees). R\* is a future opt-in; the trait surface already admits it.

use alloc::vec::Vec;

use crate::bounds::{Bounds, union_all};

/// How a full node is partitioned when it overflows.
///
/// Mirrors `index::parameters` + the split policy
/// (`index/parameters.hpp`). `MAX` is the node capacity; `MIN` the
/// minimum fill each half must reach. The `split` method takes the
/// overflowing children (as `(Bounds, index)` pairs, so the strategy is
/// payload-agnostic) and returns the two groups by index.
pub trait SplitParameters {
    /// Maximum children per node before it must split.
    const MAX: usize;
    /// Minimum children each node must keep after a split.
    const MIN: usize;

    /// Partition `entries` (a bounds-per-child list) into two groups of
    /// indices, each of size at least [`Self::MIN`].
    fn split(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>);
}

/// Quadratic split — Boost's textbook default.
///
/// `O(n²)`: pick the two children whose combined bounding box wastes the
/// most area as the seeds of the two groups, then assign each remaining
/// child to whichever group's box it enlarges least. Mirrors
/// `index/detail/rtree/quadratic/redistribute_elements.hpp`.
#[derive(Debug, Clone, Copy, Default)]
pub struct Quadratic<const MAX: usize = 8, const MIN: usize = 3>;

impl<const MAX: usize, const MIN: usize> SplitParameters for Quadratic<MAX, MIN> {
    const MAX: usize = MAX;
    const MIN: usize = MIN;

    fn split(entries: &[Bounds]) -> (Vec<usize>, Vec<usize>) {
        let n = entries.len();
        let (s1, s2) = quadratic_seeds(entries);

        let mut g1 = Vec::from([s1]);
        let mut g2 = Vec::from([s2]);
        let mut b1 = entries[s1];
        let mut b2 = entries[s2];

        let mut remaining: Vec<usize> = (0..n).filter(|&i| i != s1 && i != s2).collect();

        while let Some(idx) = remaining.pop() {
            // Force the smaller group to fill if the rest are needed to
            // reach MIN.
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
            // Otherwise assign to the group whose box enlarges least.
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

/// Linear split — cheap inserts, looser trees.
///
/// `O(n)`: pick the two children furthest apart along the axis of
/// greatest spread as seeds, then distribute the rest greedily by
/// least enlargement. Mirrors
/// `index/detail/rtree/linear/redistribute_elements.hpp`.
#[derive(Debug, Clone, Copy, Default)]
pub struct Linear<const MAX: usize = 8, const MIN: usize = 3>;

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
    use super::{Linear, Quadratic, SplitParameters};
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
    fn every_index_assigned_exactly_once() {
        let entries = line_of_boxes(9);
        let (mut g1, g2) = <Quadratic<8, 3>>::split(&entries);
        g1.extend(g2);
        g1.sort_unstable();
        assert_eq!(g1, (0..9).collect::<Vec<_>>());
    }
}
