//! The k-th-best rank tracker of the nearest search.
//!
//! A bounded max-heap keeps the best `k` values seen so far. Each entry
//! keeps its distance and value together, and the current k-th-best
//! distance is cached so rejected candidates never have to inspect the
//! heap.

use alloc::collections::BinaryHeap;
use alloc::vec::Vec;

/// The ascending distances of the best candidate values seen so far,
/// capped at `k`.
pub(crate) struct NearestBound<T> {
    ranks: BinaryHeap<Ranked<T>>,
    k: usize,
    bound: f64,
    next_ordinal: usize,
    #[cfg(test)]
    metrics: NearestBoundMetrics,
}

struct Ranked<T> {
    dist: f64,
    ordinal: usize,
    value: T,
}

impl<T> PartialEq for Ranked<T> {
    fn eq(&self, other: &Self) -> bool {
        self.dist.total_cmp(&other.dist).is_eq() && self.ordinal == other.ordinal
    }
}

impl<T> Eq for Ranked<T> {}

impl<T> PartialOrd for Ranked<T> {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T> Ord for Ranked<T> {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.dist
            .total_cmp(&other.dist)
            .then_with(|| self.ordinal.cmp(&other.ordinal))
    }
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct NearestBoundMetrics {
    pub(crate) calls: usize,
    pub(crate) partition_comparisons: usize,
    pub(crate) admissions: usize,
    pub(crate) replacements: usize,
    pub(crate) shifted_ranks: usize,
}

impl<T> NearestBound<T> {
    /// An empty tracker for the best `k` ranks, with room for
    /// `capacity` distances (the caller passes `min(k, len)` — a query
    /// can never hold more ranks than the tree has values).
    pub(crate) fn new(k: usize, capacity: usize) -> Self {
        Self {
            ranks: BinaryHeap::with_capacity(capacity),
            k,
            bound: f64::INFINITY,
            next_ordinal: 0,
            #[cfg(test)]
            metrics: NearestBoundMetrics::default(),
        }
    }

    /// The distance a candidate must beat: the k-th-best distance seen,
    /// or infinity while fewer than `k` ranks are held.
    pub(crate) fn bound(&self) -> f64 {
        self.bound
    }

    /// Insert a candidate that the caller has already proved is strictly
    /// better than [`bound`](Self::bound), dropping the last rank from
    /// the last rank when `k` are already held.
    pub(crate) fn admit_better(&mut self, dist: f64, value: T) {
        debug_assert!(dist.total_cmp(&self.bound()).is_lt());
        let rank = Ranked {
            dist,
            ordinal: self.next_ordinal,
            value,
        };
        self.next_ordinal += 1;
        if self.ranks.len() == self.k {
            #[cfg(test)]
            {
                self.metrics.replacements += 1;
            }
            *self
                .ranks
                .peek_mut()
                .expect("a full positive-k heap has a greatest rank") = rank;
        } else {
            self.ranks.push(rank);
        }
        if self.ranks.len() == self.k {
            self.bound = self
                .ranks
                .peek()
                .expect("a full positive-k heap has a greatest rank")
                .dist;
        }
        #[cfg(test)]
        {
            self.metrics.calls += 1;
            self.metrics.admissions += 1;
        }
    }

    /// Consume the ranked candidates in ascending distance order.
    pub(crate) fn into_values(self) -> Vec<T> {
        self.ranks
            .into_sorted_vec()
            .into_iter()
            .map(|rank| rank.value)
            .collect()
    }

    #[cfg(test)]
    pub(crate) fn metrics(&self) -> NearestBoundMetrics {
        self.metrics
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact rank distances, no arithmetic")]
mod tests {
    use super::NearestBound;

    #[test]
    fn unfull_bound_is_infinity() {
        let mut bound = NearestBound::new(3, 3);
        assert_eq!(bound.bound(), f64::INFINITY);
        bound.admit_better(4.0, 1u32);
        bound.admit_better(1.0, 2);
        assert_eq!(bound.bound(), f64::INFINITY);
        bound.admit_better(9.0, 3);
        assert_eq!(bound.bound(), 9.0);
        assert_eq!(bound.into_values(), [2, 1, 3]);
    }

    #[test]
    fn better_candidate_replaces_the_bound() {
        let mut bound = NearestBound::new(3, 3);
        bound.admit_better(1.0, 1u32);
        bound.admit_better(2.0, 2);
        bound.admit_better(3.0, 3);
        assert_eq!(bound.bound(), 3.0);

        bound.admit_better(2.5, 25);
        assert_eq!(bound.bound(), 2.5);
        assert_eq!(bound.into_values(), [1, 2, 25]);
    }

    #[test]
    fn ties_at_the_k_boundary_keep_the_first_admitted() {
        let mut bound = NearestBound::new(2, 2);
        bound.admit_better(5.0, 1u32);
        bound.admit_better(5.0, 2);
        assert!(5.0f64.total_cmp(&bound.bound()).is_ge());
        assert_eq!(bound.bound(), 5.0);

        bound.admit_better(1.0, 4);
        assert_eq!(bound.bound(), 5.0);
        assert_eq!(bound.into_values(), [4, 1]);
    }

    #[test]
    fn k_one_tracks_the_single_best() {
        let mut bound = NearestBound::new(1, 1);
        bound.admit_better(5.0, 1u32);
        assert_eq!(bound.bound(), 5.0);
        assert!(7.0f64.total_cmp(&bound.bound()).is_ge());
        bound.admit_better(2.0, 3);
        assert_eq!(bound.bound(), 2.0);
        assert_eq!(bound.into_values(), [3]);
    }
}
