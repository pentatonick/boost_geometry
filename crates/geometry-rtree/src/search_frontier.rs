//! A priority queue for the nearest search's node frontier.
//!
//! Entries begin in a fixed-capacity binary max-heap and drain into an
//! allocated [`BinaryHeap`] only if the frontier outgrows it. Pop order
//! is identical either way, so correctness never depends on staying
//! below the boundary.

use alloc::collections::BinaryHeap;
use core::mem::replace;
use heapless::binary_heap::{BinaryHeap as InlineHeap, Max};

/// Default capacity for the bounded search's node-only frontier. The
/// streaming iterator supplies separate measured node/value capacities
/// because those entries are smaller and have different distributions.
const DEFAULT_INLINE_CAPACITY: usize = 192;

enum FrontierEntries<T: Ord, const INLINE_CAPACITY: usize> {
    Inline(InlineHeap<T, Max, INLINE_CAPACITY>),
    Spilled(BinaryHeap<T>),
}

/// A max-priority queue with the same pop contract as [`BinaryHeap`]:
/// [`pop`](Self::pop) returns the greatest entry. The nearest search
/// keys its entries by distance REVERSED, so pops come nearest-first.
pub(crate) struct SearchFrontier<T: Ord, const INLINE_CAPACITY: usize = DEFAULT_INLINE_CAPACITY> {
    entries: FrontierEntries<T, INLINE_CAPACITY>,
    #[cfg(test)]
    metrics: FrontierMetrics,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FrontierMetrics {
    pub(crate) pushes: usize,
    pub(crate) pops: usize,
    pub(crate) high_water: usize,
}

impl<T: Ord, const INLINE_CAPACITY: usize> SearchFrontier<T, INLINE_CAPACITY> {
    /// An empty frontier backed entirely by inline storage.
    pub(crate) fn new() -> Self {
        Self {
            entries: FrontierEntries::Inline(InlineHeap::new()),
            #[cfg(test)]
            metrics: FrontierMetrics::default(),
        }
    }

    /// Push an entry, draining inline entries into an allocated heap on
    /// the first push past `INLINE_CAPACITY`.
    pub(crate) fn push(&mut self, entry: T) {
        #[cfg(test)]
        {
            self.metrics.pushes += 1;
            self.metrics.high_water = self.metrics.high_water.max(self.len() + 1);
        }
        match &mut self.entries {
            FrontierEntries::Inline(entries) => {
                if let Err(entry) = entries.push(entry) {
                    let inline = replace(entries, InlineHeap::new());
                    let mut spilled = Self::spilled_heap(inline, INLINE_CAPACITY + 1);
                    spilled.push(entry);
                    self.entries = FrontierEntries::Spilled(spilled);
                }
            }
            FrontierEntries::Spilled(entries) => entries.push(entry),
        }
    }

    /// Push a known-size batch, reserving the complete spill boundary
    /// before moving inline entries.
    pub(crate) fn extend<I>(&mut self, entries: I)
    where
        I: ExactSizeIterator<Item = T>,
    {
        let incoming = entries.len();
        #[cfg(test)]
        {
            self.metrics.pushes += incoming;
            self.metrics.high_water = self.metrics.high_water.max(self.len() + incoming);
        }
        match &mut self.entries {
            FrontierEntries::Inline(frontier)
                if frontier.capacity() >= frontier.len() + incoming =>
            {
                for entry in entries {
                    let pushed = frontier.push(entry);
                    debug_assert!(pushed.is_ok(), "capacity was checked before extending");
                }
            }
            FrontierEntries::Inline(frontier) => {
                let capacity = frontier.len() + incoming;
                let inline = replace(frontier, InlineHeap::new());
                let mut spilled = Self::spilled_heap(inline, capacity);
                spilled.extend(entries);
                self.entries = FrontierEntries::Spilled(spilled);
            }
            FrontierEntries::Spilled(frontier) => frontier.extend(entries),
        }
    }

    /// Remove and return the greatest entry, or `None` when empty.
    pub(crate) fn pop(&mut self) -> Option<T> {
        #[cfg(test)]
        {
            self.metrics.pops += 1;
        }
        match &mut self.entries {
            FrontierEntries::Inline(entries) => entries.pop(),
            FrontierEntries::Spilled(entries) => entries.pop(),
        }
    }

    /// Borrow the greatest entry without removing it.
    pub(crate) fn peek(&self) -> Option<&T> {
        match &self.entries {
            FrontierEntries::Inline(entries) => entries.peek(),
            FrontierEntries::Spilled(entries) => entries.peek(),
        }
    }

    #[cfg(test)]
    pub(crate) fn metrics(&self) -> FrontierMetrics {
        self.metrics
    }

    #[cfg(test)]
    fn len(&self) -> usize {
        match &self.entries {
            FrontierEntries::Inline(entries) => entries.len(),
            FrontierEntries::Spilled(entries) => entries.len(),
        }
    }

    fn spilled_heap(inline: InlineHeap<T, Max, INLINE_CAPACITY>, capacity: usize) -> BinaryHeap<T> {
        let mut entries = BinaryHeap::with_capacity(capacity);
        entries.extend(inline.into_vec());
        entries
    }

    #[cfg(test)]
    fn is_spilled(&self) -> bool {
        match &self.entries {
            FrontierEntries::Inline(_) => false,
            FrontierEntries::Spilled(_) => true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{DEFAULT_INLINE_CAPACITY, SearchFrontier};
    use alloc::collections::BinaryHeap;
    use alloc::vec::Vec;
    use core::cmp::Reverse;

    fn lcg_keys(n: usize) -> Vec<u64> {
        let mut state: u64 = 0x9E37_79B9_7F4A_7C15;
        (0..n)
            .map(|_| {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                state >> 32
            })
            .collect()
    }

    fn drain<const CAPACITY: usize>(
        frontier: &mut SearchFrontier<Reverse<u64>, CAPACITY>,
    ) -> Vec<u64> {
        let mut popped = Vec::new();
        while let Some(Reverse(key)) = frontier.pop() {
            popped.push(key);
        }
        popped
    }

    #[test]
    fn pops_ascending_within_inline() {
        let keys = lcg_keys(DEFAULT_INLINE_CAPACITY - 1);
        let mut frontier: SearchFrontier<Reverse<u64>> = SearchFrontier::new();
        for &key in &keys {
            frontier.push(Reverse(key));
        }
        assert!(!frontier.is_spilled());
        let mut expected = keys;
        expected.sort_unstable();
        assert_eq!(drain(&mut frontier), expected);
    }

    fn assert_spill_boundary<const CAPACITY: usize>() {
        for n in [CAPACITY, CAPACITY + 1] {
            let keys = lcg_keys(n);
            let mut frontier: SearchFrontier<Reverse<u64>, CAPACITY> = SearchFrontier::new();
            for &key in &keys {
                frontier.push(Reverse(key));
            }
            assert_eq!(frontier.is_spilled(), n > CAPACITY);
            let mut expected = keys;
            expected.sort_unstable();
            assert_eq!(drain(&mut frontier), expected);
        }
    }

    #[test]
    fn spill_boundary() {
        assert_spill_boundary::<4>();
        assert_spill_boundary::<DEFAULT_INLINE_CAPACITY>();
    }

    #[test]
    fn interleaved_push_pop_matches_binary_heap() {
        let mut frontier: SearchFrontier<Reverse<u64>> = SearchFrontier::new();
        let mut reference: BinaryHeap<Reverse<u64>> = BinaryHeap::new();
        for (i, key) in lcg_keys(2 * DEFAULT_INLINE_CAPACITY)
            .into_iter()
            .enumerate()
        {
            frontier.push(Reverse(key));
            reference.push(Reverse(key));
            if i % 3 == 0 {
                assert_eq!(frontier.pop(), reference.pop());
            }
        }
        assert!(frontier.is_spilled());
        loop {
            let (got, expected) = (frontier.pop(), reference.pop());
            assert_eq!(got, expected);
            if got.is_none() {
                break;
            }
        }
    }
}
