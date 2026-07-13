//! Allocation-budget gate for `Rtree::nearest` and the
//! `Rtree::nearest_iter` laziness budget, in its own integration
//! binary because it installs a `#[global_allocator]`.
//!
//! This binary MUST contain exactly one `#[test]`: the test harness
//! runs tests in parallel threads within one process, so any sibling
//! test's allocations would race the counter. That is also why the
//! fixture generators are local rather than `#[path]`-shared with
//! `parity` — including `fixture.rs` would pull its inline
//! `#[cfg(test)]` tests into this binary.

#![allow(
    unsafe_code,
    reason = "GlobalAlloc is an unsafe trait; the impl only counts and forwards to System"
)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering};

use geometry_rtree::{Bounds, Predicate, Rtree};

struct CountingAllocator;

static ALLOCATION_COUNT: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

const FIELD: f64 = 50_000.0;
const K: usize = 8;
const QUERY_COUNT: usize = 50;

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

fn squared_distance(p: [f64; 2], q: [f64; 2]) -> f64 {
    let dx = p[0] - q[0];
    let dy = p[1] - q[1];
    dx * dx + dy * dy
}

fn knn_scan(points: &[[f64; 2]], q: [f64; 2], k: usize) -> Vec<f64> {
    let mut dists: Vec<f64> = points.iter().map(|&p| squared_distance(p, q)).collect();
    dists.sort_unstable_by(f64::total_cmp);
    dists.truncate(k);
    dists
}

type Tree = Rtree<(Bounds, u32)>;

fn tree_of(points: &[[f64; 2]]) -> Tree {
    points
        .iter()
        .enumerate()
        .map(|(i, &p)| (Bounds::point(p), u32::try_from(i).expect("fits u32")))
        .collect()
}

fn allocations_across_queries(tree: &Tree, qs: &[[f64; 2]]) -> usize {
    let before = ALLOCATION_COUNT.load(Ordering::Relaxed);
    for &q in qs {
        black_box(tree.nearest(black_box(q), K));
    }
    ALLOCATION_COUNT.load(Ordering::Relaxed) - before
}

#[test]
#[allow(
    clippy::float_cmp,
    reason = "the oracle contract is exact f64 equality"
)]
fn per_query_allocations_constant_and_within_budget() {
    let qs = queries(QUERY_COUNT);
    let counts: Vec<usize> = [10_000, 100_000]
        .into_iter()
        .map(|n| {
            let points = uniform(n);
            let tree = tree_of(&points);
            black_box(tree.nearest(qs[0], K));
            allocations_across_queries(&tree, &qs)
        })
        .collect();
    assert_eq!(
        counts[0], counts[1],
        "per-query allocations must not scale with tree size"
    );
    // The ranked entries reuse their allocation when collected into the
    // returned value vector. The inline frontier does not allocate for
    // these K = 8 fixtures. The constancy assertion above is the
    // O(1)-in-tree-size guarantee.
    assert_eq!(
        counts[0], QUERY_COUNT,
        "nearest must allocate exactly one result buffer per query"
    );
    eprintln!(
        "[rtree-inline-frontier] expected_nearest_allocations={} observed_10k={} observed_100k={}",
        QUERY_COUNT, counts[0], counts[1],
    );

    let points = uniform(100_000);
    let tree = tree_of(&points);
    let q = qs[0];
    let (spill_k, result, spill_allocs) = [2_048, 4_096, 8_192, 16_384, 32_768]
        .into_iter()
        .find_map(|k| {
            let before = ALLOCATION_COUNT.load(Ordering::Relaxed);
            let result = tree.nearest(q, k);
            let allocations = ALLOCATION_COUNT.load(Ordering::Relaxed) - before;
            eprintln!(
                "[rtree-inline-frontier] expected_spill_allocations_above=2 observed_k={k} observed_allocations={allocations}"
            );
            (allocations > 2).then_some((k, result, allocations))
        })
        .expect("one bounded query must overflow the inline frontier");
    assert!(
        spill_allocs > 2,
        "k = {spill_k} on a 100k tree must overflow the inline frontier to exercise the spill path"
    );
    let dists: Vec<f64> = result
        .iter()
        .map(|(b, _)| squared_distance(b.min, q))
        .collect();
    assert_eq!(
        dists,
        knn_scan(&points, q, spill_k),
        "spilling query diverges from the scan oracle"
    );

    let stream_allocs: Vec<usize> = [10_000, 100_000]
        .into_iter()
        .map(|n| {
            let points = uniform(n);
            let tree = tree_of(&points);
            let before = ALLOCATION_COUNT.load(Ordering::Relaxed);
            let mut stream = tree.nearest_iter(black_box(q));
            black_box(stream.next());
            ALLOCATION_COUNT.load(Ordering::Relaxed) - before
        })
        .collect();
    assert!(
        stream_allocs[1] <= stream_allocs[0],
        "nearest_iter take-one must not allocate more on a 10x tree: {stream_allocs:?}"
    );
    // The frontier remains inline below its spill boundary.
    assert_eq!(
        stream_allocs[0], 0,
        "nearest_iter take-one must not allocate below the spill boundary"
    );
    eprintln!(
        "[rtree-inline-frontier] expected_take_one_allocations=0 observed_10k={} observed_100k={}",
        stream_allocs[0], stream_allocs[1],
    );

    let world = Predicate::Intersects(Bounds::new([0.0, 0.0], [FIELD, FIELD]));
    let dump_allocs: Vec<usize> = [10_000, 100_000]
        .into_iter()
        .map(|n| {
            let points = uniform(n);
            let tree = tree_of(&points);
            let before = ALLOCATION_COUNT.load(Ordering::Relaxed);
            let mut dump = tree.query_iter(black_box(world));
            black_box(dump.next());
            drop(dump);
            ALLOCATION_COUNT.load(Ordering::Relaxed) - before
        })
        .collect();
    assert_eq!(
        dump_allocs[0], dump_allocs[1],
        "query_iter whole-world take-one allocations must not scale with tree size: {dump_allocs:?}"
    );
}
