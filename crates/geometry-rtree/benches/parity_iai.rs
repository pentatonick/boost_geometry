//! Stable iai-callgrind parity matrix for the retained R-tree defaults.
//!
//! N = 50k, Q = 100, K = 8. Total workloads include construction;
//! `*_query_only` rows build fixtures in setup. Run with one codegen
//! unit so unrelated monomorphizations cannot perturb the comparison:
//!
//! ```text
//! RUSTFLAGS='-Ccodegen-units=1' cargo bench --bench parity_iai
//! ```

use std::hint::black_box;

use geometry_rtree::{Bounds, Predicate, Rtree};
use iai_callgrind::{library_benchmark, library_benchmark_group, main};
use rstar::primitives::GeomWithData;
use rstar::{AABB, RTree};

#[path = "../tests/parity/fixture.rs"]
#[allow(dead_code, reason = "the bench uses a subset of the shared fixture")]
mod fixture;

const N: usize = 50_000;
const Q: usize = 100;
const K: usize = 8;
const HALF: f64 = 500.0;

type BoostValue = (Bounds, u32);
type RstarValue = GeomWithData<[f64; 2], u32>;

struct BoostKnnFixture {
    tree: Rtree<BoostValue>,
    queries: Vec<[f64; 2]>,
}

struct RstarKnnFixture {
    tree: RTree<RstarValue>,
    queries: Vec<[f64; 2]>,
}

fn boost_values(points: Vec<[f64; 2]>) -> Vec<BoostValue> {
    points
        .into_iter()
        .enumerate()
        .map(|(index, point)| {
            (
                Bounds::point(point),
                u32::try_from(index).expect("N fits u32"),
            )
        })
        .collect()
}

fn rstar_values(points: Vec<[f64; 2]>) -> Vec<RstarValue> {
    points
        .into_iter()
        .enumerate()
        .map(|(index, point)| GeomWithData::new(point, u32::try_from(index).expect("N fits u32")))
        .collect()
}

fn boost_bulk_fixture(points: Vec<[f64; 2]>) -> &'static BoostKnnFixture {
    Box::leak(Box::new(BoostKnnFixture {
        tree: boost_values(points).into_iter().collect(),
        queries: fixture::queries(Q),
    }))
}

fn rstar_bulk_fixture(points: Vec<[f64; 2]>) -> &'static RstarKnnFixture {
    Box::leak(Box::new(RstarKnnFixture {
        tree: RTree::bulk_load(rstar_values(points)),
        queries: fixture::queries(Q),
    }))
}

fn boost_inserted_fixture(points: Vec<[f64; 2]>) -> &'static BoostKnnFixture {
    let mut tree = Rtree::new();
    for value in boost_values(points) {
        tree.insert(value);
    }
    Box::leak(Box::new(BoostKnnFixture {
        tree,
        queries: fixture::queries(Q),
    }))
}

fn rstar_inserted_fixture(points: Vec<[f64; 2]>) -> &'static RstarKnnFixture {
    let mut tree = RTree::new();
    for value in rstar_values(points) {
        tree.insert(value);
    }
    Box::leak(Box::new(RstarKnnFixture {
        tree,
        queries: fixture::queries(Q),
    }))
}

fn boost_bulk_uniform_fixture() -> &'static BoostKnnFixture {
    boost_bulk_fixture(fixture::uniform(N))
}

fn rstar_bulk_uniform_fixture() -> &'static RstarKnnFixture {
    rstar_bulk_fixture(fixture::uniform(N))
}

fn boost_inserted_uniform_fixture() -> &'static BoostKnnFixture {
    boost_inserted_fixture(fixture::uniform(N))
}

fn boost_inserted_clustered_fixture() -> &'static BoostKnnFixture {
    boost_inserted_fixture(fixture::clustered(N))
}

fn rstar_inserted_uniform_fixture() -> &'static RstarKnnFixture {
    rstar_inserted_fixture(fixture::uniform(N))
}

fn rstar_inserted_clustered_fixture() -> &'static RstarKnnFixture {
    rstar_inserted_fixture(fixture::clustered(N))
}

#[inline(never)]
fn rstar_knn_workload(fixture: &RstarKnnFixture) -> u64 {
    let mut sum = 0;
    for query in &fixture.queries {
        for value in fixture.tree.nearest_neighbor_iter(query).take(K) {
            sum += u64::from(value.data);
        }
    }
    black_box(sum)
}

#[inline(never)]
fn boost_streaming_workload(fixture: &BoostKnnFixture) -> u64 {
    let mut sum = 0;
    for query in &fixture.queries {
        for (_, id) in fixture.tree.nearest_iter(*query).take(K) {
            sum += u64::from(*id);
        }
    }
    black_box(sum)
}

#[inline(never)]
fn boost_bounded_workload(fixture: &BoostKnnFixture) -> u64 {
    let mut sum = 0;
    for query in &fixture.queries {
        for (_, id) in fixture.tree.nearest(*query, K) {
            sum += u64::from(*id);
        }
    }
    black_box(sum)
}

#[library_benchmark]
fn rstar_build_insert() -> RTree<RstarValue> {
    let mut tree = RTree::new();
    for value in rstar_values(fixture::uniform(N)) {
        tree.insert(value);
    }
    black_box(tree)
}

#[library_benchmark]
fn boost_build_insert() -> Rtree<BoostValue> {
    let mut tree = Rtree::new();
    for value in boost_values(fixture::uniform(N)) {
        tree.insert(value);
    }
    black_box(tree)
}

#[library_benchmark]
fn rstar_build_bulk() -> RTree<RstarValue> {
    black_box(RTree::bulk_load(rstar_values(fixture::uniform(N))))
}

#[library_benchmark]
fn boost_build_bulk() -> Rtree<BoostValue> {
    black_box(boost_values(fixture::uniform(N)).into_iter().collect())
}

#[library_benchmark]
fn rstar_knn() -> u64 {
    let fixture = RstarKnnFixture {
        tree: RTree::bulk_load(rstar_values(fixture::uniform(N))),
        queries: fixture::queries(Q),
    };
    rstar_knn_workload(&fixture)
}

#[library_benchmark]
fn boost_knn() -> u64 {
    let fixture = BoostKnnFixture {
        tree: boost_values(fixture::uniform(N)).into_iter().collect(),
        queries: fixture::queries(Q),
    };
    boost_streaming_workload(&fixture)
}

#[library_benchmark]
fn boost_knn_bounded() -> u64 {
    let fixture = BoostKnnFixture {
        tree: boost_values(fixture::uniform(N)).into_iter().collect(),
        queries: fixture::queries(Q),
    };
    boost_bounded_workload(&fixture)
}

#[library_benchmark(setup = rstar_bulk_uniform_fixture)]
fn rstar_knn_query_only(fixture: &'static RstarKnnFixture) -> u64 {
    rstar_knn_workload(fixture)
}

#[library_benchmark(setup = boost_bulk_uniform_fixture)]
fn boost_knn_query_only(fixture: &'static BoostKnnFixture) -> u64 {
    boost_streaming_workload(fixture)
}

#[library_benchmark(setup = boost_bulk_uniform_fixture)]
fn boost_knn_bounded_query_only(fixture: &'static BoostKnnFixture) -> u64 {
    boost_bounded_workload(fixture)
}

#[library_benchmark(setup = rstar_inserted_uniform_fixture)]
fn rstar_knn_inserted_query_only(fixture: &'static RstarKnnFixture) -> u64 {
    rstar_knn_workload(fixture)
}

#[library_benchmark(setup = rstar_inserted_clustered_fixture)]
fn rstar_knn_inserted_clustered_query_only(fixture: &'static RstarKnnFixture) -> u64 {
    rstar_knn_workload(fixture)
}

#[library_benchmark(setup = boost_inserted_uniform_fixture)]
fn boost_knn_inserted_query_only(fixture: &'static BoostKnnFixture) -> u64 {
    boost_bounded_workload(fixture)
}

#[library_benchmark(setup = boost_inserted_clustered_fixture)]
fn boost_knn_inserted_clustered_query_only(fixture: &'static BoostKnnFixture) -> u64 {
    boost_bounded_workload(fixture)
}

#[library_benchmark]
fn rstar_range() -> u64 {
    rstar_range_workload(fixture::uniform(N))
}

#[library_benchmark]
fn rstar_range_clustered() -> u64 {
    rstar_range_workload(fixture::clustered(N))
}

fn rstar_range_workload(points: Vec<[f64; 2]>) -> u64 {
    let tree = RTree::bulk_load(rstar_values(points));
    let mut sum = 0;
    for query in fixture::queries(Q) {
        let window = AABB::from_corners(
            [query[0] - HALF, query[1] - HALF],
            [query[0] + HALF, query[1] + HALF],
        );
        for value in tree.locate_in_envelope(&window) {
            sum += u64::from(value.data);
        }
    }
    black_box(sum)
}

#[library_benchmark]
fn boost_range() -> u64 {
    boost_range_workload(fixture::uniform(N))
}

#[library_benchmark]
fn boost_range_clustered() -> u64 {
    boost_range_workload(fixture::clustered(N))
}

fn boost_range_workload(points: Vec<[f64; 2]>) -> u64 {
    let tree: Rtree<BoostValue> = boost_values(points).into_iter().collect();
    let mut sum = 0;
    for query in fixture::queries(Q) {
        let window = Bounds::new(
            [query[0] - HALF, query[1] - HALF],
            [query[0] + HALF, query[1] + HALF],
        );
        for (_, id) in tree.query_iter(Predicate::Intersects(window)) {
            sum += u64::from(*id);
        }
    }
    black_box(sum)
}

library_benchmark_group!(
    name = parity;
    benchmarks =
        rstar_build_insert,
        boost_build_insert,
        rstar_build_bulk,
        boost_build_bulk,
        rstar_knn,
        boost_knn,
        boost_knn_bounded,
        rstar_knn_query_only,
        boost_knn_query_only,
        boost_knn_bounded_query_only,
        rstar_range,
        boost_range,
        rstar_range_clustered,
        boost_range_clustered
);

library_benchmark_group!(
    name = inserted_knn;
    benchmarks =
        rstar_knn_inserted_query_only,
        boost_knn_inserted_query_only,
        rstar_knn_inserted_clustered_query_only,
        boost_knn_inserted_clustered_query_only
);

main!(library_benchmark_groups = parity, inserted_knn);
