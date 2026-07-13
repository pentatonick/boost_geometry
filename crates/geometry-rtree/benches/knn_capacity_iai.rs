//! Query-only nearest-neighbour comparison for the retained defaults.
//!
//! Trees and queries are constructed outside Callgrind. Run with one
//! codegen unit for reproducible counts:
//!
//! ```text
//! RUSTFLAGS='-Ccodegen-units=1' cargo bench --bench knn_capacity_iai
//! ```

use std::hint::black_box;

use geometry_rtree::{Bounds, Rtree};
use iai_callgrind::{library_benchmark, library_benchmark_group, main};
use rstar::RTree;
use rstar::primitives::GeomWithData;

#[path = "../tests/parity/fixture.rs"]
#[allow(dead_code, reason = "the bench uses a subset of the shared fixture")]
mod fixture;

const N: usize = 50_000;
const Q: usize = 100;
const K: usize = 8;

type BoostValue = (Bounds, u32);
type RstarValue = GeomWithData<[f64; 2], u32>;

struct BoostFixture {
    tree: Rtree<BoostValue>,
    queries: Vec<[f64; 2]>,
}

struct RstarFixture {
    tree: RTree<RstarValue>,
    queries: Vec<[f64; 2]>,
}

fn boost_fixture(points: Vec<[f64; 2]>) -> &'static BoostFixture {
    let values = points.into_iter().enumerate().map(|(index, point)| {
        (
            Bounds::point(point),
            u32::try_from(index).expect("N fits u32"),
        )
    });
    Box::leak(Box::new(BoostFixture {
        tree: values.collect(),
        queries: fixture::queries(Q),
    }))
}

fn rstar_fixture(points: Vec<[f64; 2]>) -> &'static RstarFixture {
    let values = points
        .into_iter()
        .enumerate()
        .map(|(index, point)| GeomWithData::new(point, u32::try_from(index).expect("N fits u32")))
        .collect();
    Box::leak(Box::new(RstarFixture {
        tree: RTree::bulk_load(values),
        queries: fixture::queries(Q),
    }))
}

#[inline(never)]
fn rstar_workload(fixture: &'static RstarFixture) -> u64 {
    let mut sum = 0;
    for query in &fixture.queries {
        for value in fixture.tree.nearest_neighbor_iter(query).take(K) {
            sum += u64::from(value.data);
        }
    }
    black_box(sum)
}

#[inline(never)]
fn boost_streaming_workload(fixture: &'static BoostFixture) -> u64 {
    let mut sum = 0;
    for query in &fixture.queries {
        for (_, id) in fixture.tree.nearest_iter(*query).take(K) {
            sum += u64::from(*id);
        }
    }
    black_box(sum)
}

#[inline(never)]
fn boost_bounded_workload(fixture: &'static BoostFixture) -> u64 {
    let mut sum = 0;
    for query in &fixture.queries {
        for (_, id) in fixture.tree.nearest(*query, K) {
            sum += u64::from(*id);
        }
    }
    black_box(sum)
}

macro_rules! fixtures {
    ($boost:ident, $rstar:ident, $points:path) => {
        fn $boost() -> &'static BoostFixture {
            boost_fixture($points(N))
        }

        fn $rstar() -> &'static RstarFixture {
            rstar_fixture($points(N))
        }
    };
}

fixtures!(
    boost_uniform_fixture,
    rstar_uniform_fixture,
    fixture::uniform
);
fixtures!(
    boost_clustered_fixture,
    rstar_clustered_fixture,
    fixture::clustered
);

#[library_benchmark(setup = rstar_uniform_fixture)]
fn rstar_uniform(fixture: &'static RstarFixture) -> u64 {
    rstar_workload(fixture)
}

#[library_benchmark(setup = rstar_clustered_fixture)]
fn rstar_clustered(fixture: &'static RstarFixture) -> u64 {
    rstar_workload(fixture)
}

#[library_benchmark(setup = boost_uniform_fixture)]
fn boost_streaming_uniform(fixture: &'static BoostFixture) -> u64 {
    boost_streaming_workload(fixture)
}

#[library_benchmark(setup = boost_clustered_fixture)]
fn boost_streaming_clustered(fixture: &'static BoostFixture) -> u64 {
    boost_streaming_workload(fixture)
}

#[library_benchmark(setup = boost_uniform_fixture)]
fn boost_bounded_uniform(fixture: &'static BoostFixture) -> u64 {
    boost_bounded_workload(fixture)
}

#[library_benchmark(setup = boost_clustered_fixture)]
fn boost_bounded_clustered(fixture: &'static BoostFixture) -> u64 {
    boost_bounded_workload(fixture)
}

library_benchmark_group!(
    name = knn_capacity;
    benchmarks =
        rstar_uniform,
        rstar_clustered,
        boost_streaming_uniform,
        boost_streaming_clustered,
        boost_bounded_uniform,
        boost_bounded_clustered
);

main!(library_benchmark_groups = knn_capacity);
