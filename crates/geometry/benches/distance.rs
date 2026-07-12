//! CC1.T2 — distance benchmarks.
//!
//! Baseline for `distance` across coordinate-system families: Cartesian
//! Pythagoras, and the geographic strategies (Haversine / Andoyer). Run
//! with `cargo bench --bench distance`.

use criterion::{Criterion, black_box, criterion_group, criterion_main};

use boost_geometry::adapt::{Adapt, WithCs};
use boost_geometry::cs::{Cartesian, Degree, Geographic};
use boost_geometry::model::Point2D;
use boost_geometry::prelude::distance;

fn bench_distance(c: &mut Criterion) {
    let mut group = c.benchmark_group("distance");

    // Cartesian Pythagoras.
    let a = Point2D::<f64, Cartesian>::new(0.0, 0.0);
    let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
    group.bench_function("cartesian", |bencher| {
        bencher.iter(|| distance(black_box(&a), black_box(&b)));
    });

    // Geographic (Amsterdam → Paris), degrees on WGS84.
    let ams = WithCs::<_, Geographic<Degree>>::new(Adapt([4.90_f64, 52.37]));
    let par = WithCs::<_, Geographic<Degree>>::new(Adapt([2.35_f64, 48.86]));
    group.bench_function("geographic", |bencher| {
        bencher.iter(|| distance(black_box(&ams), black_box(&par)));
    });

    group.finish();
}

criterion_group!(benches, bench_distance);
criterion_main!(benches);
