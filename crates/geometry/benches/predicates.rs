//! CC1.T4 — within / envelope / intersects benchmarks.
//!
//! Baseline for the spatial predicates over a fixed polygon and query.
//! Run with `cargo bench --bench predicates`.

use criterion::{Criterion, black_box, criterion_group, criterion_main};

use boost_geometry::cs::Cartesian;
use boost_geometry::model::{Point2D, Polygon, Ring};
use boost_geometry::prelude::{envelope, intersects, within};

type P = Point2D<f64, Cartesian>;

/// A regular 64-gon polygon.
fn sample_polygon() -> Polygon<P> {
    let mut pts: Vec<P> = (0..64)
        .map(|i| {
            let a = core::f64::consts::TAU * f64::from(i) / 64.0;
            P::new(4.0 * a.cos(), 4.0 * a.sin())
        })
        .collect();
    pts.push(pts[0]);
    Polygon::new(Ring::from_vec(pts))
}

fn bench_predicates(c: &mut Criterion) {
    let pg = sample_polygon();
    let inside = P::new(0.0, 0.0);
    let a = pg.clone();
    let b = sample_polygon();

    let mut group = c.benchmark_group("predicates");
    group.bench_function("within_point_polygon", |bencher| {
        bencher.iter(|| within(black_box(&inside), black_box(&pg)));
    });
    group.bench_function("envelope_polygon", |bencher| {
        bencher.iter(|| envelope(black_box(&pg)));
    });
    group.bench_function("intersects_polygon_polygon", |bencher| {
        bencher.iter(|| intersects(black_box(&a), black_box(&b)));
    });
    group.finish();
}

criterion_group!(benches, bench_predicates);
criterion_main!(benches);
