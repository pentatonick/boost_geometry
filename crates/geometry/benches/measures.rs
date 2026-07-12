//! CC1.T3 — length / perimeter / area benchmarks.
//!
//! Baseline for the Cartesian measure algorithms over a fixed
//! linestring and polygon. Run with `cargo bench --bench measures`.

use criterion::{Criterion, black_box, criterion_group, criterion_main};

use boost_geometry::cs::Cartesian;
use boost_geometry::model::{Linestring, Point2D, Polygon};
use boost_geometry::prelude::{area, length, perimeter};

type P = Point2D<f64, Cartesian>;

/// A 64-vertex zig-zag linestring.
fn sample_linestring() -> Linestring<P> {
    let pts = (0..64)
        .map(|i| {
            let x = f64::from(i);
            let y = if i % 2 == 0 { 0.0 } else { 1.0 };
            P::new(x, y)
        })
        .collect();
    Linestring::from_vec(pts)
}

/// A regular 64-gon polygon.
fn sample_polygon() -> Polygon<P> {
    let mut pts: Vec<P> = (0..64)
        .map(|i| {
            let a = core::f64::consts::TAU * f64::from(i) / 64.0;
            P::new(a.cos(), a.sin())
        })
        .collect();
    pts.push(pts[0]);
    Polygon::new(boost_geometry::model::Ring::from_vec(pts))
}

fn bench_measures(c: &mut Criterion) {
    let ls = sample_linestring();
    let pg = sample_polygon();

    let mut group = c.benchmark_group("measures");
    group.bench_function("length_linestring", |b| {
        b.iter(|| length(black_box(&ls)));
    });
    group.bench_function("perimeter_polygon", |b| {
        b.iter(|| perimeter(black_box(&pg)));
    });
    group.bench_function("area_polygon", |b| {
        b.iter(|| area(black_box(&pg)));
    });
    group.finish();
}

criterion_group!(benches, bench_measures);
criterion_main!(benches);
