// A crate that depends on `boost_geometry` alone — no `geometry-trait`,
// `geometry-tag`, or `geometry-cs` in its `Cargo.toml` — can use
// `#[derive(Point)]`: the generated impls reach the kernel traits through
// the facade's `__private` re-exports, and a parameterised coordinate
// system needs no extra import. trybuild compiles this file in a scratch
// package whose only geometry dependency is the facade.
use boost_geometry::Point;

#[derive(Default, Point)]
#[geometry(cs = "Cartesian", scalar = "f64")]
struct Planar {
    x: f64,
    y: f64,
}

#[derive(Default, Point)]
#[geometry(cs = "Spherical<Degree>", scalar = "f64")]
struct LonLat {
    lon: f64,
    lat: f64,
}

fn main() {
    let a = Planar { x: 0.0, y: 0.0 };
    let b = Planar { x: 3.0, y: 4.0 };
    assert_eq!(boost_geometry::algorithm::distance(&a, &b), 5.0);
    let _ = LonLat::default();
}
