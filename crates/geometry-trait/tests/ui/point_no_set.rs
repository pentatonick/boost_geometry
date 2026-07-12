// Mirrors `boost/geometry/test/concepts/point_without_setter.cpp`.
//
// After the KC1.T1 split, a point with only a getter and no `set` is a
// *valid* read-only `Point` — the write half moved to the separate
// `PointMut` subtrait. So this fixture now COMPILES: `check_point`
// requires only `Point`. It is registered as a `pass` fixture. The
// "a mutating algorithm needs a setter" failure is covered by the
// `PointMut`-requiring fixture introduced in KC1.T3.

use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{check_point, Geometry, Point};

struct NoSet {
    v: [f64; 2],
}

impl Geometry for NoSet {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for NoSet {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;

    fn get<const D: usize>(&self) -> f64 {
        self.v[D]
    }
    // Missing: `fn set<const D: usize>(&mut self, v: f64) { … }`
}

fn main() {
    check_point::<NoSet>();
}
