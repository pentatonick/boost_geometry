// Mirrors `boost/geometry/test/concepts/point_without_coordinate_type.cpp`.
//
// The `impl Point` block here is missing the `type Scalar` associated
// type. Rustc rejects the impl with E0046 — "not all trait items
// implemented, missing: `Scalar`" — *and* refuses the subsequent
// `check_point::<NoScalar>()` call because `NoScalar` doesn't implement
// `Point`.

use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{check_point, Geometry, Point};

struct NoScalar {
    v: [f64; 2],
}

impl Geometry for NoScalar {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for NoScalar {
    // Missing: `type Scalar = f64;`
    type Cs = Cartesian;
    const DIM: usize = 2;

    fn get<const D: usize>(&self) -> f64 {
        self.v[D]
    }
}

fn main() {
    check_point::<NoScalar>();
}
