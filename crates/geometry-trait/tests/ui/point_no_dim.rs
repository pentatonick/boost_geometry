// Mirrors `boost/geometry/test/concepts/point_without_dimension.cpp`.
//
// The `impl Point` block here is missing the `const DIM` item. Rustc
// rejects the impl with E0046 — "not all trait items implemented,
// missing: `DIM`".

use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{check_point, Geometry, Point};

struct NoDim {
    v: [f64; 2],
}

impl Geometry for NoDim {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for NoDim {
    type Scalar = f64;
    type Cs = Cartesian;
    // Missing: `const DIM: usize = 2;`

    fn get<const D: usize>(&self) -> f64 {
        self.v[D]
    }
}

fn main() {
    check_point::<NoDim>();
}
