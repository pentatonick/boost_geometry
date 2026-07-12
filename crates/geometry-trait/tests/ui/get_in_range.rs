// Pair fixture for `get_out_of_range.rs`: an in-range `D` compiles
// cleanly. Registering at least one passing test in the trybuild
// harness forces `cargo build` rather than `cargo check`, which is
// what we need for the const-assert in `Xy::get` to be evaluated
// (const evaluation of monomorphisation-instantiated function
// bodies happens at codegen time).

use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut};

struct Xy {
    v: [f64; 2],
}

impl Geometry for Xy {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for Xy {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;

    fn get<const D: usize>(&self) -> f64 {
        const { assert!(D < <Xy as Point>::DIM, "Point::get: dimension out of range"); }
        self.v[D]
    }
}

impl PointMut for Xy {
    fn set<const D: usize>(&mut self, v: f64) {
        const { assert!(D < <Xy as Point>::DIM, "Point::set: dimension out of range"); }
        self.v[D] = v;
    }
}

fn main() {
    let p = Xy { v: [1.0, 2.0] };
    let _ = p.get::<0>();
    let _ = p.get::<1>();
}
