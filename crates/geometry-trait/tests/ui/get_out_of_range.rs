// Verifies that an out-of-range `D` on `Point::get::<D>()` is a
// compile error rather than a runtime panic — the central payoff
// of the const-generic access design (proposal §3.2).
//
// Mirrors the spirit of
// `boost/geometry/test/concepts/point_concept_checker.cpp`, which
// asserts the concept holds at compile time.
//
// Note: plain `self.v[D]` on a `[f64; 2]` is NOT caught by rustc
// at const-eval — array indexing with an out-of-range const-generic
// becomes a runtime panic. The idiomatic Rust spelling of Boost's
// `BOOST_STATIC_ASSERT(Dim < traits::dimension<P>::value)` is a
// `const { assert!(D < DIM) }` block at the top of `get`. We expect
// downstream `Point` impls (`Adapt<[T; N]>`, the `model::Point`,
// `derive(Point)` output) to emit exactly this guard.

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
    let p = Xy { v: [0.0, 0.0] };
    // Should fail to compile — the `const { assert! }` guard inside
    // `Xy::get` rejects `D = 2` at monomorphisation time.
    let _ = p.get::<2>();
}
