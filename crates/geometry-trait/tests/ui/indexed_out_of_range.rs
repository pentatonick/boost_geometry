// Verifies that an out-of-range `I` (corner index) on
// `IndexedAccess::get_indexed::<I, D>()` is a compile error rather
// than a runtime panic — the indexed counterpart to
// `get_out_of_range.rs` (proposal §3.2).
//
// Mirrors the spirit of
// `boost/geometry/test/concepts/box_concept_checker.cpp`, which
// asserts the Box concept holds at compile time. The Boost C++ side
// pins the corner index domain via `min_corner = 0` /
// `max_corner = 1` (`boost/geometry/core/access.hpp:36-39`); a
// `const { assert!(I < 2) }` block inside `get_indexed` is the
// idiomatic Rust spelling.

use geometry_cs::Cartesian;
use geometry_tag::{BoxTag, PointTag};
use geometry_trait::{Geometry, IndexedAccess, Point, PointMut};

struct P2(f64, f64);

impl Geometry for P2 {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for P2 {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;

    fn get<const D: usize>(&self) -> f64 {
        const { assert!(D < <P2 as Point>::DIM, "Point::get: dimension out of range"); }
        if D == 0 { self.0 } else { self.1 }
    }
}

impl PointMut for P2 {
    fn set<const D: usize>(&mut self, v: f64) {
        const { assert!(D < <P2 as Point>::DIM, "Point::set: dimension out of range"); }
        if D == 0 { self.0 = v } else { self.1 = v }
    }
}

struct B {
    corners: [[f64; 2]; 2],
}

impl Geometry for B {
    type Kind = BoxTag;
    type Point = P2;
}

impl IndexedAccess for B {
    fn get_indexed<const I: usize, const D: usize>(&self) -> f64 {
        const { assert!(I < 2, "IndexedAccess::get_indexed: index out of range"); }
        const { assert!(D < 2, "IndexedAccess::get_indexed: dimension out of range"); }
        self.corners[I][D]
    }
    fn set_indexed<const I: usize, const D: usize>(&mut self, v: f64) {
        const { assert!(I < 2, "IndexedAccess::set_indexed: index out of range"); }
        const { assert!(D < 2, "IndexedAccess::set_indexed: dimension out of range"); }
        self.corners[I][D] = v;
    }
}

fn main() {
    let b = B { corners: [[0.0; 2]; 2] };
    // Should fail to compile — the `const { assert! }` guard inside
    // `B::get_indexed` rejects `I = 2` at monomorphisation time.
    let _ = b.get_indexed::<2, 0>();
}
