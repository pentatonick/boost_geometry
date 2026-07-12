// Mirrors `boost/geometry/test/concepts/point_with_incorrect_dimension.cpp`.
//
// `DIM` is declared as 3 but the backing storage is `[f64; 2]`. The
// const-assert guard inside `get` / `set` is fine for `D in 0..3`, but
// `materialise` / `fold_dims` / any algorithm that reads dimension 2
// will index out of bounds. We surface the inconsistency at the impl
// site by writing the same `const { assert! }` guard the well-formed
// fixtures use, but checking `DIM <= LEN` against the array length.

use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{check_point, Geometry, Point, PointMut};

struct WrongDim {
    v: [f64; 2],
}

impl Geometry for WrongDim {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for WrongDim {
    type Scalar = f64;
    type Cs = Cartesian;
    // BUG: backing storage is only 2 wide.
    const DIM: usize = 3;

    fn get<const D: usize>(&self) -> f64 {
        const {
            assert!(
                <WrongDim as Point>::DIM as usize <= 2,
                "Point::DIM exceeds the backing array length"
            );
        }
        self.v[D]
    }
}

impl PointMut for WrongDim {
    fn set<const D: usize>(&mut self, v: f64) {
        const {
            assert!(
                <WrongDim as Point>::DIM as usize <= 2,
                "Point::DIM exceeds the backing array length"
            );
        }
        self.v[D] = v;
    }
}

fn main() {
    check_point::<WrongDim>();
    // Force monomorphisation so the const-assert fires.
    let mut p = WrongDim { v: [0.0; 2] };
    let _ = p.get::<0>();
    p.set::<0>(1.0);
}
