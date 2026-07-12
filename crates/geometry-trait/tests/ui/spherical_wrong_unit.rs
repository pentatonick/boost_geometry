//! Mirrors `boost/geometry/test/concepts/
//! point_spherical_with_wrong_units.cpp`: passing `f64` as the
//! angle-unit type to `Spherical<U>` (instead of `Degree` or
//! `Radian`) must fail to compile because `f64` is not an
//! `AngleUnit`.
//!
//! The point type is hand-rolled (rather than `geometry_model::Point`)
//! so the fixture stays inside the `geometry-trait` dependency layer —
//! the `AngleUnit` rejection lives in `geometry-cs` and needs nothing
//! from `geometry-model`.

use geometry_cs::Spherical;
use geometry_tag::PointTag;
use geometry_trait::{check_point, Geometry, Point};

struct BadPoint(f64, f64);

impl Geometry for BadPoint {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for BadPoint {
    type Scalar = f64;
    // `Spherical<f64>` — `f64` is not an `AngleUnit`. `Spherical<U>` is
    // declared `Spherical<U: AngleUnit>`, so naming it here is the
    // compile error.
    type Cs = Spherical<f64>;
    const DIM: usize = 2;
    fn get<const D: usize>(&self) -> f64 {
        if D == 0 {
            self.0
        } else {
            self.1
        }
    }
}

fn main() {
    let _ = check_point::<BadPoint>();
}
