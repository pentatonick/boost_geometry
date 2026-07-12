//! Mirrors `boost/geometry/test/concepts/
//! point_geographic_with_wrong_units.cpp`: same shape as
//! `spherical_wrong_unit.rs` but against `Geographic<f64>` — `f64` is
//! not an `AngleUnit`, so naming `Geographic<f64>` is a compile error.

use geometry_cs::Geographic;
use geometry_tag::PointTag;
use geometry_trait::{check_point, Geometry, Point};

struct BadPoint(f64, f64);

impl Geometry for BadPoint {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for BadPoint {
    type Scalar = f64;
    type Cs = Geographic<f64>;
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
