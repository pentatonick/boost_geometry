//! Mirrors the `ro_point` half of `boost/geometry/test/concepts/
//! point_concept_checker.cpp` lines 19-46: an immutable point type
//! that satisfies `Point` (read) but deliberately *not* `PointMut`
//! (write).
#![allow(
    clippy::float_cmp,
    reason = "Coordinates are read back unchanged from literals."
)]

use geometry_cs::Cartesian;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, check_point};
// Note: `distance` lives in geometry-algorithm. KC1.T3 keeps this
// fixture inside `geometry-trait` so it does not pull
// `geometry-algorithm` as a dev-dep on the trait crate; the matching
// distance test lives in M-KC1 (`crates/geometry/tests/readonly_point.rs`).

#[derive(Default)]
struct RoPoint(f64, f64);

impl Geometry for RoPoint {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for RoPoint {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;

    #[inline]
    fn get<const D: usize>(&self) -> f64 {
        if D == 0 { self.0 } else { self.1 }
    }
}

// Note: NO `impl PointMut for RoPoint`. That is the whole point.

#[test]
fn ro_point_satisfies_point_concept() {
    check_point::<RoPoint>();
}

#[test]
fn ro_point_round_trip_read() {
    let p = RoPoint(3.0, 4.0);
    assert_eq!(p.get::<0>(), 3.0);
    assert_eq!(p.get::<1>(), 4.0);
}
