//! KC1.T3 — handing a read-only `RoPoint` to a `PointMut`-requiring
//! site (here: `segment_start`, which materialises a fresh `Point`
//! via `Default + set::<D>`) must fail at compile time with the
//! friendly diagnostic plate from KC1.T1.

use geometry_cs::Cartesian;
use geometry_tag::{PointTag, SegmentTag};
use geometry_trait::{segment_start, Geometry, IndexedAccess, Point, Segment};

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
    fn get<const D: usize>(&self) -> f64 {
        if D == 0 {
            self.0
        } else {
            self.1
        }
    }
}
// No `impl PointMut for RoPoint`.

struct RoSegment {
    e: [[f64; 2]; 2],
}
impl Geometry for RoSegment {
    type Kind = SegmentTag;
    type Point = RoPoint;
}
impl IndexedAccess for RoSegment {
    fn get_indexed<const I: usize, const D: usize>(&self) -> f64 {
        self.e[I][D]
    }
    fn set_indexed<const I: usize, const D: usize>(&mut self, v: f64) {
        self.e[I][D] = v;
    }
}
impl Segment for RoSegment {}

fn main() {
    let s = RoSegment { e: [[0.0; 2]; 2] };
    // segment_start requires `S::Point: Default + PointMut` — RoPoint
    // is Default but not PointMut.
    let _ = segment_start(&s);
}
