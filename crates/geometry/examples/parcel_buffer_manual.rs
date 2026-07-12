//! The README quick-start, without the derive: implement the point
//! traits by hand instead of `#[derive(Point)]`. Everything else is
//! identical to `parcel_buffer.rs`.
//!
//! Run with `cargo run --example parcel_buffer_manual`.

use boost_geometry::adapt::{register_polygon, register_ring};
use boost_geometry::cs::Cartesian;
use boost_geometry::overlay::{JoinStrategy, buffer_convex_polygon, is_valid_polygon};
use boost_geometry::prelude::*;
use boost_geometry::tag::PointTag;
use boost_geometry::trait_::{Geometry, Point, PointMut};

// Your own point type, registered by hand — the escape hatch for
// computed coordinates, packed storage, or FFI structs the derive
// cannot express.
#[derive(Clone, Copy, Default)]
struct Coord {
    x: f64,
    y: f64,
}

impl Geometry for Coord {
    type Kind = PointTag;
    type Point = Coord;
}

impl Point for Coord {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;

    fn get<const D: usize>(&self) -> f64 {
        match D {
            0 => self.x,
            1 => self.y,
            _ => unreachable!(),
        }
    }
}

// Only needed by algorithms that construct points (buffer, correct).
impl PointMut for Coord {
    fn set<const D: usize>(&mut self, v: f64) {
        match D {
            0 => self.x = v,
            1 => self.y = v,
            _ => unreachable!(),
        }
    }
}

struct Boundary {
    points: Vec<Coord>,
}

struct Parcel {
    outer: Boundary,
    holes: Vec<Boundary>,
}

register_ring!(Boundary, Coord, |s| s.points.iter());
register_polygon!(
    Parcel,
    Coord,
    ring = Boundary,
    |s| outer = &s.outer,
    inners = s.holes.iter()
);

fn main() {
    let c = |x, y| Coord { x, y };

    // A 2×2 square parcel (clockwise, closed — the default ring convention).
    let parcel = Parcel {
        outer: Boundary {
            points: vec![
                c(0.0, 0.0),
                c(0.0, 2.0),
                c(2.0, 2.0),
                c(2.0, 0.0),
                c(0.0, 0.0),
            ],
        },
        holes: vec![],
    };

    println!("parcel valid: {:?}", is_valid_polygon(&parcel));

    let mut grown = buffer_convex_polygon(
        &parcel,
        1.0,
        JoinStrategy::Round {
            points_per_circle: 360,
        },
    );

    correct(&mut grown);
    println!("buffered valid: {:?}", is_valid_polygon(&grown));
    println!("area {:.3} -> {:.3}", area(&parcel), area(&grown));
}
