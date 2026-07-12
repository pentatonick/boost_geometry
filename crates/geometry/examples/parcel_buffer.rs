//! The README quick-start: register your own ring and polygon types,
//! validate them, and buffer with round joins.
//!
//! Run with `cargo run --example parcel_buffer`.

use boost_geometry::adapt::{register_polygon, register_ring};
use boost_geometry::overlay::{JoinStrategy, buffer_convex_polygon, is_valid_polygon};
use boost_geometry::prelude::*;

type P = Point2D<f64, Cartesian>;

// Your own geometry types — no wrapper, no conversion trait.
struct Boundary {
    points: Vec<P>,
}

struct Parcel {
    outer: Boundary,
    holes: Vec<Boundary>,
}

// One declaration each and the library's algorithms accept them.
register_ring!(Boundary, P, |s| s.points.iter());
register_polygon!(
    Parcel,
    P,
    ring = Boundary,
    |s| outer = &s.outer,
    inners = s.holes.iter()
);

fn main() {
    // A 2×2 square parcel (clockwise, closed — the default ring convention).
    let parcel = Parcel {
        outer: Boundary {
            points: vec![
                P::new(0.0, 0.0),
                P::new(0.0, 2.0),
                P::new(2.0, 2.0),
                P::new(2.0, 0.0),
                P::new(0.0, 0.0),
            ],
        },
        holes: vec![],
    };

    // Validate the user-owned type directly.
    println!("parcel valid: {:?}", is_valid_polygon(&parcel));

    // Buffer it outward by 1.0 with round joins — directly on the
    // user-owned type.
    let mut grown = buffer_convex_polygon(
        &parcel,
        1.0,
        JoinStrategy::Round {
            points_per_circle: 360,
        },
    );

    // Normalise ring orientation, then validate the result.
    correct(&mut grown);
    println!("buffered valid: {:?}", is_valid_polygon(&grown));
    println!("area {:.3} -> {:.3}", area(&parcel), area(&grown));
}
