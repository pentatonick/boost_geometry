// Mirrors `boost/geometry/test/concepts/point_with_incorrect_dimension.cpp`
// in spirit. A `Point2D` has its dimension fixed at the type level, so
// only the two-argument `new` constructor is in scope. Passing three
// arguments must therefore be rejected by the type checker rather than
// silently truncated.

use geometry_cs::Cartesian;
use geometry_model::Point2D;

fn main() {
    let _ = Point2D::<f64, Cartesian>::new(1.0, 2.0, 3.0);
}
