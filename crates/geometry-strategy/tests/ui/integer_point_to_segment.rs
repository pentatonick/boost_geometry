// Integer coordinates — `PointToSegment` must refuse to compile with an
// integer point. The foot of the perpendicular is a point of the segment's
// own type, which cannot hold a fractional foot, while `Pythagoras`
// measures integer coordinates in `f64`.

use geometry_cs::Cartesian;
use geometry_model::{Point2D, Segment};
use geometry_strategy::distance::DistanceStrategy;
use geometry_strategy::{PointToSegment, Pythagoras};

fn main() {
    type IntegerPoint = Point2D<i32, Cartesian>;
    let segment = Segment::new(IntegerPoint::new(0, 0), IntegerPoint::new(10, 0));
    let _ = PointToSegment(Pythagoras).distance(&IntegerPoint::new(5, 7), &segment);
}
