//! Public segment-intersection result with topology classification.
//!
//! The predicate kernel computes zero, one, or two intersection points. This
//! entry adds the proper-crossing distinction used by the turn classifier and
//! converts an arithmetic range refusal into the public overlay error.

use geometry_coords::CoordinateScalar;
use geometry_model::Segment;
use geometry_trait::{Point, PointMut, Segment as SegmentTrait, segment_end, segment_start};

use crate::operation::OverlayError;
use crate::predicate::{SegmentIntersection, segment_intersection};
use crate::turn::{Method, classify::refine_touch};

/// A non-empty intersection between two line segments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LineIntersection<P: Point> {
    /// The segments meet at one point.
    SinglePoint {
        /// The intersection coordinate.
        intersection: P,
        /// `true` only when the point lies in the interior of both segments.
        is_proper: bool,
    },
    /// The segments overlap along a collinear segment.
    Collinear {
        /// The shared closed segment.
        intersection: Segment<P>,
    },
}

/// Intersect two segments and report proper, touch, and collinear cases.
///
/// This is the public counterpart of the Cartesian intersection strategy in
/// `strategy/cartesian/intersection.hpp`. It preserves this port's explicit
/// range refusal instead of returning a potentially unreliable coordinate.
///
/// # Errors
///
/// Returns [`OverlayError::Unsupported`] when an endpoint lies outside the
/// exact-predicate range.
#[inline]
#[must_use = "line intersection can fail and its result should be used"]
pub fn line_intersection<S, P>(
    first: &S,
    second: &S,
) -> Result<Option<LineIntersection<P>>, OverlayError>
where
    S: SegmentTrait<Point = P>,
    P: PointMut + Default,
    P::Scalar: CoordinateScalar + Into<f64> + PartialEq,
{
    match segment_intersection(first, second) {
        SegmentIntersection::Disjoint => Ok(None),
        SegmentIntersection::OutOfRange => Err(OverlayError::Unsupported),
        SegmentIntersection::Single(intersection) => {
            let first_start = segment_start(first);
            let first_end = segment_end(first);
            let second_start = segment_start(second);
            let second_end = segment_end(second);
            let is_proper = refine_touch(
                &intersection,
                &first_start,
                &first_end,
                &second_start,
                &second_end,
            ) == Method::Crosses;
            Ok(Some(LineIntersection::SinglePoint {
                intersection,
                is_proper,
            }))
        }
        SegmentIntersection::Collinear { from, to } => Ok(Some(LineIntersection::Collinear {
            intersection: Segment::new(from, to),
        })),
    }
}

#[cfg(test)]
mod tests {
    use super::{LineIntersection, line_intersection};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Segment};

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn distinguishes_proper_crossing_from_endpoint_touch() {
        let diagonal = Segment::new(P::new(0.0, 0.0), P::new(2.0, 2.0));
        let crossing = Segment::new(P::new(0.0, 2.0), P::new(2.0, 0.0));
        let touching = Segment::new(P::new(2.0, 2.0), P::new(3.0, 2.0));

        assert_eq!(
            line_intersection(&diagonal, &crossing),
            Ok(Some(LineIntersection::SinglePoint {
                intersection: P::new(1.0, 1.0),
                is_proper: true,
            }))
        );
        assert_eq!(
            line_intersection(&diagonal, &touching),
            Ok(Some(LineIntersection::SinglePoint {
                intersection: P::new(2.0, 2.0),
                is_proper: false,
            }))
        );
    }
}
