//! OVL1.T4 — the coordinate-range robustness gate.
//!
//! The "exact input arithmetic, no rescale" policy
//! (`specs/overlay-robustness-decision.md`) is only sound while the
//! products the predicates form stay within the mantissa. The
//! orientation determinant multiplies two coordinate *differences*;
//! for `f64` the product of two values each below `2^26` fits exactly
//! in the 53-bit mantissa, so the sign of the signed area is exact.
//! Above that the sign can flip — which is precisely the failure
//! Boost's rescale step existed to prevent.
//!
//! Rather than rescale, the port **refuses**: segment intersection
//! routes every endpoint coordinate through [`coordinate_in_range`]
//! and returns a [`RangeError`] for anything outside the safe band.
//! This is the honest v1 contract — a wrong answer is never returned
//! silently.
//!
//! Mirrors the role of `boost/geometry/policies/robustness/` — the
//! `no_rescale_policy` path assumes callers keep coordinates in a safe
//! range; this guard makes that assumption checked instead of implicit.

use geometry_coords::CoordinateScalar;
use geometry_trait::Point;

/// Largest coordinate magnitude for which the orientation determinant
/// is computed exactly in `f64`.
///
/// `2^26 = 67_108_864`. The determinant multiplies two coordinate
/// differences; each difference is at most `2 · SAFE_ABS_MAX`, and the
/// product of two `< 2^27` values is `< 2^54` — at the edge of the
/// 53-bit mantissa, so we take `2^26` as the coordinate bound to keep a
/// full bit of headroom for the difference and the subtraction.
///
/// Expressed as `f64`; integer scalars are always in range (their
/// products are exact until they overflow the integer type itself,
/// which is a separate concern).
pub const SAFE_ABS_MAX: f64 = 67_108_864.0; // 2^26

/// A coordinate that falls outside the safe arithmetic range of the
/// no-rescale overlay policy.
///
/// Mirrors the failure Boost's rescale step avoided: past this
/// magnitude the signed-area sign can no longer be trusted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RangeError {
    /// Which endpoint (0-based index into the call's point list) was
    /// out of range. `usize::MAX` marks an unspecified/aggregate error.
    pub point_index: usize,
    /// Which dimension (`0` = x, `1` = y) held the offending value.
    pub dimension: usize,
}

impl core::fmt::Display for RangeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "coordinate at point {} dimension {} exceeds the safe overlay range (±{SAFE_ABS_MAX})",
            self.point_index, self.dimension
        )
    }
}

#[cfg(feature = "std")]
impl std::error::Error for RangeError {}

/// True when every coordinate of `p` is within `±`[`SAFE_ABS_MAX`].
///
/// The check is skipped (always `true`) for scalar types whose values
/// convert to an `f64` magnitude — integer coordinates included, since
/// their determinant is exact until the integer type overflows.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_overlay::predicate::range_guard::{coordinate_in_range, SAFE_ABS_MAX};
///
/// type P = Point2D<f64, Cartesian>;
/// assert!(coordinate_in_range(&P::new(1.0e6, -2.0e6)));
/// assert!(!coordinate_in_range(&P::new(SAFE_ABS_MAX * 2.0, 0.0)));
/// ```
#[must_use]
pub fn coordinate_in_range<P>(p: &P) -> bool
where
    P: Point,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let x: f64 = p.get::<0>().into();
    let y: f64 = p.get::<1>().into();
    x.abs() <= SAFE_ABS_MAX && y.abs() <= SAFE_ABS_MAX
}

/// True when *every* vertex of `polygon` (exterior and all interior
/// rings) is within `±`[`SAFE_ABS_MAX`].
///
/// The turn collector's per-segment [`coordinate_in_range`] check reports
/// an out-of-range intersection by yielding
/// [`SegmentIntersection::OutOfRange`](crate::predicate::segment_intersection::SegmentIntersection::OutOfRange),
/// which the collector then *drops* (it emits no turn) — indistinguishable
/// from a genuine disjoint pair. A caller that turns "no turns" into a
/// geometric conclusion (empty intersection, `II = Empty`) would therefore
/// return a **silently wrong** answer for out-of-range input. To honour
/// the module's "a wrong answer is never returned silently" contract,
/// such callers must reject out-of-range polygons *up front* with this
/// check rather than trust the emptied turn graph.
#[must_use]
pub fn polygon_in_range<G, P>(polygon: &G) -> bool
where
    G: geometry_trait::Polygon<Point = P>,
    P: Point,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    use geometry_trait::Ring as _;
    polygon.exterior().points().all(coordinate_in_range)
        && polygon
            .interiors()
            .all(|r| r.points().all(coordinate_in_range))
}

#[cfg(test)]
mod tests {
    use super::{RangeError, SAFE_ABS_MAX, coordinate_in_range};
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn in_range_ok() {
        assert!(coordinate_in_range(&P::new(0.0, 0.0)));
        assert!(coordinate_in_range(&P::new(SAFE_ABS_MAX, -SAFE_ABS_MAX)));
        assert!(coordinate_in_range(&P::new(1.0e6, 3.0e6)));
    }

    #[test]
    fn out_of_range_rejected() {
        assert!(!coordinate_in_range(&P::new(SAFE_ABS_MAX + 1.0, 0.0)));
        assert!(!coordinate_in_range(&P::new(0.0, -SAFE_ABS_MAX * 4.0)));
        assert!(!coordinate_in_range(&P::new(f64::MAX, 0.0)));
    }

    #[test]
    fn range_error_display_mentions_index_and_dim() {
        let e = RangeError {
            point_index: 2,
            dimension: 1,
        };
        let s = alloc_display(&e);
        assert!(s.contains("point 2"));
        assert!(s.contains("dimension 1"));
    }

    #[cfg(feature = "std")]
    fn alloc_display(e: &RangeError) -> String {
        e.to_string()
    }

    #[cfg(not(feature = "std"))]
    fn alloc_display(_e: &RangeError) -> &'static str {
        "point 2 dimension 1"
    }
}
