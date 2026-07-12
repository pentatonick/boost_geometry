//! `distance_dyn` — runtime-dispatched distance between two
//! [`DynGeometry`] inputs.
//!
//! Mirrors the variant-aware overload of
//! `boost::geometry::distance(g1, g2)` reached when at least one
//! argument has tag `dynamic_geometry_tag`
//! (`algorithms/distance.hpp` + `geometries/adapted/boost_variant.hpp`).
//! On the Rust side we make the dispatch explicit:
//!
//! 1. `match (a, b)` on the kind pairs.
//! 2. Each arm calls the static distance kernels from `crate::distance`
//!    on the unwrapped concrete types — dispatch inside the arm is
//!    monomorphic and produces the same code as a direct call would.
//! 3. Pairs with no static impl (e.g. `Polygon × Polygon` in v1) return
//!    `Err(DynKindMismatch)` rather than panicking.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{DynGeometry, DynKind, Linestring, Point, Segment};
use geometry_strategy::{
    DefaultDistance, DefaultDistanceStrategy, DistanceStrategy, PointToSegment, Pythagoras,
};
use geometry_tag::SameAs;
use geometry_trait::Linestring as LinestringTrait;

use crate::distance::{distance, distance_with};
use crate::dyn_error::DynKindMismatch;

/// The supported `(left, right)` kind combinations for `distance_dyn`.
/// Mirrors the per-tag-pair static specialisations in
/// `algorithms/distance.hpp` + the leaf strategies under
/// `strategies/cartesian/distance_*.hpp`.
const SUPPORTED: &[&[DynKind]] = &[
    &[DynKind::Point, DynKind::Point],
    &[DynKind::Point, DynKind::LineString], // via PointToSegment per segment
    &[DynKind::LineString, DynKind::Point], // reversed
];

/// Runtime-dispatched distance.
///
/// Returns `Ok(d)` for any pair v1's static `distance` supports
/// (point-point, and point-linestring via `PointToSegment`).
///
/// The Cartesian family bound reflects that `PointToSegment` (T24) is
/// Cartesian-only in v1; point-point works for any family, but the
/// shared bound keeps the wrapper's `where` clause a single line.
///
/// # Errors
///
/// Returns `Err(DynKindMismatch)` for any kind pair with no static
/// distance impl (e.g. polygon-polygon).
#[allow(
    clippy::match_same_arms,
    reason = "The two point↔linestring arms differ by argument order; keeping them separate documents both directions."
)]
pub fn distance_dyn<S, Cs>(
    a: &DynGeometry<S, Cs>,
    b: &DynGeometry<S, Cs>,
) -> Result<S, DynKindMismatch>
where
    S: CoordinateScalar,
    Cs: CoordinateSystem + Copy,
    Cs::Family: SameAs<CartesianFamily> + DefaultDistance<Cs::Family>,
    DefaultDistanceStrategy<Point<S, 2, Cs>, Point<S, 2, Cs>>:
        DistanceStrategy<Point<S, 2, Cs>, Point<S, 2, Cs>, Out = S> + Default,
    PointToSegment<Pythagoras>:
        DistanceStrategy<Point<S, 2, Cs>, Segment<Point<S, 2, Cs>>, Out = S>,
{
    use DynGeometry::{LineString, Point as PointArm};
    match (a, b) {
        (PointArm(p), PointArm(q)) => Ok(distance(p, q)),
        (PointArm(p), LineString(ls)) => Ok(point_to_linestring(p, ls)),
        (LineString(ls), PointArm(p)) => Ok(point_to_linestring(p, ls)),
        _ => Err(DynKindMismatch {
            got: alloc::vec![a.kind(), b.kind()],
            expected: SUPPORTED,
        }),
    }
}

/// Minimum distance from a point to any segment of a linestring.
///
/// Mirrors `algorithms/detail/distance/point_to_geometry.hpp`: the
/// point-to-linestring distance is the minimum point-to-segment
/// distance over the linestring's segments.
fn point_to_linestring<S, Cs>(p: &Point<S, 2, Cs>, ls: &Linestring<Point<S, 2, Cs>>) -> S
where
    S: CoordinateScalar,
    Cs: CoordinateSystem + Copy,
    Cs::Family: SameAs<CartesianFamily>,
    PointToSegment<Pythagoras>:
        DistanceStrategy<Point<S, 2, Cs>, Segment<Point<S, 2, Cs>>, Out = S>,
{
    let strategy = PointToSegment::<Pythagoras>::default();
    let mut best: Option<S> = None;
    let mut prev: Option<Point<S, 2, Cs>> = None;
    for pt in ls.points() {
        if let Some(start) = prev {
            let seg = Segment::new(start, *pt);
            let d = distance_with(p, &seg, strategy);
            best = Some(match best {
                Some(m) if m <= d => m,
                _ => d,
            });
        }
        prev = Some(*pt);
    }
    // Degenerate: fewer than two points. A single vertex is handled as
    // a zero-length segment (its point-to-segment distance is the
    // point-to-vertex distance); an empty linestring has distance 0.
    best.unwrap_or_else(|| match ls.points().next() {
        Some(v) => distance_with(p, &Segment::new(*v, *v), strategy),
        None => S::ZERO,
    })
}
