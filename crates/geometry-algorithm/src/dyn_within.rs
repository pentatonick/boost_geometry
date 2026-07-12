//! `within_dyn` — runtime-dispatched point-in-polygon for
//! [`DynGeometry`].
//!
//! Supported pair: `(Point, Polygon)`. Anything else returns
//! `Err(DynKindMismatch)`. Mirrors `algorithms/within.hpp`'s static
//! per-tag dispatch reached through the variant. `MultiPolygon` is not
//! yet a `WithinStrategy` target in v1, so `(Point, MultiPolygon)` is
//! unsupported for now.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{DynGeometry, DynKind, Point, Polygon};
use geometry_strategy::{WithinStrategy, WithinStrategyForKind};
use geometry_tag::{PolygonTag, SameAs};

use crate::dyn_error::DynKindMismatch;
use crate::within::within;

const SUPPORTED: &[&[DynKind]] = &[&[DynKind::Point, DynKind::Polygon]];

/// Runtime-dispatched point-in-polygon.
///
/// Returns `Ok(true/false)` for `(Point, Polygon)`.
///
/// # Errors
///
/// Returns `Err(DynKindMismatch)` for every other kind combination.
pub fn within_dyn<S, Cs>(
    a: &DynGeometry<S, Cs>,
    b: &DynGeometry<S, Cs>,
) -> Result<bool, DynKindMismatch>
where
    S: CoordinateScalar,
    Cs: CoordinateSystem,
    Cs::Family: SameAs<CartesianFamily>,
    <PolygonTag as WithinStrategyForKind>::S:
        WithinStrategy<Point<S, 2, Cs>, Polygon<Point<S, 2, Cs>>>,
{
    use DynGeometry::{Point as PointArm, Polygon as PolygonArm};
    match (a, b) {
        (PointArm(p), PolygonArm(pg)) => Ok(within(p, pg)),
        _ => Err(DynKindMismatch {
            got: alloc::vec![a.kind(), b.kind()],
            expected: SUPPORTED,
        }),
    }
}
