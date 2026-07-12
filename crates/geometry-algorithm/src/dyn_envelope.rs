//! `envelope_dyn` — runtime-dispatched envelope for [`DynGeometry`].
//!
//! Mirrors `boost::geometry::envelope(g)` reached through the variant
//! adapter (`algorithms/envelope.hpp`). Returns a
//! `model::Box<Point<S, 2, Cs>>`.
//!
//! Every single/multi variant is supported (each per-kind envelope
//! strategy resolves through the tag-keyed
//! [`geometry_strategy::EnvelopeStrategyForKind`] picker). The
//! `GeometryCollection` arm returns `Err(DynKindMismatch)`: computing
//! its envelope needs a box-merge primitive v1 does not yet ship (and
//! an empty collection has no well-defined envelope). That arm lands
//! when the merge helper does.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{Box, DynGeometry, DynKind, Linestring, Point, Polygon};
use geometry_strategy::{EnvelopeStrategy, EnvelopeStrategyForKind};
use geometry_tag::{
    LinestringTag, MultiLinestringTag, MultiPointTag, MultiPolygonTag, PointTag, PolygonTag, SameAs,
};

use crate::dyn_error::DynKindMismatch;
use crate::envelope::envelope;

const SUPPORTED: &[&[DynKind]] = &[
    &[DynKind::Point],
    &[DynKind::LineString],
    &[DynKind::Polygon],
    &[DynKind::MultiPoint],
    &[DynKind::MultiLineString],
    &[DynKind::MultiPolygon],
];

/// Runtime-dispatched envelope.
///
/// Returns `Ok(Box)` for every single/multi kind.
///
/// # Errors
///
/// Returns `Err(DynKindMismatch)` for `GeometryCollection` (see the
/// module docs — it needs a box-merge primitive v1 lacks).
pub fn envelope_dyn<S, Cs>(g: &DynGeometry<S, Cs>) -> Result<Box<Point<S, 2, Cs>>, DynKindMismatch>
where
    S: CoordinateScalar,
    Cs: CoordinateSystem,
    Cs::Family: SameAs<CartesianFamily>,
    // Each arm calls the tag-dispatched `envelope` free fn; require the
    // picked per-kind strategy for each concrete arm type to produce a
    // `Box<Point<S, 2, Cs>>`.
    <PointTag as EnvelopeStrategyForKind>::S:
        EnvelopeStrategy<Point<S, 2, Cs>, Output = Box<Point<S, 2, Cs>>>,
    <LinestringTag as EnvelopeStrategyForKind>::S:
        EnvelopeStrategy<Linestring<Point<S, 2, Cs>>, Output = Box<Point<S, 2, Cs>>>,
    <PolygonTag as EnvelopeStrategyForKind>::S:
        EnvelopeStrategy<Polygon<Point<S, 2, Cs>>, Output = Box<Point<S, 2, Cs>>>,
    <MultiPointTag as EnvelopeStrategyForKind>::S: EnvelopeStrategy<geometry_model::MultiPoint<Point<S, 2, Cs>>, Output = Box<Point<S, 2, Cs>>>,
    <MultiLinestringTag as EnvelopeStrategyForKind>::S: EnvelopeStrategy<
            geometry_model::MultiLinestring<Linestring<Point<S, 2, Cs>>>,
            Output = Box<Point<S, 2, Cs>>,
        >,
    <MultiPolygonTag as EnvelopeStrategyForKind>::S: EnvelopeStrategy<
            geometry_model::MultiPolygon<Polygon<Point<S, 2, Cs>>>,
            Output = Box<Point<S, 2, Cs>>,
        >,
{
    use DynGeometry::{
        GeometryCollection, LineString, MultiLineString, MultiPoint, MultiPolygon,
        Point as PointArm, Polygon as PolygonArm,
    };
    match g {
        PointArm(p) => Ok(envelope(p)),
        LineString(ls) => Ok(envelope(ls)),
        PolygonArm(pg) => Ok(envelope(pg)),
        MultiPoint(mp) => Ok(envelope(mp)),
        MultiLineString(ml) => Ok(envelope(ml)),
        MultiPolygon(mpg) => Ok(envelope(mpg)),
        GeometryCollection(_) => Err(DynKindMismatch {
            got: alloc::vec![g.kind()],
            expected: SUPPORTED,
        }),
    }
}
