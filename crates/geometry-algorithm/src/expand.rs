//! Expand a box to include another geometry.
//!
//! Mirrors `boost::geometry::expand` from
//! `boost/geometry/algorithms/expand.hpp` and the component-wise Cartesian
//! implementation in `algorithms/detail/expand/{point,box}.hpp`.

use geometry_model::Box as ModelBox;
use geometry_strategy::{EnvelopeStrategy, EnvelopeStrategyForKind};
use geometry_trait::{Box as BoxTrait, Geometry, IndexedAccess, PointMut, corner, fold_dims};

/// Expand `bounds` so it contains `geometry`'s envelope.
///
/// Mirrors `boost::geometry::expand(box, geometry)` from
/// `algorithms/detail/expand/interface.hpp:115-132`. The geometry kind is
/// resolved through the same public envelope strategy used by
/// [`crate::envelope()`], then each minimum/maximum coordinate widens the
/// caller-supplied box in place.
///
/// The current envelope strategy family is Cartesian; spherical and
/// geographic antimeridian-aware expansion will become available when those
/// envelope strategies are ported.
#[inline]
pub fn expand<B, G, P>(bounds: &mut B, geometry: &G)
where
    B: BoxTrait<Point = P>,
    G: Geometry<Point = P>,
    P: PointMut + Default,
    G::Kind: EnvelopeStrategyForKind,
    <G::Kind as EnvelopeStrategyForKind>::S: EnvelopeStrategy<G, Output = ModelBox<P>> + Default,
{
    expand_with(
        bounds,
        geometry,
        <<G::Kind as EnvelopeStrategyForKind>::S as Default>::default(),
    );
}

/// Expand `bounds` using an explicitly supplied envelope strategy.
///
/// Mirrors the strategy overload of `boost::geometry::expand` from
/// `algorithms/detail/expand/interface.hpp:135-153`. Returning an envelope
/// from the strategy keeps the mutation generic over any writable Box concept
/// implementation.
#[inline]
#[allow(
    clippy::needless_pass_by_value,
    reason = "envelope strategies are zero-sized or small Copy values, matching the crate's other _with entry points"
)]
pub fn expand_with<B, G, P, S>(bounds: &mut B, geometry: &G, strategy: S)
where
    B: BoxTrait<Point = P>,
    G: Geometry<Point = P>,
    P: PointMut + Default,
    S: EnvelopeStrategy<G, Output = ModelBox<P>>,
{
    let incoming = strategy.envelope(geometry);

    fold_dims((), incoming.min(), |(), _, dimension| match dimension {
        0 => expand_dimension::<B, P, 0>(bounds, &incoming),
        1 => expand_dimension::<B, P, 1>(bounds, &incoming),
        2 => expand_dimension::<B, P, 2>(bounds, &incoming),
        3 => expand_dimension::<B, P, 3>(bounds, &incoming),
        _ => unreachable!("fold_dims limits point dimensions to MAX_DIM"),
    });
}

#[inline]
fn expand_dimension<B, P, const D: usize>(bounds: &mut B, incoming: &ModelBox<P>)
where
    B: BoxTrait<Point = P>,
    P: PointMut,
{
    let incoming_min = incoming.get_indexed::<{ corner::MIN }, D>();
    let incoming_max = incoming.get_indexed::<{ corner::MAX }, D>();
    let current_min = bounds.get_indexed::<{ corner::MIN }, D>();
    let current_max = bounds.get_indexed::<{ corner::MAX }, D>();

    if incoming_min < current_min {
        bounds.set_indexed::<{ corner::MIN }, D>(incoming_min);
    }
    if incoming_max > current_max {
        bounds.set_indexed::<{ corner::MAX }, D>(incoming_max);
    }
}
