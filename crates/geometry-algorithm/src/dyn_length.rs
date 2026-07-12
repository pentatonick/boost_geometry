//! `length_dyn` — runtime-dispatched length for [`DynGeometry`].
//!
//! Mirrors `boost::geometry::length(g)` reached through the variant
//! adapter. Per Boost's convention (`algorithms/length.hpp:75-80`) the
//! length of a non-linear *leaf* kind (point, polygon, multi-point, …)
//! is `0` — so every leaf arm has a value and the wrapper returns a
//! plain scalar, never an error (KC4.T1). A `GeometryCollection` is
//! **not** a leaf: Boost's `length<geometry_collection_tag>`
//! (`length.hpp:251-265`) recursively sums the length of every member,
//! so this wrapper does too.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{DynGeometry, Linestring, Point};
use geometry_strategy::{CartesianLength, DefaultLength, DefaultLengthStrategy, LengthStrategy};
use geometry_tag::SameAs;

use crate::length::length;

/// Runtime-dispatched length.
///
/// Returns the polyline length for linear kinds (`LineString`,
/// `MultiLineString`), the recursive sum of member lengths for a
/// `GeometryCollection`, and `0` for every non-linear leaf kind —
/// matching Boost's contract (`length.hpp:75-80`, plus the
/// `geometry_collection_tag` specialisation at `:251-265`).
#[must_use]
pub fn length_dyn<S, Cs>(g: &DynGeometry<S, Cs>) -> S
where
    S: CoordinateScalar,
    Cs: CoordinateSystem,
    Cs::Family: SameAs<CartesianFamily> + DefaultLength<Cs::Family>,
    CartesianLength: LengthStrategy<Linestring<Point<S, 2, Cs>>, Out = S>,
    DefaultLengthStrategy<Linestring<Point<S, 2, Cs>>>:
        LengthStrategy<Linestring<Point<S, 2, Cs>>, Out = S> + Default,
{
    use DynGeometry::{
        GeometryCollection, LineString, MultiLineString, MultiPoint, MultiPolygon,
        Point as PointArm, Polygon,
    };
    // A collection's length is the sum of its members' lengths
    // (`length.hpp:251-265` recurses via `visit_breadth_first`). The walk
    // is an explicit work-list rather than recursion so an adversarially
    // deep `GeometryCollection` chain cannot overflow the native stack (an
    // uncatchable process abort).
    let mut total = S::ZERO;
    let mut stack = alloc::vec![g];
    while let Some(node) = stack.pop() {
        match node {
            LineString(ls) => total = total + length(ls),
            MultiLineString(ml) => {
                total = ml.0.iter().fold(total, |acc, ls| acc + length(ls));
            }
            GeometryCollection(items) => stack.extend(items.iter()),
            // Non-linear leaf kinds have no length — Boost returns 0.
            PointArm(_) | Polygon(_) | MultiPoint(_) | MultiPolygon(_) => {}
        }
    }
    total
}
