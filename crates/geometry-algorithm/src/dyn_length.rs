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

use alloc::collections::VecDeque;

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
pub fn length_dyn<S, Cs>(g: &DynGeometry<S, Cs>) -> S::Measure
where
    S: CoordinateScalar,
    Cs: CoordinateSystem,
    Cs::Family: SameAs<CartesianFamily> + DefaultLength<Cs::Family>,
    CartesianLength: LengthStrategy<Linestring<Point<S, 2, Cs>>, Out = S::Measure>,
    DefaultLengthStrategy<Linestring<Point<S, 2, Cs>>>:
        LengthStrategy<Linestring<Point<S, 2, Cs>>, Out = S::Measure> + Default,
{
    use DynGeometry::{
        GeometryCollection, LineString, MultiLineString, MultiPoint, MultiPolygon,
        Point as PointArm, Polygon,
    };
    let zero = <S::Measure as CoordinateScalar>::ZERO;
    let leaf = |node: &DynGeometry<S, Cs>| match node {
        LineString(ls) => length(ls),
        // `length<multi_linestring_tag>` sums its own members from zero
        // (`length.hpp:147-165`, `multi_sum.hpp:31-43`) before a
        // collection adds it.
        MultiLineString(ml) => ml.0.iter().fold(zero, |sum, ls| sum + length(ls)),
        // Non-linear leaf kinds have no length — Boost returns 0.
        PointArm(_) | Polygon(_) | MultiPoint(_) | MultiPolygon(_) | GeometryCollection(_) => zero,
    };
    let GeometryCollection(members) = g else {
        return leaf(g);
    };
    // A collection's length is the sum of its members' lengths, added in
    // the order of Boost's `visit_breadth_first` (`length.hpp:251-265`,
    // `algorithms/detail/visit.hpp:193-241`): a collection's own members
    // left to right, each nested collection queued and walked after them.
    // A floating-point sum depends on that order. The queue, not
    // recursion, also keeps an adversarially deep `GeometryCollection`
    // chain off the native stack (an uncatchable process abort).
    let mut total = zero;
    let mut queue = VecDeque::new();
    let mut members = members.iter();
    loop {
        for member in members {
            match member {
                GeometryCollection(nested) => queue.push_back(nested),
                _ => total = total + leaf(member),
            }
        }
        match queue.pop_front() {
            Some(nested) => members = nested.iter(),
            None => return total,
        }
    }
}
