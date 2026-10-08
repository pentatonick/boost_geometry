//! `area_dyn` — runtime-dispatched area for [`DynGeometry`].
//!
//! Same shape as `length_dyn`. Per Boost (`algorithms/area.hpp`) the
//! area of a non-areal *leaf* kind (point, linestring, multi-point,
//! multi-linestring, …) is `0`, so every leaf arm has a value and the
//! wrapper returns a plain scalar, never an error (KC4.T1). A
//! `GeometryCollection` is **not** a leaf: Boost's
//! `area<geometry_collection_tag>` (`area.hpp:273-285`) recursively sums
//! the area of every member, so this wrapper does too.

use alloc::collections::VecDeque;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{DynGeometry, Point, Polygon};
use geometry_strategy::{AreaStrategy, DefaultArea, DefaultAreaStrategy, ShoelaceMultiPolygonArea};
use geometry_tag::SameAs;

use crate::area::{area, multi_polygon_area};

/// Runtime-dispatched signed area.
///
/// Returns the signed area for areal kinds (`Polygon`, `MultiPolygon`),
/// the recursive sum of member areas for a `GeometryCollection`, and `0`
/// for every non-areal leaf kind — matching Boost's contract
/// (`area.hpp`, incl. the `geometry_collection_tag` specialisation at
/// `:273-285`).
#[must_use]
pub fn area_dyn<S, Cs>(g: &DynGeometry<S, Cs>) -> S::Measure
where
    S: CoordinateScalar,
    Cs: CoordinateSystem,
    Cs::Family: SameAs<CartesianFamily> + DefaultArea<Cs::Family>,
    DefaultAreaStrategy<Polygon<Point<S, 2, Cs>>>:
        AreaStrategy<Polygon<Point<S, 2, Cs>>, Out = S::Measure> + Default,
    ShoelaceMultiPolygonArea:
        AreaStrategy<geometry_model::MultiPolygon<Polygon<Point<S, 2, Cs>>>, Out = S::Measure>,
{
    use DynGeometry::{
        GeometryCollection, LineString, MultiLineString, MultiPoint, MultiPolygon,
        Point as PointArm, Polygon as PolygonArm,
    };
    let leaf = |node: &DynGeometry<S, Cs>| match node {
        PolygonArm(pg) => area(pg),
        MultiPolygon(mpg) => multi_polygon_area(mpg),
        // Non-areal leaf kinds have no area — Boost returns 0.
        PointArm(_)
        | LineString(_)
        | MultiPoint(_)
        | MultiLineString(_)
        | GeometryCollection(_) => <S::Measure as CoordinateScalar>::ZERO,
    };
    let GeometryCollection(members) = g else {
        return leaf(g);
    };
    // A collection's area is the sum of its members' areas, added in the
    // order of Boost's `visit_breadth_first` (`area.hpp:273-285`,
    // `algorithms/detail/visit.hpp:193-241`): a collection's own members
    // left to right, each nested collection queued and walked after them.
    // A floating-point sum depends on that order. The queue, not
    // recursion, also keeps an adversarially deep `GeometryCollection`
    // chain off the native stack (an uncatchable process abort).
    let mut total = <S::Measure as CoordinateScalar>::ZERO;
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
