//! A runtime-tagged variant geometry — one variant per OGC kind.
//!
//! Mirrors `boost::geometry::dynamic_geometry_tag` and the
//! variant adapters in
//! `boost/geometry/geometries/adapted/boost_variant.hpp` (and the
//! `boost_variant2` / `std_variant` siblings). Where the C++ side
//! uses `boost::variant<...>` and a `dynamic_geometry_tag` to dispatch,
//! the Rust port uses a plain enum and lets `match` do the dispatch —
//! one arm per OGC kind, every arm parameterised by the same
//! `(Scalar, Cs)` so the static algorithms inside each arm can still
//! be monomorphised.
//!
//! Phase positioning:
//! * KC2.T1 (this file) — the enum + `kind()` accessor.
//! * KC2.T2 — `length_dyn`, `area_dyn`, … in `geometry-algorithm`.
//! * KC2.T3 — `impl GeometryCollection for Vec<DynGeometry<...>>` so
//!   a heterogeneous vector is itself a collection.
//! * KC4   — `length`/`area` of "wrong" kinds returns 0 — depends on
//!   the variant existing so the dispatch arm has somewhere to land.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::{Cartesian, CoordinateSystem};
use geometry_tag::DynamicGeometryTag;
use geometry_trait::Geometry;

use crate::{Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon};

/// A geometry whose OGC kind is decided at runtime.
///
/// Mirrors Boost's `boost::geometry::dynamic_geometry_tag` family
/// (`core/tags.hpp:124-125`,
/// `geometries/adapted/boost_variant.hpp`). One variant per OGC kind;
/// every variant carries the matching `model::*` struct parameterised
/// by the supplied `Scalar` and `Cs`.
///
/// The parameterisation deliberately fixes a single
/// `(Scalar, Cs)` across all variants — heterogeneous *scalar* or
/// *coordinate-system* mixing inside one collection is out of scope
/// for v1 (it would require runtime CS conversion, which is a
/// `phase_07` projections concern).
///
/// # Layout
///
/// `GeometryCollection` wraps a `Vec<DynGeometry<Scalar, Cs>>` —
/// nested collections are allowed. The recursive variant is required
/// because OGC `GeometryCollection` itself is heterogeneous.
///
/// # Example
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{DynGeometry, DynKind, Point2D};
///
/// let p = DynGeometry::<f64, Cartesian>::Point(Point2D::new(1.0, 2.0));
/// assert_eq!(p.kind(), DynKind::Point);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(bound(
        serialize = "Scalar: serde::Serialize",
        deserialize = "Scalar: serde::Deserialize<'de>"
    ))
)]
pub enum DynGeometry<Scalar: CoordinateScalar, Cs: CoordinateSystem = Cartesian> {
    /// OGC `Point`. The dimension is fixed at 2D; 3D points go through
    /// `PolyhedralSurface` (or wait for the v1.1 `DynPoint3` variant —
    /// not in this task).
    Point(Point<Scalar, 2, Cs>),

    /// OGC `LineString`. Wraps [`Linestring<Point<Scalar, 2, Cs>>`].
    LineString(Linestring<Point<Scalar, 2, Cs>>),

    /// OGC `Polygon`. The two const-generic bools default to Boost's
    /// `(ClockWise = true, Closed = true)`.
    Polygon(Polygon<Point<Scalar, 2, Cs>>),

    /// OGC `MultiPoint`.
    MultiPoint(MultiPoint<Point<Scalar, 2, Cs>>),

    /// OGC `MultiLineString`.
    MultiLineString(MultiLinestring<Linestring<Point<Scalar, 2, Cs>>>),

    /// OGC `MultiPolygon`.
    MultiPolygon(MultiPolygon<Polygon<Point<Scalar, 2, Cs>>>),

    /// OGC `GeometryCollection` — a `Vec` of dyn-geometries. Nested
    /// collections are permitted.
    GeometryCollection(Vec<DynGeometry<Scalar, Cs>>),
    // OGC `PolyhedralSurface`. Reuses `crate::PolyhedralSurface` once a
    // concrete model type exists (it is currently trait-only per v1
    // T14); until then there is no variant to land here.
    // PolyhedralSurface(PolyhedralSurface<...>)
}

/// Discriminant of [`DynGeometry`]. Stable across releases.
///
/// Mirrors the result of the `geometry::detail::single_tag_of` family
/// plus Boost's `dynamic_geometry_concept`'s tag inspection
/// (`geometries/concepts/dynamic_geometry_concept.hpp`).
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DynKind {
    Point,
    LineString,
    Polygon,
    MultiPoint,
    MultiLineString,
    MultiPolygon,
    GeometryCollection,
    /// Reserved for the `PolyhedralSurface` variant once the
    /// matching concrete model type lands. v1's `PolyhedralSurface`
    /// is trait-only.
    PolyhedralSurface,
}

impl<Scalar: CoordinateScalar, Cs: CoordinateSystem> Geometry for DynGeometry<Scalar, Cs> {
    type Kind = DynamicGeometryTag;
    // A runtime-tagged geometry has no single static point type; the
    // representative choice is the 2D point every variant is built on.
    // Mirrors Boost treating a `dynamic_geometry` through its
    // `point_type` of the underlying variant elements.
    type Point = Point<Scalar, 2, Cs>;
}

impl<Scalar: CoordinateScalar, Cs: CoordinateSystem> DynGeometry<Scalar, Cs> {
    /// Discriminant of this value.
    ///
    /// Mirrors `tag<G>::type` on `dynamic_geometry_tag` Boost types
    /// (`core/tag.hpp` + the variant adapters). Pure run-time
    /// inspection — algorithms should not branch on `kind()` directly;
    /// the `_dyn` algorithm wrappers in KC2.T2 match each arm and
    /// forward to the static algorithm, keeping dispatch monomorphic
    /// per arm.
    #[must_use]
    pub fn kind(&self) -> DynKind {
        match self {
            DynGeometry::Point(_) => DynKind::Point,
            DynGeometry::LineString(_) => DynKind::LineString,
            DynGeometry::Polygon(_) => DynKind::Polygon,
            DynGeometry::MultiPoint(_) => DynKind::MultiPoint,
            DynGeometry::MultiLineString(_) => DynKind::MultiLineString,
            DynGeometry::MultiPolygon(_) => DynKind::MultiPolygon,
            DynGeometry::GeometryCollection(_) => DynKind::GeometryCollection,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Round-trip each variant through `kind()`. Mirrors the per-tag
    //! check pattern in `boost/geometry/test/core/tag.cpp`, but on the
    //! runtime discriminant instead of the compile-time tag type.

    use super::*;
    use crate::{Point2D, linestring, polygon};
    use alloc::vec;
    use geometry_cs::Cartesian;

    type S = f64;
    type Pt = Point2D<S, Cartesian>;

    #[test]
    fn kind_round_trip_every_variant() {
        let cases: Vec<(DynGeometry<S, Cartesian>, DynKind)> = vec![
            (DynGeometry::Point(Pt::new(1.0, 2.0)), DynKind::Point),
            (
                DynGeometry::LineString(linestring![(0.0, 0.0), (1.0, 1.0)]),
                DynKind::LineString,
            ),
            (
                DynGeometry::Polygon(polygon![[
                    (0.0, 0.0),
                    (1.0, 0.0),
                    (1.0, 1.0),
                    (0.0, 1.0),
                    (0.0, 0.0)
                ]]),
                DynKind::Polygon,
            ),
            (
                DynGeometry::MultiPoint(crate::MultiPoint(vec![Pt::new(0.0, 0.0)])),
                DynKind::MultiPoint,
            ),
            (
                DynGeometry::GeometryCollection(vec![]),
                DynKind::GeometryCollection,
            ),
        ];
        for (g, expected) in cases {
            assert_eq!(g.kind(), expected);
        }
    }

    #[test]
    fn geometry_collection_can_nest() {
        let nested = DynGeometry::<S, Cartesian>::GeometryCollection(vec![
            DynGeometry::Point(Pt::new(1.0, 2.0)),
            DynGeometry::GeometryCollection(vec![DynGeometry::Point(Pt::new(3.0, 4.0))]),
        ]);
        assert_eq!(nested.kind(), DynKind::GeometryCollection);
        if let DynGeometry::GeometryCollection(items) = nested {
            assert_eq!(items.len(), 2);
            assert_eq!(items[0].kind(), DynKind::Point);
            assert_eq!(items[1].kind(), DynKind::GeometryCollection);
        }
    }
}
