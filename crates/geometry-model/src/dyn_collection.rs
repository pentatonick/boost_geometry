//! [`DynGeometryCollection`] — a heterogeneous OGC `GeometryCollection`.
//!
//! Mirrors Boost's `geometry_collection<Geometry>` adapter pattern
//! (`boost/geometry/geometries/geometry_collection.hpp`) over a
//! `boost::variant`-typed element. The Rust port encodes the same
//! relationship via the runtime-tagged
//! [`crate::DynGeometry`] enum from KC2.T1.
//!
//! A `DynGeometryCollection<f64, Cartesian>` satisfies the
//! [`GeometryCollection`] trait — the *same* trait v1's homogeneous
//! `Vec<Polygon<P>>` already satisfies — and can flow through every
//! algorithm written against `G: GeometryCollection`.
//!
//! ## Why a newtype, not a bare `Vec`
//!
//! The orphan rule forbids `impl GeometryCollection for
//! Vec<DynGeometry<…>>`: both the trait (from `geometry-trait`) and
//! `Vec` (from `alloc`) are foreign, and `Vec` is not a `#[fundamental]`
//! type, so a local type parameter inside it is not enough. Wrapping
//! the vector in this crate-local newtype restores coherence — the
//! self type's head (`DynGeometryCollection`) is local.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::CoordinateSystem;
use geometry_tag::GeometryCollectionTag;
use geometry_trait::{Geometry, GeometryCollection};

use crate::DynGeometry;

/// A homogeneous-`(Scalar, Cs)`, heterogeneous-kind collection of
/// [`DynGeometry`] values — the v1.x OGC `GeometryCollection`.
///
/// # Example
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{DynGeometry, DynGeometryCollection, Point2D};
/// use geometry_trait::GeometryCollection as _;
///
/// let gc = DynGeometryCollection::<f64, Cartesian>(vec![
///     DynGeometry::Point(Point2D::new(0.0, 0.0)),
/// ]);
/// assert_eq!(gc.items().count(), 1);
/// ```
#[derive(Debug, Clone, PartialEq, Default)]
pub struct DynGeometryCollection<S: CoordinateScalar, Cs: CoordinateSystem>(
    pub Vec<DynGeometry<S, Cs>>,
);

impl<S, Cs> Geometry for DynGeometryCollection<S, Cs>
where
    S: CoordinateScalar,
    Cs: CoordinateSystem,
{
    type Kind = GeometryCollectionTag;
    // The `Point` projection on a heterogeneous collection is the
    // "representative point" of its inner element. Every variant
    // carries a `Point<S, 2, Cs>`-based geometry, so the natural
    // choice is `model::Point<S, 2, Cs>` itself.
    type Point = crate::Point<S, 2, Cs>;
}

impl<S, Cs> GeometryCollection for DynGeometryCollection<S, Cs>
where
    S: CoordinateScalar,
    Cs: CoordinateSystem,
{
    type Item = DynGeometry<S, Cs>;

    fn items(&self) -> impl ExactSizeIterator<Item = &Self::Item> {
        self.0.iter()
    }
}

#[cfg(test)]
mod tests {
    //! Construct a heterogeneous collection, iterate it through the
    //! [`GeometryCollection`] interface, and confirm the `.kind()`
    //! sequence matches construction order.

    use super::*;
    use crate::{DynKind, Point2D, linestring, polygon};
    use alloc::vec;
    use geometry_cs::Cartesian;

    type S = f64;
    type Pt = Point2D<S, Cartesian>;

    #[test]
    fn dyn_collection_iterates_in_order() {
        let xs = DynGeometryCollection::<S, Cartesian>(vec![
            DynGeometry::Point(Pt::new(0.0, 0.0)),
            DynGeometry::LineString(linestring![(0.0, 0.0), (1.0, 1.0)]),
            DynGeometry::Polygon(polygon![[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]]),
        ]);

        let kinds: Vec<DynKind> = xs.items().map(DynGeometry::kind).collect();
        assert_eq!(
            kinds,
            vec![DynKind::Point, DynKind::LineString, DynKind::Polygon]
        );
    }

    #[test]
    fn empty_collection_is_a_geometry_collection() {
        let xs = DynGeometryCollection::<S, Cartesian>(vec![]);
        assert_eq!(xs.items().count(), 0);
    }

    /// Compile-time proof that the newtype satisfies the trait.
    fn accepts_collection<C: GeometryCollection>() {}
    #[test]
    fn dyn_collection_is_a_geometry_collection() {
        accepts_collection::<DynGeometryCollection<S, Cartesian>>();
    }
}
