//! Default polyhedral-surface model backed by a polygon vector.
//!
//! Mirrors `boost::geometry::model::polyhedral_surface<Polygon>` from
//! `geometries/polyhedral_surface.hpp:37-85`.

use alloc::vec::Vec;

use geometry_tag::PolyhedralSurfaceTag;
use geometry_trait::{Geometry, Polygon, PolyhedralSurface as PolyhedralSurfaceTrait};

/// A contiguous collection of polygon faces in three-dimensional space.
///
/// Mirrors `model::polyhedral_surface<Polygon>` from
/// `geometries/polyhedral_surface.hpp:48-83`. As in Boost, construction does
/// not verify that faces are 3D or share boundary segments; those are validity
/// concerns rather than storage invariants.
#[derive(Debug, Clone, PartialEq)]
#[repr(transparent)]
pub struct PolyhedralSurface<Pg: Polygon>(pub Vec<Pg>);

impl<Pg: Polygon> PolyhedralSurface<Pg> {
    /// Construct an empty polyhedral surface.
    ///
    /// Mirrors the default constructor at
    /// `geometries/polyhedral_surface.hpp:74-76`.
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self(Vec::new())
    }

    /// Wrap an existing ordered face vector.
    ///
    /// Mirrors the initializer-list constructor at
    /// `geometries/polyhedral_surface.hpp:79-81`.
    #[inline]
    #[must_use]
    pub const fn from_vec(faces: Vec<Pg>) -> Self {
        Self(faces)
    }

    /// Append one polygon face.
    #[inline]
    pub fn push(&mut self, face: Pg) {
        self.0.push(face);
    }
}

impl<Pg: Polygon> Default for PolyhedralSurface<Pg> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<Pg: Polygon> Geometry for PolyhedralSurface<Pg> {
    type Kind = PolyhedralSurfaceTag;
    type Point = Pg::Point;
}

impl<Pg: Polygon> PolyhedralSurfaceTrait for PolyhedralSurface<Pg> {
    type Face = Pg;

    #[inline]
    fn faces(&self) -> impl ExactSizeIterator<Item = &Self::Face> {
        self.0.iter()
    }
}
