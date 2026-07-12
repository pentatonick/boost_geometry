//! Geometry concepts as Rust traits. One trait per `doc/concept/*.qbk`.
//!
//! Mirrors `boost/geometry/core/{access,tag,coordinate_type,
//! coordinate_dimension,coordinate_system}.hpp` and the
//! `boost/geometry/geometries/concepts/*_concept.hpp` headers.
//!
//! v1 surface — only the [`Geometry`] super-trait and the [`Point`]
//! concept land here. Subsequent tasks add `Box`, `Segment`,
//! `Linestring`, `Ring`, `Polygon`, and the multi / collection
//! concepts as siblings of [`Point`].

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

mod boxg;
mod check;
mod closure;
mod collection;
mod geometry;
mod indexed_access;
mod linestring;
mod multi;
mod point;
mod point_order;
mod polygon;
mod polyhedral;
mod ring;
mod segment;

pub use boxg::{Box, box_max, box_min};
pub use check::{
    check_box, check_geometry_collection, check_indexed_access, check_linestring,
    check_multi_linestring, check_multi_point, check_multi_polygon, check_point, check_polygon,
    check_polyhedral_surface, check_ring, check_segment,
};
pub use closure::Closure;
pub use collection::GeometryCollection;
pub use geometry::Geometry;
pub use indexed_access::{IndexedAccess, corner};
pub use linestring::Linestring;
pub use multi::{MultiLinestring, MultiPoint, MultiPolygon};
pub use point::{Point, PointMut, fold_dims};
pub use point_order::PointOrder;
pub use polygon::Polygon;
pub use polyhedral::PolyhedralSurface;
pub use ring::Ring;
pub use segment::{Segment, segment_end, segment_start};

// Re-export tags and tag-hierarchy markers so a downstream `impl Point`
// (or any other concept impl) has a single import path.
pub use geometry_tag::{
    Areal, BoxTag, GeometryCollectionTag, Linear, LinestringTag, Multi, MultiLinestringTag,
    MultiPointTag, MultiPolygonTag, PointTag, Pointlike, PolygonTag, Polygonal,
    PolyhedralSurfaceTag, Polylinear, RingTag, SameAs, SegmentTag, Single, Volumetric,
};
