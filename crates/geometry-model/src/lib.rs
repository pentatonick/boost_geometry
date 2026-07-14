//! Default concrete geometry types.
//!
//! Each type here implements the matching trait from `geometry-trait`.
//! Mirrors `boost/geometry/geometries/*.hpp` — the Boost.Geometry
//! "model" namespace of stock geometries (`model::point`,
//! `model::segment`, `model::box`, …) that the library ships for
//! callers who do not want to adapt their own types.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod boxg;
mod dyn_collection;
mod dyn_geometry;
mod geometry_rebind;
mod infinite_line;
mod linestring;
mod macros;
mod multi;
mod point;
mod pointing_segment;
mod polygon;
mod polyhedral_surface;
mod ring;
mod segment;

pub use boxg::Box;
pub use dyn_collection::DynGeometryCollection;
pub use dyn_geometry::{DynGeometry, DynKind};
pub use geometry_rebind::{RebindGeometry, Rebound};
pub use infinite_line::InfiniteLine;
pub use linestring::Linestring;
pub use multi::{MultiLinestring, MultiPoint, MultiPolygon};
pub use point::{Point, Point2D, Point3D};
pub use pointing_segment::PointingSegment;
pub use polygon::Polygon;
pub use polyhedral_surface::PolyhedralSurface;
pub use ring::Ring;
pub use segment::Segment;
