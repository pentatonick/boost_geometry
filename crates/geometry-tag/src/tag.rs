//! The eleven empty tag types that identify a geometry kind.
//!
//! Each tag is a zero-sized struct — its sole purpose is to serve as a
//! type-level identity used by `Geometry::Kind` and by trait-bound
//! dispatch. The C++ counterparts live in `boost/geometry/core/tags.hpp`.
//!
//! # Examples
//!
//! Every tag is a zero-sized identifier:
//!
//! ```
//! use geometry_tag::{
//!     BoxTag, GeometryCollectionTag, LinestringTag, MultiLinestringTag,
//!     MultiPointTag, MultiPolygonTag, PointTag, PolygonTag,
//!     PolyhedralSurfaceTag, RingTag, SegmentTag,
//! };
//! let _ = (
//!     PointTag, SegmentTag, LinestringTag, RingTag, PolygonTag, BoxTag,
//!     MultiPointTag, MultiLinestringTag, MultiPolygonTag,
//!     GeometryCollectionTag, PolyhedralSurfaceTag,
//! );
//! ```

/// OGC Point identifying tag — `boost/geometry/core/tags.hpp:91`.
#[derive(Debug, Default, Clone, Copy)]
pub struct PointTag;

/// Convenience segment (2-points) identifying tag — `boost/geometry/core/tags.hpp:106`.
#[derive(Debug, Default, Clone, Copy)]
pub struct SegmentTag;

/// OGC Linestring identifying tag — `boost/geometry/core/tags.hpp:94`.
#[derive(Debug, Default, Clone, Copy)]
pub struct LinestringTag;

/// Convenience linear ring identifying tag — `boost/geometry/core/tags.hpp:100`.
#[derive(Debug, Default, Clone, Copy)]
pub struct RingTag;

/// OGC Polygon identifying tag — `boost/geometry/core/tags.hpp:97`.
#[derive(Debug, Default, Clone, Copy)]
pub struct PolygonTag;

/// 2D or 3D axis-aligned box (mbr / aabb) identifying tag — `boost/geometry/core/tags.hpp:103`.
#[derive(Debug, Default, Clone, Copy)]
pub struct BoxTag;

/// OGC `MultiPoint` identifying tag — `boost/geometry/core/tags.hpp:113`.
#[derive(Debug, Default, Clone, Copy)]
pub struct MultiPointTag;

/// OGC `MultiLinestring` identifying tag — `boost/geometry/core/tags.hpp:116`.
#[derive(Debug, Default, Clone, Copy)]
pub struct MultiLinestringTag;

/// OGC `MultiPolygon` identifying tag — `boost/geometry/core/tags.hpp:119`.
#[derive(Debug, Default, Clone, Copy)]
pub struct MultiPolygonTag;

/// OGC Geometry Collection identifying tag — `boost/geometry/core/tags.hpp:122`.
#[derive(Debug, Default, Clone, Copy)]
pub struct GeometryCollectionTag;

/// OGC Polyhedral surface identifying tag — `boost/geometry/core/tags.hpp:109`.
#[derive(Debug, Default, Clone, Copy)]
pub struct PolyhedralSurfaceTag;

/// Runtime-tagged (variant) geometry identifying tag —
/// `boost/geometry/core/tags.hpp:124` (`dynamic_geometry_tag`). Marks a
/// value whose OGC kind is decided at runtime, e.g. `DynGeometry`.
#[derive(Debug, Default, Clone, Copy)]
pub struct DynamicGeometryTag;
