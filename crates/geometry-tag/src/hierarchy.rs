//! Marker traits that reproduce the C++ tag inheritance hierarchy.
//!
//! In `boost/geometry/core/tags.hpp` the kind tags inherit from category
//! tags (`point_tag : single_tag, pointlike_tag`, etc.). Rust has no struct
//! inheritance, so we replace each category with an empty marker trait and
//! implement the trait for every tag that would inherit from the C++ base.
//! A bound like `G::Kind: Linear` then automatically covers `SegmentTag`,
//! `LinestringTag`, and `MultiLinestringTag` — the equivalent of
//! `tag_cast<Tag, ..., linear_tag>` (`boost/geometry/core/tag_cast.hpp`).

use crate::tag::{
    BoxTag, GeometryCollectionTag, LinestringTag, MultiLinestringTag, MultiPointTag,
    MultiPolygonTag, PointTag, PolygonTag, PolyhedralSurfaceTag, RingTag, SegmentTag,
};

/// Single-geometry marker — counterpart of `single_tag` (`boost/geometry/core/tags.hpp:59`).
///
/// # Examples
///
/// ```
/// use geometry_tag::{PointTag, RingTag, Single};
/// fn k<T: Single>() {}
/// k::<PointTag>();
/// k::<RingTag>();
/// ```
pub trait Single {}

/// Multi-geometry marker — counterpart of `multi_tag` (`boost/geometry/core/tags.hpp:63`).
///
/// # Examples
///
/// ```
/// use geometry_tag::{Multi, MultiPointTag, MultiPolygonTag};
/// fn k<T: Multi>() {}
/// k::<MultiPointTag>();
/// k::<MultiPolygonTag>();
/// ```
pub trait Multi {}

/// Point-like category marker — counterpart of `pointlike_tag` (`boost/geometry/core/tags.hpp:66`).
///
/// # Examples
///
/// ```
/// use geometry_tag::{MultiPointTag, PointTag, Pointlike};
/// fn k<T: Pointlike>() {}
/// k::<PointTag>();
/// k::<MultiPointTag>();
/// ```
pub trait Pointlike {}

/// Linear category marker — counterpart of `linear_tag` (`boost/geometry/core/tags.hpp:69`).
///
/// # Examples
///
/// ```
/// use geometry_tag::{Linear, LinestringTag, MultiLinestringTag, SegmentTag};
/// fn k<T: Linear>() {}
/// k::<SegmentTag>();
/// k::<LinestringTag>();
/// k::<MultiLinestringTag>();
/// ```
pub trait Linear {}

/// Polylinear category marker — counterpart of `polylinear_tag : linear_tag`
/// (`boost/geometry/core/tags.hpp:72`). The C++ inheritance is expressed as
/// a Rust super-trait bound, so `T: Polylinear` automatically implies
/// `T: Linear`.
///
/// # Examples
///
/// ```
/// use geometry_tag::{LinestringTag, MultiLinestringTag, Polylinear};
/// fn k<T: Polylinear>() {}
/// k::<LinestringTag>();
/// k::<MultiLinestringTag>();
/// ```
pub trait Polylinear: Linear {}

/// Areal category marker — counterpart of `areal_tag` (`boost/geometry/core/tags.hpp:75`).
///
/// # Examples
///
/// ```
/// use geometry_tag::{Areal, BoxTag, PolygonTag, RingTag};
/// fn k<T: Areal>() {}
/// k::<BoxTag>();
/// k::<RingTag>();
/// k::<PolygonTag>();
/// ```
pub trait Areal {}

/// Polygonal category marker — counterpart of `polygonal_tag : areal_tag`
/// (`boost/geometry/core/tags.hpp:78`). The C++ inheritance is expressed as
/// a Rust super-trait bound, so `T: Polygonal` automatically implies
/// `T: Areal`.
///
/// # Examples
///
/// ```
/// use geometry_tag::{MultiPolygonTag, Polygonal, RingTag};
/// fn k<T: Polygonal>() {}
/// k::<RingTag>();
/// k::<MultiPolygonTag>();
/// ```
pub trait Polygonal: Areal {}

/// Volumetric category marker — counterpart of `volumetric_tag`
/// (`boost/geometry/core/tags.hpp:81`).
///
/// # Examples
///
/// ```
/// use geometry_tag::{PolyhedralSurfaceTag, Volumetric};
/// fn k<T: Volumetric>() {}
/// k::<PolyhedralSurfaceTag>();
/// ```
pub trait Volumetric {}

// --- PointTag : single_tag, pointlike_tag ----------------------------------
impl Single for PointTag {}
impl Pointlike for PointTag {}

// --- SegmentTag : single_tag, linear_tag -----------------------------------
impl Single for SegmentTag {}
impl Linear for SegmentTag {}

// --- LinestringTag : single_tag, polylinear_tag (-> linear_tag) ------------
impl Single for LinestringTag {}
impl Linear for LinestringTag {}
impl Polylinear for LinestringTag {}

// --- RingTag : single_tag, polygonal_tag (-> areal_tag) --------------------
impl Single for RingTag {}
impl Areal for RingTag {}
impl Polygonal for RingTag {}

// --- PolygonTag : single_tag, polygonal_tag (-> areal_tag) -----------------
impl Single for PolygonTag {}
impl Areal for PolygonTag {}
impl Polygonal for PolygonTag {}

// --- BoxTag : single_tag, areal_tag ----------------------------------------
impl Single for BoxTag {}
impl Areal for BoxTag {}

// --- MultiPointTag : multi_tag, pointlike_tag ------------------------------
impl Multi for MultiPointTag {}
impl Pointlike for MultiPointTag {}

// --- MultiLinestringTag : multi_tag, polylinear_tag (-> linear_tag) --------
impl Multi for MultiLinestringTag {}
impl Linear for MultiLinestringTag {}
impl Polylinear for MultiLinestringTag {}

// --- MultiPolygonTag : multi_tag, polygonal_tag (-> areal_tag) -------------
impl Multi for MultiPolygonTag {}
impl Areal for MultiPolygonTag {}
impl Polygonal for MultiPolygonTag {}

// --- GeometryCollectionTag : multi_tag -------------------------------------
impl Multi for GeometryCollectionTag {}

// --- PolyhedralSurfaceTag : single_tag, volumetric_tag ---------------------
impl Single for PolyhedralSurfaceTag {}
impl Volumetric for PolyhedralSurfaceTag {}
