//! Geometry kind tags and the tag-hierarchy marker traits.
//!
//! Eleven empty tag types identify each OGC geometry kind, and a set of
//! marker traits (`Single`, `Multi`, `Pointlike`, `Linear`, `Polylinear`,
//! `Areal`, `Polygonal`, `Volumetric`) reproduce the C++ struct-inheritance
//! hierarchy at the Rust trait-bound level. Together they let downstream
//! crates dispatch on tag *identity* (one impl per tag) and tag *category*
//! (one impl that covers every linear tag, every areal tag, etc.) — the
//! Rust analogue of `tag_cast<Tag, Stops...>`.
//!
//! References:
//! - `boost/geometry/core/tags.hpp` — tag hierarchy declarations.
//! - `boost/geometry/core/tag.hpp` — the `traits::tag<G>::type` metafunction.
//! - `boost/geometry/core/tag_cast.hpp` — base-tag walking, replaced here
//!   by Rust trait super-bounds.
//!
//! # Examples
//!
//! Category dispatch — one impl covers every linear tag:
//!
//! ```
//! use geometry_tag::{Linear, LinestringTag, MultiLinestringTag, SegmentTag};
//!
//! fn accepts_linear<T: Linear>() {}
//! accepts_linear::<SegmentTag>();
//! accepts_linear::<LinestringTag>();
//! accepts_linear::<MultiLinestringTag>();
//! ```

#![no_std]
#![forbid(unsafe_code)]

mod hierarchy;
mod same_as;
mod tag;

pub use hierarchy::{Areal, Linear, Multi, Pointlike, Polygonal, Polylinear, Single, Volumetric};
pub use same_as::SameAs;
pub use tag::{
    BoxTag, DynamicGeometryTag, GeometryCollectionTag, LinestringTag, MultiLinestringTag,
    MultiPointTag, MultiPolygonTag, PointTag, PolygonTag, PolyhedralSurfaceTag, RingTag,
    SegmentTag,
};
