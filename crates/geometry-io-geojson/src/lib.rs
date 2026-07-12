//! RFC 7946 `GeoJSON` reader and writer.
//!
//! Not part of Boost.Geometry; follows RFC 7946. The parser emits a
//! [`geometry_model::DynGeometry`] (a `GeoJSON` `GeometryCollection` is
//! heterogeneous); the writer serialises any concrete model geometry
//! to a `GeoJSON` string. Feature objects and property bags are out of
//! scope — only the `geometry` member's OGC-equivalent kinds.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod json;
mod parse;
mod write;

pub use json::GeoJsonError;
pub use parse::from_geojson;
pub use write::to_geojson;
