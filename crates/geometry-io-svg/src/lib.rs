//! SVG output for Cartesian geometries — a debugging convenience.
//!
//! Mirrors `boost/geometry/io/svg/svg_mapper.hpp`. Cartesian only: an
//! [`SvgMapper`] accumulates geometries, tracks their combined bounding
//! box, and emits a self-contained `<svg>` document mapping world
//! coordinates to a fixed pixel canvas (y flipped, since SVG y grows
//! downward).

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod mapper;

// feature-group: I/O — SVG
// feature-desc: Render geometries to SVG (debugging)
// feature-keep: SvgMapper
pub use mapper::SvgMapper;
