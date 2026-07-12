//! OGC Well-Known Text (WKT) reader and writer.
//!
//! Mirrors `boost/geometry/io/wkt/{read,write,wkt}.hpp`. The parser
//! emits a [`geometry_model::DynGeometry`] because WKT is heterogeneous
//! by construction (a `GEOMETRYCOLLECTION` mixes kinds); the writer
//! accepts any concrete geometry that implements the model traits.
//!
//! Reference: OGC Simple Feature Access Part 1 (SFA-1) §7 for the WKT
//! grammar.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod lexer;
mod parse;
mod write;

pub use lexer::{Token, WktError};
pub use parse::{
    from_wkt, parse_linestring, parse_multi_linestring, parse_multi_point, parse_multi_polygon,
    parse_point, parse_polygon,
};
pub use write::{WriteWkt, to_wkt, write_wkt};
