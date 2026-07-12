//! OGC Well-Known Binary (WKB) reader and writer.
//!
//! Follows OGC Simple Feature Access 06-103r4 §8 (Boost.Geometry ships
//! no WKB). The parser emits a [`geometry_model::DynGeometry`]; the
//! writer serialises any concrete model geometry to a byte vector in a
//! caller-chosen [`ByteOrder`].

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod header;
mod parse;
mod write;

pub use header::{ByteOrder, WkbError};
pub use parse::from_wkb;
pub use write::to_wkb;
