//! `PostGIS` Extended Well-Known Binary (EWKB) reader and writer.
//!
//! EWKB is OGC Well-Known Binary with a `PostGIS` header dialect: four
//! flag bits in the 32-bit type word, and an optional 32-bit
//! spatial-reference id between the type word and the body. Everything
//! after that header is byte-identical OGC WKB, so this crate is a
//! header codec — it delegates every body to `geometry-io-wkb` rather
//! than carrying a second parser.
//!
//! Only the **outermost** record carries an SRID, which is what
//! `PostGIS` writes. This is a strictly-2D reader: the `Z`, `M` and
//! bounding-box flags are refused by name rather than silently dropped.
//!
//! Empty polygons use zero rings, including inside collections. Other
//! geometry structure is preserved without validation; consumers can
//! reject short or unclosed rings and holes without an exterior.
//!
//! ```
//! use geometry_cs::Cartesian;
//! use geometry_io_ewkb::{ByteOrder, Srid, from_ewkb, to_ewkb, to_ewkb_hex};
//! use geometry_model::Point2D;
//!
//! let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
//! let bytes = to_ewkb(&p, Some(Srid::new(4326)), ByteOrder::LittleEndian);
//!
//! let read = from_ewkb(&bytes).unwrap();
//! assert_eq!(read.srid, Some(Srid::new(4326)));
//! assert_eq!(read.byte_order, ByteOrder::LittleEndian);
//!
//! // The text form of a PostGIS `geometry` column is hex EWKB.
//! assert_eq!(
//!     to_ewkb_hex(&p, Some(Srid::new(4326)), ByteOrder::LittleEndian),
//!     "0101000020E6100000000000000000F03F0000000000000040",
//! );
//! ```
//!
//! # Bringing your own polygon
//!
//! ```
//! use geometry_cs::Cartesian;
//! use geometry_io_ewkb::{ByteOrder, Srid, to_ewkb_polygon};
//! use geometry_model::{Point2D, Polygon, Ring};
//!
//! type Pt = Point2D<f64, Cartesian>;
//! let pg = Polygon::<Pt>::new(Ring::from_vec(vec![
//!     Pt::new(0.0, 0.0),
//!     Pt::new(0.0, 1.0),
//!     Pt::new(1.0, 1.0),
//!     Pt::new(0.0, 0.0),
//! ]));
//! let bytes = to_ewkb_polygon(&pg, Some(Srid::new(4326)), ByteOrder::BigEndian);
//! assert_eq!(bytes[0], 0x00); // big-endian
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod ewkb;
mod ewkb_error;
mod hex;
mod record_header;

pub use ewkb::Ewkb;
pub use ewkb_error::EwkbError;
#[doc(hidden)]
pub use geometry_io_wkb::WriteWkb;
pub use geometry_io_wkb::{ByteOrder, WkbError};
pub use geometry_srid::Srid;
// feature-group: I/O — Extended Well-Known Binary
// feature-desc: Parse and write PostGIS EWKB (WKB with an SRID header), binary and hex
pub use ewkb::from_ewkb;
// feature-group: I/O — Extended Well-Known Binary
pub use ewkb::{to_ewkb, to_ewkb_polygon};
// feature-group: I/O — Extended Well-Known Binary
pub use hex::{from_ewkb_hex, to_ewkb_hex};
