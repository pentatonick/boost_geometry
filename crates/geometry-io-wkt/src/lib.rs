//! OGC Well-Known Text (WKT) reader and writer.
//!
//! Mirrors `boost/geometry/io/wkt/{read,write,wkt}.hpp`. The parser
//! emits a [`geometry_model::DynGeometry`] because WKT is heterogeneous
//! by construction (a `GEOMETRYCOLLECTION` mixes kinds); the writer
//! accepts concrete model geometries with [`to_wkt`] and user-defined
//! polygons implementing the geometry traits with [`to_wkt_polygon`].
//!
//! Reference: OGC Simple Feature Access Part 1 (SFA-1) §7 for the WKT
//! grammar.
//!
//! ## Serialize a user-defined polygon
//!
//! Application types can implement the lightweight [`geometry_trait`] traits
//! directly; they do not need to be converted to a `geometry_model` polygon.
//!
//! ```
//! use geometry_cs::Cartesian;
//! use geometry_io_wkt::to_wkt_polygon;
//! use geometry_tag::{PointTag, PolygonTag, RingTag};
//! use geometry_trait::{Geometry, Point, Polygon, Ring};
//!
//! struct Coordinate(f64, f64);
//!
//! impl Geometry for Coordinate {
//!     type Kind = PointTag;
//!     type Point = Self;
//! }
//!
//! impl Point for Coordinate {
//!     type Scalar = f64;
//!     type Cs = Cartesian;
//!     const DIM: usize = 2;
//!
//!     fn get<const D: usize>(&self) -> f64 {
//!         match D {
//!             0 => self.0,
//!             1 => self.1,
//!             _ => unreachable!("a Coordinate has two dimensions"),
//!         }
//!     }
//! }
//!
//! struct Boundary(Vec<Coordinate>);
//!
//! impl Geometry for Boundary {
//!     type Kind = RingTag;
//!     type Point = Coordinate;
//! }
//!
//! impl Ring for Boundary {
//!     fn points(&self) -> impl ExactSizeIterator<Item = &Coordinate> + Clone {
//!         self.0.iter()
//!     }
//! }
//!
//! struct Parcel {
//!     exterior: Boundary,
//!     holes: Vec<Boundary>,
//! }
//!
//! impl Geometry for Parcel {
//!     type Kind = PolygonTag;
//!     type Point = Coordinate;
//! }
//!
//! impl Polygon for Parcel {
//!     type Ring = Boundary;
//!
//!     fn exterior(&self) -> &Boundary {
//!         &self.exterior
//!     }
//!
//!     fn interiors(&self) -> impl ExactSizeIterator<Item = &Boundary> {
//!         self.holes.iter()
//!     }
//! }
//!
//! let parcel = Parcel {
//!     exterior: Boundary(vec![
//!         Coordinate(0.0, 0.0),
//!         Coordinate(0.0, 2.0),
//!         Coordinate(2.0, 2.0),
//!         Coordinate(2.0, 0.0),
//!         Coordinate(0.0, 0.0),
//!     ]),
//!     holes: vec![],
//! };
//!
//! assert_eq!(
//!     to_wkt_polygon(&parcel),
//!     "POLYGON((0 0,0 2,2 2,2 0,0 0))"
//! );
//! ```

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
pub use write::{WriteWkt, to_wkt, to_wkt_polygon, write_wkt};
