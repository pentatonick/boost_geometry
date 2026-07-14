//! RFC 7946 `GeoJSON` reader and writer.
//!
//! Not part of Boost.Geometry; follows RFC 7946. The parser emits a
//! [`geometry_model::DynGeometry`] (a `GeoJSON` `GeometryCollection` is
//! heterogeneous); the writer serialises concrete model geometries with
//! [`to_geojson`] and any user-defined polygon implementing the geometry
//! traits with [`to_geojson_polygon`]. Feature objects and property bags
//! are out of scope — only the `geometry` member's OGC-equivalent kinds.
//!
//! ## Serialize a user-defined polygon
//!
//! Application types can implement the lightweight [`geometry_trait`] traits
//! directly; they do not need to be converted to a `geometry_model` polygon.
//!
//! ```
//! use geometry_cs::Cartesian;
//! use geometry_io_geojson::{from_geojson, to_geojson_polygon};
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
//! let geojson = to_geojson_polygon(&parcel);
//! assert_eq!(
//!     geojson,
//!     r#"{"type":"Polygon","coordinates":[[[0,0],[0,2],[2,2],[2,0],[0,0]]]}"#
//! );
//! assert!(from_geojson(&geojson).is_ok());
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod json;
mod parse;
mod write;

pub use json::GeoJsonError;
pub use parse::from_geojson;
pub use write::{to_geojson, to_geojson_polygon};
