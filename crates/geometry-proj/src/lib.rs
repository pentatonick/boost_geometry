//! Coordinate-reference-system reprojection for the geometry kernel.
//!
//! Boost.Geometry keeps its projections in `boost/geometry/srs/`
//! (`srs::projection`, `srs::transformation`), outside its documented
//! reference; this crate does not port them and uses the pure-Rust
//! [`proj4rs`] engine (no C dependency) instead. Build a [`Crs`] from
//! a proj4 string, an EPSG code, or a WKT definition, then
//! [`reproject`](reproject()) a geometry from one CRS to another in
//! place.
//!
//! ```
//! use geometry_cs::Cartesian;
//! use geometry_model::Point2D;
//! use geometry_proj::{reproject, Crs};
//! use geometry_trait::Point as _;
//!
//! let wgs84 = Crs::from_epsg(4326).unwrap();      // lon/lat, radians
//! let mercator = Crs::from_epsg(3857).unwrap();   // metres
//! let mut p = Point2D::<f64, Cartesian>::new(0.0, 0.0);
//! reproject(&mut p, &wgs84, &mercator).unwrap();
//! assert!(p.get::<0>().abs() < 1e-6);
//! ```
//!
//! # Units
//!
//! `proj4rs` carries geographic coordinates in **radians**; convert with
//! [`f64::to_radians`] / [`f64::to_degrees`] at the boundary. See
//! [`reproject`](reproject()) for detail.
//!
//! Module layout:
//!
//! * [`crs`] — the [`Crs`] type and its constructors.
//! * [`reproject`](mod@reproject) — the [`ReprojectPoints`] hook and the
//!   [`reproject`](reproject()) function.

#![forbid(unsafe_code)]

extern crate alloc;

pub mod crs;
pub mod reproject;

pub use crs::{Crs, CrsError};
// feature-group: Reprojection
// feature-desc: CRS-to-CRS point reprojection (standalone crate)
pub use reproject::{ReprojectPoints, reproject};
