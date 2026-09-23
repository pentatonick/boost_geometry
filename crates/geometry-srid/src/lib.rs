//! The `PostGIS` spatial-reference identifier (SRID).
//!
//! A spatial-reference id names the coordinate reference system a
//! geometry's ordinates are expressed in. It is a foundation noun: both
//! `PostGIS` dialect crates carry one, `geometry-io-ewkt` as the decimal
//! integer after `SRID=` and `geometry-io-ewkb` as a 32-bit field in the
//! record header. Neither owns it, so it lives here, below both.
//!
//! This crate interprets nothing. It does not resolve an id to a
//! coordinate system, does not validate one against a registry, and does
//! not rewrite values the way `PostGIS` does on ingest — see [`Srid`]'s
//! own documentation for the range `PostGIS` stores.
//!
//! ```
//! use geometry_srid::Srid;
//!
//! let srid = Srid::new(4326);
//! assert_eq!(srid.get(), 4326);
//! assert_eq!(srid.to_string(), "4326");
//! assert_eq!(Srid::UNKNOWN.get(), 0);
//! ```

#![no_std]
#![forbid(unsafe_code)]

mod srid;

pub use srid::Srid;
