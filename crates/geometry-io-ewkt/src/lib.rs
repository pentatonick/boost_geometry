//! `PostGIS` Extended Well-Known Text (EWKT) reader and writer.
//!
//! Not part of Boost.Geometry; follows the `PostGIS` manual, sections
//! "4.1.3. WKT and WKB" and "4.2.1. `PostGIS` EWKB and EWKT", and the
//! `PostGIS` reader in `liblwgeom/lwin_wkt_lex.l`. EWKT is the dialect
//! `ST_AsEWKT` emits and `ST_GeomFromEWKT` accepts: OGC WKT plus an
//! optional `SRID=<digits>;` prefix and the glued dimension-suffix
//! spellings. `ST_AsEWKT` glues only `M` (`POINTM(1 2 3)`); `PostGIS`
//! reads the glued `Z`, `M`, and `ZM` forms and the OGC-spaced
//! `POINT Z` / `POINT M` / `POINT ZM` forms alike, and this crate reads
//! all of them.
//!
//! The geometry body is delegated to [`geometry_io_wkt`]. This crate
//! scans the prefix, overwrites a glued suffix with spaces in place — a
//! same-length rewrite, so every byte offset it reports indexes the
//! string the caller passed — and hands the body over. [`from_ewkt`]
//! therefore emits a [`geometry_model::DynGeometry`], and the six
//! `parse_*` conveniences return concrete model types; each is paired
//! with the spatial-reference id in an [`Ewkt`].
//!
//! `PostGIS` treats SRID 0 as unknown ([`Srid::UNKNOWN`]) and omits the
//! prefix for it. This crate reads `SRID=0;` as `Some(Srid::UNKNOWN)`
//! and writes it back as `SRID=0;`; callers pass `None` to omit the
//! prefix.
//!
//! Every ordinate past the second is discarded, as in
//! [`geometry_io_wkt`], and nothing above 2D is ever written — so a
//! dimension suffix carries no information this model can hold.
//!
//! ## Read and write a prefixed geometry
//!
//! ```
//! use geometry_io_ewkt::{Srid, from_ewkt, to_ewkt};
//!
//! let e = from_ewkt("SRID=4326;POINTM(1 2 3)").unwrap();
//! assert_eq!(e.srid, Some(Srid::new(4326)));
//! assert_eq!(to_ewkt(&e.geometry, e.srid), "SRID=4326;POINT(1 2)");
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

mod dimension_suffix;
mod ewkt;
mod ewkt_error;
mod srid;
mod srid_prefix;

pub use ewkt_error::EwktError;
pub use geometry_io_wkt::WktError;
#[doc(hidden)]
pub use geometry_io_wkt::WriteWkt;
pub use srid::Srid;
// feature-group: I/O — Extended Well-Known Text
// feature-desc: Parse and write PostGIS EWKT (WKT with an SRID prefix)
pub use ewkt::{
    Ewkt, from_ewkt, parse_linestring, parse_multi_linestring, parse_multi_point,
    parse_multi_polygon, parse_point, parse_polygon,
};
// feature-group: I/O — Extended Well-Known Text
pub use ewkt::{to_ewkt, to_ewkt_polygon, write_ewkt};
