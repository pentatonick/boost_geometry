//! Coordinate systems, angle units, and reference spheroids.
//!
//! Mirrors `boost/geometry/core/cs.hpp` (the `cs::cartesian`,
//! `cs::spherical`, `cs::geographic`, `cs::polar` tag templates and their
//! `cs_tag` family classifier) together with
//! `boost/geometry/core/coordinate_system.hpp` (the
//! `traits::coordinate_system<Point>` typedef that picks one of those
//! tags out per point type) and `boost/geometry/srs/spheroid.hpp` (the
//! reference ellipsoid carried alongside any geographic strategy).
//!
//! Crate split per proposal §3.3: a coordinate system is a *type* with
//! an associated [`CoordinateSystem::Family`], and algorithm strategies
//! bind on the family — never on the concrete CS — so that degree and
//! radian variants share one impl.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

mod family;
mod spheroid;
mod spheroidal;
mod system;
mod unit;

pub use family::{CartesianFamily, GeographicFamily, PolarFamily, SphericalFamily};
pub use spheroid::Spheroid;
pub use spheroidal::{SpheroidalScalar, SpheroidalUnits, normalize_spheroidal_box_coordinates};
pub use system::{Cartesian, CoordinateSystem, Geographic, Polar, Spherical};
pub use unit::{AngleUnit, Degree, FromF64, Radian};
