//! Strategies bound to the Spherical coordinate-system family.
//!
//! Mirrors `boost/geometry/strategies/spherical/` — the directory of
//! Boost strategy headers keyed on `spherical_equatorial_tag`. T40
//! lands the first member, [`distance_haversine`]; later tasks add
//! point-to-segment / side / intersection kernels on the sphere.

pub mod area;
pub mod azimuth;
pub mod distance_haversine;
pub mod length;

pub use area::{SphericalArea, SphericalPolygonArea};
pub use azimuth::SphericalAzimuth;
pub use distance_haversine::{ComparableHaversine, Haversine};
pub use length::{SphericalLength, SphericalPerimeter};
