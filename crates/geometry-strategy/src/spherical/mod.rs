//! Strategies bound to the Spherical coordinate-system family.
//!
//! Mirrors `boost/geometry/strategies/spherical/` — the directory of
//! Boost strategy headers keyed on `spherical_equatorial_tag`. T40
//! lands the first member, [`distance_haversine`]; later tasks add
//! point-to-segment / side / intersection kernels on the sphere.

pub mod area;
pub mod area_chamberlain_duquette;
pub mod azimuth;
pub mod closest_points_haversine;
pub mod distance_cross_track;
pub mod distance_haversine;
mod great_circle;
pub mod length;

pub use area::{SphericalArea, SphericalPolygonArea};
pub use area_chamberlain_duquette::ChamberlainDuquetteArea;
pub use azimuth::SphericalAzimuth;
pub use closest_points_haversine::HaversineClosestPoints;
pub use distance_cross_track::CrossTrack;
pub use distance_haversine::{ComparableHaversine, Haversine};
pub use length::{SphericalLength, SphericalPerimeter};
