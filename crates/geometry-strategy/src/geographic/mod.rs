//! Strategies bound to the Geographic coordinate-system family.
//!
//! Mirrors `boost/geometry/strategies/geographic/`. Hosts the
//! geodesic distance strategies (Andoyer, Vincenty, and Thomas), their
//! Vincenty/Thomas direct counterparts, together with the
//! `spheroid_calc::SpheroidCalc` helper they share.

pub mod area;
pub mod azimuth;
mod differential;
mod direct;
mod direct_karney;
mod direct_thomas;
mod direct_vincenty;
pub mod distance_andoyer;
pub mod distance_thomas;
pub mod distance_vincenty;
mod geodesic_intersection;
mod inverse;
mod inverse_karney;
pub mod length;
mod meridian;
pub mod spheroid_calc;
mod vertex;

pub use area::{GeographicArea, GeographicPolygonArea};
pub use azimuth::GeographicAzimuth;
pub use differential::{DifferentialQuantities, differential_quantities};
pub use direct::DirectResult;
pub use direct_karney::KarneyDirect;
pub use direct_thomas::ThomasDirect;
pub use direct_vincenty::VincentyDirect;
pub use distance_andoyer::Andoyer;
pub use distance_thomas::Thomas;
pub use distance_vincenty::Vincenty;
pub use geodesic_intersection::{GeodesicIntersection, Gnomonic, Sjoberg};
pub use inverse::InverseResult;
pub use inverse_karney::{Karney, KarneyInverse};
pub use length::{GeographicLength, GeographicPerimeter};
pub use meridian::{Meridian, MeridianInverseResult, MeridianSegmentKind};
pub use vertex::{
    geographic_vertex_latitude, geographic_vertex_longitude, spherical_vertex_latitude,
    spherical_vertex_longitude,
};
