//! Strategies bound to the Geographic coordinate-system family.
//!
//! Mirrors `boost/geometry/strategies/geographic/`. Hosts the
//! geodesic distance strategies (Andoyer in T43, Vincenty in T44,
//! optional Thomas in T45) together with the
//! `spheroid_calc::SpheroidCalc` helper they share.

pub mod area;
pub mod azimuth;
pub mod distance_andoyer;
pub mod distance_thomas;
pub mod distance_vincenty;
pub mod length;
pub mod spheroid_calc;

pub use area::{GeographicArea, GeographicPolygonArea};
pub use azimuth::GeographicAzimuth;
pub use distance_andoyer::Andoyer;
pub use distance_thomas::Thomas;
pub use distance_vincenty::Vincenty;
pub use length::{GeographicLength, GeographicPerimeter};
