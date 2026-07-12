//! Strategies bound to the Cartesian coordinate-system family.
//!
//! Mirrors `boost/geometry/strategies/cartesian/` — the directory of
//! Boost strategy headers that key on `cartesian_tag`. T22 lands the
//! first member, `distance_pythagoras`; later tasks add the
//! point-to-segment projection (T24) and side / intersection kernels.

pub mod distance_projected_point;
pub mod distance_pythagoras;

pub use distance_projected_point::PointToSegment;
pub use distance_pythagoras::{ComparablePythagoras, Pythagoras};
