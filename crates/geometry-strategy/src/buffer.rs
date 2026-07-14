//! Composable buffer strategies.
//!
//! Mirrors the five strategy roles accepted by Boost.Geometry's buffer
//! interface: distance, side, join, end, and point. The strategy values are
//! data-only and live below the overlay engine so algorithms can consume them
//! without creating a dependency cycle.

use geometry_cs::{CartesianFamily, CoordinateSystem, GeographicFamily, SphericalFamily, Spheroid};
use geometry_trait::Geometry;

/// Cartesian coordinate strategy bundle for buffer construction.
///
/// Mirrors `boost::geometry::strategies::buffer::cartesian<>` from
/// `strategies/buffer/cartesian.hpp:24-31`.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct CartesianBuffer;

/// Spherical coordinate strategy bundle for buffer construction.
///
/// Mirrors `boost::geometry::strategies::buffer::spherical` from
/// `strategies/buffer/spherical.hpp:24-46`. Distances use the same unit as
/// `radius`. The overlay consumer applies this bundle through a local tangent
/// projection, an explicitly recorded approximation for local buffers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphericalBuffer {
    /// Sphere radius in the buffer distance unit.
    pub radius: f64,
}

impl SphericalBuffer {
    /// Unit-sphere strategy matching Boost's default radius.
    pub const UNIT: Self = Self { radius: 1.0 };

    /// Construct a spherical buffer strategy with an explicit radius.
    #[must_use]
    pub const fn new(radius: f64) -> Self {
        Self { radius }
    }
}

impl Default for SphericalBuffer {
    fn default() -> Self {
        Self::UNIT
    }
}

/// Geographic coordinate strategy bundle for buffer construction.
///
/// Mirrors `boost::geometry::strategies::buffer::geographic` from
/// `strategies/buffer/geographic.hpp:25-48`. The overlay consumer derives the
/// local meridional and prime-vertical scales from this spheroid before using
/// its Cartesian offset engine.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeographicBuffer {
    /// Reference spheroid used to convert angular coordinates and metric
    /// buffer distances.
    pub spheroid: Spheroid,
}

impl GeographicBuffer {
    /// WGS84 geographic buffer strategy.
    pub const WGS84: Self = Self {
        spheroid: Spheroid::WGS84,
    };

    /// Construct a geographic buffer strategy with an explicit spheroid.
    #[must_use]
    pub const fn new(spheroid: Spheroid) -> Self {
        Self { spheroid }
    }
}

impl Default for GeographicBuffer {
    fn default() -> Self {
        Self::WGS84
    }
}

/// Select the default coordinate strategy bundle for a buffer input family.
///
/// Mirrors `strategies::buffer::services::default_strategy` from
/// `strategies/buffer/services.hpp:24-40` and its Cartesian, spherical, and
/// geographic specializations.
pub trait DefaultBuffer<Family> {
    /// Default coordinate strategy for this family.
    type Strategy: Default;
}

impl DefaultBuffer<CartesianFamily> for CartesianFamily {
    type Strategy = CartesianBuffer;
}

impl DefaultBuffer<SphericalFamily> for SphericalFamily {
    type Strategy = SphericalBuffer;
}

impl DefaultBuffer<GeographicFamily> for GeographicFamily {
    type Strategy = GeographicBuffer;
}

/// Coordinate strategy selected by the point coordinate-system family of `G`.
pub type DefaultBufferStrategy<G> = <<<<G as Geometry>::Point as geometry_trait::Point>::Cs as CoordinateSystem>::Family as DefaultBuffer<<<<G as Geometry>::Point as geometry_trait::Point>::Cs as CoordinateSystem>::Family>>::Strategy;

/// Signed offset distance policy.
///
/// Mirrors `strategy::buffer::distance_symmetric` and `distance_asymmetric`
/// from `strategies/agnostic/buffer_distance_symmetric.hpp:46-104` and
/// `buffer_distance_asymmetric.hpp:45-111`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BufferDistanceStrategy {
    /// One signed distance on both sides.
    Symmetric(f64),
    /// Independent offsets on the left and right of a linear geometry.
    Asymmetric {
        /// Offset on the directed line's left side.
        left: f64,
        /// Offset on the directed line's right side.
        right: f64,
    },
}

/// Side-offset construction policy.
///
/// Mirrors `strategy::buffer::side_straight` from
/// `strategies/cartesian/buffer_side_straight.hpp:46-136`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferSideStrategy {
    /// Offset each source segment by its perpendicular normal.
    Straight,
}

/// Corner join policy.
///
/// Mirrors `strategy::buffer::join_round` and `join_miter` from
/// `strategies/cartesian/buffer_join_round.hpp:57-164` and
/// `buffer_join_miter.hpp:52-134`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BufferJoinStrategy {
    /// Circular corner approximation.
    Round {
        /// Segment count for a complete circle.
        points_per_circle: usize,
    },
    /// Offset-edge intersection capped to `limit × distance`.
    Miter {
        /// Maximum miter ratio.
        limit: f64,
    },
}

/// Linear endpoint policy.
///
/// Mirrors `strategy::buffer::end_round` and `end_flat` from
/// `strategies/cartesian/buffer_end_round.hpp:55-173` and
/// `buffer_end_flat.hpp:50-100`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BufferEndStrategy {
    /// Semicircular endpoint approximation.
    Round {
        /// Segment count for a complete circle.
        points_per_circle: usize,
    },
    /// End the buffer at the source endpoint without extension.
    Flat,
}

/// Point-buffer policy.
///
/// Mirrors `strategy::buffer::point_circle` and `point_square` from
/// `strategies/cartesian/buffer_point_circle.hpp:56-109` and
/// `buffer_point_square.hpp:51-110`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferPointStrategy {
    /// Regular-polygon circle approximation.
    Circle {
        /// Vertex count for a complete circle.
        points_per_circle: usize,
    },
    /// Axis-aligned square centered on the point.
    Square,
}

/// Complete strategy bundle accepted by `buffer_with`.
///
/// Mirrors the five explicit strategy arguments to
/// `boost::geometry::buffer` from
/// `algorithms/detail/buffer/interface.hpp:246-273`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BufferSettings {
    /// Signed distance policy.
    pub distance: BufferDistanceStrategy,
    /// Side construction policy.
    pub side: BufferSideStrategy,
    /// Corner join policy.
    pub join: BufferJoinStrategy,
    /// Linear endpoint policy.
    pub end: BufferEndStrategy,
    /// Point construction policy.
    pub point: BufferPointStrategy,
}

impl BufferSettings {
    /// Conventional round buffer using one symmetric distance.
    ///
    /// Composes Boost's distance-symmetric, side-straight, join-round,
    /// end-round, and point-circle strategies from the headers cited on the
    /// corresponding enum types.
    #[must_use]
    pub const fn round(distance: f64, points_per_circle: usize) -> Self {
        Self {
            distance: BufferDistanceStrategy::Symmetric(distance),
            side: BufferSideStrategy::Straight,
            join: BufferJoinStrategy::Round { points_per_circle },
            end: BufferEndStrategy::Round { points_per_circle },
            point: BufferPointStrategy::Circle { points_per_circle },
        }
    }
}

impl Default for BufferSettings {
    fn default() -> Self {
        Self::round(1.0, 36)
    }
}
