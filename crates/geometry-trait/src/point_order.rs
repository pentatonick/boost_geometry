//! The [`PointOrder`] enum: vertex traversal direction of a ring.
//!
//! Mirrors `boost::geometry::order_selector` from
//! `boost/geometry/core/point_order.hpp`. The Rust port drops the
//! `order_undetermined` variant — like `closure_undetermined`, Boost
//! itself flags it "(not yet supported)" in the same header.

/// Vertex traversal direction of a ring's boundary.
///
/// Mirrors `boost::geometry::order_selector` from
/// `boost/geometry/core/point_order.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::PointOrder;
/// assert_ne!(PointOrder::Clockwise, PointOrder::CounterClockwise);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PointOrder {
    /// Points are ordered clockwise.
    ///
    /// Mirrors `boost::geometry::clockwise` (value `1`) from
    /// `boost/geometry/core/point_order.hpp`. This is the Boost
    /// default (`traits::point_order<G>::value = clockwise`), and the
    /// default returned by [`crate::Ring::point_order`].
    Clockwise,
    /// Points are ordered counter-clockwise.
    ///
    /// Mirrors `boost::geometry::counterclockwise` (value `2`) from
    /// `boost/geometry/core/point_order.hpp`.
    CounterClockwise,
}
