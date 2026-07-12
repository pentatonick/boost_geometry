//! The [`Closure`] enum: whether a ring repeats its first point as
//! its last.
//!
//! Mirrors `boost::geometry::closure_selector` from
//! `boost/geometry/core/closure.hpp`. The Rust port drops the
//! `closure_undetermined` variant — Boost itself marks it
//! "(Not yet implemented)" in the same header and no algorithm in
//! the C++ kernel relies on it. If a future task needs the
//! undetermined variant it can be added without breaking the
//! existing two-variant API by introducing a new wrapper type.

/// Whether a ring repeats its first point as its last.
///
/// Mirrors `boost::geometry::closure_selector` from
/// `boost/geometry/core/closure.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::Closure;
/// assert_ne!(Closure::Open, Closure::Closed);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Closure {
    /// Last point != first point — algorithms close the ring on the fly.
    ///
    /// Mirrors `boost::geometry::open` (value `0`) from
    /// `boost/geometry/core/closure.hpp`.
    Open,
    /// Last point == first point — the ring carries the closing vertex
    /// explicitly.
    ///
    /// Mirrors `boost::geometry::closed` (value `1`) from
    /// `boost/geometry/core/closure.hpp`. This is the Boost default
    /// (`traits::closure<G>::value = closed`), and the default returned
    /// by [`crate::Ring::closure`].
    Closed,
}
