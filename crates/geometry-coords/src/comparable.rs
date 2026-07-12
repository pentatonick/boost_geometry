//! The `Comparable<T>` newtype: a distance value you can order and
//! compare without paying for the `sqrt` that turns it into a true
//! length.
//!
//! Mirrors `boost::geometry::strategy::distance::comparable::pythagoras`
//! — the squared-distance value carried by
//! `boost/geometry/strategies/cartesian/distance_pythagoras.hpp:71-117`
//! — generalised so that any comparable-form strategy (haversine,
//! point-to-segment, …) can reuse the same wrapper.

use crate::scalar::CoordinateScalar;
use core::cmp::Ordering;

/// A distance you can *compare* but should not *interpret* as a length
/// without explicitly paying for the `sqrt`.
///
/// The C++ counterpart leans on an implicit `operator T()` on the
/// strategy's result type to convert squared-distance to distance on
/// demand
/// (`boost/geometry/strategies/cartesian/distance_pythagoras.hpp:71-117`,
/// the `namespace comparable` block). We deliberately do *not* give
/// `Comparable<T>` a `Deref<Target = T>` or `Into<T>` impl: callers
/// must write `.into_distance()` to opt into the `sqrt` cost. That
/// removes a class of bug — accidentally summing squared distances —
/// that C++'s implicit conversion never quite stops.
///
/// # Examples
///
/// ```
/// use geometry_coords::Comparable;
///
/// // Squared-distance values from two pairs.
/// let near = Comparable(9.0_f64);   // |(0,0)-(0,3)|^2
/// let far  = Comparable(25.0_f64);  // |(0,0)-(0,5)|^2
///
/// // Ordering is preserved by the squared form — no `sqrt` paid.
/// assert!(near < far);
///
/// // Opt in to the `sqrt` only when you actually need a length.
/// assert_eq!(near.into_distance(), 3.0);
/// ```
#[derive(Debug, Clone, Copy)]
#[repr(transparent)]
pub struct Comparable<T: CoordinateScalar>(
    /// The underlying squared-form value. `pub` so that strategies in
    /// other crates (e.g. `geometry-strategy`'s `ComparablePythagoras`)
    /// can construct the wrapper directly without going through a
    /// constructor function.
    pub T,
);

impl<T: CoordinateScalar> Comparable<T> {
    /// Pay the `sqrt`; turn this comparable value into a real distance.
    ///
    /// Counterpart to the implicit `cartesian_distance::operator T()`
    /// conversion in
    /// `boost/geometry/strategies/cartesian/distance_pythagoras.hpp`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_coords::Comparable;
    /// assert_eq!(Comparable(25.0_f64).into_distance(), 5.0);
    /// ```
    #[inline]
    #[must_use]
    pub fn into_distance(self) -> T {
        self.0.sqrt()
    }
}

impl<T: CoordinateScalar> PartialEq for Comparable<T> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: CoordinateScalar> PartialOrd for Comparable<T> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

// Intentionally NOT impl Into<T> or Deref<Target = T>. Forcing the
// caller to write `.into_distance()` is the whole point of the wrapper.
