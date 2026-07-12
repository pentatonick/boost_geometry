//! User-facing `distance` / `distance_with` / `comparable_distance`
//! entry points.
//!
//! Mirrors the three free-function overloads Boost.Geometry users
//! reach for:
//!
//! * `boost::geometry::distance(g1, g2)` — strategy-less, picks the
//!   default strategy for the coordinate-system pair. See
//!   `boost/geometry/algorithms/distance.hpp` and the dispatch
//!   plumbing in
//!   `boost/geometry/algorithms/detail/distance/interface.hpp`.
//! * `boost::geometry::distance(g1, g2, strategy)` — explicit-strategy
//!   companion, same file.
//! * `boost::geometry::comparable_distance(g1, g2)` — the
//!   "skip the sqrt" form, from
//!   `boost/geometry/algorithms/comparable_distance.hpp`.
//!
//! The strategy trait, default-strategy projection, and
//! reverse-dispatch wrapper all live in `geometry-strategy::distance`
//! (T21/T22). T23 is the thin layer that turns them into ordinary
//! Rust functions.

use geometry_cs::CoordinateSystem;
use geometry_strategy::distance::{DefaultDistance, DefaultDistanceStrategy, DistanceStrategy};
use geometry_trait::{Geometry, Point};

/// Shorthand for the CS family of `G`'s point type. Used purely to
/// keep the `where` clauses on the three free functions readable;
/// matches the family projection spelled out long-form in the
/// definition of
/// [`geometry_strategy::distance::DefaultDistanceStrategy`].
type Family<G> = <<<G as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family;

/// Distance between two geometries using the default strategy for
/// their coordinate-system pair.
///
/// Mirrors the no-strategy overload of `boost::geometry::distance(g1, g2)`
/// from `boost/geometry/algorithms/distance.hpp` and the dispatch
/// in `boost/geometry/algorithms/detail/distance/interface.hpp`. The
/// default strategy is resolved via
/// [`DefaultDistanceStrategy`], which is the Rust analogue of Boost's
/// `boost::geometry::strategy::distance::services::default_strategy<…>::type`.
#[inline]
pub fn distance<A, B>(
    a: &A,
    b: &B,
) -> <DefaultDistanceStrategy<A, B> as DistanceStrategy<A, B>>::Out
where
    A: Geometry,
    B: Geometry,
    Family<A>: DefaultDistance<Family<B>>,
    DefaultDistanceStrategy<A, B>: DistanceStrategy<A, B> + Default,
{
    DefaultDistanceStrategy::<A, B>::default().distance(a, b)
}

/// Distance between two geometries using an explicit strategy.
///
/// Mirrors the with-strategy overload
/// `boost::geometry::distance(g1, g2, strategy)` from
/// `boost/geometry/algorithms/distance.hpp` and the dispatch
/// in `boost/geometry/algorithms/detail/distance/interface.hpp`.
///
/// The C++ overload is spelled the same as the strategy-less one and
/// disambiguated by SFINAE on the strategy concept; on the Rust side
/// the two overloads become two distinct names ([`distance`] and
/// `distance_with`) so the strategy argument never clashes with the
/// strategy-less form during type inference.
///
/// Taking the strategy by value (rather than by reference, as Boost
/// does with `Strategy const&` at
/// `boost/geometry/algorithms/detail/distance/interface.hpp:68`)
/// keeps the call site free of an `&` everyone would forget. Concrete
/// strategies are zero-sized or pointer-sized configuration objects
/// (`Pythagoras`, `Haversine`, `Andoyer { spheroid }`), so passing
/// them by value monomorphises into nothing.
#[inline]
#[allow(
    clippy::needless_pass_by_value,
    reason = "Strategies are ZST/small Copy configuration objects; by-value matches the user-facing call shape."
)]
pub fn distance_with<A, B, S>(a: &A, b: &B, strategy: S) -> S::Out
where
    A: Geometry,
    B: Geometry,
    S: DistanceStrategy<A, B>,
{
    strategy.distance(a, b)
}

/// Comparable distance — equivalent ordering, cheaper math.
///
/// Mirrors `boost::geometry::comparable_distance(g1, g2)` from
/// `boost/geometry/algorithms/comparable_distance.hpp`. The result
/// preserves the ordering of [`distance`] but skips work the ordering
/// does not need (e.g. the final `sqrt` for Pythagoras). The
/// comparable companion of the default strategy is selected via
/// [`DistanceStrategy::Comparable`], the Rust analogue of Boost's
/// `boost::geometry::strategy::distance::services::comparable_type<…>::type`.
#[inline]
pub fn comparable_distance<A, B>(
    a: &A,
    b: &B,
) -> <<DefaultDistanceStrategy<A, B> as DistanceStrategy<A, B>>::Comparable as DistanceStrategy<
    A,
    B,
>>::Out
where
    A: Geometry,
    B: Geometry,
    Family<A>: DefaultDistance<Family<B>>,
    DefaultDistanceStrategy<A, B>: DistanceStrategy<A, B> + Default,
{
    DefaultDistanceStrategy::<A, B>::default()
        .comparable()
        .distance(a, b)
}

#[cfg(test)]
mod tests {
    //! Canonical 3-4-5 triangle, exercised through every entry point.
    //! Reference value matches `geometry/test/strategies/pythagoras.cpp`
    //! (lines 50-66) and `doc/quickstart.qbk` (Cartesian distance
    //! example).

    use super::{comparable_distance, distance, distance_with};
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;
    use geometry_strategy::cartesian::Pythagoras;

    // 3² + 4² = 25 exactly and √25 = 5 exactly in IEEE-754; `assert_eq!`
    // is the spec's chosen shape (T23 spec, `Tests` block). The
    // pedantic `float_cmp` warning does not apply to integer-valued
    // floats from finite arithmetic on small integers.
    #[allow(clippy::float_cmp, reason = "3-4-5 is exact in IEEE-754 f64.")]
    #[test]
    fn cartesian_quickstart_3_4_5() {
        let a = Point2D::<f64, Cartesian>::new(0.0, 0.0);
        let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
        assert_eq!(distance(&a, &b), 5.0);
        assert_eq!(distance_with(&a, &b, Pythagoras), 5.0);
        assert_eq!(comparable_distance(&a, &b), 25.0);
    }
}
