//! The [`DistanceStrategy`] trait, default strategy selection, and
//! the [`Reversed`] argument-swapping adapter.
//!
//! Mirrors three pieces of Boost.Geometry that collaborate to make
//! `boost::geometry::distance(a, b)` work for any geometry pair in
//! any coordinate system, with implicit or explicit strategy
//! selection:
//!
//! * `boost/geometry/strategies/distance.hpp` — the
//!   `boost::geometry::strategy::distance` namespace and the per-CS
//!   strategy concept.
//! * `boost/geometry/strategies/distance/services.hpp` — the
//!   `services::default_strategy<G1, G2>` metafunction that picks
//!   the right strategy for a CS pair (Pythagoras for cartesian,
//!   Haversine / Andoyer for spherical / geographic, …).
//! * `boost/geometry/strategies/default_strategy.hpp` plus
//!   `boost/geometry/core/reverse_dispatch.hpp` and the partial
//!   specialisation at
//!   `boost/geometry/algorithms/detail/distance/interface.hpp:53-77`
//!   — the symmetry-collapsing layer that gives you
//!   `distance(segment, point)` for free once you have
//!   `distance(point, segment)`.
//!
//! T21 lands the *trait surface* only — no concrete strategies yet.
//! T22 adds `Pythagoras` / `ComparablePythagoras` and the first
//! [`DefaultDistance`] impls; T23 wires this into the free-function
//! `geometry-algorithm::distance` entry point.

use geometry_coords::CoordinateScalar;
use geometry_cs::CoordinateSystem;
use geometry_trait::{Geometry, Point};

/// A strategy for computing the distance between two geometries.
///
/// Mirrors the per-CS strategy concept declared in
/// `boost/geometry/strategies/distance.hpp` and refined per
/// coordinate system in `strategies/{cartesian,spherical,geographic}/
/// distance_*.hpp`. The Boost concept exposes
/// `apply(g1, g2)` plus a `return_type<G1, G2>` metafunction plus a
/// `comparable_type<...>` sibling; we collapse those three onto one
/// Rust trait via associated types.
///
/// # Associated items
///
/// * [`Self::Out`] — the scalar the distance comes back as.
///   Equivalent to Boost's
///   `boost::geometry::strategy::distance::services::return_type<...>`
///   (`strategies/distance/services.hpp`). Typically the wider of
///   `A`'s and `B`'s coordinate scalars after promotion.
/// * [`Self::Comparable`] — the "skip the sqrt" form of
///   this strategy. For Pythagoras this is `ComparablePythagoras`;
///   for strategies where there is nothing to save by deferring a
///   square root (Andoyer, Vincenty), set `type Comparable = Self;`
///   and the optimiser collapses the indirection. Mirrors
///   `comparable_type<...>` from the same Boost services header.
///
/// # Method shape
///
/// `distance(&self, &A, &B)` mirrors `apply(g1, g2)` on Boost's
/// strategy structs (e.g.
/// `strategies/cartesian/distance_pythagoras.hpp:75-95` for the
/// comparable form, `:99-115` for the sqrt-paying form).
/// `comparable(&self)` mirrors `get_comparable(strategy)` from
/// `strategies/distance/services.hpp`.
pub trait DistanceStrategy<A: Geometry, B: Geometry> {
    /// The output scalar type. Typically the promoted scalar of
    /// `A` and `B`. Mirrors
    /// `services::return_type<Strategy, P1, P2>::type` from
    /// `strategies/distance/services.hpp`.
    type Out: CoordinateScalar;

    /// The "skip the sqrt" form of this strategy.
    ///
    /// For Pythagoras → `ComparablePythagoras` (returns squared
    /// distance). For strategies where there is no win from a
    /// comparable form (Vincenty, Andoyer), `type Comparable = Self;`
    /// — the optimiser collapses it. Mirrors
    /// `comparable_type<Strategy>::type` from
    /// `strategies/distance/services.hpp`.
    ///
    /// The bound `DistanceStrategy<A, B, Out = Self::Out>` enforces
    /// the invariant that comparing two comparable distances gives
    /// the same ordering as comparing two real distances — both
    /// sides of the comparison must speak the same scalar type.
    type Comparable: DistanceStrategy<A, B, Out = Self::Out>;

    /// Compute the distance between `a` and `b`.
    ///
    /// Mirrors `apply(g1, g2)` on Boost strategy structs (e.g.
    /// `strategies/cartesian/distance_pythagoras.hpp:99-115`).
    fn distance(&self, a: &A, b: &B) -> Self::Out;

    /// Return the comparable-form companion of this strategy.
    ///
    /// Mirrors `services::get_comparable(strategy)` from
    /// `strategies/distance/services.hpp` — most concrete strategies
    /// implement it as a no-op constructor of the sibling
    /// `comparable::*` type.
    fn comparable(&self) -> Self::Comparable;
}

/// "Which distance strategy do we pick by default for a pair of
/// coordinate-system *families* `Self` (for the first geometry's
/// CS family) and `Other` (for the second)?"
///
/// Mirrors `boost::geometry::strategy::distance::services::
/// default_strategy<Tag1, Tag2, Point1, Point2, CsTag1, CsTag2>`
/// from `boost/geometry/strategies/distance/services.hpp`, together
/// with the per-CS specialisations in
/// `strategies/{cartesian,spherical,geographic}/distance.hpp` and
/// the umbrella `strategies/default_strategy.hpp`.
///
/// In Boost the dispatch keys on the *coordinate-system tag*
/// (`cs_tag<Point>::type`). The Rust analogue is the
/// [`CoordinateSystem::Family`] associated type defined in
/// `geometry-cs` — every concrete CS (`Cartesian`,
/// `Spherical<Degree>`, `Spherical<Radian>`, `Geographic<Degree>`,
/// …) reports one of the family marker types
/// (`CartesianFamily`, `SphericalFamily`, `GeographicFamily`,
/// `PolarFamily`).
///
/// Implementations land in `geometry-strategy` alongside each
/// concrete strategy:
///
/// ```ignore
/// impl DefaultDistance<CartesianFamily>  for CartesianFamily  { type Strategy = Pythagoras; }
/// impl DefaultDistance<SphericalFamily>  for SphericalFamily  { type Strategy = Haversine; }
/// impl DefaultDistance<GeographicFamily> for GeographicFamily { type Strategy = Andoyer; }
/// ```
///
/// The `Strategy: Default` bound matches Boost's expectation that
/// `services::default_strategy<...>::type` is default-constructible
/// (every `strategies/.../distance_*.hpp` concrete strategy has a
/// no-arg constructor).
pub trait DefaultDistance<Other> {
    /// The strategy chosen for `(Self, Other)`. Must implement
    /// `Default` because the free-function `distance(a, b)` builds
    /// it without arguments — same contract as Boost's
    /// default-constructed `services::default_strategy<...>::type`.
    type Strategy: Default;
}

/// Type alias resolving the default distance strategy for the pair
/// `(A, B)` by walking
/// `A -> A::Point -> Cs -> Family -> DefaultDistance<…B's Family…>::Strategy`.
///
/// Replaces the C++ free function
/// `boost::geometry::strategy::distance::services::
/// default_strategy<G1, G2>::type` from
/// `strategies/distance/services.hpp`, plumbed through the
/// per-CS files.
///
/// At call sites in `geometry-algorithm::distance`, this is the
/// type the strategy-less `distance(a, b)` overload monomorphises
/// against:
///
/// ```ignore
/// pub fn distance<A: Geometry, B: Geometry>(a: &A, b: &B)
///     -> <DefaultDistanceStrategy<A, B> as DistanceStrategy<A, B>>::Out
/// where
///     DefaultDistanceStrategy<A, B>: DistanceStrategy<A, B>,
/// {
///     DefaultDistanceStrategy::<A, B>::default().distance(a, b)
/// }
/// ```
pub type DefaultDistanceStrategy<A, B> =
    <<<<A as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family as DefaultDistance<
        <<<B as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family,
    >>::Strategy;

/// `Reversed<S>` lifts a `DistanceStrategy<A, B>` into a
/// `DistanceStrategy<B, A>` by calling the inner strategy with the
/// arguments swapped.
///
/// Replaces `boost::geometry::reverse_dispatch`
/// (`boost/geometry/core/reverse_dispatch.hpp`) together with the
/// partial specialisation that uses it in
/// `boost/geometry/algorithms/detail/distance/interface.hpp:53-77`.
/// Boost has one such specialisation *per algorithm*; in Rust the
/// symmetry is expressed once, at the strategy-trait layer, with a
/// single blanket impl below — every algorithm that goes through a
/// `DistanceStrategy<A, B>` automatically gets the swap.
///
/// # Cost
///
/// `#[repr(transparent)]` is deliberately *not* applied — `Reversed`
/// is a one-field tuple struct and monomorphisation collapses the
/// indirection. There is no runtime overhead: the wrapper exists
/// solely to give the trait system a *different* `Self` type to
/// dispatch on, the same way `boost::geometry::detail::distance::
/// reverse_dispatch_call` exists in C++ only to give the partial
/// specialisation something to bind to.
#[derive(Debug, Default, Clone, Copy)]
pub struct Reversed<S>(pub S);

impl<A, B, S> DistanceStrategy<B, A> for Reversed<S>
where
    A: Geometry,
    B: Geometry,
    S: DistanceStrategy<A, B>,
{
    type Out = S::Out;
    type Comparable = Reversed<S::Comparable>;

    #[inline]
    fn distance(&self, b: &B, a: &A) -> S::Out {
        self.0.distance(a, b)
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        Reversed(self.0.comparable())
    }
}

#[cfg(test)]
mod tests {
    //! Compile-time witnesses for the trait surface that the
    //! algorithm crate (T23) will rely on. No runtime assertions —
    //! a concrete strategy lands in T22.

    use super::{DefaultDistance, DefaultDistanceStrategy, DistanceStrategy, Reversed};
    use geometry_coords::CoordinateScalar;
    use geometry_cs::CoordinateSystem;
    use geometry_trait::{Geometry, Point};

    /// Witness: any type implementing `DistanceStrategy<A, B>` must
    /// expose `Out`, `Comparable`, `distance`, `comparable` with the
    /// exact bounds T23's `distance(a, b)` will depend on.
    fn _accepts_strategy<A, B, S>()
    where
        A: Geometry,
        B: Geometry,
        S: DistanceStrategy<A, B>,
        S::Out: CoordinateScalar,
        S::Comparable: DistanceStrategy<A, B, Out = S::Out>,
    {
    }

    /// Witness: `Reversed<S>` implements `DistanceStrategy<B, A>`
    /// for any `S: DistanceStrategy<A, B>`. Exercises the blanket
    /// impl above so the algorithm crate can rely on it without a
    /// concrete strategy in scope.
    fn _reversed_witness<A, B, S>()
    where
        A: Geometry,
        B: Geometry,
        S: DistanceStrategy<A, B>,
        Reversed<S>: DistanceStrategy<B, A, Out = S::Out>,
    {
    }

    /// Witness: a `Reversed<Reversed<S>>` is *not* spelled the same
    /// as `S` but does compute the same thing — the blanket impl
    /// composes. Useful because reverse-dispatch lookups from
    /// asymmetric-pair algorithms may layer two `Reversed`s in
    /// degenerate code paths.
    fn _double_reversed_witness<A, B, S>()
    where
        A: Geometry,
        B: Geometry,
        S: DistanceStrategy<A, B>,
        Reversed<Reversed<S>>: DistanceStrategy<A, B, Out = S::Out>,
    {
    }

    /// Witness: `DefaultDistanceStrategy<A, B>` resolves to the
    /// `Strategy` projection of `DefaultDistance` keyed on the two
    /// geometries' CS families, and that strategy implements
    /// `DistanceStrategy<A, B>`. This is the exact bound T23's
    /// strategy-less `distance(a, b)` will carry.
    fn _default_strategy_witness<A, B>()
    where
        A: Geometry,
        B: Geometry,
        <<A as Geometry>::Point as Point>::Cs: CoordinateSystem,
        <<B as Geometry>::Point as Point>::Cs: CoordinateSystem,
        <<<A as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family:
            DefaultDistance<<<<B as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family>,
        DefaultDistanceStrategy<A, B>: DistanceStrategy<A, B> + Default,
    {
    }

    use crate::cartesian::distance_pythagoras::{ComparablePythagoras, Pythagoras};
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type P = Point2D<f64, Cartesian>;

    /// Instantiating each type-level witness with a concrete strategy
    /// (`Pythagoras` over two cartesian points) proves the trait bounds
    /// they encode actually resolve, and executes their bodies.
    #[test]
    #[allow(clippy::used_underscore_items, reason = "the test exists to run the compile-time witness's body")]
    fn witnesses_resolve_for_pythagoras_over_cartesian_points() {
        _accepts_strategy::<P, P, Pythagoras>();
        _reversed_witness::<P, P, Pythagoras>();
        _double_reversed_witness::<P, P, Pythagoras>();
        _default_strategy_witness::<P, P>();
    }

    /// `Reversed<Pythagoras>` computes the same distance as `Pythagoras`
    /// with the arguments swapped, and its `comparable()` returns the
    /// wrapped comparable strategy (squared distance).
    #[test]
    #[allow(clippy::float_cmp, reason = "compared values are exact integer-valued literals")]
    fn reversed_swaps_arguments_and_forwards_comparable() {
        let a = P::new(0.0, 0.0);
        let b = P::new(3.0, 4.0);

        let forward = Pythagoras.distance(&a, &b);
        let reversed = Reversed(Pythagoras).distance(&b, &a);
        assert_eq!(forward, reversed);
        assert_eq!(forward, 5.0);

        // The comparable companion skips the sqrt: 3² + 4² = 25.
        let rev: Reversed<Pythagoras> = Reversed(Pythagoras);
        let cmp = DistanceStrategy::<P, P>::comparable(&rev);
        let cmp_val = cmp.distance(&b, &a);
        assert_eq!(cmp_val, ComparablePythagoras.distance(&a, &b));
        assert_eq!(cmp_val, 25.0);
    }
}
