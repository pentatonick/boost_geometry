//! Type equality expressible as a trait bound.
//!
//! Strategy impls in later crates need to constrain *"both inputs share a
//! coordinate-system family"*, which is `std::is_same<X, Y>` in C++
//! (`boost/geometry/core/cs.hpp` and the strategy `services` headers use
//! this pattern extensively). Rust's generics do not let you write `where
//! X = Y` directly, so we encode the same idea as `where X: SameAs<Y>`. The
//! `sealed` super-trait ensures only the reflexive impl can exist
//! downstream.

mod sealed {
    /// Sealed super-trait — only this crate may implement it, which is
    /// what makes `SameAs<T>` actually mean equality.
    pub trait Sealed<T> {}
}

/// `T: SameAs<U>` holds iff `T` and `U` are the same type.
///
/// Conceptually the trait-bound form of `std::is_same<X, Y>`, used by
/// strategy bounds to require that two generic parameters share a
/// coordinate-system family. Compare with the equality checks performed
/// inside `boost/geometry/strategies/distance/services.hpp` when selecting
/// a default strategy.
///
/// # Compile-time diagnostic
///
/// In practice the only way a downstream caller fails this bound is by
/// pairing a non-Cartesian point with a Cartesian-only strategy like
/// `Pythagoras` (the silent-Cartesian trap from proposal §8). The
/// `#[diagnostic::on_unimplemented]` plate below redirects that error
/// to the matching mitigation — see
/// [`WithCs`](../../geometry_adapt/struct.WithCs.html) and T31.
///
/// # Examples
///
/// ```
/// use geometry_tag::SameAs;
/// fn require_same<T: SameAs<U>, U>() {}
/// require_same::<u32, u32>();   // T == U: holds
/// // require_same::<u32, i32>(); // T != U: would fail to compile
/// ```
#[diagnostic::on_unimplemented(
    message = "coordinate-system family mismatch: `{Self}` is not the same as `{T}`",
    label = "this point's coordinate-system family is `{Self}`, but the strategy requires `{T}`",
    note = "if `{Self}` is `GeographicFamily` or `SphericalFamily`, you probably picked a \
            Cartesian-only strategy (e.g. `Pythagoras`) by accident — wrap your point in \
            `geometry_adapt::WithCs<_, Geographic<Degree>>` / `WithCs<_, Spherical<Degree>>` \
            or pick a CS-appropriate strategy (`Haversine`, `Andoyer`, `Vincenty`)",
    note = "see the silent-Cartesian discussion in the rust-port proposal §3.7 and §8"
)]
pub trait SameAs<T>: sealed::Sealed<T> {}

impl<T> sealed::Sealed<T> for T {}
impl<T> SameAs<T> for T {}
