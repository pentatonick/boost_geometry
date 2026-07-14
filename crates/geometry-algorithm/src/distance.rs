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
#[must_use]
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

/// Comparable distance using an explicitly supplied strategy.
///
/// Mirrors the strategy overload of `boost::geometry::comparable_distance`
/// from `boost/geometry/algorithms/detail/comparable_distance/interface.hpp:226-254`.
/// The supplied
/// strategy is converted through [`DistanceStrategy::comparable`] before the
/// distance is evaluated.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "distance strategies are zero-sized or small Copy values and match distance_with's public call shape"
)]
pub fn comparable_distance_with<A, B, S>(a: &A, b: &B, strategy: S) -> S::Out
where
    A: Geometry,
    B: Geometry,
    S: DistanceStrategy<A, B>,
{
    strategy.comparable().distance(a, b)
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

    /// The 1D squared-difference walk: |7 − 2| = 5, squared = 25.
    /// Exercises the `DIM == 1` dispatch arm of the Pythagoras kernel.
    #[test]
    fn one_dimensional_distance() {
        type P1 = geometry_model::Point<f64, 1, Cartesian>;
        let a = P1::new(2.0);
        let b = P1::new(7.0);
        assert!((distance(&a, &b) - 5.0).abs() < 1e-12);
        assert!((comparable_distance(&a, &b) - 25.0).abs() < 1e-12);
    }

    /// The 4D walk (`MAX_DIM`): unit steps on each of four axes give
    /// squared distance 4 and real distance 2. Exercises the `DIM == 4`
    /// dispatch arm.
    #[test]
    fn four_dimensional_distance() {
        use geometry_trait::PointMut as _;
        type P4 = geometry_model::Point<f64, 4, Cartesian>;
        let o = P4::default();
        let mut p = P4::default();
        p.set::<0>(1.0);
        p.set::<1>(1.0);
        p.set::<2>(1.0);
        p.set::<3>(1.0);
        assert!((comparable_distance(&o, &p) - 4.0).abs() < 1e-12);
        assert!((distance(&o, &p) - 2.0).abs() < 1e-12);
    }

    /// A 3D point-to-segment distance exercises the `DIM == 3` dispatch
    /// arms of the projected-point kernel. The point `(3, 4, 0)`
    /// projects onto the z-axis segment `(0,0,0)-(0,0,10)` at the start
    /// endpoint, so the distance is `sqrt(3² + 4²) = 5`.
    #[test]
    fn three_dimensional_point_to_segment() {
        use geometry_model::{Point3D, Segment};
        use geometry_strategy::PointToSegment;
        type P3 = Point3D<f64, Cartesian>;
        let p = P3::new(3.0, 4.0, 0.0);
        let s = Segment::new(P3::new(0.0, 0.0, 0.0), P3::new(0.0, 0.0, 10.0));
        let d = distance_with(&p, &s, PointToSegment::<Pythagoras>::default());
        assert!((d - 5.0).abs() < 1e-12, "got {d}");
    }

    /// The north pole reached from two distinct longitudes is one
    /// physical point (distance 0) for every geodesic strategy. The
    /// differing longitudes bypass the coordinate-equality
    /// short-circuit, so the in-formula degenerate guards report it.
    #[cfg(feature = "std")]
    #[test]
    fn geodesic_coincident_poles_at_different_longitudes_are_zero() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Geographic};
        use geometry_strategy::geographic::{Andoyer, Thomas, Vincenty};

        type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;
        let deg = |lon: f64, lat: f64| -> Gg { WithCs::new(Adapt([lon, lat])) };

        let a = deg(0.0, 90.0);
        let b = deg(180.0, 90.0);
        assert!(distance_with(&a, &b, Andoyer::WGS84).abs() < 1e-3);
        assert!(distance_with(&a, &b, Thomas::WGS84).abs() < 1e-6);
        assert!(distance_with(&a, &b, Vincenty::WGS84).abs() < 1e-3);
    }

    /// Thomas: a line whose *second* endpoint sits exactly on a pole
    /// exercises the `lat2 == ±π/2` reduced-latitude short-circuit.
    /// `(1, 80) → (0, 90)` ≈ 1116.825795 km.
    #[cfg(feature = "std")]
    #[test]
    fn thomas_second_endpoint_at_pole() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Geographic};
        use geometry_strategy::geographic::Thomas;

        type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;
        let deg = |lon: f64, lat: f64| -> Gg { WithCs::new(Adapt([lon, lat])) };

        let d = distance_with(&deg(1.0, 80.0), &deg(0.0, 90.0), Thomas::WGS84);
        assert!(
            (d / 1000.0 - 1_116.825_795).abs() < 0.012,
            "{} km",
            d / 1000.0
        );
    }

    /// Vincenty: a pair straddling the antimeridian normalises Δλ into
    /// `(-π, π]` in both directions; both describe the same 20°
    /// equatorial arc (≈ 2 226 km on WGS84).
    #[cfg(feature = "std")]
    #[test]
    fn vincenty_antimeridian_longitude_is_normalised_both_ways() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Geographic};
        use geometry_strategy::geographic::Vincenty;

        type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;
        let deg = |lon: f64, lat: f64| -> Gg { WithCs::new(Adapt([lon, lat])) };

        let east = distance_with(&deg(170.0, 0.0), &deg(-170.0, 0.0), Vincenty::WGS84);
        let west = distance_with(&deg(-170.0, 0.0), &deg(170.0, 0.0), Vincenty::WGS84);
        assert!((east - west).abs() < 1e-6, "{east} vs {west}");
        assert!(
            (east / 1000.0 - 2_226.0).abs() < 5.0,
            "{} km",
            east / 1000.0
        );
    }
}
