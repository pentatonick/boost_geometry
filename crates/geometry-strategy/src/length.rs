//! Strategy for summing the length of a sequence of points.
//!
//! Mirrors three pieces of Boost.Geometry that collaborate to make
//! `boost::geometry::length(g)` and `boost::geometry::perimeter(g)`
//! work for any linestring / ring / polygon in any coordinate system:
//!
//! * `boost/geometry/strategies/length/services.hpp` — the
//!   `services::default_strategy<G>` metafunction that picks the
//!   per-CS length strategy.
//! * `boost/geometry/strategies/length/cartesian.hpp` —
//!   `strategies::length::cartesian<>` plus its
//!   `services::default_strategy<Geometry, cartesian_tag>`
//!   specialisation; the Cartesian implementation hands a
//!   `strategy::distance::pythagoras<>` to the algorithm.
//! * `boost/geometry/algorithms/length.hpp:80-107` —
//!   `detail::length::range_length` walks the iterator pair and sums
//!   the per-segment distances; this file performs the same walk in
//!   Rust against the [`Linestring`] / [`Ring`] traits.
//!
//! T33 lands the Cartesian implementation only — Boost's
//! Spherical/Geographic length strategies arrive alongside the
//! Haversine / Andoyer / Vincenty distance strategies in later
//! tasks (T40+).

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem, GeographicFamily, SphericalFamily};
use geometry_tag::SameAs;
use geometry_trait::{Closure, Geometry, Linestring, Point, Ring};

use crate::cartesian::Pythagoras;
use crate::distance::DistanceStrategy;

/// A strategy for computing the length of a sequence of points.
///
/// Mirrors the per-CS length-strategy concept declared in
/// `boost/geometry/strategies/length/services.hpp` and refined per
/// coordinate system in `strategies/length/{cartesian,spherical,
/// geographic}.hpp`. The Boost concept exposes a `distance(p1, p2)`
/// helper that hands the algorithm a point-to-point distance kernel;
/// the Rust analogue collapses the two layers (strategy + algorithm
/// walk) into a single method [`LengthStrategy::length`] keyed on the
/// geometry type, because the walk shape is identical for every CS —
/// only the inner distance kernel changes.
///
/// # Associated items
///
/// * [`Self::Out`] — the scalar the length comes back as.
///   Equivalent to Boost's `default_length_result<Geometry>::type`
///   (`strategies/default_length_result.hpp`); typically the
///   coordinate scalar of `G`'s point type.
pub trait LengthStrategy<G: Geometry> {
    /// The output scalar type. Typically the geometry's coordinate
    /// scalar. Mirrors `default_length_result<G>::type` from
    /// `strategies/default_length_result.hpp`.
    type Out: CoordinateScalar;

    /// Sum the per-segment distances along `g`.
    ///
    /// Mirrors `detail::length::range_length::apply` from
    /// `algorithms/length.hpp:80-107` together with the CS-specific
    /// `strategies::length::*::distance` call at
    /// `strategies/length/cartesian.hpp:33-39`.
    fn length(&self, g: &G) -> Self::Out;
}

/// Cartesian length: sum of Pythagorean distances between
/// consecutive points (linestring case).
///
/// Mirrors `boost::geometry::strategies::length::cartesian<>` from
/// `strategies/length/cartesian.hpp:29-39`. The strategy carries no
/// state — `cartesian<>::distance(p1, p2)` returns a fresh
/// `strategy::distance::pythagoras<>` each call, which on the Rust
/// side is the unit-struct [`Pythagoras`] used directly below.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianLength;

/// Cartesian perimeter: sum of Pythagorean distances between
/// consecutive points of a ring, plus the closing edge when the ring
/// is open.
///
/// Separate from [`CartesianLength`] because Rust's coherence rules
/// cannot prove that a single type does not implement both
/// [`Linestring`] and [`Ring`]; splitting the strategy keeps the
/// per-tag dispatch disjoint at the impl level.
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianPerimeter;

// ---- Linestring ------------------------------------------------------
//
// Mirrors the `dispatch::length<Geometry, linestring_tag>` arm at
// `algorithms/length.hpp:132-135`, which inherits from
// `detail::length::range_length<Geometry, closed>`. A linestring is
// already "closed" in the range_length sense: the closing edge from
// last to first is *not* added — that is the perimeter case (rings).

impl<L> LengthStrategy<L> for CartesianLength
where
    L: Linestring,
    <L::Point as Point>::Cs: CoordinateSystem,
    <<L::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = <L::Point as Point>::Scalar;

    #[inline]
    fn length(&self, g: &L) -> Self::Out {
        sum_pairwise::<L::Point, _>(g.points())
    }
}

// ---- Ring ------------------------------------------------------------
//
// Mirrors the `dispatch::perimeter<Geometry, ring_tag>` arm at
// `algorithms/perimeter.hpp:66-73`, which inherits from
// `detail::length::range_length<Geometry, closure<Geometry>::value>`.
// For a closed ring the closing edge is already encoded as the
// repeated last point; for an open ring we add the explicit
// last->first segment. Boost achieves the same via
// `views::closeable_view` at `algorithms/length.hpp:90`.

impl<R> LengthStrategy<R> for CartesianPerimeter
where
    R: Ring,
    <R::Point as Point>::Cs: CoordinateSystem,
    <<R::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = <R::Point as Point>::Scalar;

    #[inline]
    fn length(&self, g: &R) -> Self::Out {
        let mut total = sum_pairwise::<R::Point, _>(g.points());
        if matches!(g.closure(), Closure::Open) {
            // An open ring leaves the closing edge implicit — add it
            // explicitly. Mirrors the `closeable_view` wrap at
            // `algorithms/length.hpp:90`.
            let mut it = g.points();
            if let Some(first) = it.next() {
                if let Some(last) = g.points().last() {
                    total = total + Pythagoras.distance(last, first);
                }
            }
        }
        total
    }
}

/// Walk `it` summing Pythagorean distances between consecutive
/// points. Returns `P::Scalar::ZERO` for an empty or single-point
/// range — same as Boost's `range_length` whose initial sum is
/// default-constructed (`return_type sum = return_type();`,
/// `algorithms/length.hpp:89`).
#[inline]
fn sum_pairwise<'a, P, I>(it: I) -> P::Scalar
where
    P: Point + 'a,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    I: IntoIterator<Item = &'a P>,
    I::IntoIter: Clone,
{
    let it = it.into_iter();
    let next = it.clone().skip(1);
    it.zip(next).fold(P::Scalar::ZERO, |acc, (a, b)| {
        acc + Pythagoras.distance(a, b)
    })
}

// Zero-on-mismatched-kind (Boost `length.hpp:75-80`: length of a
// point / polygon / multi-point is 0) is NOT expressed as a static
// `LengthStrategy` impl here: `CartesianLength` carries a blanket
// `impl<L: Linestring>` (so downstream `register_linestring!` types
// work), and Rust coherence forbids adding a disjoint concrete-type
// impl alongside a blanket one — the compiler cannot prove a foreign
// `Point`/`Polygon` will never also implement `Linestring`. The
// zero-length contract therefore lives on the *dynamic* path only:
// `geometry_algorithm::length_dyn` returns 0 for the non-linear arms
// (KC4.T1). The static `length<G: Linestring>` keeps v1's
// compile-error stance for non-linear kinds, which is a clearer signal
// when the kind is known at compile time.

// ---- Default length strategy per CS family --------------------------

/// "Which length strategy do we pick by default for this CS family?"
///
/// Mirrors v1's [`DefaultDistance`](crate::distance::DefaultDistance)
/// for the length algorithm — the Rust analogue of Boost's
/// `services::default_strategy<Geometry, cs_tag>` in
/// `strategies/length/services.hpp`, specialised per CS in
/// `strategies/length/{cartesian,spherical,geographic}.hpp`.
///
/// Length is unary, so — unlike `DefaultDistance` — there is exactly
/// one family type parameter (there is no second geometry). Each
/// family reports its default length strategy:
///
/// ```ignore
/// impl DefaultLength<CartesianFamily>  for CartesianFamily  { type Strategy = CartesianLength;  }
/// impl DefaultLength<SphericalFamily>  for SphericalFamily  { type Strategy = SphericalLength;  }
/// impl DefaultLength<GeographicFamily> for GeographicFamily { type Strategy = GeographicLength; }
/// ```
///
/// The `Strategy: Default` bound matches Boost's expectation that
/// `services::default_strategy<...>::type` is default-constructible.
pub trait DefaultLength<Family> {
    /// The length strategy chosen for this family. Must implement
    /// [`Default`] because the free-function `length(g)` builds it
    /// without arguments.
    type Strategy: Default;
}

/// Cartesian family defaults to [`CartesianLength`].
///
/// Mirrors the `services::default_strategy<Geometry, cartesian_tag>`
/// specialisation in `strategies/length/cartesian.hpp`.
impl DefaultLength<CartesianFamily> for CartesianFamily {
    type Strategy = CartesianLength;
}

/// Spherical family defaults to [`SphericalLength`](crate::spherical::SphericalLength).
///
/// Mirrors the `services::default_strategy<Geometry, spherical_tag>`
/// specialisation in `strategies/length/spherical.hpp`.
impl DefaultLength<SphericalFamily> for SphericalFamily {
    type Strategy = crate::spherical::SphericalLength;
}

/// Geographic family defaults to [`GeographicLength`](crate::geographic::GeographicLength).
///
/// Mirrors the `services::default_strategy<Geometry, geographic_tag>`
/// specialisation in `strategies/length/geographic.hpp`.
impl DefaultLength<GeographicFamily> for GeographicFamily {
    type Strategy = crate::geographic::GeographicLength;
}

/// Type alias resolving the default length strategy for geometry `G`
/// by walking `G -> G::Point -> Cs -> Family -> DefaultLength::Strategy`.
///
/// Mirrors [`DefaultDistanceStrategy`](crate::distance::DefaultDistanceStrategy)
/// for the length algorithm; the free-function `length(g)`
/// monomorphises against this at the call site.
pub type DefaultLengthStrategy<G> =
    <<<<G as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family as DefaultLength<
        <<<G as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family,
    >>::Strategy;

#[cfg(test)]
mod tests {
    //! Reference values from `geometry/test/algorithms/length/length.cpp`
    //! (lines 24-33) and the rectangle perimeter case from
    //! `algorithms/perimeter.cpp` (the classic 4x3 unit-rectangle
    //! example). Each test cites the source it mirrors.

    use super::{CartesianLength, CartesianPerimeter, LengthStrategy};
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, Ring, linestring};

    /// `length.cpp:24` — 3-4-5 segment as a two-point linestring.
    #[test]
    fn linestring_3_4_5() {
        let ls: Linestring<Point2D<f64, Cartesian>> = linestring![(0.0, 0.0), (3.0, 4.0)];
        let got = CartesianLength.length(&ls);
        assert!((got - 5.0).abs() < 1e-12);
    }

    /// `length.cpp:27` — three-point polyline, 5 + sqrt(2).
    #[test]
    fn linestring_5_plus_sqrt2() {
        let ls: Linestring<Point2D<f64, Cartesian>> =
            linestring![(0.0, 0.0), (3.0, 4.0), (4.0, 3.0)];
        let got = CartesianLength.length(&ls);
        let expected = 5.0 + 2.0_f64.sqrt();
        assert!((got - expected).abs() < 1e-12);
    }

    /// Closed ring around a 4x3 rectangle: perimeter is 14.
    #[test]
    fn closed_ring_4_3_rectangle() {
        let mut r = Ring::<Point2D<f64, Cartesian>>::new();
        r.push(Point2D::new(0.0, 0.0));
        r.push(Point2D::new(4.0, 0.0));
        r.push(Point2D::new(4.0, 3.0));
        r.push(Point2D::new(0.0, 3.0));
        r.push(Point2D::new(0.0, 0.0));
        let got = CartesianPerimeter.length(&r);
        assert!((got - 14.0).abs() < 1e-12);
    }

    /// Open ring (no repeated closing vertex) around the same
    /// rectangle: the strategy must add the implicit last->first edge.
    #[test]
    fn open_ring_4_3_rectangle() {
        let mut r = Ring::<Point2D<f64, Cartesian>, true, false>::new();
        r.push(Point2D::new(0.0, 0.0));
        r.push(Point2D::new(4.0, 0.0));
        r.push(Point2D::new(4.0, 3.0));
        r.push(Point2D::new(0.0, 3.0));
        let got = CartesianPerimeter.length(&r);
        assert!((got - 14.0).abs() < 1e-12);
    }

    // KC1.T2 witness: proves this strategy accepts a geometry whose
    // `Point` is read-only (need not implement `PointMut`). If it
    // compiles, the read-only bound is locked.
    fn _accepts_readonly_point<G, S>(s: &S, g: &G) -> S::Out
    where
        G: geometry_trait::Geometry,
        <G as geometry_trait::Geometry>::Point: geometry_trait::Point,
        S: LengthStrategy<G>,
    {
        s.length(g)
    }

    /// Invoke the read-only-point witness so its body is exercised too
    /// (the compile-time guarantee is unchanged; this just runs it).
    #[test]
    #[allow(clippy::used_underscore_items, reason = "the test exists to run the compile-time witness's body")]
    fn readonly_point_witness_computes_length() {
        let ls: Linestring<Point2D<f64, Cartesian>> = linestring![(0.0, 0.0), (3.0, 4.0)];
        let got = _accepts_readonly_point(&CartesianLength, &ls);
        assert!((got - 5.0).abs() < 1e-12);
    }
}
