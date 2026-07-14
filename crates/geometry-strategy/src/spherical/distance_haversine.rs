//! Haversine great-circle distance for the Spherical family.
//!
//! Mirrors `boost/geometry/strategies/spherical/distance_haversine.hpp`:
//!
//! * lines 44-57 — the formula commentary in the C++ header:
//!   `d = 2·asin(sqrt(sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)))`,
//!   with the inner term `h` factored out so the "comparable" form can
//!   skip both the `sqrt` and the `asin` (both monotone-increasing on
//!   `[0, 1]`).
//! * lines 63-130 — `comparable::haversine<RadiusOrSphere, CalcType>`:
//!   returns only the inner `h` — the Rust port is
//!   [`ComparableHaversine`].
//! * lines 146-200 — `haversine<RadiusOrSphere, CalcType>`: the
//!   sqrt-and-asin-paying companion that multiplies through by the
//!   sphere radius — the Rust port is [`Haversine`].
//! * lines 222-260 — the `services::*` specialisations that wire the
//!   strategy into the default-strategy lookup; reproduced here as
//!   `impl DefaultDistance<SphericalFamily> for SphericalFamily`.
//!
//! # Convention
//!
//! Boost has two spherical tags — `spherical_polar_tag` (colatitude,
//! measured from the pole) and `spherical_equatorial_tag` (latitude,
//! measured from the equator). The Rust port collapses both onto a
//! single [`Spherical<U>`](geometry_cs::Spherical) following the
//! *equatorial* convention, which is what the quickstart and the
//! Boost haversine tests use. See proposal §8.
//!
//! # Calculation-type policy
//!
//! Boost runs the inputs through
//! `util::calculation_type::geometric::binary` to pick a working
//! scalar (`boost/geometry/util/calculation_type.hpp`). The v1 Rust
//! port follows the T40 spec's "for simplicity" branch and hardcodes
//! `Scalar = f64` on both inputs — this lets the kernel reach for
//! `f64::sin` / `cos` / `asin` directly (which require `std`) without
//! growing the [`CoordinateScalar`](geometry_coords::CoordinateScalar)
//! trait surface. Mixed-scalar support folds in alongside the
//! `Promote` lattice when a real caller appears.
//!
//! `#[cfg(feature = "std")]` gates the impls: the standard library
//! provides the trig functions as inherent methods on `f64`. A
//! `no_std` build of `geometry-strategy` (default-features off) does
//! not get Haversine; that is fine — the crate compiles, just without
//! this strategy. T42+ may add a `libm` fallback alongside the
//! geographic strategies.

use geometry_cs::{CoordinateSystem, SphericalFamily};
use geometry_tag::SameAs;
use geometry_trait::Point;

use crate::distance::{DefaultDistance, DistanceStrategy};

#[cfg(feature = "std")]
use crate::normalise::{HasAngularUnits, lonlat_radians};

/// Haversine great-circle distance, parameterised by sphere radius.
///
/// Inputs follow the [`Spherical<U>`](geometry_cs::Spherical)
/// equatorial convention — see its rustdoc.
///
/// Mirrors `boost::geometry::strategy::distance::haversine<T>` from
/// `strategies/spherical/distance_haversine.hpp:146-200`. The radius
/// is supplied at construction and the output is in the *same units
/// as the radius* (metres for [`Haversine::EARTH`], miles if the
/// caller multiplies a [`Haversine::UNIT`] result by a miles-radius,
/// and so on — see the quickstart at `doc/src/examples/quick_start.cpp:137-148`).
///
/// The associated [`DistanceStrategy::Comparable`] type is
/// [`ComparableHaversine`] — it returns only the inner `h` term of
/// the formula, skipping the `sqrt` *and* the `asin` (both monotone
/// on `[0, 1]`), mirroring
/// `boost::geometry::strategy::distance::comparable::haversine`
/// (`distance_haversine.hpp:63-130`).
#[derive(Debug, Clone, Copy)]
pub struct Haversine {
    /// Sphere radius. The result of [`Haversine::distance`] is in
    /// these units.
    pub radius: f64,
}

impl Haversine {
    /// Mean Earth radius in metres — Boost's
    /// `average_earth_radius = 6_372_795.0` from
    /// `test/strategies/haversine.cpp:29`. Used as the [`Default`].
    pub const EARTH: Self = Self {
        radius: 6_372_795.0,
    };

    /// Unit sphere (`radius = 1`): the result of
    /// [`Haversine::distance`] is then the *angular* distance in
    /// radians. Matches the pattern in
    /// `doc/src/examples/quick_start.cpp:137-148`, where the
    /// quickstart multiplies a unit-sphere angle by an Earth radius
    /// expressed in miles to get distance in miles.
    pub const UNIT: Self = Self { radius: 1.0 };
}

impl Default for Haversine {
    #[inline]
    fn default() -> Self {
        Self::EARTH
    }
}

/// Comparable form of [`Haversine`].
///
/// Inputs follow the [`Spherical<U>`](geometry_cs::Spherical)
/// equatorial convention — see its rustdoc.
///
/// Returns only the inner `h` term
/// of the haversine formula
/// (`h = sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)`), skipping
/// the `sqrt`, the `asin`, and the radius multiply. Ordering matches
/// [`Haversine`] because both `sqrt` and `asin` are monotone on
/// `[0, 1]`.
///
/// Mirrors `boost::geometry::strategy::distance::comparable::haversine`
/// from `strategies/spherical/distance_haversine.hpp:63-130`.
#[derive(Debug, Default, Clone, Copy)]
pub struct ComparableHaversine;

// ---- DistanceStrategy impls -----------------------------------------
//
// The `SameAs<SphericalFamily>` bounds on both points enforce the
// spherical-only rule. A caller wiring a Cartesian or Geographic
// point through here by mistake gets the
// `#[diagnostic::on_unimplemented]` plate on `geometry_tag::SameAs`
// (the same plate Pythagoras relies on) pointing them at
// `WithCs<_, Spherical<…>>` or at the geographic strategies; the
// extra `#[diagnostic::on_unimplemented]` plate below adds a
// Haversine-specific note for the most common confusion (geographic
// vs spherical).

/// Haversine on `f64` spherical points.
///
/// Mirrors the `apply(p1, p2)` member of
/// `boost::geometry::strategy::distance::haversine` at
/// `strategies/spherical/distance_haversine.hpp:188-199`:
///
/// ```text
/// d = 2 · R · asin(sqrt(h))
///   where h = sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)
/// ```
///
/// # Diagnostics on mis-paired CS
///
/// A caller who pairs a Cartesian or Geographic point with
/// [`Haversine`] hits the `<P::Cs as CoordinateSystem>::Family:
/// SameAs<SphericalFamily>` bound below and gets the redirect plate
/// on [`geometry_tag::SameAs`] pointing them at
/// `WithCs<_, Spherical<…>>` or at the geographic strategies
/// (Andoyer, Vincenty). See T31 and proposal §3.7.
#[cfg(feature = "std")]
impl<P1, P2> DistanceStrategy<P1, P2> for Haversine
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    P1::Cs: HasAngularUnits,
    P2::Cs: HasAngularUnits,
    <P1::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
{
    type Out = f64;
    type Comparable = ComparableHaversine;

    #[inline]
    fn distance(&self, a: &P1, b: &P2) -> Self::Out {
        let h = comparable_haversine_h::<P1, P2>(a, b);
        // d = 2 · R · asin(sqrt(h)) — mirrors
        // `distance_haversine.hpp:191-194`.
        2.0 * self.radius * h.sqrt().asin()
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        ComparableHaversine
    }
}

#[cfg(feature = "std")]
impl<P1, P2> DistanceStrategy<P1, P2> for ComparableHaversine
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    P1::Cs: HasAngularUnits,
    P2::Cs: HasAngularUnits,
    <P1::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
{
    type Out = f64;
    type Comparable = Self;

    #[inline]
    fn distance(&self, a: &P1, b: &P2) -> Self::Out {
        comparable_haversine_h::<P1, P2>(a, b)
    }

    #[inline]
    fn comparable(&self) -> Self::Comparable {
        *self
    }
}

// ---- Default Spherical × Spherical = Haversine ----------------------

/// Spherical × Spherical defaults to Haversine.
///
/// Mirrors the `services::default_strategy<point_tag, point_tag, P1,
/// P2, spherical_equatorial_tag, spherical_equatorial_tag>`
/// specialisation in `strategies/spherical/distance_haversine.hpp`
/// (around the `services::*` block at lines 222-260).
impl DefaultDistance<SphericalFamily> for SphericalFamily {
    type Strategy = Haversine;
}

// ---- Shared kernel --------------------------------------------------

/// Compute the inner `h` term of the haversine formula:
///
/// ```text
/// h = sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)
/// ```
///
/// Direct port of the body of
/// `comparable::haversine::apply` at
/// `strategies/spherical/distance_haversine.hpp:111-119`. Both
/// inputs are first normalised to radians via [`lonlat_radians`], so
/// the kernel does not care whether the source CS carries `Degree`
/// or `Radian` units — that mirrors Boost's implicit
/// `math::d2r<T>()` multiplication at the entry of each spherical
/// strategy.
#[cfg(feature = "std")]
#[inline]
fn comparable_haversine_h<P1, P2>(a: &P1, b: &P2) -> f64
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    P1::Cs: HasAngularUnits,
    P2::Cs: HasAngularUnits,
{
    let (lon1, lat1) = lonlat_radians(a);
    let (lon2, lat2) = lonlat_radians(b);

    let dlat_half = (lat2 - lat1) * 0.5;
    let dlon_half = (lon2 - lon1) * 0.5;

    let s_lat = dlat_half.sin();
    let s_lon = dlon_half.sin();

    s_lat * s_lat + lat1.cos() * lat2.cos() * s_lon * s_lon
}

// ---- Tests ----------------------------------------------------------

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference values come from
    //! `geometry/test/strategies/haversine.cpp` (lines 29 and 60-76)
    //! and `geometry/doc/src/examples/quick_start.cpp:137-148`.

    use super::{ComparableHaversine, Haversine};
    use crate::distance::DistanceStrategy;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Spherical};

    #[inline]
    fn deg(lon: f64, lat: f64) -> WithCs<Adapt<[f64; 2]>, Spherical<Degree>> {
        WithCs::new(Adapt([lon, lat]))
    }

    /// `test/strategies/haversine.cpp:66-67` — Amsterdam → Paris on
    /// the Boost average earth, expected `467_270.4 m` within `1 m`.
    #[test]
    fn amsterdam_paris_average_earth() {
        let p1 = deg(4.0, 52.0);
        let p2 = deg(2.0, 48.0);
        let d = Haversine::EARTH.distance(&p1, &p2);
        assert!((d - 467_270.4).abs() < 1.0, "got {d} expected ~ 467270.4");
    }

    /// `test/strategies/haversine.cpp:67-68` — symmetric in arguments.
    #[test]
    fn symmetric_in_arguments() {
        let p1 = deg(4.0, 52.0);
        let p2 = deg(2.0, 48.0);
        let ab = Haversine::EARTH.distance(&p1, &p2);
        let ba = Haversine::EARTH.distance(&p2, &p1);
        assert!((ab - ba).abs() < 1e-6);
    }

    /// `test/strategies/haversine.cpp:69` — same arc on a unit
    /// sphere returns the angular distance (radians).
    #[test]
    fn unit_sphere_returns_radians() {
        let p1 = deg(4.0, 52.0);
        let p2 = deg(2.0, 48.0);
        let angle = Haversine::UNIT.distance(&p1, &p2);
        // metres / earth radius == radians
        let expected = 467_270.4 / 6_372_795.0;
        assert!((angle - expected).abs() < 1e-6);
    }

    /// `test/strategies/haversine.cpp:72-75` — Amsterdam → Barcelona,
    /// `1_232_906.5 m` within `1 m`.
    #[test]
    fn amsterdam_barcelona() {
        let p1 = deg(4.0, 52.0);
        let p2 = deg(2.0, 41.0);
        let d = Haversine::EARTH.distance(&p1, &p2);
        assert!(
            (d - 1_232_906.5).abs() < 1.0,
            "got {d} expected ~ 1232906.5"
        );
    }

    /// Comparable form preserves ordering vs. the real distance —
    /// the whole point of skipping `sqrt` and `asin`.
    #[test]
    fn comparable_orders_match_distance() {
        let o = deg(0.0, 0.0);
        let near = deg(1.0, 0.0);
        let far = deg(45.0, 0.0);
        let h_near = ComparableHaversine.distance(&o, &near);
        let h_far = ComparableHaversine.distance(&o, &far);
        assert!(h_near < h_far);

        let d_near = Haversine::EARTH.distance(&o, &near);
        let d_far = Haversine::EARTH.distance(&o, &far);
        assert!(d_near < d_far);
    }

    /// `doc/src/examples/quick_start.cpp:137-148` — "Distance in
    /// miles: 267.02" using the unit-sphere shortcut multiplied by
    /// the Earth's radius in miles.
    #[test]
    fn quickstart_amsterdam_paris_in_miles() {
        let ams = deg(4.90, 52.37);
        // doc says 48.86 (not 48.85)
        let par = deg(2.35, 48.86);
        let angle = Haversine::UNIT.distance(&ams, &par);
        let miles = angle * 3959.0;
        assert!(
            (miles - 267.02).abs() < 0.05,
            "got {miles} expected ~ 267.02"
        );
    }

    // KC1.T2 witness: proves this strategy accepts a read-only `Point`
    // (one that need not implement `PointMut`). If it compiles, the
    // read-only bound is locked.
    fn _accepts_readonly_point<P, S>(s: &S, a: &P, b: &P) -> S::Out
    where
        P: geometry_trait::Point,
        S: DistanceStrategy<P, P>,
    {
        s.distance(a, b)
    }

    type GP = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;

    /// `Haversine::comparable()` yields a `ComparableHaversine` that
    /// returns the sqrt-free `h` term; taking `asin(sqrt(h)) · 2R` of it
    /// recovers the full Haversine distance.
    #[test]
    fn comparable_yields_the_h_term() {
        let a = deg(4.0, 52.0);
        let b = deg(2.0, 48.0);
        let full = Haversine::UNIT.distance(&a, &b);
        let h = DistanceStrategy::<GP, GP>::comparable(&Haversine::UNIT).distance(&a, &b);
        // Reconstruct the angle from the comparable term.
        let reconstructed = 2.0 * h.sqrt().asin();
        assert!((full - reconstructed).abs() < 1e-12, "{full} vs {reconstructed}");
    }

    /// `ComparableHaversine::comparable()` returns itself, and its
    /// `distance` preserves the ordering of the full distance (the whole
    /// point of the comparable form): a nearer pair has a smaller `h`.
    #[test]
    fn comparable_of_comparable_is_itself_and_order_preserving() {
        let o = deg(0.0, 0.0);
        let near = deg(1.0, 0.0);
        let far = deg(10.0, 0.0);
        let cmp = DistanceStrategy::<GP, GP>::comparable(&ComparableHaversine);
        assert!(cmp.distance(&o, &near) < cmp.distance(&o, &far));
    }

    /// The read-only-point witness computes a distance when invoked.
    #[test]
    #[allow(clippy::used_underscore_items, reason = "the test exists to run the compile-time witness's body")]
    fn readonly_witness_computes_distance() {
        let d = _accepts_readonly_point(&Haversine::UNIT, &deg(0.0, 0.0), &deg(1.0, 0.0));
        assert!(d > 0.0, "got {d}");
    }
}
