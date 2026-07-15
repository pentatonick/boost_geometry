//! `length(&g)` and `perimeter(&p)` — see
//! `boost/geometry/algorithms/{length,perimeter}.hpp`.
//!
//! `length` dispatches to the per-CS-family default strategy via
//! [`geometry_strategy::DefaultLength`] — Cartesian linestrings resolve
//! to [`geometry_strategy::CartesianLength`], spherical to
//! [`geometry_strategy::SphericalLength`], geographic to
//! [`geometry_strategy::GeographicLength`]. `perimeter` and
//! `ring_perimeter` use the parallel
//! [`geometry_strategy::DefaultPerimeter`] family dispatch.

use geometry_cs::CoordinateSystem;
use geometry_strategy::{
    DefaultLength, DefaultLengthStrategy, DefaultPerimeter, DefaultPerimeterStrategy,
    LengthStrategy,
};
use geometry_trait::{Geometry, Linestring, Point, Polygon, Ring};

/// Shorthand for the CS family of `G`'s point type. Keeps the `where`
/// clauses readable; matches the projection in
/// [`geometry_strategy::DefaultLengthStrategy`].
type Family<G> = <<<G as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family;

/// Sum the per-segment distance along a linestring using the default
/// strategy for its coordinate-system family.
///
/// Mirrors `boost::geometry::length` from
/// `boost/geometry/algorithms/length.hpp`. The strategy is resolved
/// via [`geometry_strategy::DefaultLengthStrategy`] — Cartesian input
/// resolves to the Pythagorean [`geometry_strategy::CartesianLength`]
/// (identical to the v1 behaviour), spherical to Haversine, geographic
/// to Andoyer. For an explicit strategy use [`length_with`].
///
/// # Behaviour on "wrong" kinds
///
/// This *static* entry point deliberately compile-errors on a `Point`
/// or `Polygon` argument
/// (Boost returns 0 at runtime; when the kind is known
/// at compile time a type error is a clearer signal). The runtime
/// contract — Boost's "length of a non-linear kind is 0"
/// (`length.hpp:75-80`) — is honoured on the *dynamic* path instead:
/// [`crate::length_dyn`] returns `0` for `Point`,
/// `Polygon`, `MultiPoint`, ….
#[inline]
#[must_use]
pub fn length<G>(g: &G) -> <DefaultLengthStrategy<G> as LengthStrategy<G>>::Out
where
    G: Linestring,
    Family<G>: DefaultLength<Family<G>>,
    DefaultLengthStrategy<G>: LengthStrategy<G> + Default,
{
    DefaultLengthStrategy::<G>::default().length(g)
}

/// Length of a linestring using an explicitly supplied strategy.
///
/// Mirrors the `length(g, strategy)` overload at
/// `boost/geometry/algorithms/length.hpp`. Taking the strategy by
/// value matches the by-value call shape of
/// [`crate::distance_with`]; concrete strategies are zero-sized or
/// small `Copy` configuration objects, so this monomorphises into
/// nothing.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "Strategies are ZST/small Copy configuration objects; by-value matches the user-facing call shape."
)]
pub fn length_with<G, S>(g: &G, s: S) -> S::Out
where
    G: Geometry,
    S: LengthStrategy<G>,
{
    s.length(g)
}

/// Perimeter of a polygon: length of the exterior ring plus the sum
/// of the lengths of the interior rings.
///
/// Mirrors `boost::geometry::perimeter` from
/// `boost/geometry/algorithms/perimeter.hpp:230-236`.
#[inline]
#[must_use]
pub fn perimeter<P>(p: &P) -> <DefaultPerimeterStrategy<P> as LengthStrategy<P::Ring>>::Out
where
    P: Polygon,
    Family<P>: DefaultPerimeter<Family<P>>,
    DefaultPerimeterStrategy<P>: LengthStrategy<P::Ring> + Default,
{
    perimeter_with(p, DefaultPerimeterStrategy::<P>::default())
}

/// Perimeter of a polygon using an explicitly supplied ring-length strategy.
///
/// Mirrors the strategy overload of `boost::geometry::perimeter` from
/// `boost/geometry/algorithms/perimeter.hpp:252-258`.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "perimeter strategies are zero-sized or small Copy values, matching length_with"
)]
pub fn perimeter_with<P, S>(p: &P, strategy: S) -> S::Out
where
    P: Polygon,
    S: LengthStrategy<P::Ring>,
{
    let mut total = strategy.length(p.exterior());
    for inner in p.interiors() {
        total = total + strategy.length(inner);
    }
    total
}

/// Standalone helper to compute the perimeter of a `Ring` directly,
/// without wrapping it in a polygon. Mirrors Boost's
/// `perimeter(ring)` overload which dispatches via the
/// `ring_tag` arm of `algorithms/perimeter.hpp:66-73`.
#[inline]
#[must_use]
pub fn ring_perimeter<R>(r: &R) -> <DefaultPerimeterStrategy<R> as LengthStrategy<R>>::Out
where
    R: Ring,
    Family<R>: DefaultPerimeter<Family<R>>,
    DefaultPerimeterStrategy<R>: LengthStrategy<R> + Default,
{
    ring_perimeter_with(r, DefaultPerimeterStrategy::<R>::default())
}

/// Perimeter of a ring using an explicitly supplied strategy.
///
/// Mirrors the strategy overload of the ring-tag arm in
/// `boost/geometry/algorithms/perimeter.hpp:252-258`.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "perimeter strategies are zero-sized or small Copy values, matching length_with"
)]
pub fn ring_perimeter_with<R, S>(ring: &R, strategy: S) -> S::Out
where
    R: Ring,
    S: LengthStrategy<R>,
{
    strategy.length(ring)
}

#[cfg(test)]
mod tests {
    //! Reference values from `geometry/test/algorithms/length/length.cpp`
    //! (lines 24-33) and the classic 4x3 rectangle perimeter case.
    //! Spherical / geographic dispatch cross-checks the free-function
    //! result against a direct Haversine / Andoyer call
    //! (`length_sph.cpp` / `length_geo.cpp`).
    #![allow(
        clippy::float_cmp,
        reason = "lengths are compared with an explicit absolute tolerance, not `==`"
    )]

    use super::{length, length_with, perimeter, ring_perimeter};
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, Ring, linestring, polygon};

    #[test]
    fn length_of_3_4_5_segment() {
        let ls: Linestring<Point2D<f64, Cartesian>> = linestring![(0.0, 0.0), (3.0, 4.0)];
        let got = length(&ls);
        assert!((got - 5.0).abs() < 1e-12);
    }

    #[test]
    fn length_of_three_point_polyline() {
        let ls: Linestring<Point2D<f64, Cartesian>> =
            linestring![(0.0, 0.0), (3.0, 4.0), (4.0, 3.0)];
        let got = length(&ls);
        let expected = 5.0 + 2.0_f64.sqrt();
        assert!((got - expected).abs() < 1e-12);
    }

    #[test]
    fn perimeter_of_4_3_rectangle() {
        let p: geometry_model::Polygon<Point2D<f64, Cartesian>> =
            polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)]];
        let got = perimeter(&p);
        assert!((got - 14.0).abs() < 1e-12);
    }

    #[test]
    fn ring_perimeter_of_4_3_rectangle() {
        let mut r = Ring::<Point2D<f64, Cartesian>>::new();
        r.push(Point2D::new(0.0, 0.0));
        r.push(Point2D::new(4.0, 0.0));
        r.push(Point2D::new(4.0, 3.0));
        r.push(Point2D::new(0.0, 3.0));
        r.push(Point2D::new(0.0, 0.0));
        let got = ring_perimeter(&r);
        assert!((got - 14.0).abs() < 1e-12);
    }

    /// `length_sph.cpp` — strategy-less `length` on a spherical
    /// linestring resolves to Haversine. Uses `haversine.cpp:66-67`'s
    /// headline `(4, 52) → (2, 48) ≈ 467_270.4 m` reference, and
    /// cross-checks against a direct Haversine call.
    #[cfg(feature = "std")]
    #[test]
    fn spherical_length_dispatches_to_haversine() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Spherical};
        use geometry_strategy::{DistanceStrategy, spherical::Haversine};

        type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;
        let deg = |lon: f64, lat: f64| -> Sp { WithCs::new(Adapt([lon, lat])) };

        let ls: Linestring<Sp> = Linestring(vec![deg(4.0, 52.0), deg(2.0, 48.0)]);
        let got = length(&ls);
        let direct = Haversine::EARTH.distance(&deg(4.0, 52.0), &deg(2.0, 48.0));
        assert!((got - direct).abs() < 1e-6);
        // `haversine.cpp:66-67` — Amsterdam → Paris ≈ 467_270.4 m.
        assert!((got - 467_270.4).abs() < 1.0, "got {got} m");
    }

    /// `length_geo.cpp` — strategy-less `length` on a geographic
    /// linestring resolves to Andoyer; `(4, 52) → (3, 40)` on WGS84
    /// ≈ 1336.040 km (`andoyer.cpp:230-231`).
    #[cfg(feature = "std")]
    #[test]
    fn geographic_length_dispatches_to_andoyer() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Geographic};
        use geometry_strategy::{DistanceStrategy, geographic::Andoyer};

        type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;
        let deg = |lon: f64, lat: f64| -> Gg { WithCs::new(Adapt([lon, lat])) };

        let ls: Linestring<Gg> = Linestring(vec![deg(4.0, 52.0), deg(3.0, 40.0)]);
        let got = length(&ls);
        let direct = Andoyer::WGS84.distance(&deg(4.0, 52.0), &deg(3.0, 40.0));
        assert!((got - direct).abs() < 1e-6);
        assert!((got / 1000.0 - 1_336.039_890).abs() < 0.01);
    }

    /// `length_with` with an explicit spherical strategy reaches the
    /// same value as the default dispatch.
    #[cfg(feature = "std")]
    #[test]
    fn length_with_explicit_spherical_strategy() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Spherical};
        use geometry_strategy::SphericalLength;

        type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;
        let deg = |lon: f64, lat: f64| -> Sp { WithCs::new(Adapt([lon, lat])) };

        let ls: Linestring<Sp> = Linestring(vec![deg(0., 0.), deg(0., 10.)]);
        let via_with = length_with(&ls, SphericalLength::default());
        let via_default = length(&ls);
        assert!((via_with - via_default).abs() < 1e-9);
    }

    /// `length.hpp:89` — an empty or single-point linestring has length
    /// 0 (Boost's default-constructed sum), and so does a degenerate
    /// open ring's perimeter.
    #[test]
    fn degenerate_inputs_have_zero_length() {
        let empty: Linestring<Point2D<f64, Cartesian>> = linestring![];
        assert_eq!(length(&empty), 0.0);
        let single: Linestring<Point2D<f64, Cartesian>> = linestring![(3.0, 4.0)];
        assert_eq!(length(&single), 0.0);
        let empty_ring = Ring::<Point2D<f64, Cartesian>, true, false>::new();
        assert_eq!(ring_perimeter(&empty_ring), 0.0);
    }
}
