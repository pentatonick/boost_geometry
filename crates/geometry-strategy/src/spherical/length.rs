//! Spherical great-circle length / perimeter.
//!
//! Mirrors the per-CS dispatch arm of
//! `boost/geometry/algorithms/length.hpp` reached when the geometry's
//! CS family is `Spherical`. The Boost spherical length strategy is
//! `strategies::length::spherical` from
//! `boost/geometry/strategies/length/spherical.hpp`, which hands
//! `strategy::distance::haversine` to the per-segment walk; we mirror
//! that — [`SphericalLength`] / [`SphericalPerimeter`] are thin shells
//! around the existing [`Haversine`]
//! distance strategy.

use alloc::vec::Vec;

use geometry_cs::{CoordinateSystem, SphericalFamily};
use geometry_tag::SameAs;
use geometry_trait::{Closure, Linestring, Point, Ring};

use crate::distance::DistanceStrategy;
use crate::length::LengthStrategy;
use crate::spherical::Haversine;

/// Spherical length: sum of Haversine distances between consecutive
/// points.
///
/// Carries the same `radius` parameter as [`Haversine`]
/// so callers control the sphere size (Earth ≈ `6_372_795` m by Boost
/// convention). `Default::default()` produces
/// `SphericalLength { haversine: Haversine::EARTH }`.
///
/// Mirrors `boost::geometry::strategies::length::spherical<>` from
/// `strategies/length/spherical.hpp`.
#[derive(Debug, Default, Clone, Copy)]
pub struct SphericalLength {
    /// The inner Haversine distance kernel summed over each segment.
    pub haversine: Haversine,
}

/// Spherical perimeter — same as [`SphericalLength`] but adds the
/// closing edge for an open ring.
///
/// Separate from [`SphericalLength`] because Rust's coherence rules
/// cannot prove that a single type does not implement both
/// [`Linestring`] and [`Ring`]; splitting the strategy keeps the
/// per-tag dispatch disjoint at the impl level (the same split as
/// [`CartesianLength`](crate::length::CartesianLength) /
/// [`CartesianPerimeter`](crate::length::CartesianPerimeter)).
#[derive(Debug, Default, Clone, Copy)]
pub struct SphericalPerimeter {
    /// The inner Haversine distance kernel summed over each segment.
    pub haversine: Haversine,
}

// ---- Linestring ------------------------------------------------------
//
// Mirrors the `dispatch::length<Geometry, linestring_tag>` arm at
// `algorithms/length.hpp:132-135`, feeding the spherical Haversine
// kernel instead of Pythagoras.

impl<L> LengthStrategy<L> for SphericalLength
where
    L: Linestring,
    <L::Point as Point>::Cs: CoordinateSystem,
    <<L::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
    Haversine: DistanceStrategy<L::Point, L::Point, Out = <L::Point as Point>::Scalar>,
{
    type Out = <L::Point as Point>::Scalar;

    #[inline]
    fn length(&self, g: &L) -> Self::Out {
        let pts: Vec<&L::Point> = g.points().collect();
        let mut acc = <Self::Out as geometry_coords::CoordinateScalar>::ZERO;
        for w in pts.windows(2) {
            acc = acc + self.haversine.distance(w[0], w[1]);
        }
        acc
    }
}

// ---- Ring ------------------------------------------------------------
//
// Mirrors the `dispatch::perimeter<Geometry, ring_tag>` arm at
// `algorithms/perimeter.hpp:66-73`. For an open ring the explicit
// last->first segment is added; a closed ring already encodes the
// closing edge as its repeated last point.

impl<R> LengthStrategy<R> for SphericalPerimeter
where
    R: Ring,
    <R::Point as Point>::Cs: CoordinateSystem,
    <<R::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
    Haversine: DistanceStrategy<R::Point, R::Point, Out = <R::Point as Point>::Scalar>,
{
    type Out = <R::Point as Point>::Scalar;

    #[inline]
    fn length(&self, g: &R) -> Self::Out {
        let pts: Vec<&R::Point> = g.points().collect();
        let mut acc = <Self::Out as geometry_coords::CoordinateScalar>::ZERO;
        for w in pts.windows(2) {
            acc = acc + self.haversine.distance(w[0], w[1]);
        }
        if matches!(g.closure(), Closure::Open) {
            if let Some(first) = pts.first() {
                let last = pts.last().unwrap_or(first);
                acc = acc + self.haversine.distance(last, first);
            }
        }
        acc
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference from `boost/geometry/test/algorithms/length/length_sph.cpp`.
    //! Values are cross-checked against a direct [`Haversine`] call on
    //! the same segments (the length strategy reuses the distance
    //! strategy verbatim, so the two must agree exactly).
    #![allow(
        clippy::float_cmp,
        reason = "lengths are compared with an explicit absolute tolerance, not `==`"
    )]

    use super::{SphericalLength, SphericalPerimeter};
    use crate::distance::DistanceStrategy;
    use crate::length::LengthStrategy;
    use crate::spherical::Haversine;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Spherical};
    use geometry_model::{Linestring, Ring};

    type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;

    #[inline]
    fn deg(lon: f64, lat: f64) -> Sp {
        WithCs::new(Adapt([lon, lat]))
    }

    /// A single-segment linestring's length equals a direct Haversine
    /// call — Amsterdam → Paris on the Boost average earth.
    #[test]
    fn ams_paris_haversine_length() {
        let ls: Linestring<Sp> = Linestring(vec![deg(4.90, 52.37), deg(2.35, 48.86)]);
        let got = SphericalLength::default().length(&ls);
        let direct = Haversine::EARTH.distance(&deg(4.90, 52.37), &deg(2.35, 48.86));
        assert!((got - direct).abs() < 1e-6);
    }

    /// A three-point meridian linestring sums two equal 10° hops.
    #[test]
    fn three_point_great_circle() {
        let ls: Linestring<Sp> = Linestring(vec![deg(0., 0.), deg(0., 10.), deg(0., 20.)]);
        let got = SphericalLength::default().length(&ls);
        let segment = Haversine::EARTH.distance(&deg(0., 0.), &deg(0., 10.));
        assert!((got - segment * 2.0).abs() < 1e-6);
    }

    /// An open ring's perimeter adds the closing edge back to the
    /// first vertex.
    #[test]
    fn open_ring_closes_itself() {
        let mut r = Ring::<Sp, true, false>::new();
        r.push(deg(0., 0.));
        r.push(deg(10., 0.));
        r.push(deg(10., 10.));
        r.push(deg(0., 10.));
        let got = SphericalPerimeter::default().length(&r);
        let h = Haversine::EARTH;
        let expected = h.distance(&deg(0., 0.), &deg(10., 0.))
            + h.distance(&deg(10., 0.), &deg(10., 10.))
            + h.distance(&deg(10., 10.), &deg(0., 10.))
            + h.distance(&deg(0., 10.), &deg(0., 0.));
        assert!((got - expected).abs() < 1e-6);
    }
}
