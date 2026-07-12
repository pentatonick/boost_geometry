//! Geographic geodesic length / perimeter.
//!
//! Mirrors the per-CS dispatch arm of
//! `boost/geometry/algorithms/length.hpp` reached when the geometry's
//! CS family is `Geographic`. The Boost geographic length strategy is
//! `strategies::length::geographic` from
//! `boost/geometry/strategies/length/geographic.hpp`, which hands a
//! geodesic inverse formula to the per-segment walk; we mirror that —
//! [`GeographicLength`] / [`GeographicPerimeter`] are thin shells
//! around the existing [`Andoyer`] distance
//! strategy.

use alloc::vec::Vec;

use geometry_cs::{CoordinateSystem, GeographicFamily};
use geometry_tag::SameAs;
use geometry_trait::{Closure, Linestring, Point, Ring};

use crate::distance::DistanceStrategy;
use crate::geographic::Andoyer;
use crate::length::LengthStrategy;

/// Geographic length: sum of Andoyer geodesic distances between
/// consecutive points.
///
/// Carries the same `spheroid` parameter as
/// [`Andoyer`] so callers can pick WGS84 /
/// GDA / custom. `Default::default()` produces
/// `GeographicLength { andoyer: Andoyer::WGS84 }`.
///
/// Mirrors `boost::geometry::strategies::length::geographic<>` from
/// `strategies/length/geographic.hpp`.
#[derive(Debug, Default, Clone, Copy)]
pub struct GeographicLength {
    /// The inner Andoyer geodesic kernel summed over each segment.
    pub andoyer: Andoyer,
}

/// Geographic perimeter — same as [`GeographicLength`] but adds the
/// closing edge for an open ring.
///
/// Separate from [`GeographicLength`] because Rust's coherence rules
/// cannot prove that a single type does not implement both
/// [`Linestring`] and [`Ring`]; splitting the strategy keeps the
/// per-tag dispatch disjoint at the impl level.
#[derive(Debug, Default, Clone, Copy)]
pub struct GeographicPerimeter {
    /// The inner Andoyer geodesic kernel summed over each segment.
    pub andoyer: Andoyer,
}

// ---- Linestring ------------------------------------------------------

impl<L> LengthStrategy<L> for GeographicLength
where
    L: Linestring,
    <L::Point as Point>::Cs: CoordinateSystem,
    <<L::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
    Andoyer: DistanceStrategy<L::Point, L::Point, Out = <L::Point as Point>::Scalar>,
{
    type Out = <L::Point as Point>::Scalar;

    #[inline]
    fn length(&self, g: &L) -> Self::Out {
        let pts: Vec<&L::Point> = g.points().collect();
        let mut acc = <Self::Out as geometry_coords::CoordinateScalar>::ZERO;
        for w in pts.windows(2) {
            acc = acc + self.andoyer.distance(w[0], w[1]);
        }
        acc
    }
}

// ---- Ring ------------------------------------------------------------

impl<R> LengthStrategy<R> for GeographicPerimeter
where
    R: Ring,
    <R::Point as Point>::Cs: CoordinateSystem,
    <<R::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
    Andoyer: DistanceStrategy<R::Point, R::Point, Out = <R::Point as Point>::Scalar>,
{
    type Out = <R::Point as Point>::Scalar;

    #[inline]
    fn length(&self, g: &R) -> Self::Out {
        let pts: Vec<&R::Point> = g.points().collect();
        let mut acc = <Self::Out as geometry_coords::CoordinateScalar>::ZERO;
        for w in pts.windows(2) {
            acc = acc + self.andoyer.distance(w[0], w[1]);
        }
        if matches!(g.closure(), Closure::Open) {
            if let (Some(first), Some(last)) = (pts.first(), pts.last()) {
                acc = acc + self.andoyer.distance(last, first);
            }
        }
        acc
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    //! Reference from `boost/geometry/test/algorithms/length/length_geo.cpp`.
    //! Values are cross-checked against a direct [`Andoyer`] call on the
    //! same segments (the length strategy reuses the distance strategy
    //! verbatim, so the two must agree exactly).
    #![allow(
        clippy::float_cmp,
        reason = "lengths are compared with an explicit absolute tolerance, not `==`"
    )]

    use super::{GeographicLength, GeographicPerimeter};
    use crate::distance::DistanceStrategy;
    use crate::geographic::Andoyer;
    use crate::length::LengthStrategy;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Geographic};
    use geometry_model::{Linestring, Ring};

    type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;

    #[inline]
    fn deg(lon: f64, lat: f64) -> Gg {
        WithCs::new(Adapt([lon, lat]))
    }

    /// A single-segment linestring's length equals a direct Andoyer
    /// call — `(4, 52) → (3, 40)` on WGS84 ≈ 1336.040 km
    /// (`andoyer.cpp:230-231`).
    #[test]
    fn ams_geodesic_length() {
        let ls: Linestring<Gg> = Linestring(vec![deg(4.0, 52.0), deg(3.0, 40.0)]);
        let got = GeographicLength::default().length(&ls);
        let direct = Andoyer::WGS84.distance(&deg(4.0, 52.0), &deg(3.0, 40.0));
        assert!((got - direct).abs() < 1e-6);
        assert!((got / 1000.0 - 1_336.039_890).abs() < 0.01);
    }

    /// A three-point linestring sums two geodesic hops.
    #[test]
    fn three_point_geodesic() {
        let ls: Linestring<Gg> = Linestring(vec![deg(0., 0.), deg(0., 10.), deg(0., 20.)]);
        let got = GeographicLength::default().length(&ls);
        let a = Andoyer::WGS84;
        let expected =
            a.distance(&deg(0., 0.), &deg(0., 10.)) + a.distance(&deg(0., 10.), &deg(0., 20.));
        assert!((got - expected).abs() < 1e-6);
    }

    /// An open ring's perimeter adds the closing edge back to the first
    /// vertex.
    #[test]
    fn open_ring_closes_itself() {
        let mut r = Ring::<Gg, true, false>::new();
        r.push(deg(0., 0.));
        r.push(deg(10., 0.));
        r.push(deg(10., 10.));
        r.push(deg(0., 10.));
        let got = GeographicPerimeter::default().length(&r);
        let a = Andoyer::WGS84;
        let expected = a.distance(&deg(0., 0.), &deg(10., 0.))
            + a.distance(&deg(10., 0.), &deg(10., 10.))
            + a.distance(&deg(10., 10.), &deg(0., 10.))
            + a.distance(&deg(0., 10.), &deg(0., 0.));
        assert!((got - expected).abs() < 1e-6);
    }
}
