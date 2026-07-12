//! `centroid(&g)` — the geometric centre of `g`.
//!
//! Mirrors `boost::geometry::centroid(g, p)` from
//! `boost/geometry/algorithms/centroid.hpp` — except Boost returns
//! through an out-parameter while Rust returns the point by value. Per
//! geometry kind (all Cartesian in LA6; spherical / geographic siblings
//! land in LA8):
//!
//! * `Polygon`    → [`geometry_strategy::CartesianPolygonCentroid`]
//! * `Ring`       → [`geometry_strategy::CartesianRingCentroid`]
//! * `Linestring` → [`geometry_strategy::CartesianLinestringCentroid`]
//! * `Segment`    → [`geometry_strategy::CartesianSegmentCentroid`]
//! * `Box`        → [`geometry_strategy::CartesianBoxCentroid`]
//! * `MultiPoint` → [`geometry_strategy::CartesianMultiPointCentroid`]
//!
//! The per-kind strategy is selected type-level via
//! [`geometry_strategy::CentroidStrategyForKind`], a tag-keyed picker
//! that routes `G::Kind` to the right per-kind strategy struct. Because
//! it keys on the tag, any concept-adapted foreign type resolves through
//! the same path as the equivalent `geometry-model` value.
//! `centroid_with` takes an explicit
//! strategy.
//!
//! # Spherical / geographic centroid — deferred (LA8.T3)
//!
//! Unlike `length` / `area` / `azimuth` — whose LA8 free functions
//! dispatch by coordinate-system family — `centroid` stays Cartesian.
//! Boost ships no validated spherical / geographic centroid reference
//! values, so shipping that math would be unverifiable; see the
//! deferral note on [`geometry_strategy::CentroidStrategyForKind`]. A
//! spherical or geographic geometry is therefore a compile error here;
//! use [`centroid_with`] with an explicit strategy in the meantime.

use geometry_strategy::{CentroidStrategy, CentroidStrategyForKind};
use geometry_trait::Geometry;

/// Centroid of `g` using the per-kind Cartesian strategy.
///
/// Mirrors `boost::geometry::centroid(g, p)` from
/// `boost/geometry/algorithms/centroid.hpp`. The strategy is chosen from
/// the geometry *kind* via the tag-keyed
/// [`geometry_strategy::CentroidStrategyForKind`] picker; for an
/// explicit strategy (or a future spherical / geographic LA8 strategy)
/// use [`centroid_with`].
#[inline]
#[must_use]
pub fn centroid<G>(
    g: &G,
) -> <<G::Kind as CentroidStrategyForKind>::S as CentroidStrategy<G>>::Output
where
    G: Geometry,
    G::Kind: CentroidStrategyForKind,
    <G::Kind as CentroidStrategyForKind>::S: CentroidStrategy<G>,
{
    <<G::Kind as CentroidStrategyForKind>::S as Default>::default().centroid(g)
}

/// Centroid of `g` using an explicitly supplied strategy.
///
/// Mirrors the `centroid(g, p, strategy)` overload at
/// `boost/geometry/algorithms/centroid.hpp`. Taking the strategy by
/// value (rather than by reference) matches the by-value call shape of
/// [`crate::distance_with`]; concrete strategies are
/// zero-sized configuration objects, so this monomorphises into nothing.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "Strategies are ZST/small Copy configuration objects; by-value matches the user-facing call shape."
)]
pub fn centroid_with<G, S>(g: &G, s: S) -> S::Output
where
    G: Geometry,
    S: CentroidStrategy<G>,
{
    s.centroid(g)
}

#[cfg(test)]
mod tests {
    //! Reference values from `geometry/test/algorithms/centroid.cpp` —
    //! the same fixtures the LA6.T1 strategy tests use, reached here
    //! through the free function.
    #![allow(
        clippy::float_cmp,
        reason = "centroids are compared with an explicit absolute tolerance, not `==`"
    )]

    use super::centroid;
    use geometry_cs::Cartesian;
    use geometry_model::{Box, MultiPoint, Point2D, Polygon, Ring, Segment, linestring, polygon};
    use geometry_trait::Point as _;

    type Pt = Point2D<f64, Cartesian>;

    fn close(got: Pt, x: f64, y: f64, tol: f64) -> bool {
        (got.get::<0>() - x).abs() < tol && (got.get::<1>() - y).abs() < tol
    }

    // centroid.cpp:46 — POLYGON((0 0,0 10,10 10,10 0,0 0)) → (5, 5)
    #[test]
    fn polygon_10x10_square() {
        let pg: Polygon<Pt> = polygon![[(0., 0.), (0., 10.), (10., 10.), (10., 0.), (0., 0.)]];
        let c = centroid(&pg);
        assert!(close(c, 5.0, 5.0, 1e-9));
    }

    // centroid.cpp:139 — ring POLYGON((1 1, 1 2, 2 2, 2 1, 1 1)) → (1.5, 1.5)
    #[test]
    fn ring_unit_square_shift() {
        let r: Ring<Pt> = Ring::from_vec(vec![
            Pt::new(1., 1.),
            Pt::new(1., 2.),
            Pt::new(2., 2.),
            Pt::new(2., 1.),
            Pt::new(1., 1.),
        ]);
        let c = centroid(&r);
        assert!(close(c, 1.5, 1.5, 1e-9));
    }

    // centroid.cpp:73 — LINESTRING(1 1, 2 2, 3 3) → (2, 2)
    #[test]
    fn linestring_diagonal() {
        let ls = linestring![(1., 1.), (2., 2.), (3., 3.)];
        let c = centroid(&ls);
        assert!(close(c, 2.0, 2.0, 1e-9));
    }

    // centroid.cpp:109 — segment (1 1) → (3 3) → (2, 2)
    #[test]
    fn segment_midpoint() {
        let s = Segment::new(Pt::new(1., 1.), Pt::new(3., 3.));
        let c = centroid(&s);
        assert!(close(c, 2.0, 2.0, 1e-12));
    }

    // centroid.cpp:131 — box "POLYGON((1 2,3 4))" → (2, 3)
    #[test]
    fn box_centroid() {
        let b: Box<Pt> = Box::from_corners(Pt::new(1., 2.), Pt::new(3., 4.));
        let c = centroid(&b);
        assert!(close(c, 2.0, 3.0, 1e-12));
    }

    // MultiPoint {(0,0),(2,0),(0,2)} → (2/3, 2/3)
    #[test]
    fn multipoint_mean() {
        let mp: MultiPoint<Pt> =
            MultiPoint::from_vec(vec![Pt::new(0., 0.), Pt::new(2., 0.), Pt::new(0., 2.)]);
        let c = centroid(&mp);
        assert!(close(c, 2.0 / 3.0, 2.0 / 3.0, 1e-9));
    }
}
