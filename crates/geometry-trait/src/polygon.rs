//! The [`Polygon`] concept: an exterior ring plus zero or more interior
//! rings (holes), all sharing the same point type.
//!
//! Mirrors `doc/concept/polygon.qbk` and the model in
//! `boost/geometry/geometries/polygon.hpp`. The three polygon-only
//! metafunctions
//!   `boost::geometry::traits::ring_const_type<P>` /
//!   `boost::geometry::traits::ring_mutable_type<P>`
//!   (`boost/geometry/core/ring_type.hpp`),
//!   `boost::geometry::traits::exterior_ring<P>`
//!   (`boost/geometry/core/exterior_ring.hpp`), and
//!   `boost::geometry::traits::interior_rings<P>` /
//!   `boost::geometry::traits::interior_const_type<P>` /
//!   `boost::geometry::traits::interior_mutable_type<P>`
//!   (`boost/geometry/core/interior_rings.hpp`)
//! fold into a single associated type plus two accessor methods here.

use crate::geometry::Geometry;
use crate::ring::Ring;
use geometry_tag::PolygonTag;

/// A polygon — an exterior ring plus zero or more interior rings.
///
/// Mirrors the Polygon concept (`doc/concept/polygon.qbk`); the
/// canonical model is `boost::geometry::model::polygon` in
/// `boost/geometry/geometries/polygon.hpp`. All rings share the
/// polygon's point type, matching the constraint that
/// `traits::ring_const_type<P>::type` and the polygon's
/// `traits::point_type<P>::type` are mutually consistent
/// (`boost/geometry/core/{ring_type,point_type}.hpp`).
///
/// `Ring` is an associated type — locked in proposal §10.3 — so the
/// inner ring kind is fixed at the impl site and dispatch stays
/// monomorphic (Boost's `traits::ring_const_type<P>::type`).
///
/// As with [`crate::Linestring`] and [`crate::Ring`], the interior
/// rings are exposed via RPITIT so each impl can hand back whatever
/// iterator its underlying container provides.
///
/// # Examples
///
/// ```
/// use geometry_trait::Polygon;
/// fn hole_count<P: Polygon>(p: &P) -> usize { p.interiors().len() }
/// ```
pub trait Polygon: Geometry<Kind = PolygonTag> {
    /// The ring type used for both the exterior and the interior rings.
    ///
    /// Mirrors `boost::geometry::traits::ring_const_type<P>::type`
    /// (`boost/geometry/core/ring_type.hpp`).
    type Ring: Ring<Point = Self::Point>;

    /// The exterior ring (outer boundary) of this polygon.
    ///
    /// Mirrors `boost::geometry::traits::exterior_ring<P>::get(p)`
    /// (`boost/geometry/core/exterior_ring.hpp`).
    fn exterior(&self) -> &Self::Ring;

    /// The interior rings (holes) of this polygon, in declared order.
    ///
    /// Mirrors `boost::geometry::traits::interior_rings<P>::get(p)`
    /// (`boost/geometry/core/interior_rings.hpp`). The returned
    /// iterator is `ExactSizeIterator` so callers can ask for the
    /// hole count without consuming it — the analogue of
    /// `boost::size(traits::interior_rings<P>::get(p))`.
    fn interiors(&self) -> impl ExactSizeIterator<Item = &Self::Ring>;
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::point::Point;
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_tag::{PointTag, RingTag};

    struct Xy(f64, f64);

    impl Geometry for Xy {
        type Kind = PointTag;
        type Point = Self;
    }

    impl Point for Xy {
        type Scalar = f64;
        type Cs = Cartesian;
        const DIM: usize = 2;

        fn get<const D: usize>(&self) -> f64 {
            if D == 0 { self.0 } else { self.1 }
        }
    }

    struct VRing(Vec<Xy>);

    impl Geometry for VRing {
        type Kind = RingTag;
        type Point = Xy;
    }

    impl Ring for VRing {
        fn points(&self) -> impl ExactSizeIterator<Item = &Xy> + Clone {
            self.0.iter()
        }
    }

    struct VPoly {
        outer: VRing,
        inners: Vec<VRing>,
    }

    impl Geometry for VPoly {
        type Kind = PolygonTag;
        type Point = Xy;
    }

    impl Polygon for VPoly {
        type Ring = VRing;

        fn exterior(&self) -> &VRing {
            &self.outer
        }

        fn interiors(&self) -> impl ExactSizeIterator<Item = &VRing> {
            self.inners.iter()
        }
    }

    #[test]
    fn polygon_with_two_inner_rings() {
        let poly = VPoly {
            outer: VRing(vec![
                Xy(0.0, 0.0),
                Xy(10.0, 0.0),
                Xy(10.0, 10.0),
                Xy(0.0, 10.0),
                Xy(0.0, 0.0),
            ]),
            inners: vec![
                VRing(vec![
                    Xy(1.0, 1.0),
                    Xy(2.0, 1.0),
                    Xy(2.0, 2.0),
                    Xy(1.0, 2.0),
                    Xy(1.0, 1.0),
                ]),
                VRing(vec![
                    Xy(5.0, 5.0),
                    Xy(6.0, 5.0),
                    Xy(6.0, 6.0),
                    Xy(5.0, 6.0),
                    Xy(5.0, 5.0),
                ]),
            ],
        };

        assert_eq!(poly.exterior().points().count(), 5);
        assert_eq!(poly.interiors().count(), 2);
        let first = poly.exterior().points().next().unwrap();
        assert_eq!(first.get::<0>().to_bits(), 0.0_f64.to_bits());
        assert_eq!(first.get::<1>().to_bits(), 0.0_f64.to_bits());

        let inner_counts: Vec<usize> = poly.interiors().map(|r| r.points().count()).collect();
        assert_eq!(inner_counts, vec![5, 5]);
    }
}
