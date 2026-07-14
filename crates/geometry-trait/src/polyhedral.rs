//! The [`PolyhedralSurface`] concept: a contiguous collection of
//! polygons sharing common boundary segments.
//!
//! Mirrors `doc/concept/polyhedral_surface.qbk`; the canonical model is
//! `boost::geometry::model::polyhedral_surface` in
//! `boost/geometry/geometries/polyhedral_surface.hpp`, which derives
//! from `std::vector<Polygon>` and asserts `concepts::Polygon<Polygon>`
//! on its face type. The Rust port mirrors that assertion by bounding
//! [`PolyhedralSurface::Face`] on the [`Polygon`] concept, and pins
//! `Face::Point = Self::Point` so the surface's `point_type<PS>`
//! projection stays consistent across every face.

use crate::geometry::Geometry;
use crate::polygon::Polygon;
use geometry_tag::PolyhedralSurfaceTag;

/// A polyhedral surface — a contiguous collection of polygons in
/// 3-dimensional space that share common boundary segments.
///
/// Mirrors the `PolyhedralSurface` concept
/// (`doc/concept/polyhedral_surface.qbk`); the canonical model is
/// `boost::geometry::model::polyhedral_surface` in
/// `boost/geometry/geometries/polyhedral_surface.hpp`.
///
/// The OGC spec requires the faces to be 3D Cartesian and to share
/// boundary segments. As with the C++ model — whose constructors do
/// not check the shared-boundary invariant and defer to `is_valid()`
/// — neither invariant is enforced at the trait level. Both are
/// checked by `check_polyhedral_surface::<T>()` in T15.
///
/// # Examples
///
/// ```
/// use geometry_trait::PolyhedralSurface;
/// fn face_count<P: PolyhedralSurface>(p: &P) -> usize { p.faces().len() }
/// ```
pub trait PolyhedralSurface: Geometry<Kind = PolyhedralSurfaceTag> {
    /// The face polygon type.
    ///
    /// Mirrors the `Polygon` template parameter on
    /// `boost::geometry::model::polyhedral_surface<Polygon, …>`
    /// (`boost/geometry/geometries/polyhedral_surface.hpp`).
    type Face: Polygon<Point = Self::Point>;

    /// The faces of this polyhedral surface, in declared order.
    ///
    /// Plays the role of `boost::begin(ps)` / `boost::end(ps)` from
    /// `boost/geometry/geometries/polyhedral_surface.hpp` when read
    /// together. The returned iterator is `ExactSizeIterator` so
    /// callers can ask for the face count without consuming it.
    fn faces(&self) -> impl ExactSizeIterator<Item = &Self::Face>;
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::point::{Point, PointMut};
    use crate::ring::Ring;
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_tag::{PointTag, PolygonTag, RingTag};

    fn accepts_ps<P: PolyhedralSurface>() {}

    #[derive(Clone)]
    struct Xyz(f64, f64, f64);

    impl Geometry for Xyz {
        type Kind = PointTag;
        type Point = Self;
    }

    impl Point for Xyz {
        type Scalar = f64;
        type Cs = Cartesian;
        const DIM: usize = 3;

        fn get<const D: usize>(&self) -> f64 {
            match D {
                0 => self.0,
                1 => self.1,
                _ => self.2,
            }
        }
    }

    impl PointMut for Xyz {
        fn set<const D: usize>(&mut self, v: f64) {
            match D {
                0 => self.0 = v,
                1 => self.1 = v,
                _ => self.2 = v,
            }
        }
    }

    struct VRing(Vec<Xyz>);

    impl Geometry for VRing {
        type Kind = RingTag;
        type Point = Xyz;
    }

    impl Ring for VRing {
        fn points(&self) -> impl ExactSizeIterator<Item = &Xyz> + Clone {
            self.0.iter()
        }
    }

    struct VPoly {
        outer: VRing,
        inners: Vec<VRing>,
    }

    impl Geometry for VPoly {
        type Kind = PolygonTag;
        type Point = Xyz;
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

    struct VPolyhedral(Vec<VPoly>);

    impl Geometry for VPolyhedral {
        type Kind = PolyhedralSurfaceTag;
        type Point = Xyz;
    }

    impl PolyhedralSurface for VPolyhedral {
        type Face = VPoly;

        fn faces(&self) -> impl ExactSizeIterator<Item = &VPoly> {
            self.0.iter()
        }
    }

    /// Build a unit cube's four side faces as a synthetic polyhedral
    /// surface. v1 carries no algorithm for polyhedral surfaces, so
    /// this test only checks the trait surface compiles and `faces()`
    /// yields the right count.
    #[test]
    fn vec_backed_polyhedral_surface_satisfies_trait() {
        fn quad(a: Xyz, b: Xyz, c: Xyz, d: Xyz) -> VPoly {
            let a2 = a.clone();
            VPoly {
                outer: VRing(vec![a, b, c, d, a2]),
                inners: vec![],
            }
        }

        let ps = VPolyhedral(vec![
            quad(
                Xyz(0.0, 0.0, 0.0),
                Xyz(1.0, 0.0, 0.0),
                Xyz(1.0, 0.0, 1.0),
                Xyz(0.0, 0.0, 1.0),
            ),
            quad(
                Xyz(1.0, 0.0, 0.0),
                Xyz(1.0, 1.0, 0.0),
                Xyz(1.0, 1.0, 1.0),
                Xyz(1.0, 0.0, 1.0),
            ),
            quad(
                Xyz(1.0, 1.0, 0.0),
                Xyz(0.0, 1.0, 0.0),
                Xyz(0.0, 1.0, 1.0),
                Xyz(1.0, 1.0, 1.0),
            ),
            quad(
                Xyz(0.0, 1.0, 0.0),
                Xyz(0.0, 0.0, 0.0),
                Xyz(0.0, 0.0, 1.0),
                Xyz(0.0, 1.0, 1.0),
            ),
        ]);

        accepts_ps::<VPolyhedral>();
        assert_eq!(ps.faces().len(), 4);
        let ring_lens: Vec<usize> = ps.faces().map(|f| f.exterior().points().count()).collect();
        assert_eq!(ring_lens, vec![5, 5, 5, 5]);

        // Read each ordinate of the first face's first vertex through
        // `Point::get`, and confirm every face reports zero interior
        // rings (`interiors()` is empty for these quads).
        let first_face = ps.faces().next().unwrap();
        let v0 = first_face.exterior().points().next().unwrap();
        assert_eq!((v0.get::<0>(), v0.get::<1>(), v0.get::<2>()), (0.0, 0.0, 0.0));
        for face in ps.faces() {
            assert_eq!(face.interiors().count(), 0);
        }
    }

    /// `Xyz` is a full `PointMut`: writing each ordinate through `set`
    /// and reading it back through `get` round-trips.
    #[test]
    fn xyz_point_get_set_round_trips_every_ordinate() {
        let mut p = Xyz(0.0, 0.0, 0.0);
        p.set::<0>(1.0);
        p.set::<1>(2.0);
        p.set::<2>(3.0);
        assert_eq!((p.get::<0>(), p.get::<1>(), p.get::<2>()), (1.0, 2.0, 3.0));
    }
}
