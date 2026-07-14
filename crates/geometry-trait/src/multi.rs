//! The three "iter of a single geometry" concepts:
//! [`MultiPoint`], [`MultiLinestring`], and [`MultiPolygon`].
//!
//! Mirrors `doc/concept/multi_point.qbk`, `doc/concept/multi_linestring.qbk`,
//! and `doc/concept/multi_polygon.qbk`; the canonical models live in
//! `boost/geometry/geometries/multi_point.hpp`,
//! `boost/geometry/geometries/multi_linestring.hpp`, and
//! `boost/geometry/geometries/multi_polygon.hpp` respectively. All three
//! C++ models derive from `Container<Item, Allocator<Item>>` and are
//! consumed through `boost::range`; the Rust counterparts surface the
//! item sequence as a `…s()` iterator returned via RPITIT, matching the
//! pattern already used by [`crate::Linestring`], [`crate::Ring`], and
//! [`crate::Polygon`].

use crate::geometry::Geometry;
use crate::linestring::Linestring;
use crate::point::Point;
use crate::polygon::Polygon;
use geometry_tag::{MultiLinestringTag, MultiPointTag, MultiPolygonTag};

/// A multi-point — a collection of points belonging to each other.
///
/// Mirrors the `MultiPoint` concept (`doc/concept/multi_point.qbk`); the
/// canonical model is `boost::geometry::model::multi_point` in
/// `boost/geometry/geometries/multi_point.hpp`, which derives from
/// `std::vector<Point>` and asserts `concepts::Point<Point>` on its
/// element type. The Rust port mirrors that assertion by bounding
/// [`MultiPoint::ItemPoint`] on the [`Point`] concept.
///
/// As with [`crate::Linestring`], the element sequence is exposed via
/// RPITIT so each impl can hand back whatever iterator its underlying
/// container provides. The `ExactSizeIterator` bound mirrors the
/// random-access shape `boost::range` relies on for the C++ model:
/// callers can ask for `.len()` without consuming the source.
///
/// # Examples
///
/// ```
/// use geometry_trait::MultiPoint;
/// fn count<M: MultiPoint>(m: &M) -> usize { m.points().len() }
/// ```
pub trait MultiPoint: Geometry<Kind = MultiPointTag> {
    /// The element point type.
    ///
    /// Mirrors the `Point` template parameter on
    /// `boost::geometry::model::multi_point<Point, …>`
    /// (`boost/geometry/geometries/multi_point.hpp`). The Rust side
    /// does not require `ItemPoint == Self::Point`: the C++ model
    /// has no `Self::Point` projection of its own (its
    /// `point_type<MP>::type` is read from the element), so any
    /// [`Point`] is admissible here.
    type ItemPoint: Point;

    /// The points of this multi-point, in declared order.
    ///
    /// Plays the role of `boost::begin(mp)` / `boost::end(mp)` from
    /// `boost/geometry/geometries/multi_point.hpp` when read together.
    fn points(&self) -> impl ExactSizeIterator<Item = &Self::ItemPoint>;
}

/// A multi-linestring — a collection of linestrings belonging to each
/// other (e.g. a highway with interruptions).
///
/// Mirrors the `MultiLinestring` concept
/// (`doc/concept/multi_linestring.qbk`); the canonical model is
/// `boost::geometry::model::multi_linestring` in
/// `boost/geometry/geometries/multi_linestring.hpp`, which derives
/// from `std::vector<LineString>` and asserts
/// `concepts::Linestring<LineString>` on its element type. The Rust
/// port mirrors that assertion by bounding
/// [`MultiLinestring::ItemLinestring`] on the [`Linestring`] concept,
/// and additionally pins `ItemLinestring::Point = Self::Point` so the
/// multi's `point_type<MLS>` projection (read through any element)
/// stays consistent across the collection.
///
/// # Examples
///
/// ```
/// use geometry_trait::MultiLinestring;
/// fn count<M: MultiLinestring>(m: &M) -> usize { m.linestrings().len() }
/// ```
pub trait MultiLinestring: Geometry<Kind = MultiLinestringTag> {
    /// The element linestring type.
    ///
    /// Mirrors the `LineString` template parameter on
    /// `boost::geometry::model::multi_linestring<LineString, …>`
    /// (`boost/geometry/geometries/multi_linestring.hpp`).
    type ItemLinestring: Linestring<Point = Self::Point>;

    /// The linestrings of this multi-linestring, in declared order.
    ///
    /// Plays the role of `boost::begin(mls)` / `boost::end(mls)` from
    /// `boost/geometry/geometries/multi_linestring.hpp` when read
    /// together.
    fn linestrings(&self) -> impl ExactSizeIterator<Item = &Self::ItemLinestring>;
}

/// A multi-polygon — a collection of polygons belonging to each other
/// (e.g. Hawaii).
///
/// Mirrors the `MultiPolygon` concept (`doc/concept/multi_polygon.qbk`);
/// the canonical model is `boost::geometry::model::multi_polygon` in
/// `boost/geometry/geometries/multi_polygon.hpp`, which derives from
/// `std::vector<Polygon>` and asserts `concepts::Polygon<Polygon>` on
/// its element type. The Rust port mirrors that assertion by bounding
/// [`MultiPolygon::ItemPolygon`] on the [`Polygon`] concept, and
/// additionally pins `ItemPolygon::Point = Self::Point` so the multi's
/// `point_type<MPG>` projection stays consistent across the collection.
///
/// # Examples
///
/// ```
/// use geometry_trait::MultiPolygon;
/// fn count<M: MultiPolygon>(m: &M) -> usize { m.polygons().len() }
/// ```
pub trait MultiPolygon: Geometry<Kind = MultiPolygonTag> {
    /// The element polygon type.
    ///
    /// Mirrors the `Polygon` template parameter on
    /// `boost::geometry::model::multi_polygon<Polygon, …>`
    /// (`boost/geometry/geometries/multi_polygon.hpp`).
    type ItemPolygon: Polygon<Point = Self::Point>;

    /// The polygons of this multi-polygon, in declared order.
    ///
    /// Plays the role of `boost::begin(mpg)` / `boost::end(mpg)` from
    /// `boost/geometry/geometries/multi_polygon.hpp` when read
    /// together.
    fn polygons(&self) -> impl ExactSizeIterator<Item = &Self::ItemPolygon>;
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::point::PointMut;
    use crate::ring::Ring;
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_tag::{LinestringTag, PointTag, PolygonTag, RingTag};

    // --- Witnesses: any T satisfying the trait is accepted. These
    // are static, compile-time checks; they never run. ---
    fn accepts_mp<M: MultiPoint>() {}
    fn accepts_mls<M: MultiLinestring>() {}
    fn accepts_mpg<M: MultiPolygon>() {}

    // --- Shared 2D Cartesian point used by every test impl below. ---
    #[derive(Clone)]
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

    impl PointMut for Xy {
        fn set<const D: usize>(&mut self, v: f64) {
            if D == 0 {
                self.0 = v;
            } else {
                self.1 = v;
            }
        }
    }

    // --- Vec-backed MultiPoint. ---
    struct VMp(Vec<Xy>);

    impl Geometry for VMp {
        type Kind = MultiPointTag;
        type Point = Xy;
    }

    impl MultiPoint for VMp {
        type ItemPoint = Xy;

        fn points(&self) -> impl ExactSizeIterator<Item = &Xy> {
            self.0.iter()
        }
    }

    #[test]
    fn vec_backed_multipoint_satisfies_trait() {
        let mp = VMp(vec![Xy(0.0, 0.0), Xy(1.0, 2.0), Xy(3.0, 4.0)]);
        accepts_mp::<VMp>();
        assert_eq!(mp.points().len(), 3);
        let xs: Vec<f64> = mp.points().map(Xy::get::<0>).collect();
        assert_eq!(xs, vec![0.0, 1.0, 3.0]);
    }

    // --- Vec-backed MultiLinestring. ---
    struct VLs(Vec<Xy>);

    impl Geometry for VLs {
        type Kind = LinestringTag;
        type Point = Xy;
    }

    impl Linestring for VLs {
        fn points(&self) -> impl ExactSizeIterator<Item = &Xy> + Clone {
            self.0.iter()
        }
    }

    struct VMls(Vec<VLs>);

    impl Geometry for VMls {
        type Kind = MultiLinestringTag;
        type Point = Xy;
    }

    impl MultiLinestring for VMls {
        type ItemLinestring = VLs;

        fn linestrings(&self) -> impl ExactSizeIterator<Item = &VLs> {
            self.0.iter()
        }
    }

    #[test]
    fn vec_backed_multilinestring_satisfies_trait() {
        let mls = VMls(vec![
            VLs(vec![Xy(0.0, 0.0), Xy(1.0, 1.0)]),
            VLs(vec![Xy(2.0, 2.0), Xy(3.0, 3.0), Xy(4.0, 4.0)]),
        ]);
        accepts_mls::<VMls>();
        assert_eq!(mls.linestrings().len(), 2);
        let counts: Vec<usize> = mls.linestrings().map(|ls| ls.points().count()).collect();
        assert_eq!(counts, vec![2, 3]);
    }

    // --- Vec-backed MultiPolygon. ---
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

    struct VMpg(Vec<VPoly>);

    impl Geometry for VMpg {
        type Kind = MultiPolygonTag;
        type Point = Xy;
    }

    impl MultiPolygon for VMpg {
        type ItemPolygon = VPoly;

        fn polygons(&self) -> impl ExactSizeIterator<Item = &VPoly> {
            self.0.iter()
        }
    }

    #[test]
    fn vec_backed_multipolygon_satisfies_trait() {
        let mpg = VMpg(vec![
            VPoly {
                outer: VRing(vec![
                    Xy(0.0, 0.0),
                    Xy(1.0, 0.0),
                    Xy(1.0, 1.0),
                    Xy(0.0, 1.0),
                    Xy(0.0, 0.0),
                ]),
                inners: vec![],
            },
            VPoly {
                outer: VRing(vec![
                    Xy(2.0, 2.0),
                    Xy(3.0, 2.0),
                    Xy(3.0, 3.0),
                    Xy(2.0, 3.0),
                    Xy(2.0, 2.0),
                ]),
                inners: vec![VRing(vec![
                    Xy(2.25, 2.25),
                    Xy(2.75, 2.25),
                    Xy(2.75, 2.75),
                    Xy(2.25, 2.75),
                    Xy(2.25, 2.25),
                ])],
            },
        ]);
        accepts_mpg::<VMpg>();
        assert_eq!(mpg.polygons().len(), 2);
        let inner_counts: Vec<usize> = mpg.polygons().map(|p| p.interiors().count()).collect();
        assert_eq!(inner_counts, vec![0, 1]);

        // Read the exterior ring's vertices of the first polygon through
        // `VRing::points` and both `Xy::get::<0>` / `get::<1>`.
        let first = mpg.polygons().next().unwrap();
        let ext: Vec<(f64, f64)> = first
            .exterior()
            .points()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        assert_eq!(ext.len(), 5);
        assert_eq!(ext[2], (1.0, 1.0));

        // And the second polygon's single hole is a non-empty ring.
        let hole_len = mpg
            .polygons()
            .nth(1)
            .unwrap()
            .interiors()
            .next()
            .unwrap()
            .points()
            .count();
        assert_eq!(hole_len, 5);
    }

    /// `Xy` is a full `PointMut`: `set` writes each ordinate and `get`
    /// reads it back — covering the `get`/`set` `D == 1` branches.
    #[test]
    #[allow(clippy::float_cmp, reason = "compared values are exact integer-valued literals")]
    fn xy_get_set_round_trips_both_ordinates() {
        let mut p = Xy(0.0, 0.0);
        p.set::<0>(3.0);
        p.set::<1>(4.0);
        assert_eq!(p.get::<0>(), 3.0);
        assert_eq!(p.get::<1>(), 4.0);
    }
}
