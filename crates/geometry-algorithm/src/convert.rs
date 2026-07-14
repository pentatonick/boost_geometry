//! `convert(src) -> Dst` — typed conversions between equivalent
//! geometry kinds.
//!
//! Mirrors `boost::geometry::convert(src, dst)` from
//! `boost/geometry/algorithms/convert.hpp`. Boost writes the
//! destination through an out-parameter; the Rust port returns by
//! value. A dedicated [`Convert`] trait (rather than
//! [`core::convert::Into`]) keeps geometry conversions opt-in and
//! sidesteps the orphan-rule clash a blanket `From`/`Into` would hit
//! the moment a user adapts their own types.
//!
//! Supported pairs:
//!
//! * `Box`     → `Polygon`     — the rectangle's 5-point closed ring
//! * `Ring`    → `Polygon`     — the ring becomes the exterior
//! * `Ring`    → `Linestring`  — copy the point sequence
//! * `Segment` → `Linestring`  — two-point `(start, end)`
//! * `Point`, `Linestring`, `Polygon` → single-member multi
//!
//! `Linestring` → `Ring` is intentionally NOT shipped — the caller
//! must assert closure and orientation for the destination, which
//! Boost forces through template arguments anyway. Use
//! `Ring::from_vec(ls.0.clone())` (then [`correct`](fn@crate::correct))
//! if you need it.

use geometry_model::{
    Box, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Polygon, Ring, Segment,
};
use geometry_trait::{Point as PointTrait, PointMut};

use crate::make::make_point;

/// Convert `src` into the destination kind `Dst`.
///
/// Mirrors `boost::geometry::convert(src, dst)` from
/// `boost/geometry/algorithms/convert.hpp`. The destination is
/// inferred from the call-site annotation:
///
/// ```ignore
/// let pg: Polygon<P> = convert(&bx);
/// ```
#[must_use]
pub fn convert<Src, Dst>(src: &Src) -> Dst
where
    Src: Convert<Dst>,
{
    src.convert()
}

/// Per-pair conversion dispatch. One impl per `(Src, Dst)` pair the
/// port supports; the free [`convert`] function is the public entry.
#[doc(hidden)]
pub trait Convert<Dst> {
    fn convert(&self) -> Dst;
}

/// `Box` → `Polygon`: the rectangle's four corners wound clockwise
/// and closed back onto the first, giving a 5-point exterior ring.
///
/// Mirrors the `box → polygon` arm of
/// `boost/geometry/algorithms/convert.hpp`. Only the first two
/// dimensions participate — a box is a planar rectangle.
impl<P, const CW: bool, const CL: bool> Convert<Polygon<P, CW, CL>> for Box<P>
where
    P: PointMut + Default,
{
    fn convert(&self) -> Polygon<P, CW, CL> {
        let min_x = self.min().get::<0>();
        let min_y = self.min().get::<1>();
        let max_x = self.max().get::<0>();
        let max_y = self.max().get::<1>();
        // Clockwise from the minimum corner, closed back to the start:
        // (minx, miny) → (minx, maxy) → (maxx, maxy) → (maxx, miny) → (minx, miny).
        let ring = Ring::<P, CW, CL>::from_vec(alloc::vec![
            make_point(&[min_x, min_y]),
            make_point(&[min_x, max_y]),
            make_point(&[max_x, max_y]),
            make_point(&[max_x, min_y]),
            make_point(&[min_x, min_y]),
        ]);
        Polygon::new(ring)
    }
}

/// `Ring` → `Polygon`: the ring becomes the exterior, no holes.
///
/// Mirrors the `ring → polygon` arm of
/// `boost/geometry/algorithms/convert.hpp`.
impl<P, const CW: bool, const CL: bool> Convert<Polygon<P, CW, CL>> for Ring<P, CW, CL>
where
    P: PointTrait + Copy,
{
    fn convert(&self) -> Polygon<P, CW, CL> {
        Polygon::new(Ring::from_vec(self.0.clone()))
    }
}

/// `Ring` → `Linestring`: copy the point sequence, dropping the
/// closure / orientation type information.
///
/// Mirrors the `ring → linestring` arm of
/// `boost/geometry/algorithms/convert.hpp`.
impl<P, const CW: bool, const CL: bool> Convert<Linestring<P>> for Ring<P, CW, CL>
where
    P: PointTrait + Copy,
{
    fn convert(&self) -> Linestring<P> {
        Linestring(self.0.clone())
    }
}

/// `Segment` → `Linestring`: a two-point `(start, end)` linestring.
///
/// Mirrors the `segment → linestring` arm of
/// `boost/geometry/algorithms/convert.hpp`.
impl<P> Convert<Linestring<P>> for Segment<P>
where
    P: PointTrait + Copy,
{
    fn convert(&self) -> Linestring<P> {
        Linestring(alloc::vec![*self.start(), *self.end()])
    }
}

/// `Point` → `MultiPoint`: wrap as a single-member multi.
///
/// Mirrors the `point → multi_point` arm of
/// `boost/geometry/algorithms/convert.hpp`.
impl<P> Convert<MultiPoint<P>> for P
where
    P: PointTrait + Copy,
{
    fn convert(&self) -> MultiPoint<P> {
        MultiPoint(alloc::vec![*self])
    }
}

/// `Linestring` → `MultiLinestring`: wrap as a single-member multi.
///
/// Mirrors the `linestring → multi_linestring` arm of
/// `boost/geometry/algorithms/convert.hpp`.
impl<P> Convert<MultiLinestring<Linestring<P>>> for Linestring<P>
where
    P: PointTrait + Copy,
{
    fn convert(&self) -> MultiLinestring<Linestring<P>> {
        MultiLinestring(alloc::vec![self.clone()])
    }
}

/// `Polygon` → `MultiPolygon`: wrap as a single-member multi.
///
/// Mirrors the `polygon → multi_polygon` arm of
/// `boost/geometry/algorithms/convert.hpp`.
impl<P, const CW: bool, const CL: bool> Convert<MultiPolygon<Polygon<P, CW, CL>>>
    for Polygon<P, CW, CL>
where
    P: PointTrait + Copy,
{
    fn convert(&self) -> MultiPolygon<Polygon<P, CW, CL>> {
        MultiPolygon(alloc::vec![self.clone()])
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Converted corner coordinates are exact literals."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/convert.cpp` — a box becomes a
    //! 5-point closed polygon over the same corners; a segment becomes
    //! a two-point linestring; a point wraps into a single-member
    //! multi-point.

    use super::convert;
    use geometry_cs::Cartesian;
    use geometry_model::{Box, Linestring, MultiPoint, Point2D, Polygon, Segment};
    use geometry_trait::{Point as _, Polygon as _, Ring as _};

    type Pt = Point2D<f64, Cartesian>;

    // convert.cpp — Box → Polygon yields the rectangle's closed ring.
    #[test]
    fn box_to_polygon_five_points_and_corners() {
        let b: Box<Pt> = Box::from_corners(Pt::new(0., 0.), Pt::new(4., 3.));
        let pg: Polygon<Pt> = convert(&b);
        let pts: alloc::vec::Vec<(f64, f64)> = pg
            .exterior()
            .points()
            .map(|p| (p.get::<0>(), p.get::<1>()))
            .collect();
        // Closed ring: 5 points, first == last, all four corners present.
        assert_eq!(pts.len(), 5);
        assert_eq!(pts[0], (0., 0.));
        assert_eq!(pts[4], (0., 0.));
        assert!(pts.contains(&(0., 3.)));
        assert!(pts.contains(&(4., 3.)));
        assert!(pts.contains(&(4., 0.)));
    }

    // convert.cpp — Segment → Linestring keeps both endpoints.
    #[test]
    fn segment_to_linestring_endpoints() {
        let s = Segment::new(Pt::new(0., 0.), Pt::new(3., 4.));
        let ls: Linestring<Pt> = convert(&s);
        assert_eq!(ls.0.len(), 2);
        assert_eq!(ls.0[0].get::<0>(), 0.);
        assert_eq!(ls.0[1].get::<0>(), 3.);
    }

    #[test]
    fn point_to_multi_point_single_member() {
        let mp: MultiPoint<Pt> = convert(&Pt::new(1., 2.));
        assert_eq!(mp.0.len(), 1);
        assert_eq!(mp.0[0].get::<0>(), 1.);
    }

    // convert.cpp — Ring → Polygon: the ring becomes a hole-free
    // exterior, vertices copied verbatim.
    #[test]
    fn ring_to_polygon_becomes_hole_free_exterior() {
        use geometry_model::Ring;
        let ring: Ring<Pt> = Ring::from_vec(vec![
            Pt::new(0., 0.),
            Pt::new(1., 0.),
            Pt::new(1., 1.),
            Pt::new(0., 0.),
        ]);
        let pg: Polygon<Pt> = convert(&ring);
        assert_eq!(pg.exterior().points().count(), 4);
        assert_eq!(pg.interiors().count(), 0);
        let first = pg.exterior().points().next().unwrap();
        assert_eq!((first.get::<0>(), first.get::<1>()), (0., 0.));
    }

    // convert.cpp — Ring → Linestring copies the point sequence.
    #[test]
    fn ring_to_linestring_copies_points() {
        use geometry_model::Ring;
        let ring: Ring<Pt> = Ring::from_vec(vec![Pt::new(2., 3.), Pt::new(4., 5.), Pt::new(2., 3.)]);
        let ls: Linestring<Pt> = convert(&ring);
        assert_eq!(ls.0.len(), 3);
        assert_eq!((ls.0[1].get::<0>(), ls.0[1].get::<1>()), (4., 5.));
    }

    // convert.cpp — Linestring → MultiLinestring wraps a single member.
    #[test]
    fn linestring_to_multi_linestring_single_member() {
        use geometry_model::MultiLinestring;
        let ls: Linestring<Pt> = Linestring(vec![Pt::new(0., 0.), Pt::new(1., 1.)]);
        let mls: MultiLinestring<Linestring<Pt>> = convert(&ls);
        assert_eq!(mls.0.len(), 1);
        assert_eq!(mls.0[0].0.len(), 2);
    }

    // convert.cpp — Polygon → MultiPolygon wraps a single member.
    #[test]
    fn polygon_to_multi_polygon_single_member() {
        use geometry_model::{MultiPolygon, Ring};
        let pg: Polygon<Pt> = Polygon::new(Ring::from_vec(vec![
            Pt::new(0., 0.),
            Pt::new(1., 0.),
            Pt::new(1., 1.),
            Pt::new(0., 0.),
        ]));
        let mpg: MultiPolygon<Polygon<Pt>> = convert(&pg);
        assert_eq!(mpg.0.len(), 1);
        assert_eq!(mpg.0[0].exterior().points().count(), 4);
    }
}
