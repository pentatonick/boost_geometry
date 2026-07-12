//! `num_points(&g)` — total point count, recursive on multi / polygon
//! kinds.
//!
//! Mirrors `boost::geometry::num_points` from
//! `boost/geometry/algorithms/num_points.hpp`. Boost's free function
//! also takes an `add_for_open` flag (when `true`, an *open* ring
//! counts an extra point for the implicit closing edge); v1 ships the
//! plain count only — the flag has no v1 fixture that a value depends
//! on, so it is omitted until an open-ring caller needs it.

use geometry_tag::{
    BoxTag, LinestringTag, MultiLinestringTag, MultiPointTag, MultiPolygonTag, PointTag,
    PolygonTag, RingTag, SegmentTag,
};
use geometry_trait::{
    Box as BoxTrait, Geometry, Linestring as LinestringTrait,
    MultiLinestring as MultiLinestringTrait, MultiPoint as MultiPointTrait,
    MultiPolygon as MultiPolygonTrait, Point as PointTrait, Polygon as PolygonTrait,
    Ring as RingTrait, Segment as SegmentTrait,
};

/// Total number of points in `g`.
///
/// * `Point` → `1`
/// * `Segment` → `2` (the two endpoints)
/// * `Box` → `2^D` corner points (`num_points.hpp:100-101`) — 4 in 2D
/// * `Linestring` / `Ring` → length of the stored point sequence
/// * `Polygon` → exterior + Σ interior ring point counts
/// * `Multi*` → sum over members
///
/// Mirrors `boost::geometry::num_points(g)` from
/// `boost/geometry/algorithms/num_points.hpp`. The per-kind body is
/// selected by the tag-keyed [`NumPointsStrategyForKind`] picker, so any
/// concept-adapted foreign type resolves through the same path as the
/// equivalent `geometry-model` value.
#[inline]
#[must_use]
pub fn num_points<G>(g: &G) -> usize
where
    G: Geometry,
    G::Kind: NumPointsStrategyForKind,
    <G::Kind as NumPointsStrategyForKind>::S: NumPointsStrategy<G>,
{
    <<G::Kind as NumPointsStrategyForKind>::S as Default>::default().num_points(g)
}

/// A strategy for counting the points of `G`. Each per-kind struct below
/// carries a single concept-bounded impl, so distinct structs never
/// overlap (the distinct-struct-per-kind pattern); users call [`num_points`], not this
/// trait.
#[doc(hidden)]
pub trait NumPointsStrategy<G> {
    /// Total number of points in `g`.
    fn num_points(&self, g: &G) -> usize;
}

#[derive(Default)]
#[doc(hidden)]
pub struct PointNumPoints;
#[derive(Default)]
#[doc(hidden)]
pub struct SegmentNumPoints;
#[derive(Default)]
#[doc(hidden)]
pub struct BoxNumPoints;
#[derive(Default)]
#[doc(hidden)]
pub struct LinestringNumPoints;
#[derive(Default)]
#[doc(hidden)]
pub struct RingNumPoints;
#[derive(Default)]
#[doc(hidden)]
pub struct PolygonNumPoints;
#[derive(Default)]
#[doc(hidden)]
pub struct MultiPointNumPoints;
#[derive(Default)]
#[doc(hidden)]
pub struct MultiLinestringNumPoints;
#[derive(Default)]
#[doc(hidden)]
pub struct MultiPolygonNumPoints;

impl<G: PointTrait> NumPointsStrategy<G> for PointNumPoints {
    fn num_points(&self, _g: &G) -> usize {
        1
    }
}

impl<G: SegmentTrait> NumPointsStrategy<G> for SegmentNumPoints {
    fn num_points(&self, _g: &G) -> usize {
        2
    }
}

impl<G: BoxTrait> NumPointsStrategy<G> for BoxNumPoints {
    fn num_points(&self, _g: &G) -> usize {
        // Boost counts a box as its `2^D` corner points
        // (`num_points.hpp:100-101`: `1 << dimension`) — 4 in 2D, 8 in 3D.
        1usize << <G::Point as PointTrait>::DIM
    }
}

impl<G: LinestringTrait> NumPointsStrategy<G> for LinestringNumPoints {
    fn num_points(&self, g: &G) -> usize {
        g.points().count()
    }
}

impl<G: RingTrait> NumPointsStrategy<G> for RingNumPoints {
    fn num_points(&self, g: &G) -> usize {
        g.points().count()
    }
}

impl<G: PolygonTrait> NumPointsStrategy<G> for PolygonNumPoints {
    fn num_points(&self, g: &G) -> usize {
        let mut n = g.exterior().points().count();
        for inner in g.interiors() {
            n += inner.points().count();
        }
        n
    }
}

impl<G: MultiPointTrait> NumPointsStrategy<G> for MultiPointNumPoints {
    fn num_points(&self, g: &G) -> usize {
        g.points().count()
    }
}

impl<G: MultiLinestringTrait> NumPointsStrategy<G> for MultiLinestringNumPoints {
    fn num_points(&self, g: &G) -> usize {
        // Recurse through the public `num_points` on each member — the
        // member is a strictly-smaller kind (Linestring), so
        // monomorphisation terminates on the concrete member type.
        g.linestrings().map(num_points).sum()
    }
}

impl<G: MultiPolygonTrait> NumPointsStrategy<G> for MultiPolygonNumPoints {
    fn num_points(&self, g: &G) -> usize {
        g.polygons().map(num_points).sum()
    }
}

/// Type-level "which `NumPointsStrategy` struct does this geometry *kind*
/// use". One impl per [`geometry_tag`] kind tag, keyed on the tag (never
/// on a concept blanket, which would overlap — E0119), so any
/// concept-adapted foreign type with the same `Kind` resolves to the same
/// struct as the equivalent model value. Users call [`num_points`].
#[doc(hidden)]
pub trait NumPointsStrategyForKind {
    /// The per-kind [`NumPointsStrategy`] struct this tag is counted with.
    type S: Default;
}

impl NumPointsStrategyForKind for PointTag {
    type S = PointNumPoints;
}
impl NumPointsStrategyForKind for SegmentTag {
    type S = SegmentNumPoints;
}
impl NumPointsStrategyForKind for BoxTag {
    type S = BoxNumPoints;
}
impl NumPointsStrategyForKind for LinestringTag {
    type S = LinestringNumPoints;
}
impl NumPointsStrategyForKind for RingTag {
    type S = RingNumPoints;
}
impl NumPointsStrategyForKind for PolygonTag {
    type S = PolygonNumPoints;
}
impl NumPointsStrategyForKind for MultiPointTag {
    type S = MultiPointNumPoints;
}
impl NumPointsStrategyForKind for MultiLinestringTag {
    type S = MultiLinestringNumPoints;
}
impl NumPointsStrategyForKind for MultiPolygonTag {
    type S = MultiPolygonNumPoints;
}

#[cfg(test)]
mod tests {
    //! Reference values from `geometry/test/algorithms/num_points.cpp`.
    //! A box counts as its `2^D` corner points (`num_points.hpp:100-101`)
    //! — 4 for a 2D box.

    use super::num_points;
    use geometry_cs::Cartesian;
    use geometry_model::{
        Box, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Segment,
        linestring, polygon,
    };

    type Pt = Point2D<f64, Cartesian>;
    type Ls = Linestring<Pt>;
    type Poly = Polygon<Pt>;

    /// `num_points.cpp:80` — `POINT(0 0)` → 1.
    #[test]
    fn point_is_one() {
        assert_eq!(num_points(&Pt::new(0.0, 0.0)), 1);
    }

    /// `num_points.cpp:82` — `SEGMENT(0 0,1 1)` → 2.
    #[test]
    fn segment_is_two() {
        let s = Segment::new(Pt::new(0.0, 0.0), Pt::new(1.0, 1.0));
        assert_eq!(num_points(&s), 2);
    }

    /// `num_points.hpp:100-101` — a 2D box has `2^2 = 4` corner points.
    #[test]
    fn box_2d_is_four() {
        let b = Box::from_corners(Pt::new(0.0, 0.0), Pt::new(1.0, 1.0));
        assert_eq!(num_points(&b), 4);
    }

    /// `num_points.cpp:81` uses a 2-point linestring; the 3-point case
    /// here matches the plan's `(0,0),(1,1),(2,2)` fixture → 3.
    #[test]
    fn linestring_three_points() {
        let ls: Ls = linestring![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)];
        assert_eq!(num_points(&ls), 3);
    }

    /// `num_points.cpp:88` — `POLYGON((0 0,10 10,0 10,0 0))` (outer
    /// only, closed) → 4; a 5-point closed outer here → 5.
    #[test]
    fn polygon_outer_only() {
        let pg: Poly = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)]];
        assert_eq!(num_points(&pg), 5);
    }

    /// `num_points.cpp:89` —
    /// `POLYGON((0 0,0 10,10 10,10 0,0 0),(4 4,6 4,6 6,4 6,4 4))` → 10
    /// (5 outer + 5 inner).
    #[test]
    fn polygon_with_hole_adds_inner_count() {
        let pg: Poly = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)],
        ];
        assert_eq!(num_points(&pg), 10);
    }

    /// `num_points.cpp:91` —
    /// `MULTILINESTRING((0 0,1 1),(2 2,3 3,4 4))` → 5 (2 + 3).
    #[test]
    fn multi_linestring_sums_members() {
        let mls: MultiLinestring<Ls> = MultiLinestring(vec![
            linestring![(0.0, 0.0), (1.0, 1.0)],
            linestring![(2.0, 2.0), (3.0, 3.0), (4.0, 4.0)],
        ]);
        assert_eq!(num_points(&mls), 5);
    }

    /// `num_points.cpp:90` — `MULTIPOINT((0 0),(1 1))` → 2.
    #[test]
    fn multi_point_counts_members() {
        let mp = MultiPoint(vec![Pt::new(0.0, 0.0), Pt::new(1.0, 1.0)]);
        assert_eq!(num_points(&mp), 2);
    }

    /// `num_points.cpp:92` —
    /// `MULTIPOLYGON(((0 0,0 10,10 10,10 0,0 0)),((0 10,1 10,1 9,0 10)))`
    /// → 9 (5 + 4).
    #[test]
    fn multi_polygon_sums_members() {
        let mpg: MultiPolygon<Poly> = MultiPolygon(vec![
            polygon![[
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (0.0, 0.0)
            ]],
            polygon![[(0.0, 10.0), (1.0, 10.0), (1.0, 9.0), (0.0, 10.0)]],
        ]);
        assert_eq!(num_points(&mpg), 9);
    }
}
