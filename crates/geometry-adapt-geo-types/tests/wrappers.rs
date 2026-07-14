//! Direct exercise of the `geo-types` wrapper surface that the
//! algorithm-driven `interop.rs` milestone does not reach: the point
//! adapters ([`GeoCoord`], [`GeoPoint`]), the indexed adapters
//! ([`GeoLine`], [`GeoRect`]), the multi-container adapters
//! ([`GeoMultiPoint`], [`GeoMultiLineString`]), and the
//! [`DynGeometry`] interop in [`dyn_conversion`].
//!
//! Each test states the wrapper property it proves: the concept
//! accessors surface the correct ordinates, `new`/`into_inner` is a
//! lossless round trip, `PointMut`/`set_indexed` writes land in the
//! right slot, and the `DynGeometry` conversion preserves coordinates
//! while normalising the `Line`/`Rect`/`Triangle` kinds.

#![allow(
    clippy::float_cmp,
    reason = "all compared values are exact integer-valued ordinates, so `==` is exact"
)]

use geo_types::{
    Coord, Geometry as GtGeometry, GeometryCollection as GtGeometryCollection, Line, LineString,
    MultiLineString as GtMultiLineString, MultiPoint as GtMultiPoint,
    MultiPolygon as GtMultiPolygon, Point as GtPoint, Polygon as GtPolygon, Rect, Triangle, coord,
};
use geometry_adapt_geo_types::dyn_conversion::{from_dyn_geometry, to_dyn_geometry};
use geometry_adapt_geo_types::{
    GeoCollection, GeoCoord, GeoLine, GeoLineString, GeoMultiLineString, GeoMultiPoint, GeoPoint,
    GeoRect,
};
use geometry_cs::Cartesian;
use geometry_model::{DynGeometry, DynKind, Point2D};
use geometry_trait::{
    GeometryCollection as _, IndexedAccess as _, Linestring as _, MultiLinestring as _,
    MultiPoint as _, Point as _, PointMut as _, segment_end, segment_start,
};

type Kp = Point2D<f64, Cartesian>;

// ---- GeoCoord: point accessor, mutation, round trip -----------------

/// `GeoCoord::get::<0/1>` surfaces `x`/`y`, and `DIM` is pinned to 2.
#[test]
fn geo_coord_get_returns_x_then_y() {
    let c = GeoCoord::new(coord! { x: 3.0_f64, y: 4.0 });
    assert_eq!(c.get::<0>(), 3.0);
    assert_eq!(c.get::<1>(), 4.0);
    assert_eq!(<GeoCoord<f64> as geometry_trait::Point>::DIM, 2);
}

/// `PointMut::set::<0/1>` writes the corresponding ordinate and nothing
/// else.
#[test]
fn geo_coord_set_writes_the_addressed_ordinate() {
    let mut c = GeoCoord::new(coord! { x: 0.0_f64, y: 0.0 });
    c.set::<0>(5.0);
    assert_eq!((c.get::<0>(), c.get::<1>()), (5.0, 0.0));
    c.set::<1>(7.0);
    assert_eq!((c.get::<0>(), c.get::<1>()), (5.0, 7.0));
}

/// `From`/`into_inner` and `Deref` round-trip a `geo_types::Coord`
/// losslessly and expose the inner fields.
#[test]
fn geo_coord_round_trips_and_derefs() {
    let inner = coord! { x: 1.5_f64, y: -2.5 };
    let wrapped: GeoCoord<f64> = inner.into();
    assert_eq!(wrapped.x, 1.5); // via Deref
    let back: Coord<f64> = wrapped.into();
    assert_eq!(back, inner);
}

/// `DerefMut` mutates the underlying `geo_types::Coord` in place.
#[test]
fn geo_coord_deref_mut_mutates_inner() {
    let mut c = GeoCoord::new(coord! { x: 1.0_f64, y: 1.0 });
    c.x = 9.0; // via DerefMut
    assert_eq!(c.into_inner().x, 9.0);
}

// ---- GeoPoint: the double-newtype get/set path ----------------------

/// `GeoPoint::get::<0/1>` reaches through `Point → Coord` (`self.0.0.x`)
/// and returns the right ordinate — the layer that array-based tests
/// never touch.
#[test]
fn geo_point_get_reaches_through_the_inner_coord() {
    let p = GeoPoint::new(GtPoint::new(3.0_f64, 4.0));
    assert_eq!(p.get::<0>(), 3.0);
    assert_eq!(p.get::<1>(), 4.0);
}

/// `GeoPoint`'s `PointMut::set` writes through both newtype layers.
#[test]
fn geo_point_set_writes_through_both_layers() {
    let mut p = GeoPoint::new(GtPoint::new(0.0_f64, 0.0));
    p.set::<0>(8.0);
    p.set::<1>(9.0);
    let inner = p.into_inner();
    assert_eq!((inner.x(), inner.y()), (8.0, 9.0));
}

/// `From`/`into_inner` round-trips a `geo_types::Point` losslessly.
#[test]
fn geo_point_round_trips() {
    let inner = GtPoint::new(2.0_f64, -3.0);
    let wrapped: GeoPoint<f64> = inner.into();
    let back: GtPoint<f64> = wrapped.into();
    assert_eq!(back, inner);
}

/// `Deref` exposes the inner `geo_types::Point`'s own methods, and
/// `DerefMut` writes land in the wrapped value.
#[test]
fn geo_point_deref_and_deref_mut() {
    let mut p = GeoPoint::new(GtPoint::new(1.0_f64, 2.0));
    // Deref: call a geo_types::Point method directly on the wrapper.
    assert_eq!(p.x(), 1.0);
    assert_eq!(p.y(), 2.0);
    // DerefMut: mutate through the wrapper.
    p.set_x(9.0);
    assert_eq!(p.into_inner(), GtPoint::new(9.0_f64, 2.0));
}

// ---- GeoLine: Segment via IndexedAccess -----------------------------

/// `GeoLine` presents endpoint `0` as `start` and `1` as `end` through
/// both `get_indexed` and the `segment_start`/`segment_end` helpers.
#[test]
fn geo_line_endpoints_map_start_and_end() {
    let line = GeoLine::new(Line::new((0.0_f64, 1.0), (2.0, 3.0)));
    assert_eq!(line.get_indexed::<0, 0>(), 0.0);
    assert_eq!(line.get_indexed::<0, 1>(), 1.0);
    assert_eq!(line.get_indexed::<1, 0>(), 2.0);
    assert_eq!(line.get_indexed::<1, 1>(), 3.0);

    let a = segment_start(&line);
    let b = segment_end(&line);
    assert_eq!((a.get::<0>(), a.get::<1>()), (0.0, 1.0));
    assert_eq!((b.get::<0>(), b.get::<1>()), (2.0, 3.0));
}

/// `GeoLine::set_indexed` writes into the addressed endpoint/ordinate
/// and `into_inner` reflects the mutation exactly.
#[test]
fn geo_line_set_indexed_mutates_addressed_slot() {
    let mut line = GeoLine::new(Line::new((0.0_f64, 0.0), (0.0, 0.0)));
    line.set_indexed::<0, 0>(1.0);
    line.set_indexed::<0, 1>(2.0);
    line.set_indexed::<1, 0>(3.0);
    line.set_indexed::<1, 1>(4.0);
    let inner = line.into_inner();
    assert_eq!(inner.start, coord! { x: 1.0, y: 2.0 });
    assert_eq!(inner.end, coord! { x: 3.0, y: 4.0 });
}

/// `GeoLine::new`/`into_inner` round-trips a `geo_types::Line`.
#[test]
fn geo_line_round_trips() {
    let original = Line::new((0.0_f64, 0.0), (3.0, 4.0));
    assert_eq!(GeoLine::new(original).into_inner(), original);
}

// ---- GeoRect: Box via IndexedAccess ---------------------------------

/// `GeoRect` presents corner `0` as the min corner and `1` as the max,
/// each with its `x`/`y` ordinate.
#[test]
fn geo_rect_corners_map_min_and_max() {
    let rect = GeoRect::new(Rect::new((0.0_f64, 1.0), (3.0, 4.0)));
    assert_eq!(rect.get_indexed::<0, 0>(), 0.0);
    assert_eq!(rect.get_indexed::<0, 1>(), 1.0);
    assert_eq!(rect.get_indexed::<1, 0>(), 3.0);
    assert_eq!(rect.get_indexed::<1, 1>(), 4.0);
}

/// `GeoRect::set_indexed` updates one ordinate of one corner while
/// leaving the other three fixed; the reconstructed `Rect` reflects it.
#[test]
fn geo_rect_set_indexed_updates_one_ordinate() {
    let mut rect = GeoRect::new(Rect::new((0.0_f64, 0.0), (2.0, 2.0)));
    // Widen the max corner's x from 2 to 5; leave its y and both min
    // ordinates alone.
    rect.set_indexed::<1, 0>(5.0);
    assert_eq!(rect.get_indexed::<1, 0>(), 5.0);
    assert_eq!(rect.get_indexed::<1, 1>(), 2.0);
    assert_eq!(rect.get_indexed::<0, 0>(), 0.0);
    let inner = rect.into_inner();
    assert_eq!(inner.min(), coord! { x: 0.0, y: 0.0 });
    assert_eq!(inner.max(), coord! { x: 5.0, y: 2.0 });
}

/// `GeoRect::new`/`into_inner` round-trips a `geo_types::Rect`.
#[test]
fn geo_rect_round_trips() {
    let original = Rect::new((0.0_f64, 0.0), (3.0, 4.0));
    assert_eq!(GeoRect::new(original).into_inner(), original);
}

// ---- Multi-container concept iterators + round trips ----------------

/// `GeoMultiPoint::points` yields each member in declared order with
/// the correct ordinates, and `into_inner` reproduces the input.
#[test]
fn geo_multi_point_iterates_and_round_trips() {
    let original = GtMultiPoint::from(vec![(0.0_f64, 0.0), (1.0, 2.0), (3.0, 4.0)]);
    let wrapped = GeoMultiPoint::new(original.clone());
    let read: Vec<(f64, f64)> = wrapped
        .points()
        .map(|p| (p.get::<0>(), p.get::<1>()))
        .collect();
    assert_eq!(read, vec![(0.0, 0.0), (1.0, 2.0), (3.0, 4.0)]);
    assert_eq!(wrapped.points().len(), 3);
    assert_eq!(wrapped.into_inner(), original);
}

/// `GeoMultiLineString::linestrings` yields each member line string in
/// order, each preserving its own vertices, and round-trips.
#[test]
fn geo_multi_line_string_iterates_and_round_trips() {
    let original = GtMultiLineString::new(vec![
        LineString::from(vec![(0.0_f64, 0.0), (1.0, 1.0)]),
        LineString::from(vec![(2.0, 2.0), (3.0, 3.0), (4.0, 4.0)]),
    ]);
    let wrapped = GeoMultiLineString::new(original.clone());
    let lens: Vec<usize> = wrapped
        .linestrings()
        .map(|ls| ls.points().count())
        .collect();
    assert_eq!(lens, vec![2, 3]);
    assert_eq!(wrapped.linestrings().len(), 2);
    // First member's first vertex is (0,0), last member's last is (4,4).
    let first = wrapped.linestrings().next().unwrap();
    let first_v = first.points().next().unwrap();
    assert_eq!((first_v.get::<0>(), first_v.get::<1>()), (0.0, 0.0));
    assert_eq!(wrapped.into_inner(), original);
}

/// A `GeoLineString` reconstructed from a wrapped `LineString` keeps its
/// vertices in order — the coordinate-copy construction is lossless.
#[test]
fn geo_line_string_preserves_vertices() {
    let original = LineString::from(vec![(0.0_f64, 0.0), (3.0, 4.0), (6.0, 8.0)]);
    let wrapped = GeoLineString::new(original.clone());
    let read: Vec<(f64, f64)> = wrapped
        .points()
        .map(|c| (c.get::<0>(), c.get::<1>()))
        .collect();
    assert_eq!(read, vec![(0.0, 0.0), (3.0, 4.0), (6.0, 8.0)]);
    assert_eq!(wrapped.into_inner(), original);
}

// ---- dyn_conversion: to_dyn_geometry kinds --------------------------

/// Every non-normalised `geo_types::Geometry` variant maps to the
/// matching `DynKind`.
#[test]
fn to_dyn_geometry_maps_each_kind() {
    let cases: Vec<(GtGeometry<f64>, DynKind)> = vec![
        (GtPoint::new(1.0, 2.0).into(), DynKind::Point),
        (
            LineString::from(vec![(0.0, 0.0), (1.0, 1.0)]).into(),
            DynKind::LineString,
        ),
        (
            GtPolygon::new(
                LineString::from(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]),
                vec![],
            )
            .into(),
            DynKind::Polygon,
        ),
        (
            GtMultiPoint::from(vec![(0.0, 0.0)]).into(),
            DynKind::MultiPoint,
        ),
        (
            GtMultiLineString::new(vec![LineString::from(vec![(0.0, 0.0), (1.0, 1.0)])]).into(),
            DynKind::MultiLineString,
        ),
        (
            GtMultiPolygon::new(vec![GtPolygon::new(
                LineString::from(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]),
                vec![],
            )])
            .into(),
            DynKind::MultiPolygon,
        ),
        (
            GtGeometry::GeometryCollection(GtGeometryCollection(vec![
                GtPoint::new(0.0, 0.0).into(),
            ])),
            DynKind::GeometryCollection,
        ),
    ];
    for (g, want) in cases {
        assert_eq!(to_dyn_geometry(g).kind(), want);
    }
}

/// A `geo_types::Line` normalises to a two-vertex `DynGeometry::LineString`
/// carrying the line's endpoints in order.
#[test]
fn to_dyn_geometry_normalises_line_to_linestring() {
    let g: GtGeometry<f64> = Line::new((1.0, 2.0), (3.0, 4.0)).into();
    let dyn_g = to_dyn_geometry(g);
    assert_eq!(dyn_g.kind(), DynKind::LineString);
    match dyn_g {
        DynGeometry::LineString(ls) => {
            assert_eq!(ls.0.len(), 2);
            assert_eq!((ls.0[0].get::<0>(), ls.0[0].get::<1>()), (1.0, 2.0));
            assert_eq!((ls.0[1].get::<0>(), ls.0[1].get::<1>()), (3.0, 4.0));
        }
        _ => panic!("expected LineString"),
    }
}

/// A `geo_types::Rect` normalises to a `DynGeometry::Polygon` whose
/// exterior ring spans the rectangle's corners.
#[test]
fn to_dyn_geometry_normalises_rect_to_polygon() {
    let g: GtGeometry<f64> = Rect::new((0.0, 0.0), (2.0, 3.0)).into();
    let dyn_g = to_dyn_geometry(g);
    assert_eq!(dyn_g.kind(), DynKind::Polygon);
    match dyn_g {
        DynGeometry::Polygon(p) => {
            // Rect::to_polygon emits a closed 5-vertex ring; its x-range
            // is [0,2] and y-range [0,3].
            let xs: Vec<f64> = p
                .outer
                .0
                .iter()
                .map(geometry_trait::Point::get::<0>)
                .collect();
            let ys: Vec<f64> = p
                .outer
                .0
                .iter()
                .map(geometry_trait::Point::get::<1>)
                .collect();
            assert_eq!(xs.iter().copied().fold(f64::INFINITY, f64::min), 0.0);
            assert_eq!(xs.iter().copied().fold(f64::NEG_INFINITY, f64::max), 2.0);
            assert_eq!(ys.iter().copied().fold(f64::INFINITY, f64::min), 0.0);
            assert_eq!(ys.iter().copied().fold(f64::NEG_INFINITY, f64::max), 3.0);
        }
        _ => panic!("expected Polygon"),
    }
}

/// A `geo_types::Triangle` normalises to a `DynGeometry::Polygon` whose
/// ring carries the three triangle vertices.
#[test]
fn to_dyn_geometry_normalises_triangle_to_polygon() {
    let g: GtGeometry<f64> = Triangle::new(
        coord! {x: 0.0, y: 0.0},
        coord! {x: 4.0, y: 0.0},
        coord! {x: 0.0, y: 3.0},
    )
    .into();
    let dyn_g = to_dyn_geometry(g);
    assert_eq!(dyn_g.kind(), DynKind::Polygon);
    match dyn_g {
        DynGeometry::Polygon(p) => {
            // The three distinct vertices are present in the ring.
            let verts: Vec<(f64, f64)> = p
                .outer
                .0
                .iter()
                .map(|pt| (pt.get::<0>(), pt.get::<1>()))
                .collect();
            assert!(verts.contains(&(0.0, 0.0)));
            assert!(verts.contains(&(4.0, 0.0)));
            assert!(verts.contains(&(0.0, 3.0)));
        }
        _ => panic!("expected Polygon"),
    }
}

// ---- dyn_conversion: round trips for the modelled kinds -------------

/// Each of the seven `DynGeometry`-modelled kinds round-trips through
/// `from_dyn_geometry` → `to_dyn_geometry` back to an equal value.
#[test]
fn dyn_geometry_round_trips_each_modelled_kind() {
    let ls = geometry_model::Linestring::from_vec(vec![Kp::new(0.0, 0.0), Kp::new(1.0, 1.0)]);
    let ring = geometry_model::Ring::from_vec(vec![
        Kp::new(0.0, 0.0),
        Kp::new(1.0, 0.0),
        Kp::new(1.0, 1.0),
        Kp::new(0.0, 0.0),
    ]);
    let poly = geometry_model::Polygon::with_inners(ring.clone(), vec![]);

    let cases: Vec<DynGeometry<f64, Cartesian>> = vec![
        DynGeometry::Point(Kp::new(1.0, 2.0)),
        DynGeometry::LineString(ls.clone()),
        DynGeometry::Polygon(poly.clone()),
        DynGeometry::MultiPoint(geometry_model::MultiPoint::from_vec(vec![
            Kp::new(0.0, 0.0),
            Kp::new(5.0, 6.0),
        ])),
        DynGeometry::MultiLineString(geometry_model::MultiLinestring::from_vec(vec![ls.clone()])),
        DynGeometry::MultiPolygon(geometry_model::MultiPolygon::from_vec(vec![poly.clone()])),
        DynGeometry::GeometryCollection(vec![
            DynGeometry::Point(Kp::new(7.0, 8.0)),
            DynGeometry::LineString(ls.clone()),
        ]),
    ];

    for original in cases {
        let round_tripped = to_dyn_geometry(from_dyn_geometry(original.clone()));
        assert_eq!(round_tripped, original);
    }
}

/// A nested `GeometryCollection` recurses through both directions,
/// preserving structure and coordinates.
#[test]
fn dyn_geometry_nested_collection_round_trips() {
    let inner = DynGeometry::GeometryCollection(vec![DynGeometry::Point(Kp::new(1.0, 1.0))]);
    let outer = DynGeometry::<f64, Cartesian>::GeometryCollection(vec![
        DynGeometry::Point(Kp::new(0.0, 0.0)),
        inner,
    ]);
    let round_tripped = to_dyn_geometry(from_dyn_geometry(outer.clone()));
    assert_eq!(round_tripped, outer);
}

/// `from_dyn_geometry(DynGeometry::Point)` produces the matching
/// `geo_types::Point` geometry, exactly.
#[test]
fn from_dyn_geometry_point_is_exact() {
    let g = from_dyn_geometry(DynGeometry::<f64, Cartesian>::Point(Kp::new(1.0, 2.0)));
    assert_eq!(g, GtGeometry::Point(GtPoint::new(1.0, 2.0)));
}

// ---- GeoCollection wrapper ------------------------------------------

/// `GeoCollection` converts each `geo_types::Geometry` member into a
/// kernel `DynGeometry`, exposes them in order through `items()`, and
/// round-trips back to an equal `GeometryCollection` (the members here —
/// a point and a line string — are not subject to the Line/Rect/Triangle
/// kind-normalisation, so the round trip is exact).
#[test]
fn geo_collection_iterates_and_round_trips() {
    let original = GtGeometryCollection(vec![
        GtGeometry::Point(GtPoint::new(1.0_f64, 2.0)),
        GtGeometry::LineString(LineString::from(vec![(0.0, 0.0), (1.0, 1.0)])),
    ]);
    let gc = GeoCollection::new(original.clone());

    // `items()` yields one `DynGeometry` per member, in declared order.
    let kinds: Vec<DynKind> = gc.items().map(DynGeometry::kind).collect();
    assert_eq!(kinds, vec![DynKind::Point, DynKind::LineString]);
    assert_eq!(gc.items().count(), 2);

    // Round trip is exact for these (non-normalised) member kinds.
    assert_eq!(gc.into_inner(), original);
}

// ---- GeoRing / GeoPolygon / GeoMultiPolygon: into_inner round trips --

/// `GeoRing::new`/`into_inner` round-trips a `geo_types::LineString`
/// (the ring's backing representation) losslessly.
#[test]
fn geo_ring_round_trips() {
    use geometry_adapt_geo_types::GeoRing;
    let original = LineString::from(vec![(0.0_f64, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 0.0)]);
    let ring = GeoRing::new(original.clone());
    assert_eq!(ring.into_inner(), original);
}

/// `GeoPolygon::new`/`into_inner` round-trips a polygon *with a hole*:
/// the interior rings are rebuilt too.
#[test]
fn geo_polygon_with_hole_round_trips() {
    use geometry_adapt_geo_types::GeoPolygon;
    let exterior = LineString::from(vec![
        (0.0_f64, 0.0),
        (10.0, 0.0),
        (10.0, 10.0),
        (0.0, 10.0),
        (0.0, 0.0),
    ]);
    let hole = LineString::from(vec![(2.0_f64, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 2.0)]);
    let original = GtPolygon::new(exterior, vec![hole]);
    let poly = GeoPolygon::new(original.clone());
    let back = poly.into_inner();
    assert_eq!(back, original);
    assert_eq!(back.interiors().len(), 1);
}

/// `GeoMultiPolygon::new`/`into_inner` round-trips every member.
#[test]
fn geo_multi_polygon_round_trips() {
    use geometry_adapt_geo_types::GeoMultiPolygon;
    let a = GtPolygon::new(
        LineString::from(vec![(0.0_f64, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]),
        vec![],
    );
    let b = GtPolygon::new(
        LineString::from(vec![(5.0_f64, 5.0), (6.0, 5.0), (6.0, 6.0), (5.0, 5.0)]),
        vec![],
    );
    let original = GtMultiPolygon::new(vec![a, b]);
    let mpg = GeoMultiPolygon::new(original.clone());
    assert_eq!(mpg.into_inner(), original);
}
