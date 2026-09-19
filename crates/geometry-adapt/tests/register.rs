//! Tests for `register_linestring!`, `register_ring!`,
//! `register_polygon!`, and the three multi-geometry registrars.
//!
//! Mirrors proposal §3.7 (Option D) and the C++ snippets in
//! `boost/geometry/geometries/register/{linestring,ring}.hpp` —
//! the macros stand in for `BOOST_GEOMETRY_REGISTER_LINESTRING` /
//! `BOOST_GEOMETRY_REGISTER_RING`, and `register_polygon!` consolidates
//! the hand-written specialisations from
//! `doc/example_adapting_a_legacy_geometry_object_model.qbk`.

use geometry_adapt::{
    register_linestring, register_multi_linestring, register_multi_point, register_multi_polygon,
    register_polygon, register_ring,
};
use geometry_cs::Cartesian;
use geometry_model::Point2D;
use geometry_trait::{
    Closure, Linestring, MultiLinestring, MultiPoint, MultiPolygon, PointOrder, Polygon, Ring,
    check_linestring, check_multi_linestring, check_multi_point, check_multi_polygon,
    check_polygon, check_ring,
};

type P = Point2D<f64, Cartesian>;

// --- register_linestring! ------------------------------------------
// Proposal §3.7, Option D: the canonical "I own MyLineString, I want
// it to be a Linestring" example.

struct MyLineString {
    points: Vec<P>,
}

register_linestring!(MyLineString, P, |s| s.points.iter());

#[test]
fn user_owned_linestring_iterates() {
    let ls = MyLineString {
        points: vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            Point2D::new(2.0, 0.0),
        ],
    };
    assert_eq!(ls.points().count(), 3);
}

#[test]
fn user_owned_linestring_passes_concept_check() {
    check_linestring::<MyLineString>();
}

// --- register_ring! (defaults) -------------------------------------

struct MyRing {
    points: Vec<P>,
}

register_ring!(MyRing, P, |s| s.points.iter());

#[test]
fn user_owned_ring_defaults_are_closed_clockwise() {
    let r = MyRing {
        points: vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(1.0, 1.0),
            Point2D::new(0.0, 1.0),
            Point2D::new(0.0, 0.0),
        ],
    };
    assert_eq!(r.closure(), Closure::Closed);
    assert_eq!(r.point_order(), PointOrder::Clockwise);
    assert_eq!(r.points().count(), 5);
}

#[test]
fn user_owned_ring_passes_concept_check() {
    check_ring::<MyRing>();
}

// --- register_ring! (overridden closure + point_order) -------------

struct OpenCcwRing {
    points: Vec<P>,
}

register_ring!(
    OpenCcwRing,
    P,
    |s| s.points.iter(),
    closure = Closure::Open,
    point_order = PointOrder::CounterClockwise
);

#[test]
fn user_owned_ring_overrides_take_effect() {
    let r = OpenCcwRing {
        points: vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(0.0, 1.0),
        ],
    };
    assert_eq!(r.closure(), Closure::Open);
    assert_eq!(r.point_order(), PointOrder::CounterClockwise);
    assert_eq!(r.points().count(), 3);
}

// --- register_polygon! ---------------------------------------------

struct MyPoly {
    outer: MyRing,
    inners: Vec<MyRing>,
}

register_polygon!(
    MyPoly,
    P,
    ring = MyRing,
    |s| outer = &s.outer,
    inners = s.inners.iter()
);

#[test]
fn user_owned_polygon_exposes_outer_and_inner_rings() {
    let poly = MyPoly {
        outer: MyRing {
            points: vec![
                Point2D::new(0.0, 0.0),
                Point2D::new(10.0, 0.0),
                Point2D::new(10.0, 10.0),
                Point2D::new(0.0, 10.0),
                Point2D::new(0.0, 0.0),
            ],
        },
        inners: vec![
            MyRing {
                points: vec![
                    Point2D::new(1.0, 1.0),
                    Point2D::new(2.0, 1.0),
                    Point2D::new(2.0, 2.0),
                    Point2D::new(1.0, 2.0),
                    Point2D::new(1.0, 1.0),
                ],
            },
            MyRing {
                points: vec![
                    Point2D::new(5.0, 5.0),
                    Point2D::new(6.0, 5.0),
                    Point2D::new(6.0, 6.0),
                    Point2D::new(5.0, 6.0),
                    Point2D::new(5.0, 5.0),
                ],
            },
        ],
    };

    assert_eq!(poly.exterior().points().count(), 5);
    assert_eq!(poly.interiors().count(), 2);

    let inner_counts: Vec<usize> = poly.interiors().map(|r| r.points().count()).collect();
    assert_eq!(inner_counts, vec![5, 5]);
}

#[test]
fn user_owned_polygon_passes_concept_check() {
    check_polygon::<MyPoly>();
}

// --- register_multi_point! -----------------------------------------
// Mirrors `BOOST_GEOMETRY_REGISTER_MULTI_POINT` from
// `geometries/register/multi_point.hpp`. Unlike the Boost macro, the
// item type is explicit because Rust has no Boost.Range container base.

struct MyMultiPoint {
    points: Vec<P>,
}

register_multi_point!(MyMultiPoint, P, |s| s.points.iter());

#[test]
fn user_owned_multi_point_iterates_its_members() {
    let mp = MyMultiPoint {
        points: vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
            Point2D::new(2.0, 4.0),
        ],
    };
    assert_eq!(mp.points().count(), 3);
    // Both ordinates, in storage order: a count alone would pass for an
    // iterator that yielded the right number of the wrong points, and an
    // x-only check would miss a transposition.
    let coords: Vec<(f64, f64)> = mp.points().map(|p| (p.x(), p.y())).collect();
    assert_eq!(coords, vec![(0.0, 0.0), (1.0, 1.0), (2.0, 4.0)]);
}

/// The generated iterator is `ExactSizeIterator`, which is what lets a
/// consumer reserve capacity before walking the members. The macro
/// promises this in its return type; an iterator expression that only
/// satisfied `Iterator` would not compile against it.
#[test]
fn user_owned_multi_point_iterator_reports_its_length() {
    let mp = MyMultiPoint {
        points: vec![Point2D::new(0.0, 0.0), Point2D::new(1.0, 1.0)],
    };
    assert_eq!(mp.points().len(), 2);
}

#[test]
fn user_owned_multi_point_is_empty_when_its_storage_is() {
    let mp = MyMultiPoint { points: vec![] };
    assert_eq!(mp.points().count(), 0);
    assert_eq!(mp.points().len(), 0);
}

#[test]
fn user_owned_multi_point_passes_concept_check() {
    check_multi_point::<MyMultiPoint>();
}

// --- register_multi_linestring! ------------------------------------
// Mirrors `BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING`. The member type
// is the `MyLineString` registered above, so this also pins that a
// macro-registered type composes as another macro's item type.

struct MyMultiLineString {
    members: Vec<MyLineString>,
}

register_multi_linestring!(MyMultiLineString, P, item = MyLineString, |s| s
    .members
    .iter());

#[test]
fn user_owned_multi_linestring_exposes_its_members() {
    let ml = MyMultiLineString {
        members: vec![
            MyLineString {
                points: vec![Point2D::new(0.0, 0.0), Point2D::new(1.0, 1.0)],
            },
            MyLineString {
                points: vec![
                    Point2D::new(2.0, 2.0),
                    Point2D::new(3.0, 3.0),
                    Point2D::new(4.0, 4.0),
                ],
            },
        ],
    };
    assert_eq!(ml.linestrings().len(), 2);
    // Reaching through to each member's own points proves the item type
    // arrived as a `Linestring`, not merely as an opaque element.
    let counts: Vec<usize> = ml.linestrings().map(|l| l.points().count()).collect();
    assert_eq!(counts, vec![2, 3]);
}

#[test]
fn user_owned_multi_linestring_is_empty_when_its_storage_is() {
    let ml = MyMultiLineString { members: vec![] };
    assert_eq!(ml.linestrings().len(), 0);
}

#[test]
fn user_owned_multi_linestring_passes_concept_check() {
    check_multi_linestring::<MyMultiLineString>();
}

// --- register_multi_polygon! ---------------------------------------
// Mirrors `BOOST_GEOMETRY_REGISTER_MULTI_POLYGON`, over the `MyPoly`
// registered above — so a member carries interior rings too.

struct MyMultiPoly {
    members: Vec<MyPoly>,
}

register_multi_polygon!(MyMultiPoly, P, item = MyPoly, |s| s.members.iter());

fn unit_ring(offset: f64) -> MyRing {
    MyRing {
        points: vec![
            Point2D::new(offset, offset),
            Point2D::new(offset + 1.0, offset),
            Point2D::new(offset + 1.0, offset + 1.0),
            Point2D::new(offset, offset + 1.0),
            Point2D::new(offset, offset),
        ],
    }
}

#[test]
fn user_owned_multi_polygon_exposes_members_and_their_rings() {
    let mp = MyMultiPoly {
        members: vec![
            MyPoly {
                outer: unit_ring(0.0),
                inners: vec![],
            },
            MyPoly {
                outer: unit_ring(10.0),
                inners: vec![unit_ring(11.0), unit_ring(12.0)],
            },
        ],
    };
    assert_eq!(mp.polygons().len(), 2);
    // Each member must arrive as a `Polygon`, exterior and interiors
    // intact — the second one is the case that would survive a macro
    // that dropped interior rings.
    let interior_counts: Vec<usize> = mp.polygons().map(|p| p.interiors().count()).collect();
    assert_eq!(interior_counts, vec![0, 2]);
    assert_eq!(
        mp.polygons()
            .map(|p| p.exterior().points().count())
            .collect::<Vec<usize>>(),
        vec![5, 5]
    );
}

#[test]
fn user_owned_multi_polygon_is_empty_when_its_storage_is() {
    let mp = MyMultiPoly { members: vec![] };
    assert_eq!(mp.polygons().len(), 0);
}

#[test]
fn user_owned_multi_polygon_passes_concept_check() {
    check_multi_polygon::<MyMultiPoly>();
}
