//! Tests for `register_linestring!`, `register_ring!`, and
//! `register_polygon!`.
//!
//! Mirrors proposal §3.7 (Option D) and the C++ snippets in
//! `boost/geometry/geometries/register/{linestring,ring}.hpp` —
//! the macros stand in for `BOOST_GEOMETRY_REGISTER_LINESTRING` /
//! `BOOST_GEOMETRY_REGISTER_RING`, and `register_polygon!` consolidates
//! the hand-written specialisations from
//! `doc/example_adapting_a_legacy_geometry_object_model.qbk`.

use geometry_adapt::{register_linestring, register_polygon, register_ring};
use geometry_cs::Cartesian;
use geometry_model::Point2D;
use geometry_trait::{
    Closure, Linestring, PointOrder, Polygon, Ring, check_linestring, check_polygon, check_ring,
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
