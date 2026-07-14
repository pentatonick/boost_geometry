//! Public-facade DE-9IM tests across pointlike, linear, and areal pairs.

use boost_geometry::cs::Cartesian;
use boost_geometry::model::{Linestring, Point2D, Polygon, Ring, polygon};
use boost_geometry::overlay::{
    De9im, Dimension, OverlayError, RelateError, crosses, overlaps, relate, relation, touches,
};

type P = Point2D<f64, Cartesian>;

fn square() -> Polygon<P> {
    polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]]
}

/// `test/algorithms/relate/relate_pointlike_geometry.cpp:20-24` — equal and
/// disjoint point pairs.
#[test]
fn point_point_relations_use_point_dimension() {
    let equal = relation(&P::new(1.0, 1.0), &P::new(1.0, 1.0)).unwrap();
    assert_eq!(equal.interior_interior(), Dimension::Point);

    let disjoint = relation(&P::new(1.0, 1.0), &P::new(2.0, 2.0)).unwrap();
    assert_eq!(disjoint.interior_interior(), Dimension::Empty);
    assert_eq!(disjoint.interior_exterior(), Dimension::Point);
    assert_eq!(disjoint.exterior_interior(), Dimension::Point);
}

/// `test/algorithms/relate/relate_pointlike_geometry.cpp:166-174` — interior,
/// boundary, and exterior points against an areal geometry.
#[test]
fn point_polygon_distinguishes_interior_and_boundary() {
    let polygon = square();
    let inside = relation(&P::new(2.0, 2.0), &polygon).unwrap();
    assert_eq!(inside.interior_interior(), Dimension::Point);

    let boundary = relation(&P::new(0.0, 2.0), &polygon).unwrap();
    assert_eq!(boundary.interior_interior(), Dimension::Empty);
    assert_eq!(boundary.m[0][1], Dimension::Point);
    assert!(touches(&P::new(0.0, 2.0), &polygon).unwrap());
    assert!(relate(&P::new(2.0, 2.0), &polygon, "T********").unwrap());

    let reversed = relation(&polygon, &P::new(0.0, 2.0)).unwrap();
    for row in 0..3 {
        for column in 0..3 {
            assert_eq!(boundary.m[row][column], reversed.m[column][row]);
        }
    }
}

/// `test/algorithms/relate/relate_linear_linear.cpp:104-105` — the interiors
/// of two crossing lines meet at a point.
#[test]
fn crossing_linestrings_have_point_interior_intersection() {
    let first = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(4.0, 4.0)]);
    let second = Linestring::from_vec(vec![P::new(0.0, 4.0), P::new(4.0, 0.0)]);
    let matrix = relation(&first, &second).unwrap();
    assert_eq!(matrix.interior_interior(), Dimension::Point);
    assert_eq!(matrix.interior_exterior(), Dimension::Curve);
    assert_eq!(matrix.exterior_interior(), Dimension::Curve);
}

/// `test/algorithms/relate/relate_linear_areal.cpp:44-64` — a line crossing a
/// polygon has a curve in its interior and point boundary intersections.
#[test]
fn linestring_polygon_reports_curve_and_boundary_crossings() {
    let line = Linestring::from_vec(vec![P::new(-1.0, 2.0), P::new(5.0, 2.0)]);
    let matrix = relation(&line, &square()).unwrap();
    assert_eq!(matrix.interior_interior(), Dimension::Curve);
    assert_eq!(matrix.m[0][1], Dimension::Point);
    assert_eq!(matrix.interior_exterior(), Dimension::Curve);
    assert!(crosses(&line, &square()).unwrap());
}

fn box_at(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<P> {
    polygon![[(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]]
}

/// `test/algorithms/relate/relate_areal_areal.cpp:221-239` — colocated edge
/// and vertex clusters carry exact boundary-intersection dimensions.
#[test]
fn areal_touches_distinguish_shared_edges_and_vertices() {
    let first = box_at(0.0, 0.0, 4.0, 4.0);
    let edge_touch = box_at(4.0, 0.0, 8.0, 4.0);
    let edge_matrix = relation(&first, &edge_touch).unwrap();
    assert_eq!(edge_matrix.interior_interior(), Dimension::Empty);
    assert_eq!(edge_matrix.boundary_boundary(), Dimension::Curve);
    assert!(touches(&first, &edge_touch).unwrap());
    assert!(!overlaps(&first, &edge_touch).unwrap());

    let vertex_touch = box_at(4.0, 4.0, 8.0, 8.0);
    let vertex_matrix = relation(&first, &vertex_touch).unwrap();
    assert_eq!(vertex_matrix.interior_interior(), Dimension::Empty);
    assert_eq!(vertex_matrix.boundary_boundary(), Dimension::Point);
    assert!(touches(&first, &vertex_touch).unwrap());
}

/// `test/algorithms/relate/relate_areal_areal.cpp:222-239` — edge-aligned area
/// overlap is distinct from a pure boundary touch.
#[test]
fn collinear_boundaries_can_still_overlap_in_area() {
    let first = box_at(0.0, 0.0, 3.0, 1.0);
    let second = box_at(2.0, 0.0, 5.0, 1.0);
    let matrix = relation(&first, &second).unwrap();
    assert_eq!(matrix.interior_interior(), Dimension::Area);
    assert_eq!(matrix.boundary_boundary(), Dimension::Curve);
    assert!(overlaps(&first, &second).unwrap());
    assert!(!touches(&first, &second).unwrap());
}

/// `test/algorithms/relate/relate_areal_areal.cpp:196-219` — interior rings
/// participate in point-set topology.
#[test]
fn areal_relation_respects_polygon_holes() {
    let donut: Polygon<P> = polygon![
        [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ],
        [(3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0), (3.0, 3.0)]
    ];
    let island = box_at(4.0, 4.0, 6.0, 6.0);
    let matrix = relation(&donut, &island).unwrap();
    assert_eq!(matrix.interior_interior(), Dimension::Empty);
    assert_eq!(matrix.boundary_boundary(), Dimension::Empty);
    assert!(!touches(&donut, &island).unwrap());
    assert!(!overlaps(&donut, &island).unwrap());

    let point_in_hole = relation(&P::new(5.0, 5.0), &donut).unwrap();
    assert_eq!(point_in_hole.interior_interior(), Dimension::Empty);
    assert_eq!(point_in_hole.interior_exterior(), Dimension::Point);
}

/// `test/algorithms/relate/relate_pointlike_geometry.cpp:120-148` — endpoint,
/// interior, exterior, closed, empty, and degenerate linear locations.
#[test]
fn point_linestring_relations_cover_all_linear_locations() {
    let open = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(4.0, 0.0)]);
    let endpoint = relation(&P::new(0.0, 0.0), &open).unwrap();
    assert_eq!(endpoint.m[0][1], Dimension::Point);
    assert_eq!(endpoint.m[2][1], Dimension::Point);

    let interior = relation(&P::new(2.0, 0.0), &open).unwrap();
    assert_eq!(interior.interior_interior(), Dimension::Point);
    let exterior = relation(&P::new(2.0, 1.0), &open).unwrap();
    assert_eq!(exterior.interior_exterior(), Dimension::Point);

    let reversed = relation(&open, &P::new(2.0, 0.0)).unwrap();
    assert_eq!(reversed, interior.transposed());

    let closed = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(2.0, 0.0), P::new(0.0, 0.0)]);
    assert_eq!(
        relation(&P::new(0.0, 0.0), &closed)
            .unwrap()
            .interior_interior(),
        Dimension::Point
    );

    let empty = Linestring::<P>::new();
    assert_eq!(
        relation(&P::new(0.0, 0.0), &empty)
            .unwrap()
            .interior_exterior(),
        Dimension::Point
    );
    let degenerate = Linestring::from_vec(vec![P::new(1.0, 1.0), P::new(1.0, 1.0)]);
    assert_eq!(
        relation(&P::new(1.0, 1.0), &degenerate)
            .unwrap()
            .interior_interior(),
        Dimension::Point
    );
}

/// `test/algorithms/relate/relate_linear_linear.cpp:104-139` — disjoint,
/// endpoint-touching, overlapping, and crossing segment pairs.
#[test]
fn linear_relations_cover_disjoint_touch_and_overlap_kernels() {
    let horizontal = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(4.0, 0.0)]);
    let disjoint = Linestring::from_vec(vec![P::new(0.0, 2.0), P::new(4.0, 2.0)]);
    assert_eq!(
        relation(&horizontal, &disjoint)
            .unwrap()
            .interior_interior(),
        Dimension::Empty
    );

    let touching = Linestring::from_vec(vec![P::new(4.0, 0.0), P::new(5.0, 1.0)]);
    assert_eq!(
        relation(&horizontal, &touching)
            .unwrap()
            .boundary_boundary(),
        Dimension::Point
    );

    let overlapping = Linestring::from_vec(vec![P::new(2.0, 0.0), P::new(6.0, 0.0)]);
    assert_eq!(
        relation(&horizontal, &overlapping)
            .unwrap()
            .interior_interior(),
        Dimension::Curve
    );

    let diagonal = Linestring::from_vec(vec![P::new(0.0, -1.0), P::new(4.0, 1.0)]);
    assert!(crosses(&horizontal, &diagonal).unwrap());
}

/// `test/algorithms/relate/relate_linear_areal.cpp:44-87` — a line on an
/// areal boundary and the reversed ordered pair exercise the boundary mask.
#[test]
fn line_on_polygon_boundary_and_reversed_pair_are_related() {
    let polygon = square();
    let boundary = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(4.0, 0.0)]);
    let matrix = relation(&boundary, &polygon).unwrap();
    assert_eq!(matrix.m[0][1], Dimension::Point);
    assert_eq!(relation(&polygon, &boundary).unwrap(), matrix.transposed());

    let empty_polygon = Polygon::new(Ring::<P>::new());
    assert_eq!(
        relation(&P::new(1.0, 1.0), &empty_polygon)
            .unwrap()
            .interior_exterior(),
        Dimension::Point
    );

    let open_polygon: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(4.0, 0.0),
        P::new(4.0, 4.0),
        P::new(0.0, 4.0),
    ]));
    assert_eq!(
        relation(&P::new(0.0, 2.0), &open_polygon).unwrap().m[0][1],
        Dimension::Point
    );
}

/// `test/algorithms/relate/relate_areal_areal.cpp:76-96` — containment in
/// either order distinguishes which polygon has interior outside the other.
#[test]
fn areal_containment_covers_both_ordered_matrix_arms() {
    let outer = box_at(0.0, 0.0, 10.0, 10.0);
    let inner = box_at(2.0, 2.0, 4.0, 4.0);
    let outer_inner = relation(&outer, &inner).unwrap();
    let inner_outer = relation(&inner, &outer).unwrap();
    assert_eq!(outer_inner, inner_outer.transposed());
    assert_eq!(outer_inner.interior_interior(), Dimension::Area);
    assert_eq!(outer_inner.interior_exterior(), Dimension::Area);
    assert_eq!(inner_outer.exterior_interior(), Dimension::Area);

    let identical = relation(&outer, &outer).unwrap();
    assert_eq!(identical.interior_interior(), Dimension::Area);

    let open_outer: Polygon<P> = Polygon::new(Ring::<P>::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(0.0, 10.0),
        P::new(10.0, 10.0),
        P::new(10.0, 0.0),
    ]));
    assert_eq!(
        relation(&open_outer, &open_outer)
            .unwrap()
            .boundary_boundary(),
        Dimension::Curve
    );
}

#[test]
fn public_matrix_masks_cover_every_symbol_and_overlay_errors() {
    let matrix = De9im {
        m: [
            [Dimension::Empty, Dimension::Point, Dimension::Curve],
            [Dimension::Area, Dimension::Empty, Dimension::Point],
            [Dimension::Curve, Dimension::Area, Dimension::Empty],
        ],
    };
    assert!(matrix.matches("F012F012F").unwrap());
    assert!(matrix.matches("*T*******").unwrap());
    assert_eq!(matrix.matches("F012X012F"), Err(RelateError::InvalidMask));

    let huge = box_at(0.0, 0.0, 200_000_000.0, 200_000_000.0);
    assert_eq!(
        relate(&huge, &square(), "*********"),
        Err(RelateError::Overlay(OverlayError::Unsupported))
    );
}
