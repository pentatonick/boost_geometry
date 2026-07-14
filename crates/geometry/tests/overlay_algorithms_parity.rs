//! Public-facade tests for overlay-dependent algorithm entry points.

use boost_geometry::model::{Point2D, Polygon, Ring};
use boost_geometry::prelude::{
    Cartesian, Dimension, JoinStrategy, PointStrategy, RelateError, ValidityFailure, buffer,
    is_valid, merge_elements, relate, relation, r#union,
};
use boost_geometry::trait_::MultiPolygon as _;

type P = Point2D<f64, Cartesian>;

fn square(x: f64, y: f64, size: f64) -> Polygon<P> {
    Polygon::new(Ring::from_vec(vec![
        P::new(x, y),
        P::new(x, y + size),
        P::new(x + size, y + size),
        P::new(x + size, y),
        P::new(x, y),
    ]))
}

/// `test/algorithms/overlay/overlay.cpp:376-384` — overlapping areal union.
#[test]
fn canonical_union_is_available_from_the_facade() {
    let output = r#union(&square(0.0, 0.0, 2.0), &square(1.0, 1.0, 2.0)).unwrap();
    assert_eq!(output.polygons().count(), 1);
}

/// `test/algorithms/relate/relate_areal_areal.cpp:63-75` — relation returns
/// the matrix while relate evaluates a DE-9IM mask.
#[test]
fn relation_matrix_and_relate_mask_are_distinct_public_entries() {
    let a = square(0.0, 0.0, 2.0);
    let b = square(1.0, 1.0, 2.0);

    let matrix = relation(&a, &b).unwrap();
    assert_eq!(matrix.interior_interior(), Dimension::Area);
    assert!(relate(&a, &b, "T*T***T**").unwrap());
    assert!(!relate(&a, &b, "FF*FF****").unwrap());
    assert_eq!(relate(&a, &b, "too-short"), Err(RelateError::InvalidMask));
}

/// `test/algorithms/is_valid.cpp:1626-1634` — the generic entry dispatches to
/// a polygon validator and reports Boost's strict-policy duplicate category.
#[test]
fn generic_is_valid_reports_duplicate_points() {
    assert!(is_valid(&square(0.0, 0.0, 2.0)).is_ok());

    let duplicate: Polygon<P> = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(0.0, 2.0),
        P::new(0.0, 2.0),
        P::new(2.0, 2.0),
        P::new(2.0, 0.0),
        P::new(0.0, 0.0),
    ]));
    assert_eq!(is_valid(&duplicate), Err(ValidityFailure::DuplicatePoints));
}

/// `test/algorithms/buffer/buffer_point.cpp:13-29` and
/// `test/algorithms/buffer/buffer_polygon.cpp:266-285` — one public entry
/// dispatches by geometry kind.
#[test]
fn generic_buffer_dispatches_for_points_and_polygons() {
    let point_buffer = buffer(
        &P::new(0.0, 0.0),
        1.0,
        JoinStrategy::Miter,
        PointStrategy::Square,
    )
    .unwrap();
    assert_eq!(point_buffer.polygons().count(), 1);

    let polygon_buffer = buffer(
        &square(0.0, 0.0, 2.0),
        1.0,
        JoinStrategy::Round {
            points_per_circle: 72,
        },
        PointStrategy::Square,
    )
    .unwrap();
    assert_eq!(polygon_buffer.polygons().count(), 1);
}

/// `test/algorithms/merge_elements.cpp:41-71` — overlapping areal elements
/// coalesce while a disjoint element remains separate.
#[test]
fn merge_elements_exposes_the_areal_collection_entry() {
    let merged = merge_elements(vec![
        square(0.0, 0.0, 2.0),
        square(1.0, 1.0, 2.0),
        square(10.0, 10.0, 1.0),
    ])
    .unwrap();

    assert_eq!(merged.polygons().count(), 2);
}
