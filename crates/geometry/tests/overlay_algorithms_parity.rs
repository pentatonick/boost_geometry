//! Public-facade tests for overlay-dependent algorithm entry points.

use boost_geometry::model::{Point2D, Polygon, Ring, Segment};
use boost_geometry::overlay::{
    OverlayError,
    assemble::assemble_multipolygon,
    predicate::SegmentIntersection,
    traverse::{EnrichedRings, OverlayOp, TraversalError, enrich, enrich::Node, traverse},
    turn::{Method, Operation, OperationType, RingKind, SegmentId, Turn},
};
use boost_geometry::prelude::{
    Cartesian, Dimension, JoinStrategy, LineIntersection, PointStrategy, RelateError,
    ValidityFailure, buffer, contains_properly, is_valid, line_intersection, merge_elements,
    relate, relation, ring_area, stitch_triangles, r#union,
};
use boost_geometry::trait_::{MultiPolygon as _, Polygon as _};

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

fn traversal_square(x: f64, y: f64, size: f64) -> Ring<P> {
    Ring::from_vec(vec![
        P::new(x, y),
        P::new(x + size, y),
        P::new(x + size, y + size),
        P::new(x, y + size),
        P::new(x, y),
    ])
}

fn turn_operation(source_index: usize, segment_index: usize) -> Operation {
    Operation::new(SegmentId {
        source_index,
        ring: RingKind::Exterior,
        segment_index,
    })
}

/// `test/algorithms/overlay/overlay.cpp:376-384` — overlapping areal union.
#[test]
fn canonical_union_is_available_from_the_facade() {
    let output = r#union(&square(0.0, 0.0, 2.0), &square(1.0, 1.0, 2.0)).unwrap();
    assert_eq!(output.polygons().count(), 1);
}

/// Assembly classifies rings but does not validate them. An empty input ring
/// therefore remains an empty component instead of being silently discarded.
#[test]
fn empty_ring_assembly_uses_the_public_facade() {
    let assembled = assemble_multipolygon(vec![Ring::<P>::from_vec(Vec::new())]);
    assert_eq!(assembled.polygons().count(), 1);
    assert_eq!(assembled.polygons().next().unwrap().exterior().0.len(), 0);
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

/// DE-9IM `T**FF*FF*`: strict containment rejects every boundary contact.
#[test]
fn contains_properly_is_available_from_the_facade() {
    let container = square(0.0, 0.0, 10.0);
    let interior = square(2.0, 2.0, 2.0);
    let touches_boundary = square(0.0, 2.0, 2.0);
    let overlaps_boundary = square(9.0, 2.0, 2.0);

    assert!(contains_properly(&container, &interior).unwrap());
    assert!(!contains_properly(&container, &touches_boundary).unwrap());
    assert!(!contains_properly(&container, &overlaps_boundary).unwrap());
    assert!(!contains_properly(&interior, &container).unwrap());
}

/// The public segment entry preserves Boost's robustness refusal and exposes
/// the proper/touch distinction carried by the turn classifier.
#[test]
fn line_intersection_reports_proper_touch_collinear_and_range_cases() {
    let proper_a = Segment::new(P::new(0.0, 0.0), P::new(4.0, 4.0));
    let proper_b = Segment::new(P::new(0.0, 4.0), P::new(4.0, 0.0));
    assert_eq!(
        line_intersection(&proper_a, &proper_b),
        Ok(Some(LineIntersection::SinglePoint {
            intersection: P::new(2.0, 2.0),
            is_proper: true,
        }))
    );

    let touch = Segment::new(P::new(4.0, 4.0), P::new(5.0, 2.0));
    assert_eq!(
        line_intersection(&proper_a, &touch),
        Ok(Some(LineIntersection::SinglePoint {
            intersection: P::new(4.0, 4.0),
            is_proper: false,
        }))
    );

    let collinear = Segment::new(P::new(2.0, 2.0), P::new(6.0, 6.0));
    assert_eq!(
        line_intersection(&proper_a, &collinear),
        Ok(Some(LineIntersection::Collinear {
            intersection: Segment::new(P::new(2.0, 2.0), P::new(4.0, 4.0)),
        }))
    );

    let disjoint = Segment::new(P::new(0.0, 5.0), P::new(4.0, 5.0));
    assert_eq!(line_intersection(&proper_a, &disjoint), Ok(None));

    let too_large = Segment::new(P::new(100_000_000.0, 0.0), P::new(100_000_001.0, 1.0));
    assert_eq!(
        line_intersection(&proper_a, &too_large),
        Err(OverlayError::Unsupported)
    );
}

/// Boost's turn enrichment associates intersections at segment endpoints with
/// the existing vertex. It must not splice a duplicate turn node into either
/// public enriched-ring sequence.
#[test]
fn endpoint_turns_are_represented_by_existing_vertices() {
    let first = traversal_square(0.0, 0.0, 2.0);
    let second = first.clone();
    let turns = [
        Turn {
            point: P::new(0.0, 0.0),
            method: Method::Touch,
            operations: [turn_operation(0, 0), turn_operation(1, 0)],
            touch_only: true,
        },
        Turn {
            point: P::new(2.0, 0.0),
            method: Method::Touch,
            operations: [turn_operation(0, 0), turn_operation(1, 0)],
            touch_only: true,
        },
    ];

    let enriched = enrich(&first, &second, &turns);
    assert!(
        enriched
            .rings
            .iter()
            .flatten()
            .all(|node| matches!(node, Node::Vertex(_)))
    );
}

/// `test/algorithms/overlay/traversal.cpp` exercises all three areal walk
/// modes. The exported traversal layer must select the exterior arcs for union
/// and the asymmetric first-minus-second arcs for difference.
#[test]
fn raw_traversal_supports_union_and_difference() {
    let first = traversal_square(0.0, 0.0, 2.0);
    let second = traversal_square(1.0, 1.0, 2.0);
    let turns = boost_geometry::overlay::turn::get_turns_ring_ring(
        &first,
        0,
        RingKind::Exterior,
        &second,
        1,
        RingKind::Exterior,
    );
    let enriched = enrich(&first, &second, &turns);

    let union = traverse(&enriched, &turns, OverlayOp::Union).unwrap();
    assert_eq!(union.len(), 1);
    assert!((ring_area(&union[0]).abs() - 7.0).abs() < 1e-12);

    let difference = traverse(&enriched, &turns, OverlayOp::Difference).unwrap();
    assert_eq!(difference.len(), 1);
    assert!((ring_area(&difference[0]).abs() - 3.0).abs() < 1e-12);
}

/// A caller can construct an enriched graph through the exported low-level
/// API. Missing turn nodes are rejected deterministically rather than yielding
/// a partial ring.
#[test]
fn malformed_public_traversal_graph_is_rejected() {
    let turn = Turn {
        point: P::new(0.0, 0.0),
        method: Method::Crosses,
        operations: [turn_operation(0, 0), turn_operation(1, 0)],
        touch_only: false,
    };
    let enriched = EnrichedRings {
        rings: [Vec::new(), Vec::new()],
    };
    assert_eq!(
        traverse(&enriched, &[turn], OverlayOp::Intersection),
        Err(TraversalError::Unsupported)
    );

    let turn_node = Node::Turn {
        point: turn.point,
        turn_id: 0,
    };
    let one_node_rings = EnrichedRings {
        rings: [vec![turn_node], vec![turn_node]],
    };
    assert_eq!(
        traverse(&one_node_rings, &[turn], OverlayOp::Intersection),
        Err(TraversalError::Unsupported)
    );
    assert_eq!(
        traverse(&one_node_rings, &[turn], OverlayOp::Union),
        Err(TraversalError::Unsupported)
    );

    let malformed_turns = [
        turn,
        Turn {
            point: P::new(100.0, 100.0),
            method: Method::Crosses,
            operations: [turn_operation(0, 2), turn_operation(1, 2)],
            touch_only: false,
        },
    ];
    let no_outgoing_edge = EnrichedRings {
        rings: [
            vec![
                Node::Turn {
                    point: malformed_turns[0].point,
                    turn_id: 0,
                },
                Node::Vertex(P::new(1.0, 1.0)),
                Node::Vertex(P::new(2.0, 1.0)),
                Node::Turn {
                    point: malformed_turns[1].point,
                    turn_id: 1,
                },
                Node::Vertex(P::new(200.0, 100.0)),
                Node::Vertex(P::new(-1.0, -1.0)),
            ],
            vec![
                Node::Turn {
                    point: malformed_turns[0].point,
                    turn_id: 0,
                },
                Node::Vertex(P::new(10.0, 0.0)),
                Node::Vertex(P::new(10.0, 10.0)),
                Node::Turn {
                    point: malformed_turns[1].point,
                    turn_id: 1,
                },
                Node::Vertex(P::new(0.0, 10.0)),
            ],
        ],
    };
    assert_eq!(
        traverse(&no_outgoing_edge, &malformed_turns, OverlayOp::Intersection,),
        Err(TraversalError::Unsupported)
    );
}

/// A disjoint raw predicate outcome carries no traversal operation in Boost's
/// turn classifier. The public classifier preserves both operations as unset.
#[test]
fn disjoint_turn_classification_is_a_noop() {
    let first_start = P::new(0.0, 0.0);
    let first_end = P::new(1.0, 0.0);
    let second_start = P::new(0.0, 1.0);
    let second_end = P::new(1.0, 1.0);
    let mut turn = Turn {
        point: P::new(0.0, 0.0),
        method: Method::None,
        operations: [turn_operation(0, 0), turn_operation(1, 0)],
        touch_only: false,
    };
    boost_geometry::overlay::turn::classify::set_from_outcome(
        &mut turn,
        &SegmentIntersection::Disjoint,
        &first_start,
        &first_end,
        &second_start,
        &second_end,
    );
    assert_eq!(turn.method, Method::Disjoint);
    assert_eq!(turn.operations[0].operation, OperationType::None);
    assert_eq!(turn.operations[1].operation, OperationType::None);
}

/// Stitching consumes the native merge/union engine and removes the shared
/// diagonal between two triangles.
#[test]
fn stitch_triangles_reassembles_a_square() {
    let first = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(0.0, 1.0),
        P::new(1.0, 1.0),
        P::new(0.0, 0.0),
    ]));
    let second = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(1.0, 1.0),
        P::new(1.0, 0.0),
        P::new(0.0, 0.0),
    ]));

    let stitched = stitch_triangles([first, second]).unwrap();
    assert_eq!(stitched.polygons().count(), 1);
    assert!((ring_area(stitched.polygons().next().unwrap().exterior()).abs() - 1.0).abs() < 1e-12);
}

/// Boost's `test/algorithms/merge_elements.cpp` retains two disjoint areal
/// components. Boost has no `stitch_triangles` entry, so this adapts that
/// expectation to the public stitching API with two disjoint triangles.
#[test]
fn stitch_triangles_retains_disjoint_components() {
    let first = Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(0.0, 1.0),
        P::new(1.0, 0.0),
        P::new(0.0, 0.0),
    ]));
    let second = Polygon::new(Ring::from_vec(vec![
        P::new(2.0, 0.0),
        P::new(2.0, 1.0),
        P::new(3.0, 0.0),
        P::new(2.0, 0.0),
    ]));

    let stitched = stitch_triangles([first, second]).unwrap();
    assert_eq!(stitched.polygons().count(), 2);
    let total_area: f64 = stitched
        .polygons()
        .map(|polygon| ring_area(polygon.exterior()).abs())
        .sum();
    assert!((total_area - 1.0).abs() < 1e-12);
}

/// `test/algorithms/is_valid.cpp:1626-1634` — the generic entry dispatches to
/// a polygon validator and reports Boost's strict-policy duplicate category.
#[test]
fn generic_is_valid_reports_duplicate_points() {
    assert!(is_valid(&square(0.0, 0.0, 2.0)).is_ok());
    assert!(is_valid(square(0.0, 0.0, 2.0).exterior()).is_ok());

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

#[test]
fn traversal_failures_convert_through_the_public_error_api() {
    assert_eq!(
        OverlayError::from(TraversalError::Unsupported),
        OverlayError::Unsupported
    );
}
