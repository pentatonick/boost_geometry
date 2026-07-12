//! OVL6 milestone — spatial-predicate + validity + point-on-surface
//! parity.
//!
//! Reproduces canonical `relate` / `touches` / `overlaps` / `crosses`,
//! `is_valid`, and `point_on_surface` cases. Mirrors the case families
//! in Boost's `test/algorithms/relate/`, `test/algorithms/touches.cpp`,
//! `overlaps.cpp`, `is_valid.cpp`, and `point_on_surface.cpp`.

use geometry_algorithm::within;
use geometry_cs::Cartesian;
use geometry_model::{Point2D, Polygon, polygon};
use geometry_overlay::relate::Dimension;
use geometry_overlay::{
    OverlayError, ValidityFailure, crosses, is_valid_polygon, overlaps, point_on_surface,
    relate_matrix, touches,
};

type P = Point2D<f64, Cartesian>;

fn square(x: f64, y: f64, s: f64) -> Polygon<P> {
    polygon![[(x, y), (x, y + s), (x + s, y + s), (x + s, y), (x, y)]]
}

// ---- relate / overlaps / touches / crosses -------------------------

#[test]
fn overlapping_squares() {
    // Boundaries cross transversally → the relation is decidable.
    let a = square(0.0, 0.0, 2.0);
    let b = square(1.0, 1.0, 2.0);
    assert!(overlaps(&a, &b).unwrap());
    assert!(!touches(&a, &b).unwrap());
    assert!(!crosses(&a, &b).unwrap());
    assert_eq!(
        relate_matrix(&a, &b).unwrap().interior_interior(),
        Dimension::Area
    );
}

#[test]
fn edge_touching_squares_are_unsupported() {
    // Collinear edge contact with both interiors outside the other: the
    // turn graph cannot tell this pure touch from an edge-aligned overlap,
    // so the relation is reported unsupported rather than a wrong boolean.
    let a = square(0.0, 0.0, 2.0);
    let b = square(2.0, 0.0, 2.0);
    assert_eq!(touches(&a, &b), Err(OverlayError::Unsupported));
    assert_eq!(overlaps(&a, &b), Err(OverlayError::Unsupported));
}

#[test]
fn corner_touching_squares_are_unsupported() {
    // Meet only at the single corner (2,2) — a vertex-only contact, the
    // same ambiguous class as a vertex-crossing overlap.
    let a = square(0.0, 0.0, 2.0);
    let b = square(2.0, 2.0, 2.0);
    assert_eq!(touches(&a, &b), Err(OverlayError::Unsupported));
    assert_eq!(overlaps(&a, &b), Err(OverlayError::Unsupported));
}

#[test]
fn disjoint_squares() {
    // No boundary contact → decidable, and clearly neither.
    let a = square(0.0, 0.0, 1.0);
    let b = square(5.0, 5.0, 1.0);
    assert!(!touches(&a, &b).unwrap());
    assert!(!overlaps(&a, &b).unwrap());
    assert_eq!(
        relate_matrix(&a, &b).unwrap().interior_interior(),
        Dimension::Empty
    );
}

#[test]
fn containment_is_not_overlap() {
    // A rep-point containment (no boundary touch) makes this decidable.
    let big = square(0.0, 0.0, 10.0);
    let small = square(3.0, 3.0, 2.0);
    assert!(!overlaps(&big, &small).unwrap());
    assert!(!touches(&big, &small).unwrap());
    // Interiors do meet (small's interior is inside big).
    assert_eq!(
        relate_matrix(&big, &small).unwrap().interior_interior(),
        Dimension::Area
    );
}

// ---- is_valid -------------------------------------------------------

#[test]
fn valid_and_invalid_polygons() {
    let good: Polygon<P> = square(0.0, 0.0, 3.0);
    assert!(is_valid_polygon(&good).is_ok());

    // A self-intersecting "bow-tie" exterior.
    let bowtie: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 2.0), (2.0, 0.0), (0.0, 2.0), (0.0, 0.0)]];
    assert_eq!(
        is_valid_polygon(&bowtie),
        Err(ValidityFailure::SelfIntersection)
    );
}

// ---- point_on_surface ----------------------------------------------

#[test]
fn representative_point_is_interior() {
    // Non-convex "U" shape; the sweep point must be inside.
    let u: Polygon<P> = polygon![[
        (0.0, 0.0),
        (5.0, 0.0),
        (5.0, 5.0),
        (4.0, 5.0),
        (4.0, 1.0),
        (1.0, 1.0),
        (1.0, 5.0),
        (0.0, 5.0),
        (0.0, 0.0)
    ]];
    let p = point_on_surface(&u).unwrap();
    assert!(within(&p, &u));
}
