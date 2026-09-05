//! OVL6 milestone — spatial-predicate + validity + point-on-surface
//! parity.
//!
//! Reproduces canonical `relate` / `touches` / `overlaps` / `crosses`,
//! `is_valid`, and `point_on_surface` cases. Mirrors the case families
//! in Boost's `test/algorithms/relate/`, `test/algorithms/touches.cpp`,
//! `overlaps.cpp`, `is_valid.cpp`, and `point_on_surface.cpp`.

use geometry_algorithm::within;
use geometry_cs::Cartesian;
use geometry_model::{Point2D, Polygon, Ring, polygon};
use geometry_overlay::relate::Dimension;
use geometry_overlay::{
    ValidityFailure, crosses, is_valid_polygon, overlaps, point_on_surface, relate_matrix, touches,
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
fn edge_touching_squares_touch() {
    let a = square(0.0, 0.0, 2.0);
    let b = square(2.0, 0.0, 2.0);
    assert!(touches(&a, &b).unwrap());
    assert!(!overlaps(&a, &b).unwrap());
}

#[test]
fn corner_touching_squares_touch() {
    let a = square(0.0, 0.0, 2.0);
    let b = square(2.0, 2.0, 2.0);
    assert!(touches(&a, &b).unwrap());
    assert!(!overlaps(&a, &b).unwrap());
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

    // A self-intersecting "bow-tie" exterior. Its lobes cancel, so its
    // signed area is zero and Boost 1.83 reports the orientation failure
    // rather than the crossing — orientation is checked first.
    let bowtie: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 2.0), (2.0, 0.0), (0.0, 2.0), (0.0, 0.0)]];
    assert_eq!(
        is_valid_polygon(&bowtie),
        Err(ValidityFailure::WrongOrientation)
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

#[test]
fn representative_point_rejects_degenerate_surfaces() {
    assert!(point_on_surface(&Polygon::<P>::new(Ring::new())).is_none());
    assert!(
        point_on_surface(&Polygon::new(Ring::<P>::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(1.0, 0.0),
        ])))
        .is_none()
    );
    assert!(
        point_on_surface(&Polygon::new(Ring::<P>::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(1.0, 0.0),
            P::new(2.0, 0.0),
        ])))
        .is_none()
    );

    let with_empty_hole: Polygon<P> = Polygon::with_inners(
        Ring::<P>::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.0, 2.0),
            P::new(2.0, 2.0),
            P::new(2.0, 0.0),
            P::new(0.0, 0.0),
        ]),
        vec![Ring::<P>::new()],
    );
    assert!(point_on_surface(&with_empty_hole).is_some());
}
