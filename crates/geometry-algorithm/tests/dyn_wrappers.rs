//! Round-trip every supported combination through the matching `_dyn`
//! wrapper; assert that unsupported combinations return
//! `Err(DynKindMismatch)` with the expected `got` and a non-empty
//! `expected` list.
#![allow(
    clippy::float_cmp,
    reason = "Reference values are exact integer-valued literals."
)]

use geometry_algorithm::{area_dyn, distance_dyn, envelope_dyn, length_dyn, within_dyn};
use geometry_cs::Cartesian;
use geometry_model::{DynGeometry, DynKind, Point2D};
use geometry_model::{linestring, polygon};

type S = f64;
type Pt = Point2D<S, Cartesian>;

// -------- distance_dyn --------

#[test]
fn distance_dyn_point_point_matches_static() {
    let a = DynGeometry::<S, Cartesian>::Point(Pt::new(0.0, 0.0));
    let b = DynGeometry::<S, Cartesian>::Point(Pt::new(3.0, 4.0));
    assert_eq!(distance_dyn(&a, &b).unwrap(), 5.0);
}

#[test]
fn distance_dyn_point_linestring_is_min_over_segments() {
    // Point at (0,5); linestring along the x axis from (0,0) to (10,0).
    // Closest point on the segment is (0,0); distance = 5.
    let p = DynGeometry::<S, Cartesian>::Point(Pt::new(0.0, 5.0));
    let ls = DynGeometry::<S, Cartesian>::LineString(linestring![(0.0, 0.0), (10.0, 0.0)]);
    assert_eq!(distance_dyn(&p, &ls).unwrap(), 5.0);
    // Reversed argument order gives the same answer.
    assert_eq!(distance_dyn(&ls, &p).unwrap(), 5.0);
}

#[test]
fn distance_dyn_unsupported_pair_returns_mismatch() {
    let a = DynGeometry::<S, Cartesian>::Polygon(polygon![[
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 0.0)
    ]]);
    let b = DynGeometry::<S, Cartesian>::Polygon(polygon![[
        (2.0, 2.0),
        (3.0, 2.0),
        (3.0, 3.0),
        (2.0, 2.0)
    ]]);
    let err = distance_dyn(&a, &b).unwrap_err();
    assert_eq!(err.got, vec![DynKind::Polygon, DynKind::Polygon]);
    assert!(!err.expected.is_empty());
}

// -------- length_dyn --------

#[test]
fn length_dyn_linestring_matches_static() {
    let ls = DynGeometry::<S, Cartesian>::LineString(linestring![(0.0, 0.0), (3.0, 4.0)]);
    assert_eq!(length_dyn(&ls), 5.0);
}

#[test]
fn length_dyn_point_is_zero() {
    // KC4.T1: length of a non-linear kind is 0 (Boost length.hpp:75-80).
    let p = DynGeometry::<S, Cartesian>::Point(Pt::new(1.0, 2.0));
    assert_eq!(length_dyn(&p), 0.0);
}

#[test]
fn length_dyn_collection_sums_members() {
    // Regression: Boost's `length<geometry_collection_tag>`
    // (length.hpp:251-265) recursively sums member lengths — it does NOT
    // return 0. A collection with a length-5 line plus a zero-length
    // point has length 5, and a nested collection recurses.
    let inner = DynGeometry::<S, Cartesian>::GeometryCollection(vec![
        DynGeometry::LineString(linestring![(0.0, 0.0), (3.0, 4.0)]), // 5
    ]);
    let gc = DynGeometry::<S, Cartesian>::GeometryCollection(vec![
        DynGeometry::Point(Pt::new(9.0, 9.0)), // 0
        DynGeometry::LineString(linestring![(0.0, 0.0), (0.0, 2.0)]), // 2
        inner,                                 // 5
    ]);
    assert_eq!(length_dyn(&gc), 7.0);
    // An empty collection has zero length, not a panic.
    assert_eq!(
        length_dyn(&DynGeometry::<S, Cartesian>::GeometryCollection(vec![])),
        0.0
    );
}

// -------- area_dyn --------

#[test]
fn area_dyn_polygon_matches_static() {
    // Unit square, clockwise → Boost's positive convention area = 1.
    let pg = DynGeometry::<S, Cartesian>::Polygon(polygon![[
        (0.0, 0.0),
        (0.0, 1.0),
        (1.0, 1.0),
        (1.0, 0.0),
        (0.0, 0.0)
    ]]);
    assert_eq!(area_dyn(&pg), 1.0);
}

#[test]
fn area_dyn_linestring_is_zero() {
    // KC4.T1: area of a non-areal kind is 0 (Boost area.hpp).
    let ls = DynGeometry::<S, Cartesian>::LineString(linestring![(0.0, 0.0), (3.0, 4.0)]);
    assert_eq!(area_dyn(&ls), 0.0);
}

#[test]
fn area_dyn_collection_sums_members() {
    // Regression: Boost's `area<geometry_collection_tag>`
    // (area.hpp:273-285) recursively sums member areas — it does NOT
    // return 0. A collection with a unit square plus a zero-area line
    // has area 1, and a nested collection recurses.
    let unit_square = || {
        DynGeometry::<S, Cartesian>::Polygon(polygon![[
            (0.0, 0.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (1.0, 0.0),
            (0.0, 0.0)
        ]])
    };
    let inner = DynGeometry::<S, Cartesian>::GeometryCollection(vec![unit_square()]); // 1
    let gc = DynGeometry::<S, Cartesian>::GeometryCollection(vec![
        unit_square(),                                                // 1
        DynGeometry::LineString(linestring![(0.0, 0.0), (3.0, 4.0)]), // 0
        inner,                                                        // 1
    ]);
    assert_eq!(area_dyn(&gc), 2.0);
    // An empty collection has zero area, not a panic.
    assert_eq!(
        area_dyn(&DynGeometry::<S, Cartesian>::GeometryCollection(vec![])),
        0.0
    );
}

#[test]
fn dyn_measures_over_deeply_nested_collection_do_not_overflow() {
    // Regression: `area_dyn`/`length_dyn` summed a `GeometryCollection`
    // by recursion, so a deeply-nested chain overflowed the native stack
    // (uncatchable SIGABRT). The walk is now an explicit work-list. Wrap a
    // unit square 200k collections deep and confirm both measures still
    // compute the correct value without aborting.
    let mut g = DynGeometry::<S, Cartesian>::Polygon(polygon![[
        (0.0, 0.0),
        (0.0, 1.0),
        (1.0, 1.0),
        (1.0, 0.0),
        (0.0, 0.0)
    ]]);
    for _ in 0..200_000 {
        g = DynGeometry::GeometryCollection(vec![g]);
    }
    assert_eq!(area_dyn(&g), 1.0);
    assert_eq!(length_dyn(&g), 0.0);
    // The value's own `Drop` is still recursive (a separate model-crate
    // concern); leak it so the test exercises only the measure walk.
    core::mem::forget(g);
}

// -------- envelope_dyn --------

#[test]
fn envelope_dyn_linestring_matches_static() {
    use geometry_trait::IndexedAccess as _;
    let ls = DynGeometry::<S, Cartesian>::LineString(linestring![(1.0, 2.0), (3.0, 0.0)]);
    let b = envelope_dyn(&ls).unwrap();
    // Box min corner (1,0), max corner (3,2).
    assert_eq!(b.get_indexed::<0, 0>(), 1.0);
    assert_eq!(b.get_indexed::<0, 1>(), 0.0);
    assert_eq!(b.get_indexed::<1, 0>(), 3.0);
    assert_eq!(b.get_indexed::<1, 1>(), 2.0);
}

#[test]
fn envelope_dyn_collection_returns_mismatch() {
    let gc = DynGeometry::<S, Cartesian>::GeometryCollection(vec![]);
    let err = envelope_dyn(&gc).unwrap_err();
    assert_eq!(err.got, vec![DynKind::GeometryCollection]);
    assert!(!err.expected.is_empty());
}

// -------- within_dyn --------

#[test]
fn within_dyn_point_in_polygon_matches_static() {
    let p = DynGeometry::<S, Cartesian>::Point(Pt::new(0.5, 0.5));
    let pg = DynGeometry::<S, Cartesian>::Polygon(polygon![[
        (0.0, 0.0),
        (0.0, 1.0),
        (1.0, 1.0),
        (1.0, 0.0),
        (0.0, 0.0)
    ]]);
    assert!(within_dyn(&p, &pg).unwrap());

    let outside = DynGeometry::<S, Cartesian>::Point(Pt::new(5.0, 5.0));
    assert!(!within_dyn(&outside, &pg).unwrap());
}

#[test]
fn within_dyn_unsupported_pair_returns_mismatch() {
    let a = DynGeometry::<S, Cartesian>::Point(Pt::new(0.0, 0.0));
    let b = DynGeometry::<S, Cartesian>::Point(Pt::new(1.0, 1.0));
    let err = within_dyn(&a, &b).unwrap_err();
    assert_eq!(err.got, vec![DynKind::Point, DynKind::Point]);
    assert!(!err.expected.is_empty());
}
