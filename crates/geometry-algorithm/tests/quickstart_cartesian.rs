//! Validation milestones M1 and M3 — reproduces the numerical
//! assertions in `geometry/doc/quickstart.qbk`'s Cartesian section,
//! using the same literal inputs as
//! `geometry/doc/src/examples/quick_start.cpp` (lines 108-135), plus
//! the reference values for `length` and `area` from
//! `geometry/test/algorithms/{length,area}/` (the doc itself only
//! prints `Area: 3.015`; the with-hole and diamond area cases and the
//! length cases come from the algorithm-level test fixtures).
//!
//! Boost's quickstart prints these values:
//!
//!   Distance p1-p2 is: 1.41421
//!   Distance a-b is:   2.23607
//!   Point p is in polygon? true
//!   Area: 3.015
//!   Distance a-p is:   2.87924
//!
//! Boost's own test fixture `test/algorithms/distance/distance.cpp`
//! pins the same arithmetic at lines 69, 78 and 109; the
//! `comparable_distance` case below mirrors line 109 (comparable ==
//! squared distance).
//!
//! M3 (this file's `area` / `within` / `length` cases) tracks
//! `specs/tasks/T38-milestone_cartesian_full.md` and cites
//! `quick_start.cpp:121-129` plus `test/algorithms/area/area.cpp:45-49`
//! and `test/algorithms/length/length.cpp:24-28`.

use geometry_adapt::Adapt;
use geometry_algorithm::{area, comparable_distance, distance, length, within};
use geometry_cs::Cartesian;
use geometry_model::{Linestring, Point2D, Polygon, Ring, linestring, polygon};

/// Tolerance for the quickstart's printed values (5 decimal places).
const QUICKSTART_EPS: f64 = 1e-4;

/// The polygon literal from `doc/src/examples/quick_start.cpp:121`:
///
/// ```text
/// double points[][2] = {{2.0, 1.3}, {4.1, 3.0}, {5.3, 2.6},
///                       {2.9, 0.7}, {2.0, 1.3}};
/// ```
///
/// This is the 4-vertex polygon whose `area(poly)` Boost prints as
/// `3.015` in `doc/quickstart.qbk` (the `[quickstart_area]` block
/// expands to `quick_start.cpp:129`, which operates on this same
/// `poly`). The 12-vertex polygon used by the algorithm-level
/// `within` test fixture (see `within_quickstart_point_in_polygon`
/// below) has a different shoelace area (~4.48) and is **not** the
/// one the doc's `3.015` refers to.
fn quickstart_polygon() -> Polygon<Point2D<f64, Cartesian>> {
    polygon![[(2.0, 1.3), (4.1, 3.0), (5.3, 2.6), (2.9, 0.7), (2.0, 1.3)]]
}

#[test]
fn distance_p1_p2_is_sqrt2() {
    // quick_start.cpp:109 — `model::d2::point_xy<int> p1(1, 1), p2(2, 2);`
    // The Rust port keeps the values but uses f64; Boost.Geometry's
    // Pythagoras over `int` points promotes to a float for the sqrt.
    let p1 = Point2D::<f64, Cartesian>::new(1.0, 1.0);
    let p2 = Point2D::<f64, Cartesian>::new(2.0, 2.0);
    let d = distance(&p1, &p2);
    let expected = 2.0_f64.sqrt();
    assert!(
        (d - expected).abs() < QUICKSTART_EPS,
        "got {d}, expected ~= {expected} (quickstart prints 1.41421)",
    );
}

#[test]
fn distance_a_b_is_sqrt5() {
    // quick_start.cpp:114-116 — `int a[2] = {1,1}; int b[2] = {2,3};`
    // The C-array registration in `adapted/c_array.hpp` is what makes
    // `distance(a, b)` compile in Boost; the Rust analogue is
    // `Adapt<[T; 2]>` from `geometry-adapt::adapt_array`.
    let a = Adapt([1.0_f64, 1.0]);
    let b = Adapt([2.0_f64, 3.0]);
    let d = distance(&a, &b);
    let expected = 5.0_f64.sqrt();
    assert!(
        (d - expected).abs() < QUICKSTART_EPS,
        "got {d}, expected ~= {expected} (quickstart prints 2.23607)",
    );
}

#[test]
fn distance_a_to_tuple_p() {
    // quick_start.cpp:133 — `double d2 = distance(a, p);` where
    // `a` is the C-array `{1, 1}` from line 114 and `p` is the
    // `boost::tuple<double, double>` `(3.7, 2.0)` from line 124.
    //
    // Both sides resolve to `Cartesian` with `Scalar = f64`, which is
    // exactly the bound Pythagoras requires
    // (`distance_pythagoras.rs:60-65`), so the mixed-storage call
    // type-checks without any extra plumbing — the whole point of
    // the quickstart's mixed example.
    //
    // √((3.7 - 1)² + (2.0 - 1)²) = √(7.29 + 1.0) = √8.29 ≈ 2.87924
    let a = Adapt([1.0_f64, 1.0]);
    let p = Adapt((3.7_f64, 2.0));
    let d = distance(&a, &p);
    let expected = 2.879_236_009_777_435_f64; // √8.29
    assert!(
        (d - expected).abs() < QUICKSTART_EPS,
        "got {d}, expected ~= {expected} (quickstart prints 2.87924)",
    );
}

#[test]
fn comparable_distance_matches_squared() {
    // test/algorithms/distance/distance.cpp:109 — the comparable form
    // is the squared distance for Pythagoras, so for (0,0)→(3,0)
    // the comparable value is 9, not 3. The squared difference of
    // small integers is exact in IEEE-754 f64 (`3.0 * 3.0 == 9.0`
    // bit-for-bit), so an exact `to_bits` comparison is sound here
    // and sidesteps `clippy::float_cmp`.
    let p1 = Point2D::<f64, Cartesian>::new(0.0, 0.0);
    let p2 = Point2D::<f64, Cartesian>::new(3.0, 0.0);
    let cd = comparable_distance(&p1, &p2);
    assert_eq!(
        cd.to_bits(),
        9.0_f64.to_bits(),
        "comparable_distance({p1:?}, {p2:?}) = {cd}, want 9.0",
    );
}

// =====================================================================
// M3 — `within` and `area` on the quickstart polygon.
// =====================================================================

/// `quick_start.cpp:124-125`:
///
/// ```text
/// boost::tuple<double, double> p = boost::make_tuple(3.7, 2.0);
/// within(p, poly)  ->  true
/// ```
///
/// Note: the doc's `[quickstart_point_in_polygon]` block reuses the
/// 4-vertex `poly` constructed at line 121, so the same
/// `quickstart_polygon()` helper does double duty for both the
/// `within` and `area` assertions.
///
/// Boost's `within` finds a common point type when the point and
/// polygon are spelled with different storage shapes; the Rust port's
/// [`WithinStrategy`] is parameterised by a single `P` shared by both
/// arguments, so this file uses [`Point2D`] on both sides. The
/// mixed-storage flavour (point as `Adapt<(f64, f64)>`) lives in
/// `quickstart_adapted.rs`, where the polygon is also built from
/// adapted points.
#[test]
fn within_quickstart_point_in_polygon() {
    let p = Point2D::<f64, Cartesian>::new(3.7, 2.0);
    assert!(
        within(&p, &quickstart_polygon()),
        "(3.7, 2.0) should be inside the quickstart polygon",
    );
}

/// `doc/quickstart.qbk` line 65: `Area: 3.015`. The exact shoelace
/// over the four IEEE-754-representable vertex pairs is
/// `6.029_999_999_999_998 / 2 = 3.014_999_999_999_999`, so a 1e-3
/// tolerance matches the doc's printed precision and leaves headroom
/// for the trapezoidal-rule formulation in
/// `strategy/cartesian/area.hpp:110-111`.
#[test]
fn area_quickstart_polygon_is_3_015() {
    let a = area(&quickstart_polygon());
    assert!(
        (a - 3.015).abs() < 1e-3,
        "got {a}, expected ~= 3.015 (quickstart prints Area: 3.015)",
    );
}

// =====================================================================
// M3 — `length` from `test/algorithms/length/length.cpp:24-28`.
// =====================================================================

/// `test/algorithms/length/length.cpp:24` — `LINESTRING(0 0,3 4) → 5`.
#[test]
fn length_3_4_polyline_is_5() {
    let ls: Linestring<Point2D<f64, Cartesian>> = linestring![(0.0, 0.0), (3.0, 4.0)];
    let got = length(&ls);
    assert!(
        (got - 5.0).abs() < 1e-12,
        "got {got}, expected 5.0 (length.cpp:24)",
    );
}

/// `test/algorithms/length/length.cpp:27` —
/// `LINESTRING(0 0,3 4,4 3) → 5 + √2`.
#[test]
fn length_zigzag_with_diagonal_is_5_plus_sqrt2() {
    let ls: Linestring<Point2D<f64, Cartesian>> = linestring![(0.0, 0.0), (3.0, 4.0), (4.0, 3.0)];
    let got = length(&ls);
    let expected = 5.0 + 2.0_f64.sqrt();
    assert!(
        (got - expected).abs() < 1e-12,
        "got {got}, expected {expected} (length.cpp:27)",
    );
}

// =====================================================================
// M3 — extra `area` reference cases from
// `test/algorithms/area/area.cpp:45-49`.
// =====================================================================

/// `test/algorithms/area/area.cpp:45` — rotated unit-square diamond,
/// area = 2. The vertex order
/// `(1,1) → (2,2) → (3,1) → (2,0) → (1,1)` traverses the diamond
/// clockwise (right, down, left, up in screen coordinates) on the
/// default-CW [`Polygon`], so the signed shoelace area is positive.
#[test]
fn area_simple_diamond_is_2() {
    let p: Polygon<Point2D<f64, Cartesian>> =
        polygon![[(1.0, 1.0), (2.0, 2.0), (3.0, 1.0), (2.0, 0.0), (1.0, 1.0)]];
    let got = area(&p);
    assert!(
        (got - 2.0).abs() < 1e-12,
        "got {got}, expected 2.0 (area.cpp:45)",
    );
}

/// `test/algorithms/area/area.cpp:49` —
/// `POLYGON((0 0,0 7,4 2,2 0,0 0),(1 1,2 1,2 2,1 2,1 1)) → 15`
/// (pentagon area 16, hole area -1).
#[test]
fn area_pentagon_with_hole_is_15() {
    let outer: Ring<Point2D<f64, Cartesian>> = Ring::from_vec(vec![
        Point2D::new(0.0, 0.0),
        Point2D::new(0.0, 7.0),
        Point2D::new(4.0, 2.0),
        Point2D::new(2.0, 0.0),
        Point2D::new(0.0, 0.0),
    ]);
    let hole: Ring<Point2D<f64, Cartesian>> = Ring::from_vec(vec![
        Point2D::new(1.0, 1.0),
        Point2D::new(2.0, 1.0),
        Point2D::new(2.0, 2.0),
        Point2D::new(1.0, 2.0),
        Point2D::new(1.0, 1.0),
    ]);
    let mut p: Polygon<Point2D<f64, Cartesian>> = Polygon::new(outer);
    p.inners.push(hole);
    let got = area(&p);
    assert!(
        (got - 15.0).abs() < 1e-12,
        "got {got}, expected 15.0 (area.cpp:49)",
    );
}
