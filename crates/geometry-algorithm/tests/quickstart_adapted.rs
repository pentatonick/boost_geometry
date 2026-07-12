//! Validation milestones M2 and M3 — the same numerical assertions
//! as `quickstart_cartesian.rs`, but driven through the BYO-type entry
//! points:
//!
//!   * Path A — `Adapt<[f64; 2]>` (and `Adapt<(f64, f64)>`), the Rust
//!     analogue of `boost/geometry/geometries/adapted/c_array.hpp` and
//!     `boost_tuple.hpp`.
//!   * Path B — a user struct registered via `#[derive(Point)]` from
//!     `geometry-derive`, mirroring
//!     `BOOST_GEOMETRY_REGISTER_POINT_2D` from
//!     `geometries/register/point.hpp`.
//!   * Path C — a user-owned `Vec<P>` wrapper adapted with
//!     `register_linestring!`, mirroring
//!     `BOOST_GEOMETRY_REGISTER_LINESTRING` from
//!     `geometries/register/linestring.hpp`.
//!   * Path D — a user-owned polygon struct adapted with
//!     `register_ring!` + `register_polygon!`, the Rust analogue of
//!     the hand-specialised five-trait pattern in
//!     `doc/example_adapting_a_legacy_geometry_object_model.qbk`.
//!
//! The M2 distance assertions cover the five values quoted in M1's
//! header (`quick_start.cpp` lines 108-135 and
//! `test/algorithms/distance/distance.cpp:69,78,109`). The M3
//! additions lift the `area`, `within` and `length` assertions from
//! `quickstart_cartesian.rs` onto adapted carriers — see
//! `specs/tasks/T38-milestone_cartesian_full.md`.

use geometry_adapt::{Adapt, register_linestring, register_polygon, register_ring};
use geometry_algorithm::{area, comparable_distance, distance, length, within};
use geometry_derive::Point as DerivePoint;
use geometry_model::{Polygon as ModelPolygon, Ring as ModelRing};
use geometry_trait::{Linestring, Polygon as PolygonTrait, Ring as RingTrait};

/// Tolerance for the quickstart's printed values (5 decimal places).
const QUICKSTART_EPS: f64 = 1e-4;

// =====================================================================
// Path A — `Adapt<[T; N]>` and `Adapt<(T, T)>`.
// =====================================================================

#[test]
fn adapted_array_distance_a_b_is_sqrt5() {
    // quick_start.cpp:114-116 — `int a[2] = {1,1}; int b[2] = {2,3};`
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
fn adapted_array_to_tuple_distance_is_sqrt8_29() {
    // quick_start.cpp:133 — `distance(a, p)` with `a = {1,1}` and
    // `p = boost::tuple<double,double>(3.7, 2.0)`. Mixed-storage call
    // resolves to Cartesian/f64 on both sides.
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
fn adapted_array_comparable_distance_matches_squared() {
    // test/algorithms/distance/distance.cpp:109 — comparable form is
    // the squared distance for Pythagoras. (0,0)→(3,0) gives 9 exactly
    // in IEEE-754 f64.
    let a = Adapt([0.0_f64, 0.0]);
    let b = Adapt([3.0_f64, 0.0]);
    let cd = comparable_distance(&a, &b);
    assert_eq!(
        cd.to_bits(),
        9.0_f64.to_bits(),
        "comparable_distance({a:?}, {b:?}) = {cd}, want 9.0",
    );
}

// =====================================================================
// Path B — `#[derive(Point)]` on a user struct.
// =====================================================================

#[derive(Default, DerivePoint)]
#[geometry(cs = "Cartesian", scalar = "f64")]
struct MyPoint {
    x: f64,
    y: f64,
}

#[test]
fn derived_distance_p1_p2_is_sqrt2() {
    // quick_start.cpp:109 — `point_xy<int> p1(1,1), p2(2,2);`.
    let p1 = MyPoint { x: 1.0, y: 1.0 };
    let p2 = MyPoint { x: 2.0, y: 2.0 };
    let d = distance(&p1, &p2);
    let expected = 2.0_f64.sqrt();
    assert!(
        (d - expected).abs() < QUICKSTART_EPS,
        "got {d}, expected ~= {expected} (quickstart prints 1.41421)",
    );
}

#[test]
fn derived_distance_a_b_is_sqrt5() {
    // quick_start.cpp:114-116, lifted onto the derived struct.
    let a = MyPoint { x: 1.0, y: 1.0 };
    let b = MyPoint { x: 2.0, y: 3.0 };
    let d = distance(&a, &b);
    let expected = 5.0_f64.sqrt();
    assert!(
        (d - expected).abs() < QUICKSTART_EPS,
        "got {d}, expected ~= {expected} (quickstart prints 2.23607)",
    );
}

#[test]
fn derived_distance_a_to_tuple_p() {
    // quick_start.cpp:133, with `a` as the derived struct and `p` as
    // the adapted tuple — same Cartesian/f64 on both sides.
    let a = MyPoint { x: 1.0, y: 1.0 };
    let p = Adapt((3.7_f64, 2.0));
    let d = distance(&a, &p);
    let expected = 2.879_236_009_777_435_f64; // √8.29
    assert!(
        (d - expected).abs() < QUICKSTART_EPS,
        "got {d}, expected ~= {expected} (quickstart prints 2.87924)",
    );
}

#[test]
fn derived_comparable_distance_matches_squared() {
    // distance.cpp:109 — (0,0)→(3,0) is exact in IEEE-754 f64.
    let p1 = MyPoint { x: 0.0, y: 0.0 };
    let p2 = MyPoint { x: 3.0, y: 0.0 };
    let cd = comparable_distance(&p1, &p2);
    assert_eq!(
        cd.to_bits(),
        9.0_f64.to_bits(),
        "comparable_distance = {cd}, want 9.0",
    );
}

// =====================================================================
// Path C — `register_linestring!` on a user-owned `Vec<MyPoint>`.
// =====================================================================

struct MyLs {
    v: Vec<MyPoint>,
}

register_linestring!(MyLs, MyPoint, |s| s.v.iter());

#[test]
fn registered_linestring_has_three_points() {
    let ls = MyLs {
        v: vec![
            MyPoint { x: 0.0, y: 0.0 },
            MyPoint { x: 1.0, y: 1.0 },
            MyPoint { x: 2.0, y: 0.0 },
        ],
    };
    assert_eq!(ls.points().count(), 3);
}

/// `test/algorithms/length/length.cpp:24` —
/// `LINESTRING(0 0,3 4) → 5`, driven through the registered
/// `MyLs` carrier rather than `geometry_model::Linestring`. Exercises
/// the M3 `length` algorithm against a user-adapted linestring.
#[test]
fn registered_linestring_length_3_4_is_5() {
    let ls = MyLs {
        v: vec![MyPoint { x: 0.0, y: 0.0 }, MyPoint { x: 3.0, y: 4.0 }],
    };
    let got = length(&ls);
    assert!(
        (got - 5.0).abs() < 1e-12,
        "got {got}, expected 5.0 (length.cpp:24)",
    );
}

// =====================================================================
// Path D — `register_ring!` + `register_polygon!` on a user-owned
// polygon shape, plus the M3 `area` and `within` assertions.
//
// The `within` strategy in `geometry_strategy::within` is impl'd
// against `geometry_model::{Box, Ring, Polygon}` (not the
// `Polygon`/`Ring` *traits*) — the user-adapted polygon below
// therefore exercises `area` only. The `within` assertion uses
// `geometry_model::Polygon<Adapt<(f64, f64)>>`, so both arguments of
// the call carry adapted-tuple points and the `boost_tuple.hpp`
// analogue actually drives the test, even though the outer container
// is the stock model type.
// =====================================================================

struct MyRing {
    v: Vec<MyPoint>,
}
register_ring!(MyRing, MyPoint, |s| s.v.iter());

struct MyPoly {
    outer: MyRing,
    inners: Vec<MyRing>,
}
register_polygon!(
    MyPoly,
    MyPoint,
    ring = MyRing,
    |s| outer = &s.outer,
    inners = s.inners.iter()
);

/// `test/algorithms/area/area.cpp:45` — rotated unit-square diamond,
/// area = 2, on a fully-adapted user polygon type. Same vertices
/// (CW on a default-CW ring) as the M3 case in
/// `quickstart_cartesian.rs`.
#[test]
fn registered_polygon_area_diamond_is_2() {
    let p = MyPoly {
        outer: MyRing {
            v: vec![
                MyPoint { x: 1.0, y: 1.0 },
                MyPoint { x: 2.0, y: 2.0 },
                MyPoint { x: 3.0, y: 1.0 },
                MyPoint { x: 2.0, y: 0.0 },
                MyPoint { x: 1.0, y: 1.0 },
            ],
        },
        inners: vec![],
    };
    let got = area(&p);
    assert!(
        (got - 2.0).abs() < 1e-12,
        "got {got}, expected 2.0 (area.cpp:45)",
    );
}

/// `doc/quickstart.qbk` `Area: 3.015` on a user-adapted polygon
/// shape. The 4-vertex polygon literal is the same one
/// `doc/src/examples/quick_start.cpp:121` constructs.
#[test]
fn registered_polygon_quickstart_area_is_3_015() {
    let p = MyPoly {
        outer: MyRing {
            v: vec![
                MyPoint { x: 2.0, y: 1.3 },
                MyPoint { x: 4.1, y: 3.0 },
                MyPoint { x: 5.3, y: 2.6 },
                MyPoint { x: 2.9, y: 0.7 },
                MyPoint { x: 2.0, y: 1.3 },
            ],
        },
        inners: vec![],
    };
    let got = area(&p);
    assert!(
        (got - 3.015).abs() < 1e-3,
        "got {got}, expected ~= 3.015 (quickstart prints Area: 3.015)",
    );
}

/// `quick_start.cpp:124-125` — `within(boost::tuple<double,double>(3.7,
/// 2.0), poly)` is `true`. Boost's `point_in_polygon` finds a common
/// point type across the two arguments; the Rust [`WithinStrategy`]
/// requires the polygon's `Point` to match the query point exactly,
/// so the polygon below is `Polygon<Adapt<(f64, f64)>>` — built from
/// the same adapted-tuple shape as `p`. This mirrors the doc's
/// "tuple as point" registration end-to-end, with the model type only
/// acting as the polygon storage container.
#[test]
fn adapted_tuple_within_quickstart_polygon() {
    let p = Adapt((3.7_f64, 2.0));
    let outer = ModelRing::from_vec(vec![
        Adapt((2.0_f64, 1.3)),
        Adapt((4.1, 3.0)),
        Adapt((5.3, 2.6)),
        Adapt((2.9, 0.7)),
        Adapt((2.0, 1.3)),
    ]);
    let poly: ModelPolygon<Adapt<(f64, f64)>> = ModelPolygon::new(outer);
    assert!(
        within(&p, &poly),
        "(3.7, 2.0) should be inside the quickstart polygon",
    );

    // Sanity: round-trip the polygon's storage through the `Polygon`
    // and `Ring` traits to make sure the `Adapt<(f64, f64)>` points
    // are what the algorithm layer actually walks. Mirrors what
    // `geometry_trait::check_polygon` does for the model types.
    assert_eq!(poly.exterior().points().count(), 5);
    assert_eq!(poly.interiors().count(), 0);
}
