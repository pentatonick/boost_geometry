//! M-OVL5 — overlay-parity milestone.
//!
//! Reproduces canonical `intersection` / `union` / `difference` /
//! `sym_difference` cases, checking output areas against the exact
//! set-algebra values. Mirrors the canonical cases in Boost's
//! `test/algorithms/overlay/{intersection,union,difference,
//! sym_difference}.cpp`; area comparisons use Boost's
//! `BOOST_CHECK_CLOSE(0.001%)` tolerance.
//!
//! v1 scope is the clean areal case — simple polygons with transversal
//! crossings. Degenerate inputs (collinear shared edges, clustered
//! turns) surface as `OverlayError::Unsupported` and are out of scope
//! for this milestone; they are the deferred Boost sub-problems.

use geometry_algorithm::ring_area;
use geometry_cs::Cartesian;
use geometry_model::{MultiPolygon, Point2D, Polygon, polygon};
use geometry_overlay::{
    difference, difference_multi, intersection, intersection_multi, sym_difference, union_multi,
    union_poly,
};
use geometry_trait::{MultiPolygon as _, Point as _, Polygon as _, Ring as _};

type P = Point2D<f64, Cartesian>;

fn area(mp: &MultiPolygon<Polygon<P>>) -> f64 {
    mp.polygons()
        .map(|pg| {
            let outer = ring_area(pg.exterior()).abs();
            let holes: f64 = pg.interiors().map(|r| ring_area(r).abs()).sum();
            outer - holes
        })
        .sum()
}

/// Boost `BOOST_CHECK_CLOSE(0.001%)`.
fn close(a: f64, b: f64) {
    assert!(
        (a - b).abs() <= 1e-5 * a.abs().max(b.abs()).max(1.0),
        "expected {b}, got {a}"
    );
}

fn square(x: f64, y: f64, s: f64) -> Polygon<P> {
    polygon![[(x, y), (x + s, y), (x + s, y + s), (x, y + s), (x, y)]]
}

// ---- Star of David: a region with SIX crossings around one overlap ---
//
// Regression for the traversal fragmentation bug: two overlapping
// triangles whose boundaries cross six times. The single-region
// Weiler–Atherton walk must trace the whole hexagonal overlap as ONE
// ring, not fragment into spurious triangles.

#[test]
fn star_of_david_six_crossings() {
    let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (2.0, 4.0), (0.0, 0.0)]];
    let b: Polygon<P> = polygon![[(0.0, 3.0), (4.0, 3.0), (2.0, -1.0), (0.0, 3.0)]];

    // Each triangle has base 4, height 4 → area 8. Central hexagonal
    // overlap = 5.25.
    let inter = intersection(&a, &b).unwrap();
    assert_eq!(
        inter.polygons().count(),
        1,
        "intersection must be one hexagon"
    );
    close(area(&inter), 5.25);

    // |A ∪ B| = |A| + |B| − |A∩B| = 8 + 8 − 5.25 = 10.75.
    let uni = union_poly(&a, &b).unwrap();
    close(area(&uni), 10.75);

    // |A − B| = |A| − |A∩B| = 8 − 5.25 = 2.75.
    let diff = difference(&a, &b).unwrap();
    close(area(&diff), 2.75);
}

// ---- Union whose result has a HOLE -----------------------------------
//
// Regression for a data-loss bug: a U-shape unioned with a bar that
// bridges the prongs seals the notch into a hole. The output must be one
// polygon with that hole (filled area 32), not an empty result.

#[test]
fn union_producing_a_hole() {
    let u: Polygon<P> = polygon![[
        (0.0, 0.0),
        (6.0, 0.0),
        (6.0, 6.0),
        (4.0, 6.0),
        (4.0, 2.0),
        (2.0, 2.0),
        (2.0, 6.0),
        (0.0, 6.0),
        (0.0, 0.0)
    ]];
    let bar: Polygon<P> = polygon![[
        (-1.0, 4.0),
        (7.0, 4.0),
        (7.0, 5.0),
        (-1.0, 5.0),
        (-1.0, 4.0)
    ]];
    let out = union_poly(&u, &bar).unwrap();
    assert_eq!(out.polygons().count(), 1, "union must not vanish");
    // Filled area = outer − hole. The [2,4]×[2,4] notch (area 4) is sealed
    // as a hole; the outer outline area is 36, so filled = 32.
    close(area(&out), 32.0);
    assert_eq!(
        out.polygons().next().unwrap().interiors().count(),
        1,
        "the sealed notch must be a hole"
    );
}

// ---- Corner overlap: two unit-area-16 squares sharing a 1×1 corner ---

#[test]
fn corner_overlap_all_four_ops() {
    let a = square(0.0, 0.0, 2.0); // area 4
    let b = square(1.0, 1.0, 2.0); // area 4, overlap 1

    close(area(&intersection(&a, &b).unwrap()), 1.0);
    close(area(&union_poly(&a, &b).unwrap()), 7.0);
    close(area(&difference(&a, &b).unwrap()), 3.0);
    close(area(&difference(&b, &a).unwrap()), 3.0);
    close(area(&sym_difference(&a, &b).unwrap()), 6.0);
}

// ---- Larger rectangular overlap, both-axis offset (no shared edges) --

#[test]
fn rectangular_overlap_all_four_ops() {
    // A = [0,4]×[0,3] (area 12), B = [2,6]×[1,5] (area 16).
    // Overlap = [2,4]×[1,3] = 2×2 = 4.
    let a: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)]];
    let b: Polygon<P> = polygon![[(2.0, 1.0), (6.0, 1.0), (6.0, 5.0), (2.0, 5.0), (2.0, 1.0)]];

    close(area(&intersection(&a, &b).unwrap()), 4.0);
    close(area(&union_poly(&a, &b).unwrap()), 12.0 + 16.0 - 4.0);
    close(area(&difference(&a, &b).unwrap()), 12.0 - 4.0);
    close(area(&difference(&b, &a).unwrap()), 16.0 - 4.0);
    close(
        area(&sym_difference(&a, &b).unwrap()),
        12.0 + 16.0 - 2.0 * 4.0,
    );
}

// ---- Containment: small square wholly inside a large one -------------

#[test]
fn containment_all_four_ops() {
    let big = square(0.0, 0.0, 10.0); // area 100
    let small = square(3.0, 3.0, 2.0); // area 4, inside big

    close(area(&intersection(&big, &small).unwrap()), 4.0);
    close(area(&union_poly(&big, &small).unwrap()), 100.0);
    // small − big = empty; big − small has a hole (deferred), so only
    // the well-defined direction is asserted here.
    close(area(&difference(&small, &big).unwrap()), 0.0);
}

// ---- Disjoint: no overlap -------------------------------------------

#[test]
fn disjoint_all_four_ops() {
    let a = square(0.0, 0.0, 1.0);
    let b = square(5.0, 5.0, 1.0);

    close(area(&intersection(&a, &b).unwrap()), 0.0);
    close(area(&union_poly(&a, &b).unwrap()), 2.0);
    close(area(&difference(&a, &b).unwrap()), 1.0);
    close(area(&sym_difference(&a, &b).unwrap()), 2.0);
}

// ---- Result lobes meeting at one point -------------------------------

/// A ring that dips out of the clip box twice and grazes its edge at a
/// single vertex in between. The intersection is two polygons that touch
/// at `(5, 0)`; splicing them into one ring through that point would be
/// an invalid self-touching polygon.
///
/// C++ Boost (`boost::geometry::intersection`, 1.83) returns, in order:
/// `(1.25,0) (1,1) (3,2) (2,6) (4,7) (5,0) (1.25,0)` and
/// `(5,0) (6,1) (6.5,0) (5,0)`, with `is_valid` true.
#[test]
fn intersection_splits_lobes_meeting_at_a_point() {
    let subject: Polygon<P> = polygon![[
        (5.0, -1.0),
        (6.0, -2.0),
        (2.0, -3.0),
        (1.0, 1.0),
        (3.0, 2.0),
        (2.0, 6.0),
        (4.0, 7.0),
        (5.0, 0.0),
        (6.0, 1.0),
        (7.0, -1.0),
        (5.0, -1.0)
    ]];
    let clip = square(0.0, 0.0, 10.0);

    let result = intersection(&subject, &clip).unwrap();
    assert_eq!(
        result.polygons().count(),
        2,
        "the two lobes must stay separate polygons"
    );

    // 15.375 for the large lobe plus 0.75 for the small one; C++ Boost
    // reports the same 16.125 for this input.
    close(area(&result), 16.125);

    let mut sizes: Vec<usize> = result.polygons().map(|pg| pg.exterior().0.len()).collect();
    sizes.sort_unstable();
    assert_eq!(sizes, [4, 7], "each lobe keeps its own closed ring");
}

// ---- Multi-polygon operands ------------------------------------------

/// The multi-polygon entry points are the same overlay over both operands'
/// rings, not a decomposition into per-member pairs. Two disjoint unit
/// squares against a third that overlaps one of them:
///
/// ```text
/// A = {(0,0)-(1,1), (4,0)-(5,1)}      area 2
/// B = {(0.5,0)-(1.5,1)}               area 1
/// A ∪ B  area 2.5   A ∩ B  area 0.5   A − B  area 1.5
/// ```
#[test]
fn multi_polygon_operands() {
    let a: MultiPolygon<Polygon<P>> =
        MultiPolygon::from_vec(vec![square(0.0, 0.0, 1.0), square(4.0, 0.0, 1.0)]);
    let b: MultiPolygon<Polygon<P>> = MultiPolygon::from_vec(vec![polygon![[
        (0.5, 0.0),
        (0.5, 1.0),
        (1.5, 1.0),
        (1.5, 0.0),
        (0.5, 0.0)
    ]]]);

    close(area(&union_multi(&a, &b).unwrap()), 2.5);
    close(area(&intersection_multi(&a, &b).unwrap()), 0.5);
    close(area(&difference_multi(&a, &b).unwrap()), 1.5);
}

// ---- Where a union ring starts, when the first operand starts at a turn ----
//
// Boost begins each output ring at a turn — the first one along the *first*
// operand's boundary. Which turn that is depends on a normalisation in
// `get_turns`: an intersection landing exactly on a vertex is attached to the
// segment it **terminates**, not the one it begins. So a turn on the first
// operand's own first vertex is the *last* position on that ring, not the
// first.
//
// Reference values from C++ Boost 1.83 on the same input, through
// `scripts/geometry-ab/cpp_ops.cpp` in the tilemaker port.

fn vertices(mp: &MultiPolygon<Polygon<P>>) -> Vec<(f64, f64)> {
    mp.polygons()
        .next()
        .expect("one polygon")
        .exterior()
        .points()
        .map(|p| (p.get::<0>(), p.get::<1>()))
        .collect()
}

/// A square and a triangle sharing the square's bottom edge. Both ends of that
/// edge are corners of the union, so nothing is dropped and the only question
/// is which one the ring starts at.
///
/// The square is given starting at `(0, 0)` — itself one of the two turns.
/// Boost starts at the *other* one, `(10, 0)`, because `(0, 0)` terminates the
/// square's last segment and so comes last.
#[test]
fn a_union_ring_starts_at_the_first_turn_along_the_first_operand() {
    let square: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 10.0),
        (10.0, 10.0),
        (10.0, 0.0),
        (0.0, 0.0)
    ]];
    let triangle: Polygon<P> = polygon![[(0.0, 0.0), (10.0, 0.0), (5.0, -8.0), (0.0, 0.0)]];

    let expected = vec![
        (10.0, 0.0),
        (5.0, -8.0),
        (0.0, 0.0),
        (0.0, 10.0),
        (10.0, 10.0),
        (10.0, 0.0),
    ];
    assert_eq!(vertices(&union_poly(&square, &triangle).unwrap()), expected);

    // Rotating the square so it no longer starts at a turn must not move the
    // answer: the same turn is still the first one along its boundary.
    let rotated: Polygon<P> = polygon![[
        (0.0, 10.0),
        (10.0, 10.0),
        (10.0, 0.0),
        (0.0, 0.0),
        (0.0, 10.0)
    ]];
    assert_eq!(
        vertices(&union_poly(&rotated, &triangle).unwrap()),
        expected
    );
}
