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
    OverlayError, difference, difference_multi, intersection, intersection_multi, is_valid,
    sym_difference, sym_difference_multi, union_multi, union_poly,
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

// ---- Result lobes meeting at two or more points -----------------------
//
// Where two lobes of the result meet at two or more points, the region
// between them is not part of the result and each lobe is its own polygon.
// A walker that takes the wrong exit at such a point traces the lobes'
// common outline and cuts the region between them out as a hole touching
// the outer ring — the right area, but a polygon `is_valid` rejects as
// `DisconnectedInterior`. C++ Boost 1.83 returns the lobes separately in
// every case below.

/// Two offset squares: `A ⊖ B` is two L-shapes meeting at `(1, 2)` and
/// `(2, 1)`. C++ Boost 1.83: two polygons of area 3 each.
#[test]
fn sym_difference_of_offset_squares_is_two_valid_polygons() {
    let a = square(0.0, 0.0, 2.0);
    let b = square(1.0, 1.0, 2.0);
    let out = sym_difference(&a, &b).unwrap();
    assert_eq!(is_valid(&out), Ok(()), "{out:?}");
    assert_eq!(out.polygons().count(), 2, "{out:?}");
    close(area(&out), 6.0);
    for lobe in out.polygons() {
        close(ring_area(lobe.exterior()).abs(), 3.0);
        assert_eq!(lobe.interiors().count(), 0);
    }
}

/// A diamond inscribed in a square, touching all four sides at their
/// midpoints. `square − diamond` is the four corner triangles, and each pair
/// of neighbours meets at a midpoint. C++ Boost 1.83, on the two rings
/// corrected to its clockwise order: four polygons of area 12.5 each. (Both
/// are given counter-clockwise here; Boost reads its declared order and
/// returns nothing for them as they stand, where this overlay classifies
/// faces by containment and does not depend on the order.)
#[test]
fn difference_with_an_inscribed_diamond_is_four_triangles() {
    let sq = square(0.0, 0.0, 10.0);
    let diamond: Polygon<P> =
        polygon![[(5.0, 0.0), (10.0, 5.0), (5.0, 10.0), (0.0, 5.0), (5.0, 0.0)]];
    let out = difference(&sq, &diamond).unwrap();
    assert_eq!(is_valid(&out), Ok(()), "{out:?}");
    assert_eq!(out.polygons().count(), 4, "{out:?}");
    close(area(&out), 50.0);
    for triangle in out.polygons() {
        close(ring_area(triangle.exterior()).abs(), 12.5);
        assert_eq!(triangle.interiors().count(), 0);
    }
}

/// Two L-shapes that touch at `(1, 2)` and `(2, 1)` and nowhere else; the
/// unit square between them belongs to neither, so their union is the two
/// operands unchanged. C++ Boost 1.83: two polygons of area 3 each.
#[test]
fn union_of_two_ls_touching_at_two_points_is_two_polygons() {
    let lower: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 2.0),
        (1.0, 2.0),
        (1.0, 1.0),
        (2.0, 1.0),
        (2.0, 0.0),
        (0.0, 0.0)
    ]];
    let upper: Polygon<P> = polygon![[
        (1.0, 2.0),
        (1.0, 3.0),
        (3.0, 3.0),
        (3.0, 1.0),
        (2.0, 1.0),
        (2.0, 2.0),
        (1.0, 2.0)
    ]];
    let out = union_poly(&lower, &upper).unwrap();
    assert_eq!(is_valid(&out), Ok(()), "{out:?}");
    assert_eq!(out.polygons().count(), 2, "{out:?}");
    close(area(&out), 6.0);
    for lobe in out.polygons() {
        close(ring_area(lobe.exterior()).abs(), 3.0);
        assert_eq!(lobe.interiors().count(), 0);
    }
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
    // A ⊖ B = |A| + |B| − 2|A ∩ B| = 2 + 1 − 1.
    close(area(&sym_difference_multi(&a, &b).unwrap()), 2.0);
}

/// A member with a hole, through the multi-polygon entry points.
///
/// `multi_polygon_segments` walks each member's exterior **and its interiors**,
/// which is what makes this the same overlay Boost dispatches rather than a
/// per-exterior approximation. The hole is placed so the second operand covers
/// part of it, so every one of the four answers moves if the interior ring is
/// dropped: without it the first operand would be 100 rather than 84 and the
/// intersection 25 rather than 21.
///
/// ```text
/// A = (0,0)-(10,10) with a hole (3,3)-(7,7)      area 100 − 16 = 84
/// B = (5,5)-(15,15)                              area 100
/// A ∩ B = [5,10]² minus the hole's [5,7]² corner = 25 − 4 = 21
/// A − B = 84 − 21 = 63     A ∪ B = 184 − 21 = 163     A ⊖ B = 184 − 42 = 142
/// ```
#[test]
fn a_multi_polygon_member_carries_its_hole_into_the_overlay() {
    let holed: MultiPolygon<Polygon<P>> = MultiPolygon::from_vec(vec![polygon![
        [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0)
        ],
        [(3.0, 3.0), (3.0, 7.0), (7.0, 7.0), (7.0, 3.0), (3.0, 3.0)]
    ]]);
    let corner: MultiPolygon<Polygon<P>> = MultiPolygon::from_vec(vec![square(5.0, 5.0, 10.0)]);

    close(area(&intersection_multi(&holed, &corner).unwrap()), 21.0);
    close(area(&difference_multi(&holed, &corner).unwrap()), 63.0);
    close(area(&union_multi(&holed, &corner).unwrap()), 163.0);
    close(area(&sym_difference_multi(&holed, &corner).unwrap()), 142.0);
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

// ---- Unions whose operands share a collinear edge ------------------------
//
// Two more pieces of Boost, both taken from its source rather than guessed at:
//
//  * `traverse_with_operation` runs `clean_closing_dups_and_spikes` over every
//    ring it traverses, which erases the ring's first point while the outline
//    runs straight through it. A ring starts at a turn, and where two operands
//    share an edge a turn need not be a corner.
//  * `get_turns` walks the first operand's sections in the outer loop and the
//    second's in the inner, so two turns on the same stretch of the first
//    operand are ordered by where they sit on the *second*.
//
// Reference values from C++ Boost 1.83 on the same input.

/// Two squares sharing a whole edge. The traversal starts at `(10, 10)` — the
/// first turn — and that point sits in the middle of the union's straight top
/// side, so Boost erases it and the ring begins at `(20, 10)`. Note the
/// identical straight-through point at the *other* end of the shared edge,
/// `(10, 0)`, survives: only the start is cleaned.
#[test]
fn a_shared_edge_loses_the_ring_start_it_ran_straight_through() {
    let left: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 10.0),
        (10.0, 10.0),
        (10.0, 0.0),
        (0.0, 0.0)
    ]];
    let right: Polygon<P> = polygon![[
        (10.0, 0.0),
        (10.0, 10.0),
        (20.0, 10.0),
        (20.0, 0.0),
        (10.0, 0.0)
    ]];
    assert_eq!(
        vertices(&union_poly(&left, &right).unwrap()),
        vec![
            (20.0, 10.0),
            (20.0, 0.0),
            (10.0, 0.0),
            (0.0, 0.0),
            (0.0, 10.0),
            (20.0, 10.0),
        ]
    );
}

/// A square and a rectangle overlapping along part of one side, so both turns
/// lie on the *same* segment of the first operand. Which of them starts the
/// ring is then decided by the second operand — and rotating it moves the
/// answer, which is why the second operand's position has to be part of the
/// ordering and the fraction along the first must not outrank it.
#[test]
fn two_turns_on_one_segment_are_ordered_by_the_second_operand() {
    let square: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 100.0),
        (100.0, 100.0),
        (100.0, 0.0),
        (0.0, 0.0)
    ]];
    // Starting at (100, 30): the second operand's last segment ends there,
    // which puts (100, 70) ahead of it.
    let from_bottom: Polygon<P> = polygon![[
        (100.0, 30.0),
        (100.0, 70.0),
        (200.0, 70.0),
        (200.0, 30.0),
        (100.0, 30.0)
    ]];
    assert_eq!(
        vertices(&union_poly(&square, &from_bottom).unwrap()),
        vec![
            (100.0, 70.0),
            (200.0, 70.0),
            (200.0, 30.0),
            (100.0, 30.0),
            (100.0, 0.0),
            (0.0, 0.0),
            (0.0, 100.0),
            (100.0, 100.0),
            (100.0, 70.0),
        ]
    );

    // Rotated, (100, 30) now ends an earlier segment and takes the start.
    let from_top: Polygon<P> = polygon![[
        (100.0, 70.0),
        (200.0, 70.0),
        (200.0, 30.0),
        (100.0, 30.0),
        (100.0, 70.0)
    ]];
    assert_eq!(
        vertices(&union_poly(&square, &from_top).unwrap()),
        vec![
            (100.0, 30.0),
            (100.0, 0.0),
            (0.0, 0.0),
            (0.0, 100.0),
            (100.0, 100.0),
            (100.0, 70.0),
            (200.0, 70.0),
            (200.0, 30.0),
            (100.0, 30.0),
        ]
    );
}

/// A square and a rectangle overlapping along part of one side, running the
/// same way round. Both ends of the overlap are turns, and both carry the
/// outline straight on — so each is appended and then replaced by the next
/// turn along, which is `append_no_collinear` doing what Boost does.
#[test]
fn a_turn_that_carries_the_outline_straight_on_is_replaced() {
    let square: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 100.0),
        (100.0, 100.0),
        (100.0, 0.0),
        (0.0, 0.0)
    ]];
    let overlapping: Polygon<P> = polygon![[
        (50.0, 100.0),
        (150.0, 100.0),
        (150.0, 50.0),
        (50.0, 50.0),
        (50.0, 100.0)
    ]];
    assert_eq!(
        vertices(&union_poly(&square, &overlapping).unwrap()),
        vec![
            (150.0, 100.0),
            (150.0, 50.0),
            (100.0, 50.0),
            (100.0, 0.0),
            (0.0, 0.0),
            (0.0, 100.0),
            (150.0, 100.0),
        ]
    );
}

/// The same shape the other way up: the rectangle straddles the square, and
/// the overlap runs down one side. `(100, 100)` is the square's own corner and
/// still goes, because the turn after it — the far end of the overlap — is
/// collinear with it.
#[test]
fn the_walked_operands_own_corner_goes_too_when_a_turn_follows_it_straight() {
    let square: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 100.0),
        (100.0, 100.0),
        (100.0, 0.0),
        (0.0, 0.0)
    ]];
    let straddling: Polygon<P> = polygon![[
        (0.0, 50.0),
        (0.0, 150.0),
        (100.0, 150.0),
        (100.0, 50.0),
        (0.0, 50.0)
    ]];
    assert_eq!(
        vertices(&union_poly(&square, &straddling).unwrap()),
        vec![
            (0.0, 150.0),
            (100.0, 150.0),
            (100.0, 50.0),
            (100.0, 0.0),
            (0.0, 0.0),
            (0.0, 150.0),
        ]
    );
}

/// Two squares meeting at a single corner. Both output rings begin at that
/// corner, so the node they start at cannot separate them — Boost's `iterate`
/// tries operation 0 before operation 1 at a turn, which puts the lobe traced
/// along the *first* operand first.
#[test]
fn lobes_meeting_at_a_corner_are_ordered_by_operand() {
    let lower: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 100.0),
        (100.0, 100.0),
        (100.0, 0.0),
        (0.0, 0.0)
    ]];
    let upper: Polygon<P> = polygon![[
        (100.0, 100.0),
        (100.0, 200.0),
        (200.0, 200.0),
        (200.0, 100.0),
        (100.0, 100.0)
    ]];
    let out = union_poly(&lower, &upper).unwrap();
    let rings: Vec<Vec<(f64, f64)>> = out
        .polygons()
        .map(|pg| {
            pg.exterior()
                .points()
                .map(|p| (p.get::<0>(), p.get::<1>()))
                .collect()
        })
        .collect();
    assert_eq!(
        rings,
        vec![
            vec![
                (100.0, 100.0),
                (100.0, 0.0),
                (0.0, 0.0),
                (0.0, 100.0),
                (100.0, 100.0)
            ],
            vec![
                (100.0, 100.0),
                (100.0, 200.0),
                (200.0, 200.0),
                (200.0, 100.0),
                (100.0, 100.0)
            ],
        ]
    );
}

/// Two convex polygons crossing twice, where each crossing sits on a
/// *different* segment of the first operand but both sit in the same monotone
/// run of it.
///
/// `get_turns` partitions each operand into sections — runs of segments
/// heading the same way in both dimensions — and walks the section pairs, so
/// two turns in one section of the first operand are ordered by the section of
/// the second, not by the first's segment index. Ordering by segment alone
/// starts this ring at the other crossing.
///
/// The crossing coordinates are irrational, so the start is checked by
/// proximity rather than pinned digit for digit.
#[test]
fn turns_in_one_section_are_ordered_by_the_second_operands_section() {
    let nine: Polygon<P> = polygon![[
        (181.0, 100.0),
        (157.0, 43.0),
        (100.0, 19.0),
        (43.0, 43.0),
        (19.0, 100.0),
        (43.0, 157.0),
        (100.0, 181.0),
        (157.0, 157.0),
        (181.0, 100.0)
    ]];
    let ten: Polygon<P> = polygon![[
        (200.0, 4.0),
        (188.0, -26.0),
        (160.0, -43.0),
        (129.0, -37.0),
        (107.0, -12.0),
        (107.0, 20.0),
        (128.0, 45.0),
        (160.0, 51.0),
        (188.0, 34.0),
        (200.0, 4.0)
    ]];
    let start = vertices(&union_poly(&nine, &ten).unwrap())[0];
    // C++ Boost 1.83 begins here; ordering by segment would begin at the other
    // crossing, near (160.293, 50.822).
    assert!(
        (start.0 - 109.530_944_625_407_16).abs() < 1e-9
            && (start.1 - 23.013_029_315_960_91).abs() < 1e-9,
        "ring starts at {start:?}"
    );
}

/// A pentagon with a smaller polygon cutting a bite out of one of its edges,
/// where both ends of the bite land on the *same* segment of the pentagon.
///
/// C++: `difference` dispatches the overlay with `Reverse2 = true`, so
/// `sectionalize` reads the second operand backwards and the two turns come
/// out in the opposite order from the one their stored segments give. They tie
/// on everything the first operand can say, so that reversal is the whole
/// decision: read forwards, the ring starts at the other end of the bite.
#[test]
fn a_difference_reads_the_second_operand_backwards() {
    let pentagon: Polygon<P> = polygon![[
        (182.0, 100.0),
        (125.0, 23.0),
        (34.0, 52.0),
        (34.0, 148.0),
        (125.0, 177.0),
        (182.0, 100.0)
    ]];
    let bite: Polygon<P> = polygon![[
        (135.0, 192.0),
        (105.0, 153.0),
        (60.0, 168.0),
        (60.0, 216.0),
        (105.0, 231.0),
        (135.0, 192.0)
    ]];
    let start = vertices(&difference(&pentagon, &bite).unwrap())[0];
    // C++ Boost 1.83 begins here; reading the second operand forwards would
    // begin at the other end of the bite, near (122.962, 176.351).
    assert!(
        (start.0 - 77.966_292_134_831_46).abs() < 1e-9
            && (start.1 - 162.011_235_955_056_18).abs() < 1e-9,
        "ring starts at {start:?}"
    );
}

/// The same pentagon against a nonagon that clips three separate pieces off
/// it, so the result is three polygons and their order is what is under test.
///
/// C++: `add_rings` emits the traversed rings in the order `traverse` started
/// them, which is where `get_turns` put each one's starting turn — not the
/// order the rings happened to be traced in. Two of these three start in the
/// same section of the first operand and are separated only by the second
/// operand's segment, so ordering by anything else swaps them.
#[test]
fn difference_pieces_come_out_in_the_order_their_turns_were_collected() {
    let pentagon: Polygon<P> = polygon![[
        (182.0, 100.0),
        (125.0, 23.0),
        (34.0, 52.0),
        (34.0, 148.0),
        (125.0, 177.0),
        (182.0, 100.0)
    ]];
    let nonagon: Polygon<P> = polygon![[
        (161.0, 91.0),
        (145.0, 49.0),
        (106.0, 27.0),
        (63.0, 34.0),
        (33.0, 69.0),
        (33.0, 113.0),
        (62.0, 148.0),
        (106.0, 155.0),
        (145.0, 133.0),
        (161.0, 91.0)
    ]];
    let pieces = difference(&pentagon, &nonagon).unwrap();
    let sizes: Vec<usize> = pieces
        .polygons()
        .map(|pg| pg.exterior().points().count())
        .collect();
    // C++ Boost 1.83: the corner by (125, 23) first, then the body, then the
    // sliver by (34, 52). Tracing order alone puts the body first.
    assert_eq!(sizes, vec![4, 10, 4], "piece order");
    let corner: Vec<(f64, f64)> = pieces
        .polygons()
        .next()
        .expect("three pieces")
        .exterior()
        .points()
        .map(|p| (p.get::<0>(), p.get::<1>()))
        .collect();
    assert!(
        (corner[0].0 - 143.706_689_536_878_23).abs() < 1e-9
            && (corner[0].1 - 48.270_440_251_572_325).abs() < 1e-9,
        "first piece starts at {:?}",
        corner[0]
    );
}

/// A polygon whose ring runs straight through its last vertex into its first,
/// differenced against something that does not touch it.
///
/// C++: nothing traverses this ring — no turn lands on it, so `add_rings`
/// copies it out of its operand with `convert_ring`, which appends nothing and
/// drops nothing. Closing it the way the traversal closes a *traced* ring puts
/// the last vertex through `append_no_collinear`, which sees the straight run
/// into the first vertex and takes it off.
///
/// This is what reached tilemaker: the dissolve it uses to repair a polygon
/// finishes with `difference(outers, inners)`, and a repaired piece that
/// nothing else touches came back a vertex short.
#[test]
fn an_untouched_ring_keeps_the_vertex_it_runs_straight_through() {
    let sliver: Polygon<P> = polygon![[(3.0, 3.0), (2.0, 4.0), (3.0, 5.0), (3.0, 4.0), (3.0, 3.0)]];
    let elsewhere = square(20.0, 20.0, 4.0);
    let kept = vertices(&difference(&sliver, &elsewhere).unwrap());
    assert_eq!(
        kept,
        vec![(3.0, 3.0), (2.0, 4.0), (3.0, 5.0), (3.0, 4.0), (3.0, 3.0)]
    );
}

/// Three separate pieces, one of which a hole cuts into, and two of which no
/// turn lands on at all — with the third sharing a vertex with one of them.
///
/// C++: `add_rings` emits the untouched rings first, under their own
/// `ring_identifier`, so they keep the operand's order; the traversed one
/// follows. The shared vertex is what makes this bite: an arrangement that
/// merges coincident points gives the second and third pieces a node in
/// common, so ordering the untouched rings by any vertex puts them the wrong
/// way round. Only the ring each cycle came out of says which is which.
///
/// This is the shape the vendored dissolve hands to `difference` after it has
/// split a self-intersecting ring, which is where tilemaker met it.
#[test]
fn untouched_pieces_keep_their_operands_order() {
    let pieces: MultiPolygon<Polygon<P>> = MultiPolygon(vec![
        polygon![[
            (3139.0, 3263.0),
            (3104.0, 3325.0),
            (3_103.231_759_656_652_2, 3_336.523_605_150_214_7),
            (3139.0, 3263.0)
        ]],
        polygon![[
            (3103.0, 3344.0),
            (3099.0, 3363.0),
            (3103.0, 3346.0),
            (3103.0, 3344.0)
        ]],
        polygon![[
            (3139.0, 3263.0),
            (3165.0, 3210.0),
            (3162.0, 3216.0),
            (3139.0, 3263.0)
        ]],
    ]);
    let bite: MultiPolygon<Polygon<P>> = MultiPolygon(vec![polygon![[
        (3_103.231_759_656_652_2, 3_336.523_605_150_214_7),
        (3103.0, 3337.0),
        (3103.0, 3340.0),
        (3_103.231_759_656_652_2, 3_336.523_605_150_214_7)
    ]]]);
    let result = difference_multi(&pieces, &bite).unwrap();
    let starts: Vec<(f64, f64)> = result
        .polygons()
        .map(|pg| {
            let first = pg.exterior().points().next().expect("a ring");
            (first.get::<0>(), first.get::<1>())
        })
        .collect();
    // C++ Boost 1.83: the two untouched pieces in operand order, then the one
    // the bite ran through.
    assert_eq!(
        starts,
        vec![
            (3103.0, 3344.0),
            (3139.0, 3263.0),
            (3_103.231_759_656_652_2, 3_336.523_605_150_214_7)
        ]
    );
}

/// Assembly ties break toward the *smallest* container: an island inside
/// a hole stays a separate polygon member when the outer ring is the one
/// being traversed (the untouched hole and island are emitted first, so
/// the island meets its smaller container before the outer).
#[test]
fn union_multi_keeps_an_island_in_a_hole_when_the_outer_is_traversed() {
    let a: MultiPolygon<Polygon<P>> = MultiPolygon::from_vec(vec![
        polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(2.0, 2.0), (2.0, 8.0), (8.0, 8.0), (8.0, 2.0), (2.0, 2.0)]
        ],
        polygon![[(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0), (4.0, 4.0)]],
    ]);
    let b: MultiPolygon<Polygon<P>> = MultiPolygon::from_vec(vec![polygon![[
        (9.0, 4.0),
        (11.0, 4.0),
        (11.0, 6.0),
        (9.0, 6.0),
        (9.0, 4.0)
    ]]]);
    let out = union_multi(&a, &b).unwrap();
    assert_eq!(
        out.polygons().count(),
        2,
        "island stays a separate polygon: {out:?}"
    );
    close(area(&out), 70.0);
    assert_eq!(
        out.polygons()
            .map(|pg| pg.interiors().count())
            .sum::<usize>(),
        1
    );
    assert_eq!(is_valid(&out), Ok(()));
}

// ---- Every result over this file's fixtures is a valid multi-polygon ----
//
// The area checks above cannot tell two separate lobes from one polygon
// whose hole touches its outer ring at two points: the filled area is the
// same either way. `is_valid` can, and this sweep is what would have caught
// the walker taking the wrong exit at a pinch point. The pairs are this
// file's fixtures, listed once more so that every test above keeps its own
// copy.

#[expect(
    clippy::too_many_lines,
    reason = "one fixture pair per test above, read as a table"
)]
fn polygon_pairs() -> Vec<(&'static str, Polygon<P>, Polygon<P>)> {
    let square_100: Polygon<P> = polygon![[
        (0.0, 0.0),
        (0.0, 100.0),
        (100.0, 100.0),
        (100.0, 0.0),
        (0.0, 0.0)
    ]];
    let pentagon: Polygon<P> = polygon![[
        (182.0, 100.0),
        (125.0, 23.0),
        (34.0, 52.0),
        (34.0, 148.0),
        (125.0, 177.0),
        (182.0, 100.0)
    ]];
    vec![
        (
            "star of david",
            polygon![[(0.0, 0.0), (4.0, 0.0), (2.0, 4.0), (0.0, 0.0)]],
            polygon![[(0.0, 3.0), (4.0, 3.0), (2.0, -1.0), (0.0, 3.0)]],
        ),
        (
            "union producing a hole",
            polygon![[
                (0.0, 0.0),
                (6.0, 0.0),
                (6.0, 6.0),
                (4.0, 6.0),
                (4.0, 2.0),
                (2.0, 2.0),
                (2.0, 6.0),
                (0.0, 6.0),
                (0.0, 0.0)
            ]],
            polygon![[
                (-1.0, 4.0),
                (7.0, 4.0),
                (7.0, 5.0),
                (-1.0, 5.0),
                (-1.0, 4.0)
            ]],
        ),
        (
            "corner overlap",
            square(0.0, 0.0, 2.0),
            square(1.0, 1.0, 2.0),
        ),
        (
            "rectangular overlap",
            polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)]],
            polygon![[(2.0, 1.0), (6.0, 1.0), (6.0, 5.0), (2.0, 5.0), (2.0, 1.0)]],
        ),
        ("containment", square(0.0, 0.0, 10.0), square(3.0, 3.0, 2.0)),
        ("disjoint", square(0.0, 0.0, 1.0), square(5.0, 5.0, 1.0)),
        (
            "lobes meeting at a point",
            polygon![[
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
            ]],
            square(0.0, 0.0, 10.0),
        ),
        (
            "square and triangle sharing an edge",
            polygon![[
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (0.0, 0.0)
            ]],
            polygon![[(0.0, 0.0), (10.0, 0.0), (5.0, -8.0), (0.0, 0.0)]],
        ),
        (
            "squares sharing a whole edge",
            polygon![[
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (0.0, 0.0)
            ]],
            polygon![[
                (10.0, 0.0),
                (10.0, 10.0),
                (20.0, 10.0),
                (20.0, 0.0),
                (10.0, 0.0)
            ]],
        ),
        (
            "two turns on one segment",
            square_100.clone(),
            polygon![[
                (100.0, 30.0),
                (100.0, 70.0),
                (200.0, 70.0),
                (200.0, 30.0),
                (100.0, 30.0)
            ]],
        ),
        (
            "overlap along part of one side",
            square_100.clone(),
            polygon![[
                (50.0, 100.0),
                (150.0, 100.0),
                (150.0, 50.0),
                (50.0, 50.0),
                (50.0, 100.0)
            ]],
        ),
        (
            "rectangle straddling the square",
            square_100.clone(),
            polygon![[
                (0.0, 50.0),
                (0.0, 150.0),
                (100.0, 150.0),
                (100.0, 50.0),
                (0.0, 50.0)
            ]],
        ),
        (
            "lobes meeting at a corner",
            square_100,
            polygon![[
                (100.0, 100.0),
                (100.0, 200.0),
                (200.0, 200.0),
                (200.0, 100.0),
                (100.0, 100.0)
            ]],
        ),
        (
            "nonagon crossing a decagon",
            polygon![[
                (181.0, 100.0),
                (157.0, 43.0),
                (100.0, 19.0),
                (43.0, 43.0),
                (19.0, 100.0),
                (43.0, 157.0),
                (100.0, 181.0),
                (157.0, 157.0),
                (181.0, 100.0)
            ]],
            polygon![[
                (200.0, 4.0),
                (188.0, -26.0),
                (160.0, -43.0),
                (129.0, -37.0),
                (107.0, -12.0),
                (107.0, 20.0),
                (128.0, 45.0),
                (160.0, 51.0),
                (188.0, 34.0),
                (200.0, 4.0)
            ]],
        ),
        (
            "pentagon with a bite",
            pentagon.clone(),
            polygon![[
                (135.0, 192.0),
                (105.0, 153.0),
                (60.0, 168.0),
                (60.0, 216.0),
                (105.0, 231.0),
                (135.0, 192.0)
            ]],
        ),
        (
            "pentagon clipped by a nonagon",
            pentagon,
            polygon![[
                (161.0, 91.0),
                (145.0, 49.0),
                (106.0, 27.0),
                (63.0, 34.0),
                (33.0, 69.0),
                (33.0, 113.0),
                (62.0, 148.0),
                (106.0, 155.0),
                (145.0, 133.0),
                (161.0, 91.0)
            ]],
        ),
        (
            "untouched sliver",
            polygon![[(3.0, 3.0), (2.0, 4.0), (3.0, 5.0), (3.0, 4.0), (3.0, 3.0)]],
            square(20.0, 20.0, 4.0),
        ),
        (
            "inscribed diamond",
            square(0.0, 0.0, 10.0),
            polygon![[(5.0, 0.0), (10.0, 5.0), (5.0, 10.0), (0.0, 5.0), (5.0, 0.0)]],
        ),
        (
            "two ls touching at two points",
            polygon![[
                (0.0, 0.0),
                (0.0, 2.0),
                (1.0, 2.0),
                (1.0, 1.0),
                (2.0, 1.0),
                (2.0, 0.0),
                (0.0, 0.0)
            ]],
            polygon![[
                (1.0, 2.0),
                (1.0, 3.0),
                (3.0, 3.0),
                (3.0, 1.0),
                (2.0, 1.0),
                (2.0, 2.0),
                (1.0, 2.0)
            ]],
        ),
    ]
}

type MultiPolygonPair = (
    &'static str,
    MultiPolygon<Polygon<P>>,
    MultiPolygon<Polygon<P>>,
);

fn multi_polygon_pairs() -> Vec<MultiPolygonPair> {
    vec![
        (
            "multi-polygon operands",
            MultiPolygon::from_vec(vec![square(0.0, 0.0, 1.0), square(4.0, 0.0, 1.0)]),
            MultiPolygon::from_vec(vec![polygon![[
                (0.5, 0.0),
                (0.5, 1.0),
                (1.5, 1.0),
                (1.5, 0.0),
                (0.5, 0.0)
            ]]]),
        ),
        (
            "member with a hole",
            MultiPolygon::from_vec(vec![polygon![
                [
                    (0.0, 0.0),
                    (10.0, 0.0),
                    (10.0, 10.0),
                    (0.0, 10.0),
                    (0.0, 0.0)
                ],
                [(3.0, 3.0), (3.0, 7.0), (7.0, 7.0), (7.0, 3.0), (3.0, 3.0)]
            ]]),
            MultiPolygon::from_vec(vec![square(5.0, 5.0, 10.0)]),
        ),
        (
            "untouched pieces",
            MultiPolygon(vec![
                polygon![[
                    (3139.0, 3263.0),
                    (3104.0, 3325.0),
                    (3_103.231_759_656_652_2, 3_336.523_605_150_214_7),
                    (3139.0, 3263.0)
                ]],
                polygon![[
                    (3103.0, 3344.0),
                    (3099.0, 3363.0),
                    (3103.0, 3346.0),
                    (3103.0, 3344.0)
                ]],
                polygon![[
                    (3139.0, 3263.0),
                    (3165.0, 3210.0),
                    (3162.0, 3216.0),
                    (3139.0, 3263.0)
                ]],
            ]),
            MultiPolygon(vec![polygon![[
                (3_103.231_759_656_652_2, 3_336.523_605_150_214_7),
                (3103.0, 3337.0),
                (3103.0, 3340.0),
                (3_103.231_759_656_652_2, 3_336.523_605_150_214_7)
            ]]]),
        ),
        (
            "island in a hole",
            MultiPolygon::from_vec(vec![
                polygon![
                    [
                        (0.0, 0.0),
                        (10.0, 0.0),
                        (10.0, 10.0),
                        (0.0, 10.0),
                        (0.0, 0.0)
                    ],
                    [(2.0, 2.0), (2.0, 8.0), (8.0, 8.0), (8.0, 2.0), (2.0, 2.0)]
                ],
                polygon![[(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0), (4.0, 4.0)]],
            ]),
            MultiPolygon::from_vec(vec![polygon![[
                (9.0, 4.0),
                (11.0, 4.0),
                (11.0, 6.0),
                (9.0, 6.0),
                (9.0, 4.0)
            ]]]),
        ),
    ]
}

fn record(
    failures: &mut Vec<String>,
    name: &str,
    operation: &str,
    result: Result<MultiPolygon<Polygon<P>>, OverlayError>,
) {
    match result {
        Ok(out) => {
            if let Err(failure) = is_valid(&out) {
                failures.push(format!(
                    "{name}: {operation} is invalid ({failure:?}): {out:?}"
                ));
            }
        }
        Err(error) => failures.push(format!("{name}: {operation} failed ({error:?})")),
    }
}

#[test]
fn every_boolean_result_over_the_fixtures_is_valid() {
    let mut failures = Vec::new();
    for (name, a, b) in polygon_pairs() {
        record(&mut failures, name, "intersection", intersection(&a, &b));
        record(&mut failures, name, "union", union_poly(&a, &b));
        record(&mut failures, name, "difference", difference(&a, &b));
        record(
            &mut failures,
            name,
            "reverse difference",
            difference(&b, &a),
        );
        record(
            &mut failures,
            name,
            "sym_difference",
            sym_difference(&a, &b),
        );
    }
    for (name, a, b) in multi_polygon_pairs() {
        record(
            &mut failures,
            name,
            "intersection",
            intersection_multi(&a, &b),
        );
        record(&mut failures, name, "union", union_multi(&a, &b));
        record(&mut failures, name, "difference", difference_multi(&a, &b));
        record(
            &mut failures,
            name,
            "reverse difference",
            difference_multi(&b, &a),
        );
        record(
            &mut failures,
            name,
            "sym_difference",
            sym_difference_multi(&a, &b),
        );
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

// ---- An operand whose hole touches its own exterior -----------------------
//
// OGC-valid: a hole may touch the exterior at one point. Such an operand used
// to fail every operation with `Unsupported` — even against a polygon far
// away — or come back as an invalid polygon, because the arrangement only
// cut edges where the two operands meet. Boost passes the touch point in a
// traced ring only where its self turn survives enrichment: kept by a
// difference on its first operand, dropped by a union and on a difference's
// second operand.
//
// Reference values from Boost (`aed7bc3`) on the same input.

fn rings(mp: &MultiPolygon<Polygon<P>>) -> Vec<Vec<Vec<(f64, f64)>>> {
    mp.polygons()
        .map(|polygon| {
            core::iter::once(polygon.exterior())
                .chain(polygon.interiors())
                .map(|ring| {
                    ring.points()
                        .map(|p| (p.get::<0>(), p.get::<1>()))
                        .collect()
                })
                .collect()
        })
        .collect()
}

/// A clockwise ring from its stored points, closing point included.
fn ring_of(points: &[(f64, f64)]) -> geometry_model::Ring<P> {
    geometry_model::Ring::from_vec(points.iter().map(|&(x, y)| P::new(x, y)).collect())
}

/// The square `(0, 0)`–`(10, 10)` with a triangular hole touching its left
/// side at `(0, 5)`.
fn square_with_touching_hole() -> Polygon<P> {
    Polygon {
        outer: ring_of(&[
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0),
        ]),
        inners: vec![ring_of(&[(0.0, 5.0), (3.0, 4.0), (3.0, 6.0), (0.0, 5.0)])],
    }
}

#[test]
fn a_hole_touching_its_exterior_does_not_stop_the_overlay() {
    let a = square_with_touching_hole();
    let far: Polygon<P> = Polygon {
        outer: ring_of(&[
            (20.0, 20.0),
            (20.0, 21.0),
            (21.0, 21.0),
            (21.0, 20.0),
            (20.0, 20.0),
        ]),
        inners: vec![],
    };
    let a_rings = vec![
        vec![
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0),
        ],
        vec![(0.0, 5.0), (3.0, 4.0), (3.0, 6.0), (0.0, 5.0)],
    ];
    let far_rings = vec![vec![
        (20.0, 20.0),
        (20.0, 21.0),
        (21.0, 21.0),
        (21.0, 20.0),
        (20.0, 20.0),
    ]];

    assert_eq!(intersection(&a, &far).unwrap().0.len(), 0);
    assert_eq!(rings(&difference(&a, &far).unwrap()), vec![a_rings.clone()]);
    assert_eq!(
        rings(&difference(&far, &a).unwrap()),
        vec![far_rings.clone()]
    );
    assert_eq!(
        rings(&union_poly(&a, &far).unwrap()),
        vec![a_rings.clone(), far_rings.clone()]
    );
    assert_eq!(
        rings(&sym_difference(&a, &far).unwrap()),
        vec![a_rings, far_rings]
    );
    let matrix = geometry_overlay::relate::relate(&a, &far).unwrap();
    assert_eq!(matrix.matches("FF2FF1212"), Ok(true));
}

#[test]
fn a_touch_point_reaches_a_traced_ring_as_boost_keeps_its_self_turn() {
    let a = square_with_touching_hole();
    let b: Polygon<P> = Polygon {
        outer: ring_of(&[
            (5.0, -1.0),
            (5.0, 2.0),
            (12.0, 2.0),
            (12.0, -1.0),
            (5.0, -1.0),
        ]),
        inners: vec![],
    };
    let hole = vec![(0.0, 5.0), (3.0, 4.0), (3.0, 6.0), (0.0, 5.0)];
    let b_minus_a = vec![
        (5.0, 0.0),
        (10.0, 0.0),
        (10.0, 2.0),
        (12.0, 2.0),
        (12.0, -1.0),
        (5.0, -1.0),
        (5.0, 0.0),
    ];

    assert_eq!(
        rings(&intersection(&a, &b).unwrap()),
        vec![vec![vec![
            (10.0, 2.0),
            (10.0, 0.0),
            (5.0, 0.0),
            (5.0, 2.0),
            (10.0, 2.0)
        ]]]
    );
    // A union drops the self turn: the exterior runs straight past (0, 5).
    assert_eq!(
        rings(&union_poly(&a, &b).unwrap()),
        vec![vec![
            vec![
                (10.0, 2.0),
                (12.0, 2.0),
                (12.0, -1.0),
                (5.0, -1.0),
                (5.0, 0.0),
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 2.0),
            ],
            hole.clone(),
        ]]
    );
    // A difference keeps it on its first operand.
    assert_eq!(
        rings(&difference(&a, &b).unwrap()),
        vec![vec![
            vec![
                (10.0, 2.0),
                (5.0, 2.0),
                (5.0, 0.0),
                (0.0, 0.0),
                (0.0, 5.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 2.0),
            ],
            hole.clone(),
        ]]
    );
    assert_eq!(
        rings(&difference(&b, &a).unwrap()),
        vec![vec![b_minus_a.clone()]]
    );
    // Boost's symmetric difference is the union of the two differences, which
    // starts its rings at their first turn and keeps (0, 5) from `a - b`.
    assert_eq!(
        rings(&sym_difference(&a, &b).unwrap()),
        vec![
            vec![
                vec![
                    (5.0, 0.0),
                    (0.0, 0.0),
                    (0.0, 5.0),
                    (0.0, 10.0),
                    (10.0, 10.0),
                    (10.0, 2.0),
                    (5.0, 2.0),
                    (5.0, 0.0),
                ],
                hole,
            ],
            vec![b_minus_a],
        ]
    );
}

/// A difference walks its second operand backwards, which turns that
/// operand's own touches into turns the difference discards.
#[test]
fn a_difference_drops_its_second_operands_touch_point() {
    let a: Polygon<P> = Polygon {
        outer: ring_of(&[
            (6.0, 11.0),
            (12.0, 11.0),
            (12.0, 5.0),
            (6.0, 5.0),
            (6.0, 11.0),
        ]),
        inners: vec![ring_of(&[(6.0, 7.0), (10.0, 6.0), (10.0, 8.0), (6.0, 7.0)])],
    };
    let b: Polygon<P> = Polygon {
        outer: ring_of(&[
            (1.0, 10.0),
            (7.0, 10.0),
            (7.0, 3.0),
            (1.0, 3.0),
            (1.0, 10.0),
        ]),
        inners: vec![],
    };
    assert_eq!(
        rings(&difference(&b, &a).unwrap()),
        vec![
            vec![vec![
                (6.0, 10.0),
                (6.0, 5.0),
                (7.0, 5.0),
                (7.0, 3.0),
                (1.0, 3.0),
                (1.0, 10.0),
                (6.0, 10.0),
            ]],
            vec![vec![(7.0, 7.25), (7.0, 6.75), (6.0, 7.0), (7.0, 7.25)]],
        ]
    );
}

// ---- Operands closer than the snap distance -------------------------------
//
// Each area is the exact union, intersection or difference of the same
// operands, computed over rational coordinates.

fn assert_area(result: &MultiPolygon<Polygon<P>>, expected: f64, tolerance: f64) {
    assert!(
        (area(result) - expected).abs() <= tolerance,
        "area {}, expected {expected}",
        area(result)
    );
    assert_eq!(is_valid(result), Ok(()));
}

/// A corner of the second operand lies on the first's side to within
/// `4e-17`; it is a node of that side, and the union is exactly
/// `21.539_991_243_181_32`.
#[test]
fn a_corner_on_the_other_operands_side_splits_it() {
    let first: Polygon<P> = polygon![[
        (-2.834_867_136_566_298_3, -0.240_026_311_507_251_95),
        (5.350_046_161_024_702, 6.759_973_688_492_748),
        (6.0, 6.0),
        (6.649_953_838_975_298, 5.240_026_311_507_252),
        (-1.534_959_458_615_702, -1.759_973_688_492_748_2),
        (-2.834_867_136_566_298_3, -0.240_026_311_507_251_95)
    ]];
    let second: Polygon<P> = polygon![[
        (6.0, 6.0),
        (-2.184_913_297_591, -1.0),
        (-1.534_959_458_615_702, -1.759_973_688_492_748_2),
        (6.649_953_838_975_298, 5.240_026_311_507_252),
        (6.0, 6.0)
    ]];
    assert_area(
        &union_poly(&first, &second).unwrap(),
        21.539_991_243_181_32,
        1e-12,
    );
}

/// Two corners `1.1e-7` apart, one of them `8e-10` off the other operand's
/// side: the union is exactly `5.641_005_586_168_003`.
#[test]
fn corners_a_hair_apart_meet_as_one_node() {
    let first: Polygon<P> = polygon![[
        (3.871_827_111_269, -6.56),
        (5.0, -6.552_027),
        (4.982_332_493_137_211_5, -4.052_089_428_939_224),
        (3.854_159_604_406_211_4, -4.060_062_428_939_224),
        (3.871_827_111_269, -6.56)
    ]];
    let second: Polygon<P> = polygon![[
        (5.0, -6.552_027),
        (3.871_827, -6.56),
        (3.889_494_505_120_371_8, -9.059_937_571_073_089),
        (5.017_667_505_120_372, -9.051_964_571_073_09),
        (5.0, -6.552_027)
    ]];
    assert_area(
        &union_poly(&first, &second).unwrap(),
        5.641_005_586_168_003,
        1e-12,
    );
}

/// A corner `1e-16` off the other operand's side, close to that side's end:
/// the union is exactly `6.961_258_485_236_265`.
#[test]
fn a_corner_near_the_end_of_the_other_operands_side_splits_it() {
    let first: Polygon<P> = polygon![[
        (3.289_430_186_072_662, 8.450_358_499_090_111),
        (-9.0, 5.99),
        (-9.019_630_581_561_66, 6.088_054_272_051_508),
        (3.367_908_418_438_340_3, 8.568_054_272_051_508),
        (3.469_288_180_649_861_4, 8.588_350_620_687_39),
        (3.486_193_244_172_151_5, 8.486_350_538_426_045),
        (5.718_654_244_172_152, -4.983_649_461_573_957),
        (5.62, -5.0),
        (5.521_345_755_827_848_5, -5.016_350_538_426_043),
        (3.289_430_186_072_662, 8.450_358_499_090_111)
    ]];
    let second: Polygon<P> = polygon![[
        (8.698_996_602_308_519, -8.908_134_141_947_654),
        (4.905_496_834_748_098, 2.055_808_113_869_982),
        (5.094_503_165_251_902, 2.121_203_886_130_018),
        (9.094_503_165_251_902, -9.439_563_666_380_982),
        (9.0, -9.472_261_552_511),
        (8.905_496_834_748_098, -9.504_959_438_641_018),
        (8.804_835_460_789_615, -9.214_028_752_178_416),
        (5.62, -5.0),
        (3.387_539, 8.47),
        (3.486_193_244_172_151_5, 8.486_350_538_426_045),
        (5.714_627_504_235_232, -4.959_353_320_700_586),
        (8.698_996_602_308_519, -8.908_134_141_947_654)
    ]];
    assert_area(
        &union_poly(&first, &second).unwrap(),
        6.961_258_485_236_265,
        1e-12,
    );
}

/// Near copies, every corner moved by about `1.3e-11`, meet in slivers whose
/// sides no sample tells apart; the coarser snap folds them away. Exactly,
/// the intersection is `0.038_567_711_844_505_81`, the union
/// `0.038_567_711_855_494_756`, and the differences slivers of
/// `7.582_710_056_581_652e-12` and `3.406_235_830_787_749_5e-12`.
#[test]
fn near_copies_of_a_triangle_overlay_at_the_coarser_snap() {
    let first: Polygon<P> = polygon![[
        (0.456_218_146_513_262_65, 0.732_058_269_216_545_6),
        (-0.934_732_979_560_381_7, 0.662_937_870_479_171_6),
        (-0.837_845_952_606_957_6, 0.723_207_632_707_494_4),
        (0.456_218_146_513_262_65, 0.732_058_269_216_545_6)
    ]];
    let second: Polygon<P> = polygon![[
        (0.456_218_146_519_395_4, 0.732_058_269_223_536_7),
        (-0.934_732_979_556_378_4, 0.662_937_870_474_206_8),
        (-0.837_845_952_597_424_6, 0.723_207_632_697_539_7),
        (0.456_218_146_519_395_4, 0.732_058_269_223_536_7)
    ]];
    assert_area(
        &intersection(&first, &second).unwrap(),
        0.038_567_711_844_505_81,
        1e-9,
    );
    assert_area(
        &union_poly(&first, &second).unwrap(),
        0.038_567_711_855_494_756,
        1e-9,
    );
    assert_area(
        &difference(&first, &second).unwrap(),
        7.582_710_056_581_652e-12,
        1e-9,
    );
    assert_area(
        &difference(&second, &first).unwrap(),
        3.406_235_830_787_749_5e-12,
        1e-9,
    );
}

/// The same for near copies of a non-convex pentagon. Exactly, the
/// intersection is `0.539_421_122_374_855_4`, the union
/// `0.539_421_122_394_412_5`, and the differences slivers of
/// `6.765_446_330_153_769e-12` and `1.279_165_614_888_985e-11`.
#[test]
fn near_copies_of_a_pentagon_overlay_at_the_coarser_snap() {
    let first: Polygon<P> = polygon![[
        (1.438_648_239_019_548, 0.278_700_427_972_134_23),
        (1.297_884_040_206_567_2, -0.182_558_336_075_767),
        (0.211_277_622_182_558_4, 0.124_249_129_601_213_48),
        (0.351_637_726_863_143_3, 0.574_242_620_638_961_5),
        (0.022_656_587_314_283_994, 0.667_200_160_810_969_4),
        (1.438_648_239_019_548, 0.278_700_427_972_134_23)
    ]];
    let second: Polygon<P> = polygon![[
        (1.438_648_239_027_125_4, 0.278_700_427_980_983_54),
        (1.297_884_040_212_346_6, -0.182_558_336_068_275_54),
        (0.211_277_622_175_015_7, 0.124_249_129_592_335_13),
        (0.351_637_726_860_659_8, 0.574_242_620_637_164_6),
        (0.022_656_587_306_078_772, 0.667_200_160_801_088_2),
        (1.438_648_239_027_125_4, 0.278_700_427_980_983_54)
    ]];
    assert_area(
        &intersection(&first, &second).unwrap(),
        0.539_421_122_374_855_4,
        1e-9,
    );
    assert_area(
        &union_poly(&first, &second).unwrap(),
        0.539_421_122_394_412_5,
        1e-9,
    );
    assert_area(
        &difference(&first, &second).unwrap(),
        6.765_446_330_153_769e-12,
        1e-9,
    );
    assert_area(
        &difference(&second, &first).unwrap(),
        1.279_165_614_888_985e-11,
        1e-9,
    );
}
