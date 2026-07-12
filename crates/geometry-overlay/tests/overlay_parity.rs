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
use geometry_overlay::{difference, intersection, sym_difference, union_poly};
use geometry_trait::{MultiPolygon as _, Polygon as _};

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
