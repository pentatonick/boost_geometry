//! M-RT2 / M-RT3 — insert and query parity milestones.
//!
//! Builds trees from a fixed point fixture with each split strategy and
//! checks structural invariants (all values present, balanced depth) and
//! query correctness (spatial windows and nearest-neighbour return
//! exactly the ground-truth set computed by brute force). Mirrors the
//! insert-and-query fixtures in Boost's
//! `test/index/rtree/test_rtree.hpp`.

use geometry_cs::Cartesian;
use geometry_model::Point2D;
use geometry_rtree::{Bounds, Linear, Predicate, Quadratic, QueryPredicate, Rtree, satisfies};
use geometry_trait::Point as _;

type P = Point2D<f64, Cartesian>;

/// A deterministic 1000-point fixture on a 100×10 lattice.
fn fixture() -> Vec<P> {
    (0..1000)
        .map(|i| P::new(f64::from(i % 100), f64::from(i / 100)))
        .collect()
}

/// Ground-truth brute-force window query.
fn brute_window(points: &[P], win: Bounds) -> usize {
    points
        .iter()
        .filter(|p| {
            let x = p.get::<0>();
            let y = p.get::<1>();
            x >= win.min[0] && x <= win.max[0] && y >= win.min[1] && y <= win.max[1]
        })
        .count()
}

#[test]
fn insert_parity_quadratic_and_linear() {
    let pts = fixture();

    let mut q: Rtree<P, Quadratic> = Rtree::new();
    let mut l: Rtree<P, Linear> = Rtree::new();
    for p in &pts {
        q.insert(*p);
        l.insert(*p);
    }

    // All values present.
    assert_eq!(q.len(), 1000);
    assert_eq!(l.len(), 1000);

    // Balanced depth: a tree of 1000 values with MAX=8 is at most a few
    // levels deep. (Boost's parity target is "within 10%"; both strategies
    // stay well inside a small absolute bound here.)
    assert!(
        q.height() >= 3 && q.height() <= 8,
        "quadratic height {}",
        q.height()
    );
    assert!(
        l.height() >= 3 && l.height() <= 10,
        "linear height {}",
        l.height()
    );
}

#[test]
fn bulk_load_parity() {
    let pts = fixture();
    let t: Rtree<P> = pts.iter().copied().collect();
    assert_eq!(t.len(), 1000);
    // STR packing yields a shallow, balanced tree.
    assert!(t.height() <= 5, "bulk height {}", t.height());
}

#[test]
fn query_window_matches_brute_force() {
    let pts = fixture();
    let mut t: Rtree<P> = Rtree::new();
    for p in &pts {
        t.insert(*p);
    }

    for win in [
        Bounds::new([0.0, 0.0], [9.0, 9.0]),
        Bounds::new([20.0, 2.0], [40.0, 6.0]),
        Bounds::new([95.0, 0.0], [99.0, 9.0]),
        Bounds::new([50.0, 5.0], [50.0, 5.0]),
    ] {
        let hits = t.query(Predicate::Intersects(win));
        assert_eq!(
            hits.len(),
            brute_window(&pts, win),
            "window {win:?} mismatch"
        );
    }
}

#[test]
fn query_result_sets_are_the_same_points() {
    let pts = fixture();
    let mut t: Rtree<P> = Rtree::new();
    for p in &pts {
        t.insert(*p);
    }
    let win = Bounds::new([10.0, 2.0], [15.0, 4.0]);
    let mut hits: Vec<(f64, f64)> = t
        .query(Predicate::CoveredBy(win))
        .iter()
        .map(|p| (p.get::<0>(), p.get::<1>()))
        .collect();
    hits.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let mut brute: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| (p.get::<0>(), p.get::<1>()))
        .filter(|&(x, y)| (10.0..=15.0).contains(&x) && (2.0..=4.0).contains(&y))
        .collect();
    brute.sort_by(|lhs, rhs| lhs.partial_cmp(rhs).unwrap());

    assert_eq!(hits, brute);
}

/// Logical `and`, `not`, and value-level `satisfies` predicates retain their
/// public leaf semantics and conservative subtree contracts.
#[test]
fn logical_query_predicates_compose_through_the_public_api() {
    let tree: Rtree<P> = [
        P::new(0.0, 0.0),
        P::new(1.0, 1.0),
        P::new(2.0, 2.0),
        P::new(5.0, 5.0),
    ]
    .into_iter()
    .collect();
    let window = Bounds::new([0.0, 0.0], [3.0, 3.0]);
    let selected = tree.query_with(
        Predicate::Intersects(window).and(satisfies(|point: &P| point.get::<0>() >= 1.0)),
    );
    assert_eq!(selected.len(), 2);

    let outside = tree.query_with(!Predicate::Intersects(window));
    assert_eq!(outside.len(), 1);
    assert!((outside[0].get::<0>() - 5.0).abs() < f64::EPSILON);

    let condition = satisfies(|point: &P| point.get::<1>() >= 0.0);
    assert!(<_ as QueryPredicate<P>>::matches(
        &condition,
        &P::new(1.0, 1.0)
    ));
    assert!(<_ as QueryPredicate<P>>::could_match(&condition, &window));
    assert!(!<_ as QueryPredicate<P>>::covers_all(&condition, &window));

    let conjunction = Predicate::Intersects(window).and(Predicate::CoveredBy(window));
    assert!(<_ as QueryPredicate<P>>::could_match(&conjunction, &window));
    assert!(<_ as QueryPredicate<P>>::covers_all(&conjunction, &window));

    let negated = !Predicate::Intersects(window);
    assert!(!<_ as QueryPredicate<P>>::could_match(&negated, &window));
    assert!(!<_ as QueryPredicate<P>>::covers_all(&negated, &window));
}

#[test]
#[allow(
    clippy::many_single_char_names,
    reason = "x/y/d are the conventional coordinate and distance names"
)]
fn nearest_matches_brute_force() {
    let pts = fixture();
    let mut t: Rtree<P> = Rtree::new();
    for p in &pts {
        t.insert(*p);
    }

    let query = [42.3, 4.1];
    let k = 10;

    // Tree answer.
    let mut tree_near: Vec<(f64, f64)> = t
        .nearest(query, k)
        .iter()
        .map(|p| (p.get::<0>(), p.get::<1>()))
        .collect();

    // Brute-force k nearest.
    let mut all: Vec<(f64, (f64, f64))> = pts
        .iter()
        .map(|p| {
            let (x, y) = (p.get::<0>(), p.get::<1>());
            let d = (x - query[0]).powi(2) + (y - query[1]).powi(2);
            (d, (x, y))
        })
        .collect();
    all.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let mut brute_near: Vec<(f64, f64)> = all.iter().take(k).map(|(_, p)| *p).collect();

    tree_near.sort_by(|a, b| a.partial_cmp(b).unwrap());
    brute_near.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(tree_near, brute_near);
}
