// Ported from Boost.Geometry:
// boost/geometry/index/test/rtree/test_rtree.hpp
// boost/geometry/index/test/rtree/rtree_insert_remove.cpp
// boost/geometry/index/detail/predicates.hpp

use boost_geometry::rtree::{
    Bounds, Indexable, Linear, Predicate, Quadratic, QueryPredicate, RStarSplit, Rtree,
    SplitParameters, not, satisfies,
};

#[derive(Clone, Debug, PartialEq)]
struct Entry {
    id: u32,
    bounds: Bounds,
}

impl Entry {
    fn new(id: u32, min: [f64; 2], max: [f64; 2]) -> Self {
        Self {
            id,
            bounds: Bounds::new(min, max),
        }
    }

    fn point(id: u32, point: [f64; 2]) -> Self {
        Self {
            id,
            bounds: Bounds::point(point),
        }
    }
}

impl Indexable for Entry {
    fn bounds(&self) -> Bounds {
        self.bounds
    }
}

fn sorted_ids(values: impl IntoIterator<Item = impl core::borrow::Borrow<Entry>>) -> Vec<u32> {
    let mut ids: Vec<u32> = values.into_iter().map(|value| value.borrow().id).collect();
    ids.sort_unstable();
    ids
}

fn query_ids<P: QueryPredicate<Entry>>(tree: &Rtree<Entry>, predicate: P) -> Vec<u32> {
    sorted_ids(tree.query_with(predicate))
}

#[test]
fn boost_box_predicates_and_combinators_use_public_facade() {
    let query = Bounds::new([0.0, 0.0], [10.0, 10.0]);
    let tree: Rtree<Entry> = [
        Entry::new(0, [2.0, 2.0], [3.0, 3.0]),
        Entry::new(1, [8.0, 8.0], [12.0, 12.0]),
        Entry::new(2, [-1.0, -1.0], [11.0, 11.0]),
        Entry::new(3, [20.0, 20.0], [21.0, 21.0]),
        Entry::point(4, [0.0, 5.0]),
        Entry::new(5, [0.0, 0.0], [10.0, 10.0]),
    ]
    .into_iter()
    .collect();

    assert_eq!(
        sorted_ids(tree.query(Predicate::Intersects(query))),
        [0, 1, 2, 4, 5]
    );
    assert_eq!(sorted_ids(tree.query(Predicate::Disjoint(query))), [3]);
    assert_eq!(
        sorted_ids(tree.query(Predicate::CoveredBy(query))),
        [0, 4, 5]
    );
    assert_eq!(sorted_ids(tree.query(Predicate::Within(query))), [0, 5]);
    assert_eq!(sorted_ids(tree.query(Predicate::Contains(query))), [2, 5]);
    assert_eq!(sorted_ids(tree.query(Predicate::Covers(query))), [2, 5]);
    assert_eq!(sorted_ids(tree.query(Predicate::Overlaps(query))), [1]);

    let boundary_point = Bounds::point([0.0, 5.0]);
    assert!(tree.query(Predicate::Contains(boundary_point)).is_empty());
    assert_eq!(
        sorted_ids(tree.query(Predicate::Covers(boundary_point))),
        [2, 4, 5]
    );

    let even_intersections =
        Predicate::Intersects(query).and(satisfies(|entry: &Entry| entry.id.is_multiple_of(2)));
    assert_eq!(query_ids(&tree, even_intersections), [0, 2, 4]);
    assert_eq!(query_ids(&tree, not(Predicate::Intersects(query))), [3]);
}

fn box_intersects(value: &Bounds, query: &Bounds) -> bool {
    value.min[0] <= query.max[0]
        && value.max[0] >= query.min[0]
        && value.min[1] <= query.max[1]
        && value.max[1] >= query.min[1]
}

fn box_covered_by(value: &Bounds, query: &Bounds) -> bool {
    query.min[0] <= value.min[0]
        && query.max[0] >= value.max[0]
        && query.min[1] <= value.min[1]
        && query.max[1] >= value.max[1]
}

fn box_within(value: &Bounds, query: &Bounds) -> bool {
    value.min[0] < value.max[0] && value.min[1] < value.max[1] && box_covered_by(value, query)
}

fn box_covers(value: &Bounds, query: &Bounds) -> bool {
    box_covered_by(query, value)
}

fn box_contains(value: &Bounds, query: &Bounds) -> bool {
    query.min[0] < query.max[0] && query.min[1] < query.max[1] && box_covers(value, query)
}

fn box_overlaps(value: &Bounds, query: &Bounds) -> bool {
    value.min[0] < query.max[0]
        && value.max[0] > query.min[0]
        && value.min[1] < query.max[1]
        && value.max[1] > query.min[1]
        && !box_covers(value, query)
        && !box_covers(query, value)
}

#[test]
fn every_predicate_prunes_a_deep_tree_without_diverging_from_an_independent_scan() {
    type Oracle = fn(&Bounds, &Bounds) -> bool;
    let entries: Vec<Entry> = (0u32..240)
        .map(|id| {
            let x = f64::from((id * 37) % 101) - 20.0;
            let y = f64::from((id * 53) % 89) - 15.0;
            if id.is_multiple_of(11) {
                Entry::point(id, [x, y])
            } else {
                Entry::new(
                    id,
                    [x, y],
                    [x + f64::from(id % 7 + 1), y + f64::from(id % 5 + 1)],
                )
            }
        })
        .collect();
    let mut tree: Rtree<Entry> = Rtree::new();
    tree.extend(entries.clone());
    assert!(tree.height() > 2);

    for query in [
        Bounds::new([-10.0, -5.0], [25.0, 30.0]),
        Bounds::new([30.0, 20.0], [55.0, 60.0]),
        Bounds::new([-30.0, -20.0], [90.0, 80.0]),
        Bounds::point([0.0, 0.0]),
    ] {
        let cases: [(Predicate, Oracle); 7] = [
            (Predicate::Intersects(query), box_intersects),
            (Predicate::Disjoint(query), |value, query| {
                !box_intersects(value, query)
            }),
            (Predicate::CoveredBy(query), box_covered_by),
            (Predicate::Within(query), box_within),
            (Predicate::Contains(query), box_contains),
            (Predicate::Covers(query), box_covers),
            (Predicate::Overlaps(query), box_overlaps),
        ];
        for (predicate, oracle) in cases {
            let expected = entries.iter().filter(|entry| oracle(&entry.bounds, &query));
            assert_eq!(
                sorted_ids(tree.query(predicate)),
                sorted_ids(expected),
                "deep-tree scan divergence for {predicate:?} and {query:?}"
            );
        }
    }
}

fn assert_tree_matches_scan<Params: SplitParameters>(tree: &Rtree<Entry, Params>, scan: &[Entry]) {
    assert_eq!(tree.len(), scan.len());
    assert_eq!(sorted_ids(tree.iter()), sorted_ids(scan));

    let expected_bounds = scan
        .iter()
        .map(Indexable::bounds)
        .reduce(|left, right| left.union(&right));
    assert_eq!(tree.bounds(), expected_bounds);

    let window = Bounds::new([15.0, -1.0], [66.0, 8.0]);
    let expected_hits = scan.iter().filter(|value| window.intersects(&value.bounds));
    assert_eq!(
        sorted_ids(tree.query(Predicate::Intersects(window))),
        sorted_ids(expected_hits)
    );

    let mut expected_nearest: Vec<&Entry> = scan.iter().collect();
    expected_nearest.sort_by(|left, right| {
        left.bounds
            .comparable_min_distance_to([43.25, 2.0])
            .total_cmp(&right.bounds.comparable_min_distance_to([43.25, 2.0]))
            .then_with(|| left.id.cmp(&right.id))
    });
    let expected_distances: Vec<f64> = expected_nearest
        .iter()
        .take(8)
        .map(|value| value.bounds.comparable_min_distance_to([43.25, 2.0]))
        .collect();
    let actual_distances: Vec<f64> = tree
        .nearest([43.25, 2.0], 8)
        .into_iter()
        .map(|value| value.bounds.comparable_min_distance_to([43.25, 2.0]))
        .collect();
    assert_eq!(actual_distances, expected_distances);
}

fn exercise_condense_tree<Params: SplitParameters>() {
    let mut scan: Vec<Entry> = (0..160)
        .map(|id| Entry::point(id, [f64::from(id % 20) * 4.0, f64::from(id / 20)]))
        .collect();
    let mut tree: Rtree<Entry, Params> = scan.clone().into_iter().collect();
    assert_tree_matches_scan(&tree, &scan);

    for id in (0..160).step_by(3) {
        let target = scan.iter().find(|value| value.id == id).unwrap().clone();
        assert_eq!(tree.remove(&target), 1);
        scan.retain(|value| value != &target);
    }
    assert_tree_matches_scan(&tree, &scan);

    let outsider = Entry::point(10_000, [0.0, 0.0]);
    assert_eq!(tree.remove(&outsider), 0);

    while let Some(target) = scan.pop() {
        assert_eq!(tree.remove(&target), 1);
    }
    assert!(tree.is_empty());
    assert_eq!(tree.height(), 1);
    assert_eq!(tree.bounds(), None);

    tree.extend([Entry::point(200, [2.0, 2.0]), Entry::point(201, [3.0, 3.0])]);
    assert_eq!(sorted_ids(&tree), [200, 201]);
    tree.clear();
    assert!(tree.is_empty());
}

#[test]
fn remove_condenses_and_reuses_default_tree() {
    exercise_condense_tree::<boost_geometry::rtree::AsymmetricRStarSplit<6, 2, 12, 4, 4, 4>>();
}

#[test]
fn remove_condenses_linear_quadratic_and_rstar_trees() {
    exercise_condense_tree::<Linear<8, 3>>();
    exercise_condense_tree::<Quadratic<8, 3>>();
    exercise_condense_tree::<RStarSplit<8, 3>>();
}

#[test]
fn count_remove_range_clone_and_iteration_match_boost_container_operations() {
    let duplicate = Entry::point(7, [7.0, 7.0]);
    let other = Entry::point(8, [8.0, 8.0]);
    let mut tree: Rtree<Entry> = [duplicate.clone(), duplicate.clone(), other.clone()]
        .into_iter()
        .collect();

    assert_eq!(tree.count(&duplicate), 2);
    assert_eq!(tree.remove(&duplicate), 1);
    assert_eq!(tree.count(&duplicate), 1);

    let cloned = tree.clone();
    assert_eq!(sorted_ids(&cloned), [7, 8]);
    assert_eq!(tree.remove_all([&duplicate, &other]), 2);
    assert!(tree.is_empty());
}

#[cfg(feature = "serde")]
#[test]
fn serde_round_trip_restores_a_queryable_mutable_tree_through_the_facade() {
    type Parameters = Quadratic<8, 3>;
    let tree: Rtree<(Bounds, u32), Parameters> = (0..40)
        .map(|id| (Bounds::point([f64::from(id), f64::from(id % 5)]), id))
        .collect();

    let encoded = serde_json::to_string(&tree).unwrap();
    let mut restored: Rtree<(Bounds, u32), Parameters> = serde_json::from_str(&encoded).unwrap();

    assert_eq!(restored.len(), tree.len());
    let window = Predicate::Intersects(Bounds::new([10.0, -1.0], [19.0, 6.0]));
    let mut ids: Vec<u32> = restored
        .query(window)
        .into_iter()
        .map(|(_, id)| *id)
        .collect();
    ids.sort_unstable();
    assert_eq!(ids, (10..20).collect::<Vec<_>>());
    assert_eq!(restored.remove(&(Bounds::point([12.0, 2.0]), 12)), 1);
}
