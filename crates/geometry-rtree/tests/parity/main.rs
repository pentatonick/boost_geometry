mod fixture;

use fixture::{
    FIELD, clustered, duplicates, knn_scan, one_point, queries, range_scan, reverse_sorted_by_x,
    sorted_by_x, squared_distance, uniform, vertical_line,
};
use geometry_rtree::{
    AsymmetricQuadratic, AsymmetricRStarSplit, Bounds, Predicate, Quadratic, Rtree, SplitParameters,
};

const K: usize = 8;
const HALF: f64 = 500.0;
const QUERY_COUNT: usize = 20;

type BoostTree<P = Quadratic> = Rtree<(Bounds, u32), P>;

fn families() -> [Vec<[f64; 2]>; 3] {
    [uniform(5_000), clustered(5_000), duplicates(4_000)]
}

fn boost_tree<P: SplitParameters>(points: &[[f64; 2]]) -> BoostTree<P> {
    points
        .iter()
        .enumerate()
        .map(|(i, &p)| (Bounds::point(p), u32::try_from(i).expect("fits u32")))
        .collect()
}

fn boost_knn_distances<P: SplitParameters>(tree: &BoostTree<P>, q: [f64; 2], k: usize) -> Vec<f64> {
    tree.nearest(q, k)
        .iter()
        .map(|(b, _)| squared_distance(b.min, q))
        .collect()
}

fn boost_range_ids<P: SplitParameters>(
    tree: &BoostTree<P>,
    min: [f64; 2],
    max: [f64; 2],
) -> Vec<u32> {
    let mut ids: Vec<u32> = tree
        .query(Predicate::Intersects(Bounds::new(min, max)))
        .iter()
        .map(|(_, id)| *id)
        .collect();
    ids.sort_unstable();
    ids
}

fn window(q: [f64; 2]) -> ([f64; 2], [f64; 2]) {
    ([q[0] - HALF, q[1] - HALF], [q[0] + HALF, q[1] + HALF])
}

#[allow(clippy::float_cmp, reason = "R2 mandates exact f64 sequence equality")]
fn knn_parity_case<P: SplitParameters>() {
    for points in families() {
        let tree = boost_tree::<P>(&points);
        for q in queries(QUERY_COUNT) {
            assert_eq!(
                boost_knn_distances(&tree, q, K),
                knn_scan(&points, q, K),
                "boost knn diverges from scan oracle at query {q:?}"
            );
        }
    }
}

#[test]
fn boost_knn_matches_scan_max6() {
    knn_parity_case::<Quadratic<6, 2>>();
}

#[test]
fn boost_knn_matches_scan_max8() {
    knn_parity_case::<Quadratic<8, 3>>();
}

#[test]
fn boost_knn_matches_scan_max16() {
    knn_parity_case::<Quadratic<16, 4>>();
}

#[test]
fn boost_knn_matches_scan_max32() {
    knn_parity_case::<Quadratic<32, 9>>();
}

#[test]
fn boost_knn_matches_scan_branch8_leaf32() {
    knn_parity_case::<AsymmetricQuadratic<8, 3, 32, 9>>();
}

#[test]
fn boost_knn_matches_scan_rstar_split_branch8_leaf32() {
    knn_parity_case::<AsymmetricRStarSplit<8, 3, 32, 9>>();
}

fn range_parity_case<P: SplitParameters>() {
    for points in families() {
        let tree = boost_tree::<P>(&points);
        for q in queries(QUERY_COUNT) {
            let (min, max) = window(q);
            assert_eq!(
                boost_range_ids(&tree, min, max),
                range_scan(&points, min, max),
                "boost range diverges from scan oracle at query {q:?}"
            );
        }
    }
}

#[test]
fn boost_range_matches_scan_max6() {
    range_parity_case::<Quadratic<6, 2>>();
}

#[test]
fn boost_range_matches_scan_max8() {
    range_parity_case::<Quadratic<8, 3>>();
}

#[test]
fn boost_range_matches_scan_max16() {
    range_parity_case::<Quadratic<16, 4>>();
}

#[test]
fn boost_range_matches_scan_max32() {
    range_parity_case::<Quadratic<32, 9>>();
}

#[test]
fn boost_range_matches_scan_branch8_leaf32() {
    range_parity_case::<AsymmetricQuadratic<8, 3, 32, 9>>();
}

#[test]
fn boost_range_matches_scan_rstar_split_branch8_leaf32() {
    range_parity_case::<AsymmetricRStarSplit<8, 3, 32, 9>>();
}

fn adversarial_families() -> [Vec<[f64; 2]>; 4] {
    [
        sorted_by_x(5_000),
        reverse_sorted_by_x(5_000),
        one_point(5_000),
        vertical_line(5_000),
    ]
}

#[allow(clippy::float_cmp, reason = "R4 mandates exact f64 sequence equality")]
fn adversarial_parity_case<P: SplitParameters>() {
    for points in adversarial_families() {
        let tree = boost_tree::<P>(&points);
        assert_eq!(tree.len(), points.len());
        for q in queries(QUERY_COUNT) {
            let (min, max) = window(q);
            assert_eq!(
                boost_range_ids(&tree, min, max),
                range_scan(&points, min, max),
                "adversarial-build range diverges from scan oracle at query {q:?}"
            );
            assert_eq!(
                boost_knn_distances(&tree, q, K),
                knn_scan(&points, q, K),
                "adversarial-build knn diverges from scan oracle at query {q:?}"
            );
        }
    }
}

#[test]
fn adversarial_bulk_inputs_match_scan_max6() {
    adversarial_parity_case::<Quadratic<6, 2>>();
}

#[test]
fn adversarial_bulk_inputs_match_scan_max8() {
    adversarial_parity_case::<Quadratic<8, 3>>();
}

#[test]
fn adversarial_bulk_inputs_match_scan_max16() {
    adversarial_parity_case::<Quadratic<16, 4>>();
}

#[test]
fn adversarial_bulk_inputs_match_scan_max32() {
    adversarial_parity_case::<Quadratic<32, 9>>();
}

#[test]
fn adversarial_bulk_inputs_match_scan_branch8_leaf32() {
    adversarial_parity_case::<AsymmetricQuadratic<8, 3, 32, 9>>();
}

#[test]
fn adversarial_bulk_inputs_match_scan_rstar_split_branch8_leaf32() {
    adversarial_parity_case::<AsymmetricRStarSplit<8, 3, 32, 9>>();
}

#[allow(clippy::float_cmp, reason = "R4 mandates exact f64 sequence equality")]
fn insert_split_stress_case<P: SplitParameters>() {
    let points = one_point(5_000);
    let mut tree: BoostTree<P> = Rtree::new();
    for (i, &p) in points.iter().enumerate() {
        tree.insert((Bounds::point(p), u32::try_from(i).expect("fits u32")));
    }
    assert_eq!(tree.len(), points.len());
    let q = points[0];
    assert_eq!(
        boost_knn_distances(&tree, q, K),
        knn_scan(&points, q, K),
        "insert-built all-equal-key knn diverges from scan oracle"
    );
    let (min, max) = window(q);
    assert_eq!(
        boost_range_ids(&tree, min, max),
        range_scan(&points, min, max),
        "insert-built all-equal-key range diverges from scan oracle"
    );
}

#[test]
fn insert_split_survives_all_equal_keys_max6() {
    insert_split_stress_case::<Quadratic<6, 2>>();
}

#[test]
fn insert_split_survives_all_equal_keys_max8() {
    insert_split_stress_case::<Quadratic<8, 3>>();
}

#[test]
fn insert_split_survives_all_equal_keys_max16() {
    insert_split_stress_case::<Quadratic<16, 4>>();
}

#[test]
fn insert_split_survives_all_equal_keys_max32() {
    insert_split_stress_case::<Quadratic<32, 9>>();
}

#[test]
fn insert_split_survives_all_equal_keys_branch8_leaf32() {
    insert_split_stress_case::<AsymmetricQuadratic<8, 3, 32, 9>>();
}

#[test]
fn insert_split_survives_all_equal_keys_rstar_split_branch8_leaf32() {
    insert_split_stress_case::<AsymmetricRStarSplit<8, 3, 32, 9>>();
}

#[test]
fn bulk_build_is_deterministic() {
    let points = uniform(10_000);
    let first = boost_tree::<Quadratic>(&points);
    let second = boost_tree::<Quadratic>(&points);
    assert_eq!(first.height(), second.height());
    for q in queries(QUERY_COUNT) {
        let (min, max) = window(q);
        let predicate = Predicate::Intersects(Bounds::new(min, max));
        let first_ids: Vec<u32> = first.query_iter(predicate).map(|(_, id)| *id).collect();
        let second_ids: Vec<u32> = second.query_iter(predicate).map(|(_, id)| *id).collect();
        assert_eq!(
            first_ids, second_ids,
            "two bulk loads of the same input diverge at query {q:?}"
        );
    }
}

fn boost_predicate_ids<P: SplitParameters>(tree: &BoostTree<P>, predicate: Predicate) -> Vec<u32> {
    let mut ids: Vec<u32> = tree.query(predicate).iter().map(|(_, id)| *id).collect();
    ids.sort_unstable();
    ids
}

// `value.contains(query)` expanded for a degenerate point box: a point
// value only contains the query box `[min, max]` when the point sits
// at or "outside" min/max in the inverted sense a containing box
// needs — point <= min on the low corner, point >= max on the high
// corner. Do NOT "fix" this into `range_scan`'s `>= min && <= max`;
// that is the opposite predicate (point inside the window).
fn contains_scan(points: &[[f64; 2]], min: [f64; 2], max: [f64; 2]) -> Vec<u32> {
    points
        .iter()
        .enumerate()
        .filter(|(_, p)| p[0] <= min[0] && p[0] >= max[0] && p[1] <= min[1] && p[1] >= max[1])
        .map(|(i, _)| u32::try_from(i).expect("fixture sizes fit u32"))
        .collect()
}

#[test]
fn window_size_predicate_matrix_matches_scan() {
    for points in [uniform(5_000), clustered(5_000)] {
        let tree = boost_tree::<Quadratic>(&points);
        for q in queries(QUERY_COUNT) {
            for half in [50.0, 500.0, 5_000.0, 50_000.0] {
                let min = [q[0] - half, q[1] - half];
                let max = [q[0] + half, q[1] + half];
                let window = Bounds::new(min, max);
                let point_hits = range_scan(&points, min, max);
                for (predicate, oracle) in [
                    (Predicate::Intersects(window), point_hits.clone()),
                    (Predicate::CoveredBy(window), point_hits.clone()),
                    (
                        Predicate::Contains(window),
                        contains_scan(&points, min, max),
                    ),
                ] {
                    assert_eq!(
                        boost_predicate_ids(&tree, predicate),
                        oracle,
                        "boost diverges from scan oracle for {predicate:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn covers_matches_a_degenerate_point_window() {
    let points = duplicates(4_000);
    let tree = boost_tree::<Quadratic>(&points);
    let target = points[0];
    let window = Bounds::point(target);
    let oracle = contains_scan(&points, target, target);
    assert!(
        oracle.len() >= 4,
        "the duplicates fixture must place >= 4 copies of the first point, got {}",
        oracle.len()
    );
    assert_eq!(
        boost_predicate_ids(&tree, Predicate::Covers(window)),
        oracle,
        "boost Covers diverges from scan oracle on a non-empty degenerate window"
    );
    assert!(boost_predicate_ids(&tree, Predicate::Contains(window)).is_empty());
}

#[test]
fn query_iter_collect_equals_query() {
    for points in families() {
        let tree = boost_tree::<Quadratic>(&points);
        for q in queries(QUERY_COUNT) {
            let (min, max) = window(q);
            let predicate = Predicate::Intersects(Bounds::new(min, max));
            let mut eager: Vec<u32> = tree.query(predicate).iter().map(|(_, id)| *id).collect();
            let mut lazy: Vec<u32> = tree.query_iter(predicate).map(|(_, id)| *id).collect();
            eager.sort_unstable();
            lazy.sort_unstable();
            assert_eq!(lazy, eager, "query_iter diverges from query at query {q:?}");
        }
    }
}

#[test]
#[allow(clippy::float_cmp, reason = "R3 mandates exact f64 sequence equality")]
fn nearest_iter_take_matches_nearest() {
    for points in families() {
        let tree = boost_tree::<Quadratic>(&points);
        for q in queries(QUERY_COUNT) {
            for k in [1, 8, 64] {
                let stream: Vec<f64> = tree
                    .nearest_iter(q)
                    .take(k)
                    .map(|(b, _)| squared_distance(b.min, q))
                    .collect();
                assert_eq!(
                    stream,
                    boost_knn_distances(&tree, q, k),
                    "nearest_iter take({k}) diverges from nearest at query {q:?}"
                );
            }
        }
    }
}

#[test]
fn nearest_iter_full_drain_is_sorted() {
    for points in families() {
        let tree = boost_tree::<Quadratic>(&points);
        let q = queries(1)[0];
        let drained: Vec<(f64, u32)> = tree
            .nearest_iter(q)
            .map(|(b, id)| (squared_distance(b.min, q), *id))
            .collect();
        assert!(
            drained
                .windows(2)
                .all(|pair| pair[0].0.total_cmp(&pair[1].0).is_le()),
            "full-drain distances must be non-decreasing (R2)"
        );
        let mut ids: Vec<u32> = drained.iter().map(|(_, id)| *id).collect();
        ids.sort_unstable();
        let every_id: Vec<u32> = (0..u32::try_from(points.len()).expect("fits u32")).collect();
        assert_eq!(
            ids, every_id,
            "full drain must yield every value exactly once (R2)"
        );
    }
}

#[test]
fn iterators_are_fused() {
    let points = uniform(500);
    let tree = boost_tree::<Quadratic>(&points);
    let q = queries(1)[0];
    let (min, max) = window(q);

    let field = Predicate::Intersects(Bounds::new([0.0, 0.0], [FIELD, FIELD]));
    let mut range = tree.query_iter(field);
    assert_eq!(range.by_ref().count(), points.len());
    for _ in 0..3 {
        assert!(range.next().is_none());
    }

    let mut stream = tree.nearest_iter(q);
    assert_eq!(stream.by_ref().count(), points.len());
    for _ in 0..3 {
        assert!(stream.next().is_none());
    }

    let empty = boost_tree::<Quadratic>(&[]);
    assert!(
        empty
            .query_iter(Predicate::Intersects(Bounds::new(min, max)))
            .next()
            .is_none()
    );
    assert!(empty.nearest_iter(q).next().is_none());

    let no_match = Predicate::Intersects(Bounds::new([-2.0, -2.0], [-1.0, -1.0]));
    assert!(tree.query_iter(no_match).next().is_none());
}

#[test]
fn knn_k_zero_returns_nothing() {
    let points = uniform(100);
    let q = queries(1)[0];
    assert!(knn_scan(&points, q, 0).is_empty());
    assert!(boost_knn_distances(&boost_tree::<Quadratic>(&points), q, 0).is_empty());
}

#[test]
#[allow(clippy::float_cmp, reason = "R2 mandates exact f64 sequence equality")]
fn knn_k_beyond_len_returns_all_ascending() {
    let points = uniform(50);
    let q = queries(1)[0];
    let oracle = knn_scan(&points, q, 100);
    assert_eq!(oracle.len(), 50);
    assert_eq!(
        boost_knn_distances(&boost_tree::<Quadratic>(&points), q, 100),
        oracle
    );
}

#[test]
#[allow(clippy::float_cmp, reason = "R2 mandates exact f64 sequence equality")]
fn knn_k_equals_len_returns_all_ascending() {
    let points = uniform(64);
    let q = queries(1)[0];
    let oracle = knn_scan(&points, q, 64);
    assert_eq!(oracle.len(), 64);
    assert_eq!(
        boost_knn_distances(&boost_tree::<Quadratic>(&points), q, 64),
        oracle
    );
}

#[test]
#[allow(clippy::float_cmp, reason = "R2 mandates exact f64 sequence equality")]
fn knn_query_coincident_with_a_stored_point() {
    let points = uniform(500);
    let q = points[123];
    let oracle = knn_scan(&points, q, K);
    assert_eq!(oracle[0], 0.0);
    assert_eq!(
        boost_knn_distances(&boost_tree::<Quadratic>(&points), q, K),
        oracle
    );
}

#[test]
fn empty_tree_returns_nothing() {
    let points: Vec<[f64; 2]> = Vec::new();
    let boost = boost_tree::<Quadratic>(&points);
    let q = queries(1)[0];
    let (min, max) = window(q);
    assert!(boost_knn_distances(&boost, q, K).is_empty());
    assert!(boost_range_ids(&boost, min, max).is_empty());
}

#[test]
fn window_covering_the_field_returns_every_id() {
    let points = uniform(1_000);
    let min = [0.0, 0.0];
    let max = [FIELD, FIELD];
    let all: Vec<u32> = (0..1_000).collect();
    assert_eq!(range_scan(&points, min, max), all);
    assert_eq!(
        boost_range_ids(&boost_tree::<Quadratic>(&points), min, max),
        all
    );
}

#[test]
fn window_boundary_touch_is_inclusive() {
    let points = vec![[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]];
    let boost = boost_tree::<Quadratic>(&points);
    for (min, max, expected) in [
        ([0.0, 0.0], [1.0, 1.0], vec![0u32, 1]),
        ([1.0, 1.0], [3.0, 3.0], vec![1, 2]),
        ([2.0, 2.0], [2.0, 2.0], vec![2]),
    ] {
        assert_eq!(range_scan(&points, min, max), expected);
        assert_eq!(boost_range_ids(&boost, min, max), expected);
    }
}
