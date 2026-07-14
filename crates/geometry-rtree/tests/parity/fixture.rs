pub const FIELD: f64 = 50_000.0;
pub const CLUSTER_COUNT: usize = 16;
pub const CLUSTER_RADIUS: f64 = 100.0;

pub struct Lcg {
    state: u64,
}

impl Lcg {
    #[must_use]
    pub fn new() -> Self {
        Self {
            state: 0x9E37_79B9_7F4A_7C15,
        }
    }

    #[allow(
        clippy::cast_precision_loss,
        reason = "state >> 11 keeps 53 bits, exact in f64"
    )]
    pub fn next_f64(&mut self) -> f64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.state >> 11) as f64 / (1u64 << 53) as f64
    }
}

impl Default for Lcg {
    fn default() -> Self {
        Self::new()
    }
}

#[must_use]
pub fn uniform(n: usize) -> Vec<[f64; 2]> {
    let mut lcg = Lcg::default();
    (0..n)
        .map(|_| [lcg.next_f64() * FIELD, lcg.next_f64() * FIELD])
        .collect()
}

#[must_use]
pub fn clustered(n: usize) -> Vec<[f64; 2]> {
    let mut lcg = Lcg::new();
    let centers: Vec<[f64; 2]> = (0..CLUSTER_COUNT)
        .map(|_| [lcg.next_f64() * FIELD, lcg.next_f64() * FIELD])
        .collect();
    (0..n)
        .map(|i| {
            let center = centers[i % CLUSTER_COUNT];
            [
                center[0] + lcg.next_f64() * 2.0 * CLUSTER_RADIUS - CLUSTER_RADIUS,
                center[1] + lcg.next_f64() * 2.0 * CLUSTER_RADIUS - CLUSTER_RADIUS,
            ]
        })
        .collect()
}

#[must_use]
pub fn duplicates(n: usize) -> Vec<[f64; 2]> {
    let mut lcg = Lcg::new();
    (0..n / 4)
        .flat_map(|_| {
            let p = [lcg.next_f64() * FIELD, lcg.next_f64() * FIELD];
            [p; 4]
        })
        .collect()
}

#[must_use]
pub fn sorted_by_x(n: usize) -> Vec<[f64; 2]> {
    let mut points = uniform(n);
    points.sort_unstable_by(|a, b| a[0].total_cmp(&b[0]));
    points
}

#[must_use]
pub fn reverse_sorted_by_x(n: usize) -> Vec<[f64; 2]> {
    let mut points = sorted_by_x(n);
    points.reverse();
    points
}

#[must_use]
pub fn one_point(n: usize) -> Vec<[f64; 2]> {
    let mut lcg = Lcg::new();
    let p = [lcg.next_f64() * FIELD, lcg.next_f64() * FIELD];
    vec![p; n]
}

#[must_use]
pub fn vertical_line(n: usize) -> Vec<[f64; 2]> {
    let mut lcg = Lcg::new();
    let x = lcg.next_f64() * FIELD;
    (0..n).map(|_| [x, lcg.next_f64() * FIELD]).collect()
}

// A query takes stream draws 3j+1 and 3j+3 (a discarded draw between);
// a point takes consecutive draws 2i+1, 2i+2 — the pairs can never
// coincide, so the query set is disjoint from the point stream.
#[must_use]
pub fn queries(q: usize) -> Vec<[f64; 2]> {
    let mut lcg = Lcg::new();
    (0..q)
        .map(|_| {
            let x = lcg.next_f64() * FIELD;
            lcg.next_f64();
            let y = lcg.next_f64() * FIELD;
            [x, y]
        })
        .collect()
}

#[must_use]
pub fn squared_distance(p: [f64; 2], q: [f64; 2]) -> f64 {
    let dx = p[0] - q[0];
    let dy = p[1] - q[1];
    dx * dx + dy * dy
}

#[must_use]
pub fn knn_scan(points: &[[f64; 2]], q: [f64; 2], k: usize) -> Vec<f64> {
    let mut dists: Vec<f64> = points.iter().map(|&p| squared_distance(p, q)).collect();
    dists.sort_unstable_by(f64::total_cmp);
    dists.truncate(k);
    dists
}

#[must_use]
pub fn range_scan(points: &[[f64; 2]], min: [f64; 2], max: [f64; 2]) -> Vec<u32> {
    points
        .iter()
        .enumerate()
        .filter(|(_, p)| p[0] >= min[0] && p[0] <= max[0] && p[1] >= min[1] && p[1] <= max[1])
        .map(|(i, _)| u32::try_from(i).expect("fixture sizes fit u32"))
        .collect()
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "the oracle contract is exact f64 equality (R2)"
)]
mod tests {
    const POINTS: [[f64; 2]; 4] = [[0.0, 0.0], [3.0, 4.0], [1.0, 0.0], [0.0, 2.0]];

    #[test]
    fn knn_scan_hand_case() {
        assert_eq!(super::knn_scan(&POINTS, [0.0, 0.0], 3), [0.0, 1.0, 4.0]);
        assert_eq!(super::knn_scan(&POINTS, [0.0, 0.0], 0), [0.0; 0]);
        assert_eq!(
            super::knn_scan(&POINTS, [0.0, 0.0], 9),
            [0.0, 1.0, 4.0, 25.0]
        );
    }

    #[test]
    fn range_scan_hand_case() {
        assert_eq!(
            super::range_scan(&POINTS, [0.0, 0.0], [1.0, 2.0]),
            [0, 2, 3]
        );
        assert_eq!(super::range_scan(&POINTS, [3.0, 4.0], [3.0, 4.0]), [1]);
        assert_eq!(
            super::range_scan(&POINTS, [8.0, 8.0], [9.0, 9.0]),
            [0u32; 0]
        );
    }
}
