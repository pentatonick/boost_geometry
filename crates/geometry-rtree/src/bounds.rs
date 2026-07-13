//! The axis-aligned bounding box used for tree keys.
//!
//! Mirrors the role of `boost::geometry::model::box` inside the index
//! (`index/detail/bounded_view.hpp`, `index/detail/rtree/node/`). The
//! index does a lot of box arithmetic — area, enlargement, union,
//! intersection — so the port uses a small concrete `Bounds` value
//! rather than the generic `model::Box`, keeping the hot path free of
//! trait dispatch.
//!
//! 2D, `f64` for v1.

/// An axis-aligned bounding box in the plane.
///
/// Mirrors the minimum bounding rectangle stored at every node of a
/// Boost rtree.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    /// Lower corner `[x_min, y_min]`.
    pub min: [f64; 2],
    /// Upper corner `[x_max, y_max]`.
    pub max: [f64; 2],
}

impl Bounds {
    /// A box from explicit corners.
    #[must_use]
    pub const fn new(min: [f64; 2], max: [f64; 2]) -> Self {
        Self { min, max }
    }

    /// A degenerate box at a single point.
    #[must_use]
    pub const fn point(p: [f64; 2]) -> Self {
        Self { min: p, max: p }
    }

    /// The area (2D measure) of the box.
    #[must_use]
    pub fn area(&self) -> f64 {
        (self.max[0] - self.min[0]) * (self.max[1] - self.min[1])
    }

    /// Half the perimeter — Boost's "margin", used by the R\* split.
    #[must_use]
    pub fn half_perimeter(&self) -> f64 {
        (self.max[0] - self.min[0]) + (self.max[1] - self.min[1])
    }

    /// The smallest box containing both `self` and `other`.
    #[must_use]
    pub fn union(&self, other: &Bounds) -> Bounds {
        Bounds {
            min: [self.min[0].min(other.min[0]), self.min[1].min(other.min[1])],
            max: [self.max[0].max(other.max[0]), self.max[1].max(other.max[1])],
        }
    }

    /// How much `self`'s area would grow to also contain `other` —
    /// Boost's enlargement metric for `choose_next_node`.
    #[must_use]
    pub fn enlargement(&self, other: &Bounds) -> f64 {
        self.union(other).area() - self.area()
    }

    /// Whether the two boxes share any point (closed boxes, so touching
    /// counts).
    #[must_use]
    pub fn intersects(&self, other: &Bounds) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
    }

    /// Whether `self` fully contains `other`.
    #[must_use]
    pub fn contains(&self, other: &Bounds) -> bool {
        self.min[0] <= other.min[0]
            && self.max[0] >= other.max[0]
            && self.min[1] <= other.min[1]
            && self.max[1] >= other.max[1]
    }

    /// The minimum distance from a query point to this box (0 inside).
    /// Used by nearest-neighbour pruning.
    #[must_use]
    pub fn min_distance_to(&self, p: [f64; 2]) -> f64 {
        self.comparable_min_distance_to(p).sqrt()
    }

    /// The SQUARED minimum distance from a query point to this box —
    /// same ordering as [`min_distance_to`](Self::min_distance_to)
    /// without the square root, for hot comparison paths. The analogue
    /// of `boost::geometry::comparable_distance`.
    #[must_use]
    pub fn comparable_min_distance_to(&self, p: [f64; 2]) -> f64 {
        let dx = clamp_gap(p[0], self.min[0], self.max[0]);
        let dy = clamp_gap(p[1], self.min[1], self.max[1]);
        dx * dx + dy * dy
    }

    /// The centroid, used by the STR bulk-load sort.
    #[must_use]
    pub fn center(&self) -> [f64; 2] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
        ]
    }
}

/// Distance from `v` to the interval `[lo, hi]` (0 if inside).
fn clamp_gap(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo - v
    } else if v > hi {
        v - hi
    } else {
        0.0
    }
}

/// The union of a non-empty slice of boxes.
///
/// # Panics
///
/// Panics if `boxes` is empty — callers hold that invariant (a node
/// always has at least one child).
#[must_use]
pub fn union_all(boxes: &[Bounds]) -> Bounds {
    let mut acc = boxes[0];
    for b in &boxes[1..] {
        acc = acc.union(b);
    }
    acc
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "box arithmetic on exact integer-valued literals"
)]
mod tests {
    use super::{Bounds, union_all};

    #[test]
    fn area_and_perimeter() {
        let b = Bounds::new([0.0, 0.0], [2.0, 3.0]);
        assert_eq!(b.area(), 6.0);
        assert_eq!(b.half_perimeter(), 5.0);
    }

    #[test]
    fn union_and_enlargement() {
        let a = Bounds::new([0.0, 0.0], [1.0, 1.0]);
        let b = Bounds::new([2.0, 2.0], [3.0, 3.0]);
        let u = a.union(&b);
        assert_eq!(u, Bounds::new([0.0, 0.0], [3.0, 3.0]));
        // a has area 1; union has area 9; enlargement is 8.
        assert_eq!(a.enlargement(&b), 8.0);
    }

    #[test]
    fn intersects_and_contains() {
        let a = Bounds::new([0.0, 0.0], [4.0, 4.0]);
        let inside = Bounds::new([1.0, 1.0], [2.0, 2.0]);
        let overlap = Bounds::new([3.0, 3.0], [5.0, 5.0]);
        let apart = Bounds::new([9.0, 9.0], [10.0, 10.0]);
        assert!(a.contains(&inside));
        assert!(a.intersects(&overlap));
        assert!(!a.contains(&overlap));
        assert!(!a.intersects(&apart));
    }

    #[test]
    fn min_distance() {
        let b = Bounds::new([0.0, 0.0], [2.0, 2.0]);
        assert_eq!(b.min_distance_to([1.0, 1.0]), 0.0); // inside
        assert_eq!(b.min_distance_to([5.0, 1.0]), 3.0); // right of it
        assert_eq!(b.min_distance_to([5.0, 6.0]), 5.0); // corner (3,4)→5
        assert_eq!(b.comparable_min_distance_to([5.0, 6.0]), 25.0);
    }

    #[test]
    fn union_all_of_many() {
        let boxes = [
            Bounds::point([1.0, 1.0]),
            Bounds::point([-2.0, 3.0]),
            Bounds::point([4.0, -1.0]),
        ];
        assert_eq!(union_all(&boxes), Bounds::new([-2.0, -1.0], [4.0, 3.0]));
    }
}
