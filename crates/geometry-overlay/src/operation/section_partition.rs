//! The order `get_turns` puts its turns in.
//!
//! `get_turns` does not compare every segment of one operand against every
//! segment of the other. It cuts each into **sections** — runs of consecutive
//! segments heading the same way — and hands the two lists to
//! `geometry::partition`, which recursively halves the plane and visits the
//! pairs of sections that can still meet. The turns land in `m_turns` in the
//! order that walk finds them, `traverse` starts a ring at the first turn it
//! has not used, and `add_rings` emits the rings in the order `traverse`
//! made them — so this order is the order of the polygons in the result.
//!
//! Under seventeen sections on either side `partition` skips the division and
//! runs the plain nested loop, which is why a single small polygon against
//! another comes out in plain section order and a multi-polygon does not.
//!
//! Mirrors `boost/geometry/algorithms/detail/partition.hpp`
//! (`partition::apply` and `partition_two_ranges::apply`).

use alloc::vec::Vec;

/// The `min_elements` `partition::apply` defaults to. Both collections must
/// be *larger* than this for the division to happen at all.
const MIN_ELEMENTS: usize = 16;

/// C++: `recurse_ok`'s `level < 100`.
const MAX_LEVEL: usize = 100;

/// An axis-aligned box, which is all `partition` knows about a section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Bounds {
    pub min: [f64; 2],
    pub max: [f64; 2],
}

impl Bounds {
    pub(crate) fn around(first: [f64; 2], second: [f64; 2]) -> Self {
        Self {
            min: [first[0].min(second[0]), first[1].min(second[1])],
            max: [first[0].max(second[0]), first[1].max(second[1])],
        }
    }

    pub(crate) fn expand(&mut self, other: &Self) {
        for axis in 0..2 {
            self.min[axis] = self.min[axis].min(other.min[axis]);
            self.max[axis] = self.max[axis].max(other.max[axis]);
        }
    }

    /// C++: `! disjoint_box_box`, which compares with `<` — so two boxes that
    /// merely touch do overlap.
    fn overlaps(&self, other: &Self) -> bool {
        (0..2).all(|axis| self.max[axis] >= other.min[axis] && other.max[axis] >= self.min[axis])
    }

    /// C++: `divide_box`, splitting at the midpoint of one dimension.
    #[expect(
        clippy::manual_midpoint,
        reason = "C++ divides the interval as `(mi + ma) / 2`, and where the two                   disagree the split lands on a different coordinate and the walk                   visits a different order"
    )]
    fn halves(&self, axis: usize) -> (Self, Self) {
        let middle = (self.min[axis] + self.max[axis]) / 2.0;
        let mut lower = *self;
        let mut upper = *self;
        lower.max[axis] = middle;
        upper.min[axis] = middle;
        (lower, upper)
    }
}

fn enclosing(bounds: &[Bounds], of: &[usize]) -> Bounds {
    let mut total = bounds[of[0]];
    for &index in &of[1..] {
        total.expand(&bounds[index]);
    }
    total
}

/// C++: `divide_into_subsets`. A box that reaches into both halves is
/// "exceeding" and is matched against everything rather than descending.
fn split(
    bounds: &[Bounds],
    of: &[usize],
    lower_box: &Bounds,
    upper_box: &Bounds,
) -> [Vec<usize>; 3] {
    let mut lower = Vec::new();
    let mut upper = Vec::new();
    let mut exceeding = Vec::new();
    for &index in of {
        let in_lower = lower_box.overlaps(&bounds[index]);
        let in_upper = upper_box.overlaps(&bounds[index]);
        match (in_lower, in_upper) {
            (true, true) => exceeding.push(index),
            (true, false) => lower.push(index),
            (false, true) => upper.push(index),
            // C++: "Is nowhere", which the overlaps policy may allow.
            (false, false) => {}
        }
    }
    [lower, upper, exceeding]
}

/// C++: `recurse_ok`, which also caps the depth at 100 levels.
fn deep_enough(of: &[usize], level: usize) -> bool {
    of.len() >= MIN_ELEMENTS && level < MAX_LEVEL
}

/// One run of the divide-and-conquer, over one pair of section lists.
struct Walk<'a> {
    first: &'a [Bounds],
    second: &'a [Bounds],
    visited: Vec<(usize, usize)>,
}

impl Walk<'_> {
    /// C++: `handle_two`, the quadratic fallback — first collection outer.
    fn pair_up(&mut self, ones: &[usize], twos: &[usize]) {
        for &one in ones {
            for &two in twos {
                self.visited.push((one, two));
            }
        }
    }

    /// Descend if both sides are still worth dividing, else match them all.
    fn narrow(&mut self, box_: &Bounds, ones: &[usize], twos: &[usize], level: usize, axis: usize) {
        if deep_enough(ones, level) && deep_enough(twos, level) {
            self.descend(box_, ones, twos, level + 1, 1 - axis);
        } else {
            self.pair_up(ones, twos);
        }
    }

    /// The straddlers of one side against both halves of the other.
    ///
    /// C++ decides this for the two halves *together* — all three lists have
    /// to be big enough or none of them descends — so it is not two
    /// independent `narrow` calls.
    fn against_both_halves(
        &mut self,
        straddlers: &[usize],
        halves: (&[usize], &[usize]),
        straddlers_lead: bool,
        level: usize,
        axis: usize,
    ) {
        let (lower, upper) = halves;
        let bounds = if straddlers_lead {
            self.first
        } else {
            self.second
        };
        if deep_enough(lower, level) && deep_enough(upper, level) && deep_enough(straddlers, level)
        {
            let box_ = enclosing(bounds, straddlers);
            let (level, axis) = (level + 1, 1 - axis);
            if straddlers_lead {
                self.descend(&box_, straddlers, lower, level, axis);
                self.descend(&box_, straddlers, upper, level, axis);
            } else {
                self.descend(&box_, lower, straddlers, level, axis);
                self.descend(&box_, upper, straddlers, level, axis);
            }
        } else if straddlers_lead {
            self.pair_up(straddlers, lower);
            self.pair_up(straddlers, upper);
        } else {
            self.pair_up(lower, straddlers);
            self.pair_up(upper, straddlers);
        }
    }

    /// C++: `partition_two_ranges<Dimension, Box>::apply`.
    fn descend(
        &mut self,
        box_: &Bounds,
        ones: &[usize],
        twos: &[usize],
        level: usize,
        axis: usize,
    ) {
        let (lower_box, upper_box) = box_.halves(axis);
        let [lower1, upper1, exceeding1] = split(self.first, ones, &lower_box, &upper_box);
        let [lower2, upper2, exceeding2] = split(self.second, twos, &lower_box, &upper_box);

        if !exceeding1.is_empty() {
            let mut box_ = enclosing(self.first, &exceeding1);
            if !exceeding2.is_empty() {
                box_.expand(&enclosing(self.second, &exceeding2));
            }
            self.narrow(&box_, &exceeding1, &exceeding2, level, axis);
            self.against_both_halves(&exceeding1, (&lower2, &upper2), true, level, axis);
        }
        if !exceeding2.is_empty() {
            self.against_both_halves(&exceeding2, (&lower1, &upper1), false, level, axis);
        }
        self.narrow(&lower_box, &lower1, &lower2, level, axis);
        self.narrow(&upper_box, &upper1, &upper2, level, axis);
    }
}

/// Every pair of sections `get_turns` looks at, in the order it looks.
///
/// C++: `geometry::partition<box_type>::apply(sec1, sec2, visitor, …)`.
fn visit_order(first: &[Bounds], second: &[Bounds]) -> Vec<(usize, usize)> {
    if first.is_empty() || second.is_empty() {
        return Vec::new();
    }
    let ones: Vec<usize> = (0..first.len()).collect();
    let twos: Vec<usize> = (0..second.len()).collect();
    let mut walk = Walk {
        first,
        second,
        visited: Vec::new(),
    };
    if first.len() > MIN_ELEMENTS && second.len() > MIN_ELEMENTS {
        let mut total = enclosing(first, &ones);
        total.expand(&enclosing(second, &twos));
        walk.descend(&total, &ones, &twos, 0, 0);
    } else {
        walk.pair_up(&ones, &twos);
    }
    walk.visited
}

/// Where each pair of sections falls in that order.
///
/// A pair the walk never reaches cannot hold a turn — the two sections' boxes
/// would have to be apart — so a lookup that misses sorts last rather than
/// claiming a position.
pub(crate) struct VisitRank(Vec<((usize, usize), usize)>);

impl VisitRank {
    pub(crate) fn of(first: &[Bounds], second: &[Bounds]) -> Self {
        let mut ranked: Vec<((usize, usize), usize)> = visit_order(first, second)
            .into_iter()
            .enumerate()
            .map(|(rank, pair)| (pair, rank))
            .collect();
        ranked.sort_unstable();
        ranked.dedup_by_key(|(pair, _)| *pair);
        Self(ranked)
    }

    pub(crate) fn rank(&self, first: usize, second: usize) -> usize {
        self.0
            .binary_search_by_key(&(first, second), |&(pair, _)| pair)
            .map_or(usize::MAX, |at| self.0[at].1)
    }
}
