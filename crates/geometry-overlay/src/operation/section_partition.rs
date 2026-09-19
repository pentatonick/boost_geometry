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

#[cfg(test)]
mod tests {
    //! What the walk must reach, and the order it must reach it in.
    //!
    //! `partition` is not merely a faster nested loop. Which pairs it visits
    //! decides which turns exist at all, and the order it visits them in is the
    //! order they land in `m_turns` — and so the order of the polygons in the
    //! result. Two claims carry that, and both are checked here against a
    //! layout that forces every arm of the division to run.

    use super::{Bounds, MIN_ELEMENTS, VisitRank, visit_order};

    /// A box by its lower corner and its extent, which is how the layouts below
    /// read most clearly.
    fn box_at(x: f64, y: f64, width: f64, height: f64) -> Bounds {
        Bounds::around([x, y], [x + width, y + height])
    }

    /// A layout that makes the division do all of its work.
    ///
    /// Sixteen boxes wholly left of the midpoint, sixteen wholly right of it,
    /// and sixteen straddling it. Sixteen is exactly `min_elements`, so each of
    /// the three lists is large enough for `recurse_ok` and the straddlers
    /// descend against both halves rather than falling back on the quadratic
    /// arm — the case `divide_into_subsets` exists for. The staircase in `y`
    /// makes the next level's split, which is on the other dimension, cut the
    /// lists again instead of handing them all to one half.
    ///
    /// `lean` shifts the second operand off the first so the two are not the
    /// same list, which is what a real pair of operands looks like.
    fn straddled_belt(lean: f64) -> Vec<Bounds> {
        /// The staircase's rise per box, walked as a `f64` so no index is ever
        /// cast into one.
        fn staircase(mut place: impl FnMut(f64)) {
            let mut along = 0.0_f64;
            for _ in 0..MIN_ELEMENTS {
                place(along);
                along += 2.0;
            }
        }

        let mut boxes = Vec::new();
        staircase(|along| boxes.push(box_at(along + lean, along + lean, 1.0, 1.0)));
        staircase(|along| boxes.push(box_at(100.0 + along + lean, along + lean, 1.0, 1.0)));
        staircase(|along| boxes.push(box_at(30.0, along + lean, 75.0, 1.0)));
        boxes
    }

    /// `count` boxes marching one unit apart, each ten units wide, so every one
    /// of them overlaps every other and the layout says nothing about the order
    /// beyond what the walk chooses.
    fn overlapping_row(count: usize) -> Vec<Bounds> {
        let mut boxes = Vec::new();
        let mut at = 0.0_f64;
        for _ in 0..count {
            boxes.push(box_at(at, 0.0, 10.0, 10.0));
            at += 1.0;
        }
        boxes
    }

    fn overlapping_pairs(first: &[Bounds], second: &[Bounds]) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();
        for (one, first_box) in first.iter().enumerate() {
            for (two, second_box) in second.iter().enumerate() {
                if first_box.overlaps(second_box) {
                    pairs.push((one, two));
                }
            }
        }
        pairs
    }

    /// The division must not lose a pair that could hold a turn.
    ///
    /// This is the property the whole recursion rests on: a pair the walk skips
    /// is a pair `get_turns` never tests, so if two sections whose boxes meet
    /// could be skipped, a turn could go missing and the traversal would trace
    /// a ring that is not there. The walk is allowed to visit *more* than the
    /// overlapping pairs — the quadratic fallback does, and `handle_two` then
    /// finds nothing — but never fewer.
    #[test]
    fn the_division_reaches_every_pair_whose_boxes_meet() {
        let first = straddled_belt(0.0);
        let second = straddled_belt(0.5);
        // The division only happens above the threshold; this layout is well
        // past it, or the test would be checking the nested loop instead.
        assert!(first.len() > MIN_ELEMENTS && second.len() > MIN_ELEMENTS);

        let visited = visit_order(&first, &second);
        let expected = overlapping_pairs(&first, &second);
        assert!(
            !expected.is_empty(),
            "the layout has to make the two operands meet"
        );
        for pair in &expected {
            assert!(
                visited.contains(pair),
                "section pair {pair:?} overlaps but the walk never visited it"
            );
        }
    }

    /// The division splits the work, it does not repeat it — and a pair's rank
    /// is the position the walk reached it at.
    ///
    /// Every arm of `divide_into_subsets` takes a disjoint slice of the two
    /// index lists: the straddlers go against each half separately, the halves
    /// only against their own counterpart. So no pair is handed to `handle_two`
    /// twice, and each pair's rank is simply where it fell in the walk. A
    /// straddler paired against a whole list rather than against each half in
    /// turn would show up here as a repeat.
    #[test]
    fn the_division_visits_each_pair_once_and_ranks_it_where_it_fell() {
        let first = straddled_belt(0.0);
        let second = straddled_belt(0.5);
        let visited = visit_order(&first, &second);
        let ranks = VisitRank::of(&first, &second);

        let mut seen = visited.clone();
        seen.sort_unstable();
        let before = seen.len();
        seen.dedup();
        assert_eq!(before, seen.len(), "the walk visited some pair twice");

        for (at, &(one, two)) in visited.iter().enumerate() {
            assert_eq!(
                ranks.rank(one, two),
                at,
                "pair ({one}, {two}) must rank where the walk reached it"
            );
        }
    }

    /// A pair the walk never reaches sorts last rather than claiming a place.
    ///
    /// Its two boxes are apart, so no turn can lie in it; giving it a real rank
    /// would put a turn that does not exist ahead of one that does.
    #[test]
    fn a_pair_the_walk_never_reaches_sorts_last() {
        let first = straddled_belt(0.0);
        let second = straddled_belt(0.5);
        let ranks = VisitRank::of(&first, &second);

        // The first operand's opening box sits at the far left, the second
        // operand's seventeenth at the far right; nothing puts them together.
        let (left, right) = (0, MIN_ELEMENTS);
        assert!(
            !first[left].overlaps(&second[right]),
            "the two boxes have to be apart for this to be the unreached case"
        );
        assert_eq!(ranks.rank(left, right), usize::MAX);
        // And a section index that does not exist at all.
        assert_eq!(ranks.rank(first.len(), 0), usize::MAX);
    }

    /// At the threshold the walk is the plain nested loop, first operand outer.
    ///
    /// `partition::apply` divides only when **both** collections are *larger*
    /// than `min_elements`, so sixteen against sixteen is `handle_two` over
    /// everything and the ranks come out row-major. This is why a single small
    /// polygon against another comes out in plain section order, and one more
    /// section on each side is enough to change the answer's polygon order.
    #[test]
    fn at_the_threshold_the_walk_is_the_plain_nested_loop() {
        let boxes = overlapping_row(MIN_ELEMENTS);
        let ranks = VisitRank::of(&boxes, &boxes);
        for one in 0..MIN_ELEMENTS {
            for two in 0..MIN_ELEMENTS {
                assert_eq!(
                    ranks.rank(one, two),
                    one * MIN_ELEMENTS + two,
                    "at the threshold ({one}, {two}) must keep its nested-loop place"
                );
            }
        }

        // One more on each side and the division takes over, so the order is no
        // longer row-major. These boxes all overlap, so every pair is still
        // reached — only its position changes.
        let wider = overlapping_row(MIN_ELEMENTS + 1);
        let divided = VisitRank::of(&wider, &wider);
        assert!(
            (0..=MIN_ELEMENTS)
                .flat_map(|one| (0..=MIN_ELEMENTS).map(move |two| (one, two)))
                .any(|(one, two)| divided.rank(one, two) != one * (MIN_ELEMENTS + 1) + two),
            "above the threshold the division must reorder the pairs"
        );
    }

    /// An operand with no sections has no pairs, and asks for no walk.
    #[test]
    fn an_empty_operand_yields_no_pairs() {
        let boxes = [box_at(0.0, 0.0, 1.0, 1.0)];
        assert!(visit_order(&[], &boxes).is_empty());
        assert!(visit_order(&boxes, &[]).is_empty());
        assert_eq!(VisitRank::of(&[], &boxes).rank(0, 0), usize::MAX);
    }
}
