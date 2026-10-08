//! Where two Cartesian segments meet.
//!
//! Mirrors `strategy::intersection::cartesian_segments`
//! (`strategies/cartesian/intersection.hpp`) read through the
//! `segments_intersection_points` policy
//! (`policies/relate/intersection_points.hpp`): the kernel behind Boost's
//! `intersects` and `disjoint` of two segments, and the meeting point of
//! `closest_points`. Whether the segments meet is decided with Boost's
//! epsilon tests — [`CoordinateScalar::side_by_triangle`] for the sides,
//! [`CoordinateScalar::tolerant_eq`] for coincident points, and
//! [`CoordinateScalar::nearly_parallel`] for segments rounding leaves
//! parallel — so two segments meet here exactly where they meet in Boost.
#![allow(
    clippy::similar_names,
    reason = "`dx_a`/`dy_a`/`dx_b`/`dy_b` are Boost's names for the two segments' offsets, kept so the arithmetic reads against `segment_intersection_info`."
)]

use core::cmp::Ordering;

use geometry_coords::CoordinateScalar;

/// How two segments `p = p1 → p2` and `q = q1 → q2` meet, as Boost's
/// `cartesian_segments` classifies them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentMeeting {
    /// They do not meet.
    Disjoint,
    /// They meet first at `p1`: both are that point, `p` is a point on
    /// `q`, or the segments share `p1`.
    AtP1,
    /// They meet first at `p2`, which they share.
    AtP2,
    /// They meet first at `q1`: `q` is a point on `p`.
    AtQ1,
    /// They overlap along a line. `axis` is the ordinate Boost orders them
    /// on, and `positions` where `p1`, `p2` lie along `q` and `q1`, `q2`
    /// along `p`: Boost's `position_value`, `0` before the start, `1` at
    /// it, `2` inside, `3` at the end, `4` after it.
    Collinear { axis: usize, positions: [u8; 4] },
    /// They cross between their endpoints.
    Crossing,
    /// Boost reads them as collinear but has no axis to order them on —
    /// both are flat along both axes, within rounding — and reports a
    /// crossing it has no ratios for.
    Unordered,
}

/// How the segments `p1 → p2` and `q1 → q2` meet.
///
/// C++: `cartesian_segments::apply` and `unified`.
#[must_use]
pub fn segment_meeting<T: CoordinateScalar>(
    p1: (T, T),
    p2: (T, T),
    q1: (T, T),
    q2: (T, T),
) -> SegmentMeeting {
    let p_is_point = same_point(p1, p2);
    let q_is_point = same_point(q1, q2);
    if p_is_point && q_is_point {
        // C++ compares `p1` with `q2`.
        return if same_point(p1, q2) {
            SegmentMeeting::AtP1
        } else {
            SegmentMeeting::Disjoint
        };
    }
    if disjoint_by_range(p1.0, p2.0, q1.0, q2.0) || disjoint_by_range(p1.1, p2.1, q1.1, q2.1) {
        return SegmentMeeting::Disjoint;
    }
    let sides_p = (
        T::side_by_triangle(q1, q2, p1),
        T::side_by_triangle(q1, q2, p2),
    );
    if same_side(sides_p) {
        return SegmentMeeting::Disjoint;
    }
    let sides_q = (
        T::side_by_triangle(p1, p2, q1),
        T::side_by_triangle(p1, p2, q2),
    );
    if same_side(sides_q) {
        return SegmentMeeting::Disjoint;
    }
    let collinear = [sides_p.0, sides_p.1, sides_q.0, sides_q.1]
        .iter()
        .all(|side| *side == Ordering::Equal)
        || T::nearly_parallel(p1, p2, q1, q2);
    if collinear {
        if let Some(axis) = collinear_axis(p1, p2, q1, q2, p_is_point, q_is_point) {
            return relate_collinear(axis, p1, p2, q1, q2, p_is_point, q_is_point);
        }
    }
    if same_point(p1, q1) || same_point(p1, q2) {
        return SegmentMeeting::AtP1;
    }
    if same_point(p2, q1) || same_point(p2, q2) {
        return SegmentMeeting::AtP2;
    }
    if collinear {
        SegmentMeeting::Unordered
    } else {
        SegmentMeeting::Crossing
    }
}

/// The point where the segments `p1 → p2` and `q1 → q2` first meet,
/// classified as `meeting`: Boost's `intersections[0]`.
///
/// C++: `segments_intersection_points` and
/// `segment_intersection_info::calculate`, which picks the segment the
/// crossing is interpolated on — the one it lies nearer an end of, or else
/// the shorter — and moves a crossing of nearly collinear segments that
/// rounding put past an end back onto it.
pub(crate) fn meeting_point(
    p1: (f64, f64),
    p2: (f64, f64),
    q1: (f64, f64),
    q2: (f64, f64),
    meeting: SegmentMeeting,
) -> Option<(f64, f64)> {
    match meeting {
        SegmentMeeting::Disjoint => None,
        SegmentMeeting::AtP1 => Some(p1),
        SegmentMeeting::AtP2 => Some(p2),
        SegmentMeeting::AtQ1 => Some(q1),
        SegmentMeeting::Collinear { axis, positions } => {
            Some(collinear_start(axis, positions, p1, p2, q1, q2))
        }
        SegmentMeeting::Crossing => {
            let (dx_a, dy_a) = (p2.0 - p1.0, p2.1 - p1.1);
            let (dx_b, dy_b) = (q2.0 - q1.0, q2.1 - q1.1);
            // C++: `cramers_rule`, once for each segment.
            let ra = Ratio::new(
                dx_b * (p1.1 - q1.1) - dy_b * (p1.0 - q1.0),
                dx_a * dy_b - dy_a * dx_b,
            );
            let rb = Ratio::new(
                dx_a * (q1.1 - p1.1) - dy_a * (q1.0 - p1.0),
                dx_b * dy_a - dy_b * dx_a,
            );
            let mut point = crossing(p1, p2, q1, q2, ra, rb);
            if ra.possibly_collinear() && rb.possibly_collinear() {
                assign_if_exceeds(&mut point, p1, p2);
                assign_if_exceeds(&mut point, q1, q2);
            }
            Some(point)
        }
        SegmentMeeting::Unordered => {
            let zero = Ratio::new(0.0, 1.0);
            Some(crossing(p1, p2, q1, q2, zero, zero))
        }
    }
}

/// Whether `a` and `b` are one point: C++ `equals_point_point`.
fn same_point<T: CoordinateScalar>(a: (T, T), b: (T, T)) -> bool {
    a.0.tolerant_eq(b.0) && a.1.tolerant_eq(b.1)
}

/// Both endpoints strictly on one side: C++ `side_info::same`.
fn same_side((first, second): (Ordering, Ordering)) -> bool {
    first == second && first != Ordering::Equal
}

/// Whether the ranges `[p1, p2]` and `[q1, q2]` of one ordinate are apart
/// by more than rounding: C++ `disjoint_by_range`, with `math::smaller`.
fn disjoint_by_range<T: CoordinateScalar>(p1: T, p2: T, q1: T, q2: T) -> bool {
    let (min_p, max_p) = if p1 > p2 { (p2, p1) } else { (p1, p2) };
    let (min_q, max_q) = if q1 > q2 { (q2, q1) } else { (q1, q2) };
    let smaller = |a: T, b: T| a < b && !a.tolerant_eq(b);
    smaller(max_p, min_q) || smaller(max_q, min_p)
}

/// The ordinate collinear segments are ordered on, if any: C++
/// `is_x_more_significant`.
fn collinear_axis<T: CoordinateScalar>(
    p1: (T, T),
    p2: (T, T),
    q1: (T, T),
    q2: (T, T),
    p_is_point: bool,
    q_is_point: bool,
) -> Option<usize> {
    let spread = |a: T, b: T| (b.to_measure() - a.to_measure()).abs();
    let (dx_a, dy_a) = (spread(p1.0, p2.0), spread(p1.1, p2.1));
    let (dx_b, dy_b) = (spread(q1.0, q2.0), spread(q1.1, q2.1));
    // `0` orders on x, `1` on y.
    let x_unless = |x_wins: bool| usize::from(!x_wins);
    if p_is_point {
        return Some(x_unless(dx_b >= dy_b));
    }
    if q_is_point {
        return Some(x_unless(dx_a >= dy_a));
    }
    // C++: `std::min`, which keeps the first of two equal values.
    let min_dx = if dx_b < dx_a { dx_b } else { dx_a };
    let min_dy = if dy_b < dy_a { dy_b } else { dy_a };
    if min_dx == min_dy {
        (min_dx > <T::Measure as CoordinateScalar>::ZERO).then_some(0)
    } else {
        Some(x_unless(min_dx > min_dy))
    }
}

/// How collinear segments overlap along `axis`: C++ `relate_collinear`
/// and `relate_one_degenerate`.
fn relate_collinear<T: CoordinateScalar>(
    axis: usize,
    p1: (T, T),
    p2: (T, T),
    q1: (T, T),
    q2: (T, T),
    p_is_point: bool,
    q_is_point: bool,
) -> SegmentMeeting {
    let along = |point: (T, T)| if axis == 0 { point.0 } else { point.1 };
    if p_is_point {
        return if within(along(p1), along(q1), along(q2)) {
            SegmentMeeting::AtP1
        } else {
            SegmentMeeting::Disjoint
        };
    }
    if q_is_point {
        return if within(along(q1), along(p1), along(p2)) {
            SegmentMeeting::AtQ1
        } else {
            SegmentMeeting::Disjoint
        };
    }
    let positions = [
        position(along(p1), along(q1), along(q2)),
        position(along(p2), along(q1), along(q2)),
        position(along(q1), along(p1), along(p2)),
        position(along(q2), along(p1), along(p2)),
    ];
    let [a1, a2, _, _] = positions;
    if (a1 < 1 && a2 < 1) || (a1 > 3 && a2 > 3) {
        SegmentMeeting::Disjoint
    } else {
        SegmentMeeting::Collinear { axis, positions }
    }
}

/// Whether `d` lies on `[s1, s2]` as the ratio `(d − s1) / (s2 − s1)`
/// does: C++ `segment_ratio::on_segment`.
fn within<T: CoordinateScalar>(d: T, s1: T, s2: T) -> bool {
    let (mut numerator, mut denominator) = (
        d.to_measure() - s1.to_measure(),
        s2.to_measure() - s1.to_measure(),
    );
    let zero = <T::Measure as CoordinateScalar>::ZERO;
    if denominator < zero {
        numerator = zero - numerator;
        denominator = zero - denominator;
    }
    numerator >= zero && numerator <= denominator
}

/// Where `c` lies along `from → to`: C++ `position_value` — `0` before
/// `from`, `1` at it, `2` between, `3` at `to`, `4` after it.
fn position<T: CoordinateScalar>(c: T, from: T, to: T) -> u8 {
    if c.tolerant_eq(from) {
        1
    } else if c.tolerant_eq(to) {
        3
    } else if from < to {
        if c < from {
            0
        } else if c > to {
            4
        } else {
            2
        }
    } else if c > from {
        0
    } else if c < to {
        4
    } else {
        2
    }
}

/// The first point of a collinear overlap: C++ `segments_collinear` takes
/// `p1` if it is on `q`, else `q1` if inside `p`, else `p2` if on `q`, else
/// `q2` if inside `p`, and of the first two it finds, the one nearer the
/// start of `p`.
fn collinear_start(
    axis: usize,
    [a1, a2, b1, b2]: [u8; 4],
    p1: (f64, f64),
    p2: (f64, f64),
    q1: (f64, f64),
    q2: (f64, f64),
) -> (f64, f64) {
    let along = |point: (f64, f64)| if axis == 0 { point.0 } else { point.1 };
    let length_p = along(p2) - along(p1);
    // C++: `rb_from` and `rb_to`, the ends of `q` as fractions of `p`,
    // pinned to `p`'s ends where `p`'s ends coincide with them.
    let mut q1_on_p = Ratio::new(along(q1) - along(p1), length_p);
    let mut q2_on_p = Ratio::new(along(q2) - along(p1), length_p);
    match a1 {
        1 => q1_on_p = Ratio::new(0.0, 1.0),
        3 => q2_on_p = Ratio::new(0.0, 1.0),
        _ => {}
    }
    match a2 {
        1 => q1_on_p = Ratio::new(1.0, 1.0),
        3 => q2_on_p = Ratio::new(1.0, 1.0),
        _ => {}
    }
    let candidates = [
        ((1..=3).contains(&a1), p1, Ratio::new(0.0, 1.0)),
        (b1 == 2, q1, q1_on_p),
        ((1..=3).contains(&a2), p2, Ratio::new(1.0, 1.0)),
        (b2 == 2, q2, q2_on_p),
    ];
    let mut found = candidates.iter().filter(|(on, _, _)| *on);
    let Some(&(_, first, first_on_p)) = found.next() else {
        unreachable!("a collinear overlap has a first point");
    };
    match found.next() {
        Some(&(_, second, second_on_p)) if second_on_p.less(first_on_p) => second,
        _ => first,
    }
}

/// The crossing interpolated on whichever segment Boost prefers: C++
/// `segment_intersection_info::calculate` with `detail_usage::use_a`.
fn crossing(
    p1: (f64, f64),
    p2: (f64, f64),
    q1: (f64, f64),
    q2: (f64, f64),
    ra: Ratio,
    rb: Ratio,
) -> (f64, f64) {
    let (dx_a, dy_a) = (p2.0 - p1.0, p2.1 - p1.1);
    let (dx_b, dy_b) = (q2.0 - q1.0, q2.1 - q1.1);
    let length_a = dx_a * dx_a + dy_a * dy_a;
    let length_b = dx_b * dx_b + dy_b * dy_b;
    let longest = length_a.max(length_b);
    let use_a = longest <= 0.0 || {
        let (relative_a, relative_b) = (1.0 - length_a / longest, 1.0 - length_b / longest);
        5.0 * ra.edge_value() + relative_a > 5.0 * rb.edge_value() + relative_b
    };
    let (start, dx, dy, ratio) = if use_a {
        (p1, dx_a, dy_a, ra)
    } else {
        (q1, dx_b, dy_b, rb)
    };
    (
        start.0 + ratio.numerator * dx / ratio.denominator,
        start.1 + ratio.numerator * dy / ratio.denominator,
    )
}

/// Move a crossing that lies past an end of `s1 → s2` onto that end: C++
/// `assign_if_exceeds`.
fn assign_if_exceeds(point: &mut (f64, f64), s1: (f64, f64), s2: (f64, f64)) {
    let exceeds = |end: (f64, f64), other: (f64, f64)| {
        let past = |c: f64, c0: f64, c1: f64| {
            if c0 < c1 {
                c < c0 && !c.tolerant_eq(c0)
            } else if c0 > c1 {
                c > c0 && !c.tolerant_eq(c0)
            } else {
                false
            }
        };
        past(point.0, end.0, other.0) || past(point.1, end.1, other.1)
    };
    if exceeds(s1, s2) {
        *point = s1;
    } else if exceeds(s2, s1) {
        *point = s2;
    }
}

/// A fraction along a segment: C++ `segment_ratio<double>`, kept with a
/// positive denominator and its approximation scaled by a million.
#[derive(Debug, Clone, Copy)]
struct Ratio {
    numerator: f64,
    denominator: f64,
    approximation: f64,
}

impl Ratio {
    const SCALE: f64 = 1_000_000.0;

    fn new(numerator: f64, denominator: f64) -> Self {
        let (numerator, denominator) = if denominator < 0.0 {
            (-numerator, -denominator)
        } else {
            (numerator, denominator)
        };
        let approximation = if denominator == 0.0 {
            0.0
        } else {
            numerator * Self::SCALE / denominator
        };
        Self {
            numerator,
            denominator,
            approximation,
        }
    }

    /// How near an end the fraction is, `0` in the middle to `1` at
    /// either end and beyond: C++ `edge_value`.
    fn edge_value(self) -> f64 {
        let value = 2.0 * (0.5 - self.approximation / Self::SCALE).abs();
        if value > 1.0 { 1.0 } else { value }
    }

    /// C++ `possibly_collinear(1.0e-3)`.
    fn possibly_collinear(self) -> bool {
        self.denominator.abs() < 1.0e-3
    }

    /// C++ `operator<`: the approximations, unless they are close, then
    /// the fractions themselves, equal within `math::equals`.
    fn less(self, other: Self) -> bool {
        if (self.approximation - other.approximation).abs() < 50.0 {
            let (a, b) = (
                self.numerator / self.denominator,
                other.numerator / other.denominator,
            );
            !a.tolerant_eq(b) && a < b
        } else {
            self.approximation < other.approximation
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "meeting points are endpoints or exact interpolations, compared bit for bit"
    )]

    use super::{Ratio, SegmentMeeting, assign_if_exceeds, meeting_point, segment_meeting};

    /// Classify and locate in one step, as `closest_points` does.
    fn meet(
        p1: (f64, f64),
        p2: (f64, f64),
        q1: (f64, f64),
        q2: (f64, f64),
    ) -> (SegmentMeeting, Option<(f64, f64)>) {
        let meeting = segment_meeting(p1, p2, q1, q2);
        (meeting, meeting_point(p1, p2, q1, q2, meeting))
    }

    /// Two points meet only where they coincide, compared `p1` with `q2`.
    #[test]
    fn two_points_meet_only_where_they_coincide() {
        let a = (1.0, 2.0);
        assert_eq!(meet(a, a, a, a), (SegmentMeeting::AtP1, Some(a)));
        let b = (1.0, 3.0);
        assert_eq!(meet(a, a, b, b), (SegmentMeeting::Disjoint, None));
    }

    /// Segments sharing only their second endpoint meet there.
    #[test]
    fn segments_sharing_p2_meet_at_p2() {
        let shared = (1.0, 1.0);
        assert_eq!(
            meet((0.0, 0.0), shared, shared, (2.0, 0.0)),
            (SegmentMeeting::AtP2, Some(shared))
        );
    }

    /// A point on a segment meets it at the point, whichever way the
    /// segment runs; a point on its line, just past an end, does not.
    #[test]
    fn a_point_meets_a_segment_it_lies_on() {
        let p = (0.5, 0.5);
        assert_eq!(
            meet(p, p, (0.0, 0.0), (1.0, 1.0)),
            (SegmentMeeting::AtP1, Some(p))
        );
        assert_eq!(
            meet(p, p, (1.0, 1.0), (0.0, 0.0)),
            (SegmentMeeting::AtP1, Some(p))
        );
        // Within rounding of the end for the range test, past it for the
        // ratio.
        let past = (1.0 + f64::EPSILON, 1.0 + f64::EPSILON);
        assert_eq!(
            segment_meeting(past, past, (0.0, 0.0), (1.0, 1.0)),
            SegmentMeeting::Disjoint
        );
    }

    /// A segment meets a point lying on it at that point.
    #[test]
    fn a_segment_meets_a_point_on_it_at_q1() {
        let q = (0.5, 0.5);
        assert_eq!(
            meet((0.0, 0.0), (1.0, 1.0), q, q),
            (SegmentMeeting::AtQ1, Some(q))
        );
        let past = (1.0 + f64::EPSILON, 1.0 + f64::EPSILON);
        assert_eq!(
            segment_meeting((0.0, 0.0), (1.0, 1.0), past, past),
            SegmentMeeting::Disjoint
        );
    }

    /// Segments so short their cross product rounds to nothing are
    /// collinear with no axis to order them on: sharing an end they meet
    /// there, and crossing they are `Unordered`, located at the start of
    /// `q` with no ratio to move along it.
    #[test]
    fn segments_flat_on_both_axes_have_no_order() {
        let t = 1.0e-9;
        assert_eq!(
            meet((0.0, 0.0), (0.0, t), (0.0, 0.0), (t, 0.0)),
            (SegmentMeeting::AtP1, Some((0.0, 0.0)))
        );
        assert_eq!(
            meet((0.0, -t), (0.0, t), (-t, 0.0), (t, 0.0)),
            (SegmentMeeting::Unordered, Some((-t, 0.0)))
        );
    }

    /// `q` running backwards along `x` puts `p`'s end after it at position
    /// `0`, and of two overlap starts the one nearer `p1` wins: here `q2`.
    #[test]
    fn collinear_overlap_starts_nearest_p1() {
        let (meeting, point) = meet((0.0, 0.0), (3.0, 0.0), (2.0, 0.0), (1.0, 0.0));
        assert_eq!(
            meeting,
            SegmentMeeting::Collinear {
                axis: 0,
                positions: [4, 0, 2, 2]
            }
        );
        assert_eq!(point, Some((1.0, 0.0)));
    }

    /// Endpoints `p` shares with `q` pin `q`'s ends to `p`'s fractions
    /// `0` and `1`, whichever end of `q` they are.
    #[test]
    fn collinear_overlap_pins_shared_ends() {
        // p1 at q1: the overlap starts at p1.
        assert_eq!(
            meet((0.0, 0.0), (2.0, 0.0), (0.0, 0.0), (1.0, 0.0)).1,
            Some((0.0, 0.0))
        );
        // p1 at q2.
        assert_eq!(
            meet((1.0, 0.0), (3.0, 0.0), (0.0, 0.0), (1.0, 0.0)).1,
            Some((1.0, 0.0))
        );
        // p2 at q1, q2 inside p and nearer p1.
        assert_eq!(
            meet((0.0, 0.0), (2.0, 0.0), (2.0, 0.0), (1.0, 0.0)).1,
            Some((1.0, 0.0))
        );
        // p2 at q2, q1 inside p and nearer p1.
        assert_eq!(
            meet((0.0, 0.0), (2.0, 0.0), (1.0, 0.0), (2.0, 0.0)).1,
            Some((1.0, 0.0))
        );
    }

    /// Two overlap starts whose approximations differ by under fifty
    /// millionths are told apart by their exact fractions: `p1` at `0`
    /// precedes `q1` at `1e-5`.
    #[test]
    fn close_overlap_starts_are_compared_exactly() {
        assert_eq!(
            meet((0.0, 0.0), (1.0, 0.0), (1.0e-5, 0.0), (-1.0, 0.0)).1,
            Some((0.0, 0.0))
        );
    }

    /// Short crossing segments have a Cramer denominator under `1e-3`, so
    /// their crossing is checked against both segments' ends; inside both,
    /// it stays.
    #[test]
    fn a_short_crossing_inside_both_segments_stays() {
        assert_eq!(
            meet(
                (0.0, 0.0),
                (0.007_812_5, 0.0),
                (0.003_906_25, -0.007_812_5),
                (0.003_906_25, 0.007_812_5)
            ),
            (SegmentMeeting::Crossing, Some((0.003_906_25, 0.0)))
        );
    }

    /// A crossing past either end of a segment moves onto that end; one
    /// within it, or on a segment flat along the ordinate, stays.
    #[test]
    fn a_crossing_past_an_end_moves_onto_it() {
        let (s1, s2) = ((0.0, 0.0), (1.0, 2.0));
        let mut before = (-0.5, 1.0);
        assign_if_exceeds(&mut before, s1, s2);
        assert_eq!(before, s1);
        let mut after = (0.5, 2.5);
        assign_if_exceeds(&mut after, s1, s2);
        assert_eq!(after, s2);
        let mut reversed = (0.5, 2.5);
        assign_if_exceeds(&mut reversed, s2, s1);
        assert_eq!(reversed, s2);
        let mut inside = (0.5, 1.0);
        assign_if_exceeds(&mut inside, s1, s2);
        assert_eq!(inside, (0.5, 1.0));
        let mut flat = (5.0, 5.0);
        assign_if_exceeds(&mut flat, (5.0, 5.0), (5.0, 5.0));
        assert_eq!(flat, (5.0, 5.0));
    }

    /// A ratio over zero approximates to zero, as Boost's `segment_ratio`
    /// does, and so sits at an end.
    #[test]
    fn a_ratio_over_zero_approximates_to_zero() {
        let ratio = Ratio::new(1.0, 0.0);
        assert_eq!(ratio.approximation, 0.0);
        assert_eq!(ratio.edge_value(), 1.0);
    }
}
