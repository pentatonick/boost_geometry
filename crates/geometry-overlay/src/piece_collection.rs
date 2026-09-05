//! The rings Boost's buffer walks, at a distance of zero.
//!
//! `buffer_inserter` does not offset a ring and hand it back. It cuts the ring
//! into **pieces** — one per side, one per join — and appends each piece's
//! generated points to an **offsetted ring**; the turns between those rings are
//! then found, classified, and traversed.
//!
//! At a distance of zero the sides offset onto the segments themselves, so the
//! offsetted ring is the input ring with one addition: at every **concave**
//! corner Boost adds two `buffered_concave` pieces, each contributing the
//! corner again, so that vertex appears three times. A convex corner gets no
//! join at all, because `join_miter` returns early when the two offset points
//! coincide — which at zero they always do.
//!
//! That is not cosmetic. `repair_one_polygon` falls back on `buffer(0)` when
//! the dissolve gives up, and it is the only thing standing between an
//! outline with a collapsed exterior and nothing at all.
//!
//! Mirrors `boost/geometry/algorithms/detail/buffer/buffer_inserter.hpp`
//! (`buffer_range::iterate`, `add_join`, `get_join_type`) and
//! `buffered_piece_collection.hpp` (`add_side_piece`, `add_range_to_piece`).

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_model::Ring;
use geometry_trait::{Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait};

use crate::predicate::orientation::{Sign, orientation_2d};

/// What `get_join_type` makes of a corner.
///
/// C++: `strategy::buffer::join_selector`, chosen from the side of the corner
/// (`side == -1` convex, `+1` concave) and, when the three points are
/// collinear, whether the third continues past the second or turns back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Join {
    /// Nothing is added: at a distance of zero the two offset points coincide
    /// and `join_miter` declines.
    Convex,
    /// Two `buffered_concave` pieces, each of which appends the corner again.
    Concave,
    /// Collinear and continuing: two consecutive sides, nothing between them.
    Continue,
    /// Collinear and turning back. An end cap, which a closed ring cannot ask
    /// for at zero width, so nothing is added.
    Spike,
}

/// C++: `buffer_range::get_join_type`.
fn join_at<P>(before: &P, corner: &P, after: &P) -> Join
where
    P: Point,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    match orientation_2d(before, corner, after) {
        Sign::Negative => Join::Convex,
        Sign::Positive => Join::Concave,
        // C++: `same_direction`, which is `direction_code(...) == 1` — the
        // perpendicular through the corner puts the third point beyond it.
        Sign::Collinear => {
            let dot = (corner.get::<0>().into() - before.get::<0>().into())
                * (after.get::<0>().into() - corner.get::<0>().into())
                + (corner.get::<1>().into() - before.get::<1>().into())
                    * (after.get::<1>().into() - corner.get::<1>().into());
            if dot > 0.0 {
                Join::Continue
            } else {
                Join::Spike
            }
        }
    }
}

fn same_point<P>(left: &P, right: &P) -> bool
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    left.get::<0>().tolerant_eq(right.get::<0>()) && left.get::<1>().tolerant_eq(right.get::<1>())
}

/// The ring as `closed_clockwise_view` presents it: distinct points, closed.
///
/// The name is Boost's and so is the behaviour, which is less than the name
/// suggests: the view is keyed on the ring *type*'s declared order and
/// closure, not on how the ring is actually wound, so for a type that is
/// already closed and clockwise it is the identity. The winding a ring is
/// stored with is what the joins are judged against — which is how an interior
/// ring, stored the other way round by `correct`, gets the opposite corners.
fn clockwise_view<R, P>(ring: &R) -> Vec<P>
where
    R: RingTrait<Point = P>,
    P: Point + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let mut points: Vec<P> = Vec::new();
    for point in ring.points() {
        if points.last().is_none_or(|last| !same_point(last, point)) {
            points.push(*point);
        }
    }
    while points.len() > 1 && same_point(&points[0], &points[points.len() - 1]) {
        points.pop();
    }
    if points.len() < 3 {
        return points;
    }
    points.push(points[0]);
    points
}

/// The offsetted ring a zero-width buffer generates for one ring.
///
/// C++: `buffer_range::iterate` over the sides, with `add_join` between each
/// consecutive pair and a closing join at the first vertex.
fn offsetted_ring<P>(closed: &[P]) -> Vec<P>
where
    P: Point + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let count = closed.len() - 1;
    let mut out: Vec<P> = Vec::with_capacity(count * 2);
    out.push(closed[0]);
    for index in 0..count {
        // The join at `closed[index]` is emitted before the side leaving it,
        // and the first side has no join before it.
        if index > 0 {
            let before = closed[index - 1];
            let corner = closed[index];
            let after = closed[index + 1];
            if join_at(&before, &corner, &after) == Join::Concave {
                out.push(corner);
                out.push(corner);
            }
        }
        out.push(closed[index + 1]);
    }
    // C++: `buffer_inserter_ring::iterate` adds a closing join at the first
    // vertex once the sides are done.
    if count >= 2 {
        let before = closed[count - 1];
        let corner = closed[0];
        let after = closed[1];
        if join_at(&before, &corner, &after) == Join::Concave {
            out.push(corner);
            out.push(corner);
        }
    }
    out
}

/// Every offsetted ring a polygon generates at a distance of zero.
///
/// A ring that collapses to a single distinct point becomes a point buffer,
/// which at zero width is a ring of coincident points enclosing nothing; it is
/// generated so the ring indices line up with Boost's, and dropped by the area
/// test downstream. `None` means the geometry asks for something this arm does
/// not implement.
pub(crate) fn zero_width_rings<G, P>(polygon: &G) -> Vec<Ring<P>>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let mut rings = Vec::new();
    for ring in core::iter::once(polygon.exterior()).chain(polygon.interiors()) {
        let closed = clockwise_view(ring);
        if closed.len() < 4 {
            // Fewer than three distinct points: nothing with area, and Boost's
            // point buffer at zero width encloses nothing either.
            continue;
        }
        rings.push(Ring::from_vec(offsetted_ring(&closed)));
    }
    rings
}

/// One segment of an offsetted ring, tagged with the ring and its position.
struct Step {
    ring: usize,
    at: usize,
    from: [f64; 2],
    to: [f64; 2],
}

/// Where two offsetted rings meet, which is the whole of what the rest of
/// Boost's pipeline reasons about.
///
/// C++: `get_piece_turns`, which skips the pairs that are neighbours by
/// construction — a side and the join beside it always touch, and that is not
/// a turn.
fn turns<P>(rings: &[Ring<P>]) -> Vec<[f64; 2]>
where
    P: Point + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    let mut steps: Vec<Step> = Vec::new();
    for (ring, points) in rings.iter().enumerate() {
        let coords: Vec<[f64; 2]> = points
            .points()
            .map(|point| [point.get::<0>().into(), point.get::<1>().into()])
            .collect();
        // The concave joins put zero-length segments into the ring. They
        // cannot meet anything, and leaving them in makes two segments that
        // are neighbours look three apart — which is the difference between a
        // ring that meets itself and one that merely turns a corner.
        for pair in coords.windows(2) {
            // Exact: these came out of the same vertex, so they are the same
            // bits or they are a real segment.
            #[expect(clippy::float_cmp, reason = "recognising a repeated vertex")]
            let repeated = pair[0][0] == pair[1][0] && pair[0][1] == pair[1][1];
            if repeated {
                continue;
            }
            steps.push(Step {
                ring,
                at: steps.iter().filter(|step| step.ring == ring).count(),
                from: pair[0],
                to: pair[1],
            });
        }
    }
    let mut found = Vec::new();
    for (index, one) in steps.iter().enumerate() {
        let length = steps.iter().filter(|step| step.ring == one.ring).count();
        for two in steps.iter().skip(index + 1) {
            if one.ring == two.ring {
                let apart = two.at - one.at;
                if apart <= 1 || apart + 1 >= length {
                    continue;
                }
            }
            if let Some(point) = meeting_point(one, two) {
                found.push(point);
            }
        }
    }
    found
}

fn meeting_point(one: &Step, two: &Step) -> Option<[f64; 2]> {
    let cross = |a: [f64; 2], b: [f64; 2]| a[0] * b[1] - a[1] * b[0];
    let sub = |a: [f64; 2], b: [f64; 2]| [a[0] - b[0], a[1] - b[1]];
    let run = sub(one.to, one.from);
    let other = sub(two.to, two.from);
    let denominator = cross(run, other);
    if denominator == 0.0 {
        return None;
    }
    let offset = sub(two.from, one.from);
    let along = cross(offset, other) / denominator;
    let across = cross(offset, run) / denominator;
    if !(0.0..=1.0).contains(&along) || !(0.0..=1.0).contains(&across) {
        return None;
    }
    Some([one.from[0] + along * run[0], one.from[1] + along * run[1]])
}

/// Whether the offsetted rings are the answer on their own.
///
/// C++: `discard_rings` drops every offsetted ring that has a turn, so what
/// comes out is the traversal's rings plus the offsetted rings that met
/// nothing. Where nothing met anything the traversal has no work and the rings
/// stand as they are — which is the case `repair_one_polygon` needs.
///
/// Where they do meet, what survives rests on `check_turn_in_original`, and
/// that rests on the winding strategy's verdict for a turn point `get_turns`
/// computed: a crossing part-way along two segments is decided by the last bits
/// of a determinant. Reproducing the verdict without reproducing the arithmetic
/// that produced the point was tried and measured — right 132 times of 300 and
/// wrong 148 — so this declines instead of guessing.
pub(crate) enum ZeroWidthOutcome {
    /// No ring meets another: the offsetted rings are the answer.
    RingsStand,
    /// Something met something, and what survives needs the buffer traversal.
    NeedsTraversal,
}

pub(crate) fn zero_width_outcome<P>(rings: &[Ring<P>]) -> ZeroWidthOutcome
where
    P: Point + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
{
    if turns(rings).is_empty() {
        ZeroWidthOutcome::RingsStand
    } else {
        ZeroWidthOutcome::NeedsTraversal
    }
}

#[cfg(test)]
mod tests {
    //! Checked against C++ Boost 1.83's `buffer` with
    //! `distance_symmetric<double>(0.0)`, `side_straight`, `join_miter`,
    //! `end_flat` and `point_square` — the strategies tilemaker's
    //! `repair_one_polygon` passes.

    use super::{ZeroWidthOutcome, zero_width_outcome, zero_width_rings};
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, Ring};
    use geometry_trait::{Point as _, Ring as _};

    type P = Point2D<f64, Cartesian>;

    fn ring(points: &[(f64, f64)]) -> Ring<P> {
        let mut ring = Ring::new();
        for &(x, y) in points {
            ring.push(P::new(x, y));
        }
        ring
    }

    fn points_of(ring: &Ring<P>) -> Vec<(f64, f64)> {
        ring.points()
            .map(|point| (point.get::<0>(), point.get::<1>()))
            .collect()
    }

    /// One concave corner in a ring that is otherwise convex.
    ///
    /// C++ returns the ring with that corner's vertex three times over — once
    /// from the side and twice from the pair of `buffered_concave` pieces —
    /// and every other vertex once.
    #[test]
    fn a_concave_corner_is_emitted_three_times() {
        let polygon = Polygon::new(ring(&[
            (3186.0, 2762.0),
            (3063.0, 2666.0),
            (2953.0, 2536.0),
            (2965.0, 2762.0),
            (3023.0, 2920.0),
            (3074.0, 2999.0),
            (3186.0, 2762.0),
        ]));
        let rings = zero_width_rings(&polygon);
        assert_eq!(rings.len(), 1);
        assert_eq!(
            points_of(&rings[0]),
            vec![
                (3186.0, 2762.0),
                (3063.0, 2666.0),
                (3063.0, 2666.0),
                (3063.0, 2666.0),
                (2953.0, 2536.0),
                (2965.0, 2762.0),
                (3023.0, 2920.0),
                (3074.0, 2999.0),
                (3186.0, 2762.0),
            ]
        );
        assert!(matches!(
            zero_width_outcome(&rings),
            ZeroWidthOutcome::RingsStand
        ));
    }

    /// The same ring, listed from its concave corner instead of a convex one.
    ///
    /// C++ emits the join **between** two sides, so the corner the ring is
    /// listed from has no side before it — its join is the closing one
    /// `buffer_inserter_ring::iterate` adds once the sides are done, and it
    /// lands at the end of the offsetted ring rather than in the middle. The
    /// corner is tripled either way: once by the ring's own first point, twice
    /// by the pair of `buffered_concave` pieces the closing join contributes.
    ///
    /// Skipping that closing join is not a cosmetic loss. It is the difference
    /// between a ring of nine points and one of seven, and the piece count is
    /// what the rest of the pipeline indexes by.
    #[test]
    fn a_concave_corner_listed_first_is_tripled_by_the_closing_join() {
        let polygon = Polygon::new(ring(&[
            (3063.0, 2666.0),
            (2953.0, 2536.0),
            (2965.0, 2762.0),
            (3023.0, 2920.0),
            (3074.0, 2999.0),
            (3186.0, 2762.0),
            (3063.0, 2666.0),
        ]));
        let rings = zero_width_rings(&polygon);
        assert_eq!(rings.len(), 1);
        assert_eq!(
            points_of(&rings[0]),
            vec![
                (3063.0, 2666.0),
                (2953.0, 2536.0),
                (2965.0, 2762.0),
                (3023.0, 2920.0),
                (3074.0, 2999.0),
                (3186.0, 2762.0),
                (3063.0, 2666.0),
                (3063.0, 2666.0),
                (3063.0, 2666.0),
            ]
        );
        assert!(matches!(
            zero_width_outcome(&rings),
            ZeroWidthOutcome::RingsStand
        ));
    }

    /// A corner whose three points are collinear adds nothing.
    ///
    /// `get_join_type` cannot read a side from a zero turn, so C++ falls back
    /// on `same_direction`: the third point either carries on past the corner —
    /// two consecutive sides with nothing between them, `join_continue` — or
    /// turns back along the side it came in on, which is an end cap a closed
    /// ring never asks for. Neither generates a piece, so the vertex on this
    /// ring's first edge survives exactly once and the ring comes back
    /// unaltered. Treating a zero turn as concave instead would triple it.
    #[test]
    fn a_corner_that_carries_straight_on_adds_no_join() {
        let listed = [
            (0.0, 0.0),
            (0.0, 5.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 0.0),
        ];
        let polygon = Polygon::new(ring(&listed));
        let rings = zero_width_rings(&polygon);
        assert_eq!(rings.len(), 1);
        assert_eq!(
            points_of(&rings[0]),
            listed.to_vec(),
            "a clockwise ring is all convex corners, and (0,5) carries straight on"
        );
    }

    /// A corner that turns back on itself is the other collinear case.
    ///
    /// The needle's tip has the side arriving and the side leaving running
    /// along the same line in opposite directions — `same_direction` is false,
    /// so C++ reaches `join_spike`, which is an end-cap strategy a closed ring
    /// has no cap to apply. Nothing is added at the tip.
    ///
    /// Both **feet** of the needle are a different matter: leaving the bottom
    /// edge and rejoining it are each a left turn on a clockwise ring, so both
    /// are reflex and both are tripled. That asymmetry — nothing at the tip,
    /// two joins at the feet — is what separates the turn-back case from the
    /// carry-straight-on case above, where the collinear corner is also silent
    /// but its neighbours are convex.
    #[test]
    fn a_corner_that_turns_back_adds_no_join_but_its_feet_do() {
        let polygon = Polygon::new(ring(&[
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (5.0, 0.0),
            (5.0, -5.0),
            (5.0, 0.0),
            (0.0, 0.0),
        ]));
        let rings = zero_width_rings(&polygon);
        assert_eq!(rings.len(), 1);
        assert_eq!(
            points_of(&rings[0]),
            vec![
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (5.0, 0.0),
                (5.0, 0.0),
                (5.0, 0.0),
                (5.0, -5.0),
                (5.0, 0.0),
                (5.0, 0.0),
                (5.0, 0.0),
                (0.0, 0.0),
            ],
            "the tip adds nothing; both feet of the needle are reflex"
        );
    }

    /// An exterior and a hole are two offsetted rings, and each is judged
    /// against the winding it is stored with.
    ///
    /// `correct` stores an interior ring the other way round from its
    /// exterior, and `closed_clockwise_view` does not undo that — it keys on
    /// the ring *type*, not on how the ring is wound. So the same square shape
    /// is all convex corners as an exterior and all reflex ones as a hole:
    /// five points out for the outer, and thirteen for the inner, where each of
    /// the four corners is tripled and the closing join accounts for the last.
    ///
    /// This is also the only shape here with two rings to compare, so it is
    /// what puts `get_piece_turns` across a ring boundary. The neighbour skip
    /// applies within one ring only; two rings that never touch have no turn
    /// between them however close they run, and the pair stands.
    #[test]
    fn an_exterior_and_its_hole_are_judged_by_their_own_windings() {
        let polygon = Polygon::with_inners(
            ring(&[
                (0.0, 0.0),
                (0.0, 20.0),
                (20.0, 20.0),
                (20.0, 0.0),
                (0.0, 0.0),
            ]),
            vec![ring(&[
                (5.0, 5.0),
                (15.0, 5.0),
                (15.0, 15.0),
                (5.0, 15.0),
                (5.0, 5.0),
            ])],
        );
        let rings = zero_width_rings(&polygon);
        assert_eq!(rings.len(), 2);
        assert_eq!(
            points_of(&rings[0]),
            vec![
                (0.0, 0.0),
                (0.0, 20.0),
                (20.0, 20.0),
                (20.0, 0.0),
                (0.0, 0.0)
            ],
            "the exterior's corners are all convex, so none adds a join"
        );
        assert_eq!(
            points_of(&rings[1]).len(),
            13,
            "the hole's four corners are all reflex, so each is tripled"
        );
        assert!(matches!(
            zero_width_outcome(&rings),
            ZeroWidthOutcome::RingsStand
        ));
    }

    /// A hole that reaches out and touches its exterior is declined.
    ///
    /// The two offsetted rings now meet, so `discard_rings` would drop them
    /// both and what comes out is whatever the buffer traversal makes of the
    /// turn — which is the part that is not ported. Deciding it needs the
    /// winding strategy's verdict on a turn point, so this says so rather than
    /// guessing. Contrast the ring above, where the same two rings stand
    /// because nothing between them met.
    #[test]
    fn a_hole_touching_its_exterior_is_declined() {
        let polygon = Polygon::with_inners(
            ring(&[
                (0.0, 0.0),
                (0.0, 20.0),
                (20.0, 20.0),
                (20.0, 0.0),
                (0.0, 0.0),
            ]),
            vec![ring(&[
                (0.0, 5.0),
                (15.0, 5.0),
                (15.0, 15.0),
                (0.0, 15.0),
                (0.0, 5.0),
            ])],
        );
        let rings = zero_width_rings(&polygon);
        assert_eq!(rings.len(), 2);
        assert!(matches!(
            zero_width_outcome(&rings),
            ZeroWidthOutcome::NeedsTraversal
        ));
    }

    /// A ring that crosses itself needs the turns, the check against the
    /// original and the traversal, none of which is ported.
    #[test]
    fn a_ring_that_meets_itself_is_declined() {
        let polygon = Polygon::new(ring(&[
            (0.0, 0.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 10.0),
            (0.0, 0.0),
        ]));
        let rings = zero_width_rings(&polygon);
        assert!(matches!(
            zero_width_outcome(&rings),
            ZeroWidthOutcome::NeedsTraversal
        ));
    }

    /// The outline that sent tilemaker here: an exterior collapsed to one
    /// point, and an interior that still encloses something. C++ drops the
    /// exterior — a point buffer of zero width holds nothing — and hands back
    /// the interior, whose stored winding is what its corners are judged
    /// against.
    #[test]
    fn a_collapsed_exterior_leaves_its_interior_standing() {
        let polygon = Polygon::with_inners(
            ring(&[
                (11.230_769_230_770_715, 4_095.000_000_000_000_5),
                (11.230_769_230_770_715, 4_095.000_000_000_000_5),
            ]),
            vec![ring(&[
                (-11.0, 4091.0),
                (-10.0, 4091.0),
                (-10.0, 4_083.000_000_000_000_5),
                (-2.0, 4087.0),
                (-1.0, 4089.0),
                (-4.0, 4092.0),
                (-8.0, 4092.0),
                (-9.0, 4093.0),
                (-11.0, 4091.0),
            ])],
        );
        let rings = zero_width_rings(&polygon);
        assert_eq!(rings.len(), 1, "the collapsed exterior encloses nothing");
        assert!(matches!(
            zero_width_outcome(&rings),
            ZeroWidthOutcome::RingsStand
        ));
        // C++ Boost 1.83: twenty-one points, six corners tripled.
        assert_eq!(points_of(&rings[0]).len(), 21);
    }
}
