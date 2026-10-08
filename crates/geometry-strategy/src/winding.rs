//! The Cartesian point-location kernel: the winding rule, with on-segment
//! detection.
//!
//! Mirrors `strategy::within::cartesian_winding`
//! (`strategies/cartesian/point_in_poly_winding.hpp`). Boost locates a
//! point in a ring or polygon (`within`, `covered_by`), on a segment or
//! linestring (`intersects`), and in the point rows of `relate` with this
//! one kernel, fed one segment at a time, so a point a rounding error off
//! an edge is on it — or off it — to every one of those predicates alike.
//! Its side test is [`CoordinateScalar::side_by_triangle`], and it compares
//! a point's ordinates with a segment's by [`CoordinateScalar::tolerant_eq`],
//! as Boost compares them by `math::equals`.

use core::cmp::Ordering;

use geometry_coords::CoordinateScalar;

/// Where a point lies relative to the segments a [`Winding`] walked:
/// Boost's `-1` / `0` / `1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointLocation {
    /// Off every segment, and wound around zero times (`-1`).
    Exterior,
    /// On a segment (`0`).
    Boundary,
    /// Off every segment, and wound around (`1`).
    Interior,
}

/// A winding walk around one point, one segment at a time.
///
/// Mirrors the state Boost's `cartesian_winding_base::counter` keeps: the
/// winding count, and whether the point was found on a segment.
#[derive(Debug, Clone, Copy, Default)]
pub struct Winding {
    count: i32,
    touches: bool,
}

impl Winding {
    /// Fold the segment `s1 → s2` into the walk around `point`. Returns
    /// `false` once the point is found on a segment: the walk can stop.
    ///
    /// C++: `cartesian_winding_base::apply` with its `check_touch`,
    /// `calculate_count` and `side_equal`.
    pub fn apply<T: CoordinateScalar>(&mut self, point: (T, T), s1: (T, T), s2: (T, T)) -> bool {
        let (px, py) = point;
        let eq1 = s1.0.tolerant_eq(px);
        let eq2 = s2.0.tolerant_eq(px);
        // C++: `check_touch` — a segment standing vertical at the point's
        // x is on the point or out of the count.
        if eq1 && eq2 {
            if (s1.1 <= py && s2.1 >= py) || (s2.1 <= py && s1.1 >= py) {
                self.touches = true;
            }
            return !self.touches;
        }
        let count = if eq1 {
            if s2.0 > px { 1 } else { -1 }
        } else if eq2 {
            if s1.0 > px { -1 } else { 1 }
        } else if s1.0 < px && s2.0 > px {
            2
        } else if s2.0 < px && s1.0 > px {
            -2
        } else {
            0
        };
        if count != 0 {
            let side = if count == 1 || count == -1 {
                // A segment ending at the point's x: the point is above,
                // below, or at that end.
                let end = if eq1 { s1.1 } else { s2.1 };
                if py.tolerant_eq(end) {
                    0
                } else if py < end {
                    -count
                } else {
                    count
                }
            } else {
                match T::side_by_triangle(s1, s2, point) {
                    Ordering::Greater => 1,
                    Ordering::Less => -1,
                    Ordering::Equal => 0,
                }
            };
            if side == 0 {
                self.touches = true;
                self.count = 0;
                return false;
            }
            if side * count > 0 {
                self.count += count;
            }
        }
        !self.touches
    }

    /// Where the point lies, by the segments walked so far.
    ///
    /// C++: `cartesian_winding_base::result`.
    #[must_use]
    pub fn location(self) -> PointLocation {
        if self.touches {
            PointLocation::Boundary
        } else if self.count == 0 {
            PointLocation::Exterior
        } else {
            PointLocation::Interior
        }
    }
}

/// Walk the polyline through `vertices` around `point`, stopping at the
/// first segment the point is on: [`PointLocation::Boundary`] iff the point
/// lies on the polyline.
///
/// C++: `detail::within::point_in_range`
/// (`algorithms/detail/within/point_in_geometry.hpp`).
pub fn range_location<T: CoordinateScalar>(
    point: (T, T),
    vertices: impl IntoIterator<Item = (T, T)>,
) -> PointLocation {
    let mut winding = Winding::default();
    let mut vertices = vertices.into_iter();
    if let Some(mut previous) = vertices.next() {
        for vertex in vertices {
            if !winding.apply(point, previous, vertex) {
                break;
            }
            previous = vertex;
        }
    }
    winding.location()
}

/// Where `point` lies relative to the ring through `vertices`. A sequence
/// that does not repeat its first vertex is closed onto it; an empty one is
/// outside by convention.
///
/// C++: `point_in_geometry<Ring>` at
/// `algorithms/detail/within/point_in_geometry.hpp`, which walks the ring
/// as its `closed_clockwise_view` presents it.
pub fn ring_location<T, I>(point: (T, T), vertices: I) -> PointLocation
where
    T: CoordinateScalar,
    I: IntoIterator<Item = (T, T)>,
    I::IntoIter: Clone,
{
    let vertices = vertices.into_iter();
    let Some(first) = vertices.clone().next() else {
        return PointLocation::Exterior;
    };
    let repeats_first = vertices.clone().nth(1).is_some()
        && vertices
            .clone()
            .last()
            .is_some_and(|last| last.0 == first.0 && last.1 == first.1);
    range_location(point, vertices.chain((!repeats_first).then_some(first)))
}

/// Where `point` lies relative to the polygon with the ring `exterior` and
/// the rings `holes`: inside the exterior, a point inside a hole is outside
/// the polygon, and one on a hole's ring is on its boundary.
///
/// C++: `point_in_geometry<Polygon>` at
/// `algorithms/detail/within/point_in_geometry.hpp`.
pub fn polygon_location<T, I, H>(point: (T, T), exterior: I, holes: H) -> PointLocation
where
    T: CoordinateScalar,
    I: IntoIterator<Item = (T, T)>,
    I::IntoIter: Clone,
    H: IntoIterator<Item = I>,
{
    let location = ring_location(point, exterior);
    if location != PointLocation::Interior {
        return location;
    }
    for hole in holes {
        match ring_location(point, hole) {
            PointLocation::Exterior => {}
            PointLocation::Boundary => return PointLocation::Boundary,
            PointLocation::Interior => return PointLocation::Exterior,
        }
    }
    PointLocation::Interior
}

#[cfg(test)]
mod tests {
    use super::{PointLocation, Winding};

    /// A transcription of Boost's per-segment branches with the side taken
    /// as the sign of the cross product, which `side_by_triangle` is on
    /// small integer coordinates: the count a segment contributes, or
    /// `i32::MIN` for a touch.
    #[allow(clippy::float_cmp)]
    fn reference_step(p: (f64, f64), s1: (f64, f64), s2: (f64, f64)) -> i32 {
        let ((px, py), (s1x, s1y), (s2x, s2y)) = (p, s1, s2);
        let eq1 = s1x == px;
        let eq2 = s2x == px;
        if eq1 && eq2 {
            let (lo, hi) = if s1y <= s2y { (s1y, s2y) } else { (s2y, s1y) };
            return if lo <= py && py <= hi { i32::MIN } else { 0 };
        }

        let count = if eq1 {
            if s2x > px { 1 } else { -1 }
        } else if eq2 {
            if s1x > px { -1 } else { 1 }
        } else if s1x < px && s2x > px {
            2
        } else if s2x < px && s1x > px {
            -2
        } else {
            0
        };
        if count == 0 {
            return 0;
        }

        let side = if count == 1 || count == -1 {
            let sey = if eq1 { s1y } else { s2y };
            if py == sey {
                0
            } else if py < sey {
                -count
            } else {
                count
            }
        } else {
            let cross = (s2x - s1x) * (py - s1y) - (s2y - s1y) * (px - s1x);
            if cross > 0.0 {
                1
            } else if cross < 0.0 {
                -1
            } else {
                0
            }
        };
        if side == 0 {
            i32::MIN
        } else if side * count > 0 {
            count
        } else {
            0
        }
    }

    #[test]
    fn a_step_matches_the_reference_branch_matrix() {
        let values = [-2.0, -1.0, 0.0, 1.0, 2.0];
        for &px in &values {
            for &py in &values {
                for &s1x in &values {
                    for &s1y in &values {
                        for &s2x in &values {
                            for &s2y in &values {
                                let mut winding = Winding::default();
                                winding.apply((px, py), (s1x, s1y), (s2x, s2y));
                                let code = if winding.location() == PointLocation::Boundary {
                                    i32::MIN
                                } else {
                                    winding.count
                                };
                                assert_eq!(
                                    code,
                                    reference_step((px, py), (s1x, s1y), (s2x, s2y)),
                                    "point=({px}, {py}), segment=({s1x}, {s1y})→({s2x}, {s2y})"
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    /// A point interpolated onto an edge is on it, though the rounding
    /// leaves it a little off the line: Boost reports `0` for both
    /// interpolations below.
    #[test]
    fn a_point_interpolated_onto_a_segment_is_on_it() {
        for (s1, s2, point) in [
            (
                (-0.000_524_070_745_816_217_3, 8.845_845_059_190_366e-5),
                (-0.000_260_089_666_903_841_53, 0.000_207_840_077_192_388_92),
                (-0.000_302_994_753_968_636_77, 0.000_188_436_871_840_194_38),
            ),
            (
                (0.991_289_671_020_925_6, -0.059_472_984_955_104_11),
                (-0.481_291_971_343_984_7, -0.531_338_077_906_607_3),
                (0.769_494_689_965_235, -0.130_543_617_876_987_26),
            ),
        ] {
            let mut winding = Winding::default();
            winding.apply(point, s1, s2);
            assert_eq!(winding.location(), PointLocation::Boundary);
        }
    }

    /// A vertical edge through the point touches it and stops the walk:
    /// the point is on the boundary whatever the edges after it wind.
    #[test]
    fn a_touch_stops_the_walk_on_the_boundary() {
        let vertices = [
            (0.0, -1.0),
            (1.0, -1.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, -1.0),
        ];
        assert_eq!(
            super::range_location((1.0, 0.0), vertices),
            PointLocation::Boundary
        );
        assert_eq!(
            super::range_location((0.5, 0.0), vertices),
            PointLocation::Interior
        );
        // No vertices wind nothing.
        assert_eq!(
            super::range_location::<f64>((0.5, 0.0), []),
            PointLocation::Exterior
        );
    }
}
