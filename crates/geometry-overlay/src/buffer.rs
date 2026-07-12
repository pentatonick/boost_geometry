//! OVL7 — `buffer`: grow a geometry outward by a fixed distance.
//!
//! Mirrors `boost/geometry/algorithms/buffer.hpp` and the buffer
//! strategies under `strategies/buffer/`. A buffer offsets every part of
//! the input outward by `distance`, rounding or mitering the corners,
//! and unions the offset pieces into an output polygon.
//!
//! v1 scope: **positive** distance buffers of a point (→ a circle) and a
//! convex polygon (→ the polygon grown with rounded corners). The
//! general non-convex / negative-distance buffer, which needs the full
//! offset-and-self-union machinery, is deferred — it builds on the same
//! overlay `union` this module already uses.
//!
//! Join / end / point strategies are modelled as small enums
//! ([`JoinStrategy`], [`PointStrategy`]) mirroring Boost's
//! `join_round` / `join_miter` and `point_circle` / `point_square`
//! strategy types.

// Segment counts convert freely between `usize` and `f64` to lay out
// circle / arc vertices; the values are small angular subdivisions where
// the sub-mantissa precision loss and the non-negative truncation are
// intentional and harmless.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "angular vertex-count arithmetic; values are small and non-negative"
)]
// Zero-length guards and closing-vertex identity compare `f64`s exactly
// on purpose — these are degenerate-case gates, not tolerance checks.
#![allow(clippy::float_cmp, reason = "exact degenerate-case guards")]

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem, FromF64};
use geometry_model::{Polygon, Ring};
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait};

/// How to fill the wedge at a convex corner of the offset boundary.
///
/// Mirrors `strategy::buffer::join_round` / `join_miter`
/// (`strategies/buffer/buffer_join_round.hpp` and friends).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinStrategy {
    /// Fill the corner with a circular arc of `points_per_circle`
    /// segments. Boost's `join_round`.
    Round {
        /// Segment count of a full circle; the arc uses a proportional
        /// share.
        points_per_circle: usize,
    },
    /// Extend the two offset edges until they meet at a sharp point.
    /// Boost's `join_miter`.
    ///
    /// The miter length is currently **uncapped**: Boost's
    /// `miter_limit` (default 5.0 × distance,
    /// `strategies/buffer/buffer_join_miter.hpp`) is not yet
    /// implemented, so a near-180° corner produces a proportionally
    /// long spike. Capping is deferred with the rest of the
    /// non-convex buffer work.
    Miter,
}

/// How to approximate a buffered point.
///
/// Mirrors `strategy::buffer::point_circle` / `point_square`
/// (`strategies/buffer/buffer_point_circle.hpp`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointStrategy {
    /// Approximate the buffer disc with a regular polygon of
    /// `points_per_circle` vertices. Boost's `point_circle`.
    Circle {
        /// Vertex count of the approximating polygon.
        points_per_circle: usize,
    },
    /// Approximate the buffer with an axis-aligned square. Boost's
    /// `point_square`.
    Square,
}

/// Buffer a point by `distance`, producing the disc (or square)
/// approximation.
///
/// Mirrors the point arm of `boost::geometry::buffer` with a
/// `point_circle` / `point_square` strategy
/// (`strategies/buffer/buffer_point_circle.hpp`).
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_overlay::buffer::{buffer_point, PointStrategy};
/// use geometry_algorithm::ring_area;
///
/// type P = Point2D<f64, Cartesian>;
/// let disc = buffer_point(&P::new(0.0, 0.0), 1.0, PointStrategy::Circle { points_per_circle: 360 });
/// // Area of the 360-gon closely approximates π.
/// assert!((ring_area(&disc).abs() - core::f64::consts::PI).abs() < 1e-3);
/// ```
#[must_use]
pub fn buffer_point<P>(center: &P, distance: f64, strategy: PointStrategy) -> Ring<P>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let cx: f64 = center.get::<0>().into();
    let cy: f64 = center.get::<1>().into();
    match strategy {
        PointStrategy::Circle { points_per_circle } => {
            circle_ring(cx, cy, distance, points_per_circle.max(3))
        }
        PointStrategy::Square => {
            let d = distance;
            // Fully-qualified `alloc::vec!`: only the `Vec` *type* is
            // imported (line 33), and the bare `vec!` macro is not in the
            // `no_std` prelude — matches the crate idiom in `assemble.rs`
            // / `traverse/state.rs`.
            Ring::from_vec(alloc::vec![
                make_point(cx - d, cy - d),
                make_point(cx + d, cy - d),
                make_point(cx + d, cy + d),
                make_point(cx - d, cy + d),
                make_point(cx - d, cy - d),
            ])
        }
    }
}

/// Buffer a **convex** polygon outward by a positive `distance`, rounding
/// the corners per `join`.
///
/// Each vertex of a convex polygon becomes a circular arc of radius
/// `distance` in the offset boundary; the arcs are joined by the offset
/// edges. Mirrors the convex case of `boost::geometry::buffer`
/// (`algorithms/buffer.hpp`) with a `join_round` strategy.
///
/// # Panics
///
/// Does not panic; a polygon with fewer than 3 exterior vertices returns
/// an empty ring's polygon.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::buffer::{buffer_convex_polygon, JoinStrategy};
/// use geometry_algorithm::ring_area;
/// use geometry_trait::Polygon as _;
///
/// type P = Point2D<f64, Cartesian>;
/// let sq: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
/// let grown = buffer_convex_polygon(&sq, 1.0, JoinStrategy::Round { points_per_circle: 720 });
/// // Area = s² + 4·s·d + π·d² = 4 + 8 + π.
/// let expected = 4.0 + 8.0 + core::f64::consts::PI;
/// assert!((ring_area(grown.exterior()).abs() - expected).abs() < 5e-2);
/// ```
#[must_use]
pub fn buffer_convex_polygon<G, P>(polygon: &G, distance: f64, join: JoinStrategy) -> Polygon<P>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64> + FromF64,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let mut verts: Vec<(f64, f64)> = distinct_vertices(polygon.exterior());
    if verts.len() < 3 {
        return Polygon::new(Ring::new());
    }

    // Normalise winding to geometrically counter-clockwise so the
    // right-hand normal `(dy, -dx)` is genuinely outward and the corner
    // arc sweeps the short (convex) way. A negative *math* signed area
    // (CCW-positive convention) means the ring is clockwise and must be
    // reversed. Reversing a ring negates its signed area without changing
    // the point set, so the buffered shape is unaffected.
    if signed_area_ccw_positive(&verts) < 0.0 {
        verts.reverse();
    }

    // The outward normal of a CCW edge points to its right. For each
    // edge, offset both endpoints outward; at each vertex, connect the
    // incoming and outgoing offset points with an arc (round) or their
    // intersection (miter). Scratch vertices stay `(f64, f64)` — the
    // trig-heavy kernel computes in `f64` (mirroring Boost's promoted
    // floating type) and materialises `P` only at the output boundary.
    let n = verts.len();
    let mut boundary: Vec<(f64, f64)> = Vec::new();

    for i in 0..n {
        let prev = verts[(i + n - 1) % n];
        let curr = verts[i];
        let next = verts[(i + 1) % n];

        // Outward normals of the two edges meeting at `curr`.
        let n_in = outward_normal(curr.0 - prev.0, curr.1 - prev.1);
        let n_out = outward_normal(next.0 - curr.0, next.1 - curr.1);

        // Offset positions of `curr` along each edge's normal.
        let p_in = (curr.0 + n_in.0 * distance, curr.1 + n_in.1 * distance);
        let p_out = (curr.0 + n_out.0 * distance, curr.1 + n_out.1 * distance);

        boundary.push(p_in);
        match join {
            JoinStrategy::Round { points_per_circle } => {
                push_corner_arc(
                    &mut boundary,
                    curr,
                    p_in,
                    p_out,
                    distance,
                    points_per_circle.max(3),
                );
            }
            JoinStrategy::Miter => {
                // True miter: the intersection of the two offset edge
                // lines, `curr + s · (2d / |s|²)` with `s = n_in + n_out`
                // (`|s| = 2·cos(θ/2)`, so the point sits at
                // `d / cos(θ/2)` along the outward bisector). Mirrors
                // `strategy::buffer::join_miter`
                // (`strategies/buffer/buffer_join_miter.hpp`), minus the
                // miter_limit cap (documented on the enum variant).
                //
                // Degenerate guards: a zero-length edge yields a (0,0)
                // normal and an anti-parallel pair yields `s == (0,0)`;
                // in both cases there is no finite/meaningful miter
                // point, so emit none — the `p_in` / `p_out` offset
                // points already bracket the corner.
                let n_in_ok = n_in.0 != 0.0 || n_in.1 != 0.0;
                let n_out_ok = n_out.0 != 0.0 || n_out.1 != 0.0;
                let sx = n_in.0 + n_out.0;
                let sy = n_in.1 + n_out.1;
                let len2 = sx * sx + sy * sy;
                if n_in_ok && n_out_ok && len2 > 0.0 {
                    let scale = 2.0 * distance / len2;
                    boundary.push((curr.0 + sx * scale, curr.1 + sy * scale));
                }
            }
        }
        boundary.push(p_out);
    }

    // Close the ring.
    if let Some(first) = boundary.first().copied() {
        boundary.push(first);
    }
    Polygon::new(Ring::from_vec(
        boundary
            .into_iter()
            .map(|(x, y)| make_point(x, y))
            .collect(),
    ))
}

/// Materialise an output point from the `f64` kernel coordinates.
fn make_point<P>(x: f64, y: f64) -> P
where
    P: PointMut + Default,
    P::Scalar: FromF64,
{
    let mut p = P::default();
    p.set::<0>(P::Scalar::from_f64(x));
    p.set::<1>(P::Scalar::from_f64(y));
    p
}

/// A regular-polygon approximation of a circle, CCW, closed.
fn circle_ring<P>(cx: f64, cy: f64, r: f64, segments: usize) -> Ring<P>
where
    P: PointMut + Default + Copy,
    P::Scalar: FromF64,
{
    let mut pts = Vec::with_capacity(segments + 1);
    let step = core::f64::consts::TAU / segments as f64;
    for k in 0..segments {
        let a = step * k as f64;
        pts.push(make_point(cx + r * a.cos(), cy + r * a.sin()));
    }
    pts.push(pts[0]);
    Ring::from_vec(pts)
}

/// Distinct consecutive vertices of a ring as `f64` pairs (drops the
/// closing repeat).
fn distinct_vertices<R>(ring: &R) -> Vec<(f64, f64)>
where
    R: RingTrait,
    <R::Point as Point>::Scalar: Into<f64>,
{
    let mut pts: Vec<(f64, f64)> = ring
        .points()
        .map(|p| (p.get::<0>().into(), p.get::<1>().into()))
        .collect();
    if pts.len() >= 2 {
        let first = pts[0];
        let last = pts[pts.len() - 1];
        if first == last {
            pts.pop();
        }
    }
    pts
}

/// The standard math signed area of the vertex ring (counter-clockwise
/// positive), via the shoelace sum over the closed loop. Used only to
/// detect winding for normalisation.
fn signed_area_ccw_positive(verts: &[(f64, f64)]) -> f64 {
    let n = verts.len();
    let mut acc = 0.0;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        acc += a.0 * b.1 - b.0 * a.1;
    }
    acc * 0.5
}

/// The outward unit normal of a directed CCW edge with delta
/// `(dx, dy)` (pointing to the edge's right).
fn outward_normal(dx: f64, dy: f64) -> (f64, f64) {
    let len = (dx * dx + dy * dy).sqrt();
    if len == 0.0 {
        return (0.0, 0.0);
    }
    // Right-hand normal of (dx, dy) is (dy, -dx).
    (dy / len, -dx / len)
}

/// Push the arc that rounds a convex corner, from offset point `from`
/// to `to`, centred on the original vertex `center` at radius
/// `distance`.
fn push_corner_arc(
    out: &mut Vec<(f64, f64)>,
    center: (f64, f64),
    from: (f64, f64),
    to: (f64, f64),
    distance: f64,
    points_per_circle: usize,
) {
    let (cx, cy) = center;
    let a0 = (from.1 - cy).atan2(from.0 - cx);
    let mut a1 = (to.1 - cy).atan2(to.0 - cx);
    // Sweep the short way, counter-clockwise (positive) for a convex CCW
    // corner.
    while a1 < a0 {
        a1 += core::f64::consts::TAU;
    }
    let sweep = a1 - a0;
    let steps = ((sweep / core::f64::consts::TAU) * points_per_circle as f64).ceil() as usize;
    let steps = steps.max(1);
    for k in 1..steps {
        let a = a0 + sweep * (k as f64 / steps as f64);
        out.push((cx + distance * a.cos(), cy + distance * a.sin()));
    }
}

#[cfg(test)]
mod tests {
    //! OVL7 done-when: buffered areas match the closed-form values.
    //! Mirrors `test/algorithms/buffer/`.

    use super::{JoinStrategy, PointStrategy, buffer_convex_polygon, buffer_point};
    use geometry_algorithm::ring_area;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};
    use geometry_trait::Polygon as _;

    type P = Point2D<f64, Cartesian>;

    fn close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "expected {b}, got {a}");
    }

    #[test]
    fn point_circle_area_approximates_pi_r_squared() {
        let disc = buffer_point(
            &P::new(0.0, 0.0),
            2.0,
            PointStrategy::Circle {
                points_per_circle: 720,
            },
        );
        // π·r² = π·4.
        close(ring_area(&disc).abs(), core::f64::consts::PI * 4.0, 1e-2);
    }

    #[test]
    fn point_square_area() {
        let sq = buffer_point(&P::new(0.0, 0.0), 3.0, PointStrategy::Square);
        // A square of half-side 3 → side 6 → area 36.
        close(ring_area(&sq).abs(), 36.0, 1e-9);
    }

    #[test]
    fn convex_square_round_buffer_area() {
        let sq: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let grown = buffer_convex_polygon(
            &sq,
            1.0,
            JoinStrategy::Round {
                points_per_circle: 720,
            },
        );
        // s² + 4·s·d + π·d² = 4 + 8 + π.
        let expected = 4.0 + 8.0 + core::f64::consts::PI;
        close(ring_area(grown.exterior()).abs(), expected, 1e-2);
    }

    #[test]
    fn convex_triangle_round_buffer_grows() {
        let tri: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (0.0, 0.0)]];
        let base = ring_area(tri.exterior()).abs(); // 6
        let grown = buffer_convex_polygon(
            &tri,
            0.5,
            JoinStrategy::Round {
                points_per_circle: 360,
            },
        );
        // The buffered area must exceed the original.
        assert!(ring_area(grown.exterior()).abs() > base);
    }

    #[test]
    fn buffer_is_winding_independent() {
        // Regression: the same square listed clockwise and counter-
        // clockwise must buffer to the same grown area. The winding
        // normalisation makes the outward offset direction correct for
        // both.
        let ccw: Polygon<P> =
            polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let cw: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]];
        let j = JoinStrategy::Round {
            points_per_circle: 720,
        };
        let expected = 4.0 + 8.0 + core::f64::consts::PI;
        let grown_from_counterclockwise =
            ring_area(buffer_convex_polygon(&ccw, 1.0, j).exterior()).abs();
        let grown_from_clockwise = ring_area(buffer_convex_polygon(&cw, 1.0, j).exterior()).abs();
        close(grown_from_counterclockwise, expected, 5e-2);
        close(grown_from_clockwise, expected, 5e-2);
    }

    #[test]
    fn miter_square_area_is_16() {
        // Regression: the old Miter arm placed the corner point at
        // distance d along the bisector (ON the round arc), yielding
        // 14.83 — smaller than even the round buffer. A true miter
        // corner is the offset-edge intersection at √2·d, so the
        // buffered 2×2 square is s² + 4·s·d + 4·d² = 16 exactly.
        let sq: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let grown = buffer_convex_polygon(&sq, 1.0, JoinStrategy::Miter);
        close(ring_area(grown.exterior()).abs(), 16.0, 1e-9);
    }

    #[test]
    fn miter_contains_near_corner_probe() {
        // A point at distance 0.99 < d from the input corner, in the
        // 22.5° direction, was EXCLUDED by the old chord-cut corner.
        use geometry_algorithm::within;
        let sq: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let grown = buffer_convex_polygon(&sq, 1.0, JoinStrategy::Miter);
        let ang = 22.5_f64.to_radians();
        let probe = P::new(2.0 + 0.99 * ang.cos(), 2.0 + 0.99 * ang.sin());
        assert!(
            within(&probe, &grown),
            "buffer must contain points within d"
        );
    }

    #[test]
    fn miter_is_superset_of_round_by_area() {
        // A miter fills the wedge beyond the round arc, so its area
        // can never be below the round join's.
        let j_round = JoinStrategy::Round {
            points_per_circle: 720,
        };
        let square: Polygon<P> =
            polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let triangle: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (0.0, 0.0)]];
        for pg in [square, triangle] {
            let m =
                ring_area(buffer_convex_polygon(&pg, 1.0, JoinStrategy::Miter).exterior()).abs();
            let r = ring_area(buffer_convex_polygon(&pg, 1.0, j_round).exterior()).abs();
            assert!(m >= r - 1e-9, "miter {m} must not be below round {r}");
        }
    }

    #[test]
    fn non_model_polygon_buffers_like_the_model_polygon() {
        // The generic signature accepts any `Polygon` trait impl — a
        // hand-rolled type must buffer to the same area as the same
        // shape held in a model polygon.
        use geometry_model::Ring;
        use geometry_tag::PolygonTag;
        use geometry_trait::{Geometry, Polygon as PolygonTrait};

        struct Parcel {
            outer: Ring<P>,
        }
        impl Geometry for Parcel {
            type Kind = PolygonTag;
            type Point = P;
        }
        impl PolygonTrait for Parcel {
            type Ring = Ring<P>;
            fn exterior(&self) -> &Ring<P> {
                &self.outer
            }
            fn interiors(&self) -> impl ExactSizeIterator<Item = &Ring<P>> {
                core::iter::empty()
            }
        }

        let pts = vec![
            P::new(0.0, 0.0),
            P::new(2.0, 0.0),
            P::new(2.0, 2.0),
            P::new(0.0, 2.0),
            P::new(0.0, 0.0),
        ];
        let parcel = Parcel {
            outer: Ring::from_vec(pts.clone()),
        };
        let model: Polygon<P> = Polygon::new(Ring::from_vec(pts));
        let j = JoinStrategy::Round {
            points_per_circle: 360,
        };
        let a = ring_area(buffer_convex_polygon(&parcel, 1.0, j).exterior()).abs();
        let b = ring_area(buffer_convex_polygon(&model, 1.0, j).exterior()).abs();
        close(a, b, 1e-12);
    }

    #[test]
    fn miter_is_winding_independent() {
        // Same square listed CW and CCW buffers to the same miter area.
        let ccw: Polygon<P> =
            polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
        let cw: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]];
        close(
            ring_area(buffer_convex_polygon(&ccw, 1.0, JoinStrategy::Miter).exterior()).abs(),
            16.0,
            1e-9,
        );
        close(
            ring_area(buffer_convex_polygon(&cw, 1.0, JoinStrategy::Miter).exterior()).abs(),
            16.0,
            1e-9,
        );
    }
}
