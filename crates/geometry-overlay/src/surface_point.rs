//! OVL6.T3 — `point_on_surface`.
//!
//! Mirrors `boost/geometry/algorithms/point_on_surface.hpp`: pick a
//! point **guaranteed to lie in the interior** of an areal geometry.
//! Boost uses a horizontal sweep at a representative height and takes
//! the midpoint of the widest interior span the scanline cuts; the port
//! does the same.
//!
//! Unlike the centroid, this point is always inside the polygon even for
//! non-convex or holed shapes — which is exactly why overlay,
//! labelling, and `relate` need it.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_trait::{Point, PointMut, Polygon as PolygonTrait, Ring as RingTrait};

/// A point guaranteed to lie in the interior of `polygon`, or `None`
/// for a degenerate (zero-area) polygon.
///
/// Sweeps a horizontal line at the average of the exterior ring's
/// minimum and maximum y, collects the x-values where it crosses the
/// boundary (exterior and holes), and returns the midpoint of the
/// widest gap between consecutive crossings that lies in the interior.
///
/// Mirrors `boost::geometry::point_on_surface`
/// (`algorithms/point_on_surface.hpp`).
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::surface_point::point_on_surface;
/// use geometry_trait::Point as _;
///
/// type P = Point2D<f64, Cartesian>;
/// let pg: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
/// let p = point_on_surface(&pg).unwrap();
/// // The representative point is inside the square.
/// assert!(p.get::<0>() > 0.0 && p.get::<0>() < 4.0);
/// assert!(p.get::<1>() > 0.0 && p.get::<1>() < 4.0);
/// ```
#[inline]
#[must_use]
pub fn point_on_surface<G, P>(polygon: &G) -> Option<P>
where
    G: PolygonTrait<Point = P>,
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar,
{
    let outer: Vec<P> = polygon.exterior().points().copied().collect();
    if outer.len() < 3 {
        return None;
    }

    // Representative sweep height: the average of the exterior's y-range.
    let mut ymin = outer[0].get::<1>();
    let mut ymax = ymin;
    for p in &outer {
        let y = p.get::<1>();
        if y < ymin {
            ymin = y;
        }
        if y > ymax {
            ymax = y;
        }
    }
    let two = P::Scalar::ONE + P::Scalar::ONE;
    let sweep_y = (ymin + ymax) / two;

    // Collect x-crossings of the sweep line with every ring.
    let mut xs: Vec<P::Scalar> = Vec::new();
    collect_crossings(&outer, sweep_y, &mut xs);
    for hole in polygon.interiors() {
        let hpts: Vec<P> = hole.points().copied().collect();
        collect_crossings(&hpts, sweep_y, &mut xs);
    }

    if xs.len() < 2 {
        return None;
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));

    // The interior spans are the odd gaps (between crossing 0-1, 2-3, …).
    // Take the midpoint of the widest such span.
    let mut best: Option<(P::Scalar, P::Scalar)> = None; // (width, mid_x)
    let mut i = 0;
    while i + 1 < xs.len() {
        let lo = xs[i];
        let hi = xs[i + 1];
        let width = hi - lo;
        let mid = (lo + hi) / two;
        match best {
            Some((bw, _)) if bw >= width => {}
            _ => best = Some((width, mid)),
        }
        i += 2;
    }

    let (_, mid_x) = best?;
    let mut p = P::default();
    p.set::<0>(mid_x);
    p.set::<1>(sweep_y);
    Some(p)
}

/// Append the x-coordinates where the horizontal line `y = sweep_y`
/// crosses the edges of the vertex ring `pts`.
fn collect_crossings<P>(pts: &[P], sweep_y: P::Scalar, out: &mut Vec<P::Scalar>)
where
    P: Point,
    P::Scalar: CoordinateScalar,
{
    let n = pts.len();
    if n < 2 {
        return;
    }
    for k in 0..n {
        let a = &pts[k];
        let b = &pts[(k + 1) % n];
        let ay = a.get::<1>();
        let by = b.get::<1>();
        // Half-open crossing test to avoid double-counting a shared
        // vertex: the edge crosses the sweep if exactly one endpoint is
        // strictly above it.
        if (ay > sweep_y) != (by > sweep_y) {
            let ax = a.get::<0>();
            let bx = b.get::<0>();
            let t = (sweep_y - ay) / (by - ay);
            out.push(ax + t * (bx - ax));
        }
    }
}

#[cfg(test)]
mod tests {
    //! OVL6.T3 done-when: the returned point is inside the polygon.
    //! Mirrors `test/algorithms/point_on_surface.cpp`.

    use super::point_on_surface;
    use geometry_algorithm::within;
    use geometry_cs::Cartesian;
    use geometry_model::{Point2D, Polygon, polygon};

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn inside_a_square() {
        let pg: Polygon<P> = polygon![[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]];
        let p = point_on_surface(&pg).unwrap();
        assert!(within(&p, &pg));
    }

    #[test]
    fn inside_an_l_shape() {
        // Non-convex L: the centroid could fall outside, but the sweep
        // point must be inside.
        let pg: Polygon<P> = polygon![[
            (0.0, 0.0),
            (6.0, 0.0),
            (6.0, 2.0),
            (2.0, 2.0),
            (2.0, 6.0),
            (0.0, 6.0),
            (0.0, 0.0)
        ]];
        let p = point_on_surface(&pg).unwrap();
        assert!(within(&p, &pg));
    }

    #[test]
    fn avoids_a_hole() {
        // A big square with a big central hole; the representative point
        // must land in the ring of material, not the hole.
        let pg: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ],
            [(3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0), (3.0, 3.0)]
        ];
        let p = point_on_surface(&pg).unwrap();
        assert!(within(&p, &pg));
    }
}
