//! `densify(g, max_distance)` — insert intermediate points so no
//! segment exceeds `max_distance`.
//!
//! Mirrors `boost::geometry::densify(g_in, g_out, max_distance)` from
//! `boost/geometry/algorithms/densify.hpp`. Boost takes the output
//! through an out-parameter; the Rust port returns it by value.

use geometry_strategy::{CartesianDensify, DensifyStrategy};

/// Insert evenly-spaced intermediate vertices into `g` so that no
/// output segment is longer than `max_distance`.
///
/// Mirrors `boost::geometry::densify` from
/// `boost/geometry/algorithms/densify.hpp`. Straight-line
/// interpolation only splits an edge — it never adds length — so the
/// total length of the output equals that of the input.
///
/// # Panics
///
/// Panics if `max_distance` is not strictly positive (zero, negative,
/// or NaN) — Boost throws `geometry::invalid_input_exception` for
/// `max_distance <= 0` (`algorithms/densify.hpp`); the Rust port
/// panics with a clear message.
#[inline]
#[must_use]
pub fn densify<G>(g: &G, max_distance: f64) -> <CartesianDensify as DensifyStrategy<G>>::Output
where
    CartesianDensify: DensifyStrategy<G>,
{
    assert!(
        max_distance > 0.0,
        "densify: max_distance must be positive, got {max_distance} \
         (Boost throws invalid_input_exception for max_distance <= 0)"
    );
    CartesianDensify.densify(g, max_distance)
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Densified coordinates are exact literals."
)]
mod tests {
    //! Reference from `boost/geometry/test/algorithms/densify.cpp:42-65`:
    //! densification splits edges without changing total length, and no
    //! output segment exceeds `max_distance`.

    use super::densify;
    use crate::{length, num_points};
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D, linestring};
    use geometry_strategy::{DistanceStrategy, Pythagoras};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn segment_of_length_10_max_2_5_yields_6_points() {
        // Boost `densify.hpp`: `n = int(10 / 2.5) = 4` intermediate
        // points → `n + 1 = 5` sub-segments of length 2 (< max 2.5).
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        let out = densify(&ls, 2.5);
        assert_eq!(num_points(&out), 6);
        let xs: alloc::vec::Vec<f64> = out.0.iter().map(geometry_trait::Point::get::<0>).collect();
        assert_eq!(xs, alloc::vec![0.0, 2.0, 4.0, 6.0, 8.0, 10.0]);
    }

    #[test]
    fn length_preserved_through_densify() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (3., 4.), (6., 0.)];
        let original = length(&ls);
        let out = densify(&ls, 1.0);
        assert!((length(&out) - original).abs() < 1e-9);
    }

    #[test]
    fn no_output_segment_exceeds_max_distance() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.), (10., 7.)];
        let out = densify(&ls, 1.0);
        let pts: alloc::vec::Vec<&Pt> = geometry_trait::Linestring::points(&out).collect();
        for w in pts.windows(2) {
            assert!(Pythagoras.distance(w[0], w[1]) <= 1.0 + 1e-9);
        }
    }

    /// An empty linestring densifies to empty; a single point copies
    /// through; a zero-length segment gains no intermediate points.
    #[test]
    fn degenerate_inputs_copy_through() {
        let empty: Linestring<Pt> = linestring![];
        assert_eq!(num_points(&densify(&empty, 1.0)), 0);

        let single: Linestring<Pt> = linestring![(3., 4.)];
        let out = densify(&single, 1.0);
        assert_eq!(num_points(&out), 1);

        let zero_len: Linestring<Pt> = linestring![(0., 0.), (0., 0.)];
        assert_eq!(num_points(&densify(&zero_len, 1.0)), 2);
    }

    /// A 3D linestring interpolates the third ordinate too (the `2 =>`
    /// arms of the strategy's per-dimension blend).
    #[test]
    fn three_d_segment_interpolates_z() {
        use geometry_model::Point3D;
        type P3 = Point3D<f64, Cartesian>;
        let ls: Linestring<P3> =
            Linestring::from_vec(alloc::vec![P3::new(0., 0., 0.), P3::new(10., 0., 10.)]);
        // len = √200 ≈ 14.14, max 5.1 → n = 2 intermediates at 1/3, 2/3.
        let out = densify(&ls, 5.1);
        let zs: alloc::vec::Vec<f64> = out.0.iter().map(geometry_trait::Point::get::<2>).collect();
        assert_eq!(zs.len(), 4);
        assert!((zs[1] - 10.0 / 3.0).abs() < 1e-9);
        assert!((zs[2] - 20.0 / 3.0).abs() < 1e-9);
    }

    /// A 4D linestring interpolates the fourth ordinate (the `3 =>` arms).
    #[test]
    #[allow(clippy::float_cmp, reason = "midpoint ordinates are exact literals")]
    fn four_d_segment_interpolates_all_ordinates() {
        use geometry_model::Point;
        use geometry_trait::{Point as _, PointMut as _};
        type P4 = Point<f64, 4, Cartesian>;
        let mut a = P4::default();
        a.set::<3>(0.0);
        let mut b = P4::default();
        b.set::<0>(10.0);
        b.set::<3>(4.0);
        let ls: Linestring<P4> = Linestring::from_vec(alloc::vec![a, b]);
        // len ≈ 10.77, max 6 → one intermediate at t = 0.5.
        let out = densify(&ls, 6.0);
        assert_eq!(num_points(&out), 3);
        assert_eq!(out.0[1].get::<0>(), 5.0);
        assert_eq!(out.0[1].get::<3>(), 2.0);
    }

    #[test]
    #[should_panic(expected = "max_distance must be positive")]
    fn zero_max_distance_panics() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        let _ = densify(&ls, 0.0);
    }

    #[test]
    #[should_panic(expected = "max_distance must be positive")]
    fn negative_max_distance_panics() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        let _ = densify(&ls, -1.0);
    }

    #[test]
    #[should_panic(expected = "max_distance must be positive")]
    fn nan_max_distance_panics() {
        let ls: Linestring<Pt> = linestring![(0., 0.), (10., 0.)];
        let _ = densify(&ls, f64::NAN);
    }
}
