//! Construct a Point / Box / Segment from a coordinate slice.
//!
//! Mirrors `boost::geometry::make<P>(values...)` and its
//! `make<Box>` / `make<Segment>` siblings from
//! `boost/geometry/algorithms/make.hpp`. The counterpart of
//! [`crate::assign::assign_values`]: same contract but
//! returns a fresh value instead of writing into an existing one. The
//! Boost overloads are variadic; the Rust port takes a slice for
//! arity-agnostic ergonomics (see [`crate::assign`] for the
//! trade-off).

use geometry_model::{Box, Segment};
use geometry_trait::PointMut;

use crate::assign::assign_values;

/// Construct a [`PointMut`] from a slice of coordinates.
///
/// Mirrors `boost::geometry::make<Point>(v0, v1, ...)` from
/// `boost/geometry/algorithms/make.hpp`.
///
/// # Panics
///
/// Same as [`crate::assign::assign_values`]: panics if
/// `values.len() < P::DIM`.
#[must_use]
pub fn make_point<P>(values: &[P::Scalar]) -> P
where
    P: PointMut + Default,
{
    let mut p = P::default();
    assign_values(&mut p, values);
    p
}

/// Construct a [`Box`] from two corner coordinate slices.
///
/// The `min_coords` slice fills the minimum corner and `max_coords`
/// the maximum corner. Mirrors `boost::geometry::make<Box>(...)` from
/// `boost/geometry/algorithms/make.hpp`.
///
/// # Panics
///
/// Panics if either slice is shorter than `P::DIM` (see
/// [`make_point`]).
#[must_use]
pub fn make_box<P>(min_coords: &[P::Scalar], max_coords: &[P::Scalar]) -> Box<P>
where
    P: PointMut + Default,
{
    Box::from_corners(make_point(min_coords), make_point(max_coords))
}

/// Construct a [`Segment`] from two endpoint coordinate slices.
///
/// Mirrors `boost::geometry::make<Segment>(...)` from
/// `boost/geometry/algorithms/make.hpp`.
///
/// # Panics
///
/// Panics if either slice is shorter than `P::DIM` (see
/// [`make_point`]).
#[must_use]
pub fn make_segment<P>(start_coords: &[P::Scalar], end_coords: &[P::Scalar]) -> Segment<P>
where
    P: PointMut + Default,
{
    Segment::new(make_point(start_coords), make_point(end_coords))
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Constructed coordinates are read back as exact literals."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/make.cpp:39-74` — a slice is
    //! materialised into a fresh point / box / segment.

    use super::{make_box, make_point, make_segment};
    use geometry_cs::Cartesian;
    use geometry_model::{Box, Point2D, Point3D, Segment};
    use geometry_trait::Point as _;

    type Pt2 = Point2D<f64, Cartesian>;
    type Pt3 = Point3D<f64, Cartesian>;

    // make.cpp:39-40
    #[test]
    fn point2_from_slice() {
        let p: Pt2 = make_point(&[123.0, 456.0]);
        assert_eq!(p.get::<0>(), 123.0);
        assert_eq!(p.get::<1>(), 456.0);
    }

    // make.cpp:47-49
    #[test]
    fn point3_from_slice() {
        let p: Pt3 = make_point(&[123.0, 456.0, 789.0]);
        assert_eq!(p.get::<0>(), 123.0);
        assert_eq!(p.get::<1>(), 456.0);
        assert_eq!(p.get::<2>(), 789.0);
    }

    // make.cpp:57-60
    #[test]
    fn box_from_corners() {
        let b: Box<Pt2> = make_box(&[123.0, 456.0], &[789.0, 1011.0]);
        assert_eq!(b.min().get::<0>(), 123.0);
        assert_eq!(b.min().get::<1>(), 456.0);
        assert_eq!(b.max().get::<0>(), 789.0);
        assert_eq!(b.max().get::<1>(), 1011.0);
    }

    #[test]
    fn segment_from_endpoints() {
        let s: Segment<Pt2> = make_segment(&[0.0, 0.0], &[3.0, 4.0]);
        assert_eq!(s.start().get::<0>(), 0.0);
        assert_eq!(s.end().get::<0>(), 3.0);
    }
}
