//! Infinite 2D line in general form, `a*x + b*y + c = 0`.
//!
//! Mirrors `boost::geometry::model::infinite_line` from
//! `geometries/infinite_line.hpp:29-52` and the operations in
//! `arithmetic/infinite_line_functions.hpp:24-107`.

use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_trait::Point;

use crate::Point2D;

/// An infinite line in the general form `a*x + b*y + c = 0`.
///
/// Mirrors `model::infinite_line<Type>` from
/// `geometries/infinite_line.hpp:35-52`. It is intentionally not a
/// [`geometry_trait::Geometry`] because the C++ source likewise notes that the
/// type is not conceptized.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InfiniteLine<T: CoordinateScalar = f64> {
    /// Coefficient of x. A horizontal line has `a == 0`.
    pub a: T,
    /// Coefficient of y. A vertical line has `b == 0`.
    pub b: T,
    /// Constant term. A line through the origin has `c == 0`.
    pub c: T,
    /// Whether the coefficients have been normalized by a caller.
    pub normalized: bool,
}

impl<T: CoordinateScalar> InfiniteLine<T> {
    /// Construct a line from its general-form coefficients.
    #[inline]
    #[must_use]
    pub const fn new(a: T, b: T, c: T) -> Self {
        Self {
            a,
            b,
            c,
            normalized: false,
        }
    }

    /// Construct the infinite line through `(x1, y1)` and `(x2, y2)`.
    ///
    /// Mirrors `detail::make::make_infinite_line` from
    /// `algorithms/detail/make/make.hpp:21-32`:
    /// `a = y1-y2`, `b = x2-x1`, `c = -a*x1-b*y1`.
    #[inline]
    #[must_use]
    pub fn from_coordinates(x1: T, y1: T, x2: T, y2: T) -> Self {
        let a = y1 - y2;
        let b = x2 - x1;
        let c = -(a * x1) - b * y1;
        Self::new(a, b, c)
    }

    /// Construct the infinite line through two points.
    ///
    /// Mirrors `make_infinite_line(PointA, PointB)` from
    /// `algorithms/detail/make/make.hpp:34-40`.
    #[inline]
    #[must_use]
    pub fn from_points<P>(start: &P, end: &P) -> Self
    where
        P: Point<Scalar = T>,
    {
        Self::from_coordinates(
            start.get::<0>(),
            start.get::<1>(),
            end.get::<0>(),
            end.get::<1>(),
        )
    }

    /// Return the unnormalized signed side measure at `(x, y)`.
    ///
    /// Positive is left, negative is right, and zero is on the line.
    /// Mirrors `arithmetic::side_value` from
    /// `arithmetic/infinite_line_functions.hpp:70-93`.
    #[inline]
    #[must_use]
    pub fn side_value(&self, x: T, y: T) -> T {
        self.a * x + self.b * y + self.c
    }

    /// Return whether both directional coefficients are zero.
    ///
    /// Mirrors `arithmetic::is_degenerate` from
    /// `arithmetic/infinite_line_functions.hpp:99-104`.
    #[inline]
    #[must_use]
    pub fn is_degenerate(&self) -> bool {
        self.a == T::ZERO && self.b == T::ZERO
    }

    /// Calculate the intersection of two non-parallel infinite lines.
    ///
    /// Mirrors `arithmetic::intersection_point` and
    /// `assign_intersection_point` from
    /// `arithmetic/infinite_line_functions.hpp:32-67`. `None` represents
    /// parallel or collinear lines.
    #[inline]
    #[must_use]
    pub fn intersection(&self, other: &Self) -> Option<Point2D<T, Cartesian>> {
        let denominator = self.a * other.b - self.b * other.a;
        if denominator == T::ZERO {
            return None;
        }
        let x = (self.b * other.c - self.c * other.b) / denominator;
        let y = (self.c * other.a - self.a * other.c) / denominator;
        Some(Point2D::new(x, y))
    }
}

impl<T: CoordinateScalar> Default for InfiniteLine<T> {
    #[inline]
    fn default() -> Self {
        Self::new(T::ZERO, T::ZERO, T::ZERO)
    }
}
