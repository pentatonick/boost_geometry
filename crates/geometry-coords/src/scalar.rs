//! The `CoordinateScalar` trait: the bound every algorithm in the
//! kernel places on a coordinate value.
//!
//! Distilled from the implicit set of operations Boost.Geometry's
//! strategies assume of their `CalculationType` — see
//! `boost/geometry/util/calculation_type.hpp` for how the C++ side
//! picks that working type, and `boost/geometry/util/math.hpp` for
//! the primitive operations (`abs`, `sqrt`) it then invokes on it.

use core::cmp::Ordering;
use core::ops::{Add, Div, Mul, Neg, Sub};

/// Numeric type usable as a geometry coordinate.
///
/// The operator and `Copy + PartialOrd` bounds capture what Boost's
/// strategies require of their `coordinate_type<P>` / `CalculationType`
/// (see `boost/geometry/util/calculation_type.hpp`). The `ZERO`/`ONE`
/// constants and `sqrt`/`abs` methods mirror the primitives in
/// `boost/geometry/util/math.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_coords::CoordinateScalar;
/// fn norm<T: CoordinateScalar>(x: T, y: T) -> T {
///     (x * x + y * y).sqrt()
/// }
/// assert_eq!(norm(3.0_f64, 4.0), 5.0);
/// ```
pub trait CoordinateScalar:
    Copy
    + PartialOrd
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
{
    /// The additive identity. Counterpart to literal `T(0)` in
    /// `boost/geometry/util/math.hpp`.
    const ZERO: Self;
    /// The multiplicative identity. Counterpart to literal `T(1)` in
    /// `boost/geometry/util/math.hpp`.
    const ONE: Self;

    /// The scalar a measure of these coordinates — a distance, a length,
    /// an area, a centroid's weighted sums — is computed and returned in:
    /// `f64` for an integer, the scalar itself otherwise.
    ///
    /// Counterpart to `boost::geometry::promote_floating_point`
    /// (`boost/geometry/util/promote_floating_point.hpp`): Boost computes
    /// those measures of integer coordinates in `double`, where a square
    /// root or a half-unit area exists and products cannot overflow.
    ///
    /// Boost also computes the measures of `float` coordinates in `double`
    /// (`select_most_precise<float, double>`); this port computes and
    /// returns them in `f32`, so an `f32` geometry measures as `f32`
    /// arithmetic does. Side tests are not measures: those promote `f32` to
    /// `f64` as Boost does ([`Self::side_by_triangle`]).
    type Measure: CoordinateScalar;

    /// This coordinate as a [`Self::Measure`].
    #[must_use]
    fn to_measure(self) -> Self::Measure;

    /// A measure back as a coordinate: truncated toward zero for an
    /// integer, as Boost's `numeric_cast` does, and saturated at the
    /// integer's range; the value itself otherwise.
    #[must_use]
    fn from_measure(measure: Self::Measure) -> Self;

    /// Square root.
    ///
    /// For integers this is a placeholder: a measure that needs a `sqrt`
    /// is computed in [`Self::Measure`], so calling this on `i32`/`i64`
    /// indicates a kernel bug.
    ///
    /// Counterpart to `boost::geometry::math::sqrt`
    /// (`boost/geometry/util/math.hpp`).
    #[must_use]
    fn sqrt(self) -> Self;

    /// Absolute value. Counterpart to `boost::geometry::math::abs`
    /// (`boost/geometry/util/math.hpp`).
    #[must_use]
    fn abs(self) -> Self;

    /// Equality the way the kernel means it.
    ///
    /// Counterpart to `boost::geometry::math::equals`
    /// (`boost/geometry/util/math.hpp`) under `equals_default_policy`:
    /// exact for an integer, and for a float, equal when the difference is
    /// within one epsilon of the larger magnitude — or of `1`, so that two
    /// values near zero still have to agree to an absolute epsilon.
    ///
    /// This is not a convenience. Boost's side predicate calls three points
    /// collinear when any *two* of them are equal by this rule, so a pair a
    /// few last bits apart at a large coordinate is coincident to the whole
    /// kernel, and every predicate built on the side test follows.
    #[must_use]
    fn tolerant_eq(self, other: Self) -> bool;

    /// Whether the value is finite: neither infinite nor NaN.
    ///
    /// Counterpart to `boost::math::isfinite`, which Boost's centroid
    /// strategies ask of a measure before they divide by it
    /// (`strategies/cartesian/centroid_bashein_detmer.hpp:216-217`). Every
    /// value of a scalar without infinities is finite, which is the default.
    #[must_use]
    #[inline]
    fn is_finite(self) -> bool {
        true
    }

    /// The highest value: `util::bounds<T>::highest()` (`util/bounds.hpp`),
    /// the minimum corner of the inverse box Boost makes the envelope of
    /// nothing (`algorithms/detail/envelope/initialize.hpp:61-79`).
    /// `numeric::bounds` answers `T()` for a type it does not know, which is
    /// the default.
    #[must_use]
    #[inline]
    fn highest() -> Self {
        Self::ZERO
    }

    /// The lowest value: `util::bounds<T>::lowest()`, the maximum corner of
    /// that inverse box. `T()` by default, as [`Self::highest`] is.
    #[must_use]
    #[inline]
    fn lowest() -> Self {
        Self::ZERO
    }

    /// The sign of the cross product `u × v` of two coordinate
    /// differences, `u = u_to − u_from` and `v = v_to − v_from`: positive
    /// where `v` turns left of `u`, negative where it turns right, zero
    /// where they are collinear, and `None` where it has no sign (a NaN).
    ///
    /// This is the side and turn test every Cartesian predicate rests on.
    /// A float evaluates `ux·vy − uy·vx` in itself. An integer evaluates
    /// it exactly, in a wider integer, as Boost's `side_by_triangle` does
    /// through `promote_integral`, so no pair of coordinates overflows it.
    #[must_use]
    fn cross_sign(
        u_from: (Self, Self),
        u_to: (Self, Self),
        v_from: (Self, Self),
        v_to: (Self, Self),
    ) -> Option<Ordering> {
        let (ux, uy) = (u_to.0 - u_from.0, u_to.1 - u_from.1);
        let (vx, vy) = (v_to.0 - v_from.0, v_to.1 - v_from.1);
        (ux * vy - uy * vx).partial_cmp(&Self::ZERO)
    }

    /// The sign of the dot product `u · v` of two coordinate differences,
    /// as [`Self::cross_sign`] takes them: positive where `u` and `v` point
    /// the same way, negative where they point apart. Exact for an
    /// integer, like [`Self::cross_sign`].
    #[must_use]
    fn dot_sign(
        u_from: (Self, Self),
        u_to: (Self, Self),
        v_from: (Self, Self),
        v_to: (Self, Self),
    ) -> Option<Ordering> {
        let (ux, uy) = (u_to.0 - u_from.0, u_to.1 - u_from.1);
        let (vx, vy) = (v_to.0 - v_from.0, v_to.1 - v_from.1);
        (ux * vx + uy * vy).partial_cmp(&Self::ZERO)
    }

    /// Where `p` lies along the direction `a → b`: `Greater` beyond `b`,
    /// `Less` short of it (or nowhere, a NaN, as Boost's `-1`), and `Equal`
    /// level with `b` or for a direction of no length.
    ///
    /// C++: `direction_code<cartesian_tag>`
    /// (`algorithms/detail/direction_code.hpp:47-85`), the sign of `p` in
    /// the equation of the line through `b` square to `a → b`, which is
    /// the sign of `(b − a) · (p − b)`. An integer evaluates that exactly,
    /// as Boost does in the integer type. A float evaluates the line's
    /// equation as Boost does, so a point within a rounding error of `b`
    /// falls on the side Boost's rounding puts it.
    #[must_use]
    fn direction_code(a: (Self, Self), b: (Self, Self), p: (Self, Self)) -> Ordering {
        Self::dot_sign(a, b, b, p).unwrap_or(Ordering::Less)
    }

    /// The side of `p` relative to the directed line `p1 → p2`, by Boost's
    /// default Cartesian side strategy: `Greater` to the left, `Less` to the
    /// right — and for a point that has no side (a NaN), as Boost's `-1` —
    /// and `Equal` on the line.
    ///
    /// C++: `strategy::side::side_by_triangle<>::apply`
    /// (`strategy/cartesian/side_by_triangle.hpp`), the side test behind
    /// point in polygon, point on segment, segment intersection and spikes.
    /// An integer evaluates the determinant exactly, as `promote_integral`
    /// makes Boost's, and so does any scalar whose arithmetic is exact. A
    /// float follows Boost's floating-point path: a triple of which two
    /// points are equal by [`Self::tolerant_eq`] is collinear; the triple
    /// is rotated to start at its lexicographically smallest point; the
    /// determinant of the differences is evaluated in `f64`; and one within
    /// an epsilon of zero — scaled by the largest difference, or `1` — is
    /// zero. A point a rounding error off a line is on it, as it is to
    /// Boost.
    #[must_use]
    fn side_by_triangle(p1: (Self, Self), p2: (Self, Self), p: (Self, Self)) -> Ordering {
        Self::cross_sign(p1, p2, p1, p).unwrap_or(Ordering::Less)
    }

    /// The side of `p` relative to the directed line `p1 → p2`, exactly:
    /// `Greater` to the left, `Less` to the right (or no side, as Boost's
    /// `-1`), `Equal` on the line.
    ///
    /// C++: `strategy::side::side_robust<void, fp_equals_policy>`
    /// (`strategy/cartesian/side_robust.hpp`), the side test of
    /// `convex_hull` and `is_convex`. A float is evaluated by the adaptive
    /// [`crate::precise_math::orient2d`] in `f64`, as Boost evaluates it
    /// in `double`; an integer exactly, in a wider integer, where Boost
    /// converts it to `double` first and so agrees up to `2^53`.
    #[must_use]
    fn side_robust(p1: (Self, Self), p2: (Self, Self), p: (Self, Self)) -> Ordering {
        Self::cross_sign(p1, p2, p1, p).unwrap_or(Ordering::Less)
    }

    /// Whether rounding leaves the directions `p1 → p2` and `q1 → q2`
    /// indistinguishable from parallel, so that two segments whose side
    /// tests say they cross are read as collinear instead.
    ///
    /// C++: the Cramer's-rule denominator test of
    /// `strategy::intersection::cartesian_segments`
    /// (`strategies/cartesian/intersection.hpp`): `dx_a·dy_b − dy_a·dx_b`
    /// equal to zero by `math::equals`, its epsilon scaled by the largest
    /// of the four differences, or `1`. A float evaluates it in its own
    /// precision, as Boost does. An exact scalar has no rounding to hide a
    /// parallel pair behind: segments its exact side tests let through as
    /// crossing are not parallel, so it answers `false`.
    #[must_use]
    fn nearly_parallel(
        p1: (Self, Self),
        p2: (Self, Self),
        q1: (Self, Self),
        q2: (Self, Self),
    ) -> bool {
        let _ = (p1, p2, q1, q2);
        false
    }
}

macro_rules! impl_scalar_float {
    ($($t:ty),*) => { $(
        impl CoordinateScalar for $t {
            const ZERO: Self = 0.0;
            const ONE:  Self = 1.0;
            type Measure = Self;
            #[inline]
            fn to_measure(self) -> Self { self }
            #[inline]
            fn from_measure(measure: Self) -> Self { measure }
            #[inline]
            fn sqrt(self) -> Self { crate::math::sqrt(self) }
            #[inline]
            fn abs(self)  -> Self { crate::math::abs(self) }
            #[inline]
            fn is_finite(self) -> bool { <$t>::is_finite(self) }
            #[inline]
            fn highest() -> Self { <$t>::MAX }
            #[inline]
            fn lowest() -> Self { <$t>::MIN }
            #[inline]
            fn tolerant_eq(self, other: Self) -> bool {
                if self == other {
                    return true;
                }
                if !self.is_finite() || !other.is_finite() {
                    return false;
                }
                // C++: `greatest(abs(a), abs(b), T(1))`, the factor
                // `equals_default_policy` supplies.
                let factor = crate::math::abs(self)
                    .max(crate::math::abs(other))
                    .max(1.0);
                crate::math::abs(self - other) <= <$t>::EPSILON * factor
            }
            fn side_by_triangle(
                p1: (Self, Self),
                p2: (Self, Self),
                p: (Self, Self),
            ) -> Ordering {
                let equal = |a: (Self, Self), b: (Self, Self)| {
                    a.0.tolerant_eq(b.0) && a.1.tolerant_eq(b.1)
                };
                if equal(p1, p2) || equal(p1, p) || equal(p2, p) {
                    return Ordering::Equal;
                }
                // C++: `compare::cartesian<compare::less, compare::equals_epsilon>`
                // picks the rotation, so the determinant is the same for
                // every rotation of one triple.
                let less = |a: (Self, Self), b: (Self, Self)| {
                    if !a.0.tolerant_eq(b.0) {
                        a.0 < b.0
                    } else if !a.1.tolerant_eq(b.1) {
                        a.1 < b.1
                    } else {
                        false
                    }
                };
                let (a, b, c) = if less(p, p1) {
                    if less(p, p2) { (p, p1, p2) } else { (p2, p, p1) }
                } else if less(p1, p2) {
                    (p1, p2, p)
                } else {
                    (p2, p, p1)
                };
                // C++: `side_value` takes the differences in the coordinate
                // type and promotes them to `double`.
                let (dx, dy) = (f64::from(b.0 - a.0), f64::from(b.1 - a.1));
                let (dpx, dpy) = (f64::from(c.0 - a.0), f64::from(c.1 - a.1));
                let side = dx * dpy - dy * dpx;
                // C++: `equals_by_policy(side, 0, equals_factor_policy(dx, dy,
                // dpx, dpy))`.
                let factor = crate::math::abs(dx)
                    .max(crate::math::abs(dy))
                    .max(crate::math::abs(dpx))
                    .max(crate::math::abs(dpy))
                    .max(1.0);
                if side == 0.0
                    || (side.is_finite() && crate::math::abs(side) <= f64::EPSILON * factor)
                {
                    Ordering::Equal
                } else if side > 0.0 {
                    Ordering::Greater
                } else {
                    Ordering::Less
                }
            }
            fn direction_code(a: (Self, Self), b: (Self, Self), p: (Self, Self)) -> Ordering {
                // C++: `make_perpendicular_line(a, b, b)`, in the coordinate
                // type, and `arithmetic::is_degenerate`.
                let line_a = b.0 - a.0;
                let line_b = -(a.1 - b.1);
                if line_a.tolerant_eq(0.0) && line_b.tolerant_eq(0.0) {
                    return Ordering::Equal;
                }
                let line_c = -line_a * b.0 - line_b * b.1;
                // C++: `arithmetic::side_value`.
                let side = line_a * p.0 + line_b * p.1 + line_c;
                if side == 0.0 {
                    Ordering::Equal
                } else if side > 0.0 {
                    Ordering::Greater
                } else {
                    Ordering::Less
                }
            }
            fn side_robust(p1: (Self, Self), p2: (Self, Self), p: (Self, Self)) -> Ordering {
                let point = |q: (Self, Self)| [f64::from(q.0), f64::from(q.1)];
                let side = crate::precise_math::orient2d(point(p1), point(p2), point(p));
                if side == 0.0 {
                    Ordering::Equal
                } else if side > 0.0 {
                    Ordering::Greater
                } else {
                    Ordering::Less
                }
            }
            fn nearly_parallel(
                p1: (Self, Self),
                p2: (Self, Self),
                q1: (Self, Self),
                q2: (Self, Self),
            ) -> bool {
                let (dx_a, dy_a) = (p2.0 - p1.0, p2.1 - p1.1);
                let (dx_b, dy_b) = (q2.0 - q1.0, q2.1 - q1.1);
                let denominator = dx_a * dy_b - dy_a * dx_b;
                // C++: `equals_by_policy(denominator, 0,
                // equals_factor_policy(dx_a, dy_a, dx_b, dy_b))`.
                let factor = crate::math::abs(dx_a)
                    .max(crate::math::abs(dy_a))
                    .max(crate::math::abs(dx_b))
                    .max(crate::math::abs(dy_b))
                    .max(1.0);
                denominator == 0.0
                    || (denominator.is_finite()
                        && crate::math::abs(denominator) <= <$t>::EPSILON * factor)
            }
        }
    )* };
}
impl_scalar_float!(f32, f64);

// Integer support: lets callers feed integer-coordinate geometries
// (think `model::Point<i32, 2, Cartesian>`) into the kernel. A measure is
// computed in `f64` (`Measure`), and a side or turn test exactly in `i128`
// (`cross_sign`, `dot_sign`), so neither overflows nor truncates. Calling
// `sqrt` here therefore signals that a kernel bypassed `Measure`, and the
// panic that `unreachable!` produces is the diagnostic we want.
macro_rules! impl_scalar_int {
    ($($t:ty),*) => { $(
        impl CoordinateScalar for $t {
            const ZERO: Self = 0;
            const ONE:  Self = 1;
            type Measure = f64;
            #[inline]
            fn highest() -> Self { <$t>::MAX }
            #[inline]
            fn lowest() -> Self { <$t>::MIN }
            #[inline]
            #[allow(
                clippy::cast_precision_loss,
                clippy::cast_lossless,
                reason = "Boost converts an integer coordinate to `double` the same way; one cast serves `i64`, which has no lossless conversion"
            )]
            fn to_measure(self) -> f64 { self as f64 }
            #[inline]
            #[allow(
                clippy::cast_possible_truncation,
                reason = "truncation toward zero is the conversion Boost's `numeric_cast` makes"
            )]
            fn from_measure(measure: f64) -> Self { measure as $t }
            #[inline]
            fn sqrt(self) -> Self {
                unreachable!(
                    "integer sqrt called on `{}`: a measure must be computed in `Measure` first",
                    core::stringify!($t),
                )
            }
            #[inline]
            fn abs(self) -> Self { <$t>::abs(self) }
            #[inline]
            fn tolerant_eq(self, other: Self) -> bool { self == other }
            #[inline]
            fn cross_sign(
                u_from: (Self, Self),
                u_to: (Self, Self),
                v_from: (Self, Self),
                v_to: (Self, Self),
            ) -> Option<Ordering> {
                let u = WideDifference::between(u_from, u_to);
                let v = WideDifference::between(v_from, v_to);
                Some(u.x.product(v.y).cmp(&u.y.product(v.x)))
            }
            #[inline]
            fn dot_sign(
                u_from: (Self, Self),
                u_to: (Self, Self),
                v_from: (Self, Self),
                v_to: (Self, Self),
            ) -> Option<Ordering> {
                let u = WideDifference::between(u_from, u_to);
                let v = WideDifference::between(v_from, v_to);
                Some(u.x.product(v.x).cmp(&u.y.product(v.y).negated()))
            }
        }
    )* };
}
impl_scalar_int!(i32, i64);

/// The difference of two integer coordinate pairs, held in `i128` where it
/// cannot overflow: two `i64` coordinates differ by less than `2^64`.
struct WideDifference {
    x: WideOrdinate,
    y: WideOrdinate,
}

impl WideDifference {
    fn between<T: Into<i128>>(from: (T, T), to: (T, T)) -> Self {
        Self {
            x: WideOrdinate(to.0.into() - from.0.into()),
            y: WideOrdinate(to.1.into() - from.1.into()),
        }
    }
}

/// One ordinate of a [`WideDifference`], under `2^64` in magnitude.
#[derive(Clone, Copy)]
struct WideOrdinate(i128);

impl WideOrdinate {
    /// The exact product of two ordinates. Each magnitude is under `2^64`,
    /// so the product's is under `2^128` and fits a `u128`; the sign is
    /// kept apart.
    fn product(self, other: Self) -> SignedProduct {
        SignedProduct {
            negative: (self.0 < 0) != (other.0 < 0),
            magnitude: self.0.unsigned_abs() * other.0.unsigned_abs(),
        }
    }
}

/// An exact product of two [`WideOrdinate`]s: a sign and a `u128`
/// magnitude.
#[derive(Clone, Copy)]
struct SignedProduct {
    negative: bool,
    magnitude: u128,
}

impl SignedProduct {
    fn negated(self) -> Self {
        Self {
            negative: !self.negative,
            ..self
        }
    }
}

impl PartialEq for SignedProduct {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for SignedProduct {}

impl PartialOrd for SignedProduct {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for SignedProduct {
    fn cmp(&self, other: &Self) -> Ordering {
        // Zero carries no sign, whichever its factors had.
        let below_zero = |product: &Self| product.negative && product.magnitude != 0;
        match (below_zero(self), below_zero(other)) {
            (false, false) => self.magnitude.cmp(&other.magnitude),
            (true, true) => other.magnitude.cmp(&self.magnitude),
            (false, true) => Ordering::Greater,
            (true, false) => Ordering::Less,
        }
    }
}
