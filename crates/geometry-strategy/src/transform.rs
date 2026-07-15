//! `TransformStrategy<P>` and affine implementations.
//!
//! The trait mirrors the per-CS transform-strategy concept from
//! `boost/geometry/strategies/transform/services.hpp`; the affine
//! matrices mirror
//! `boost::geometry::strategy::transform::matrix_transformer` and its
//! `translate_/rotate_/scale_transformer` siblings from
//! `boost/geometry/strategies/transform/matrix_transformers.hpp`.
//!
//! Boost's `apply(src, dst)` mutates the destination through an
//! out-parameter; the Rust port returns the destination by value,
//! which makes the output type explicit and lines up with the KC1
//! `PointMut` bound used to build it.

use core::marker::PhantomData;

use geometry_coords::CoordinateScalar;
use geometry_cs::{Cartesian, CartesianFamily, CoordinateSystem};
use geometry_model::{Point2D, Point3D};
use geometry_tag::SameAs;
use geometry_trait::{Point, PointMut};

/// A function from one [`Point`] to another, used by
/// [`transform`](../../geometry_algorithm/fn.transform.html).
///
/// The output may have a different scalar type or coordinate system
/// than the input. [`Self::Output`] must be
/// `PointMut + Default` because the strategy builds it via
/// `Default + set::<D>`.
pub trait TransformStrategy<P: Point> {
    /// The destination point type.
    type Output: PointMut + Default;

    /// Apply the transform to `src` and return a fresh destination
    /// point.
    ///
    /// Mirrors Boost's `apply(src, dst)`, returning `dst` by value.
    fn transform(&self, src: &P) -> Self::Output;
}

/// Named constructor for 2D translation strategies.
#[derive(Debug, Default, Clone, Copy)]
pub struct Translate;

impl Translate {
    /// Translate by `(x, y)`.
    #[must_use]
    pub fn by<T: CoordinateScalar>(x: T, y: T) -> Affine2<T> {
        Affine2::translation(x, y)
    }
}

/// Named constructor for 2D scale strategies.
#[derive(Debug, Default, Clone, Copy)]
pub struct Scale;

impl Scale {
    /// Scale the two axes independently.
    #[must_use]
    pub fn by<T: CoordinateScalar>(x: T, y: T) -> Affine2<T> {
        Affine2::scale(x, y)
    }

    /// Scale both axes by the same factor.
    #[must_use]
    pub fn uniform<T: CoordinateScalar>(factor: T) -> Affine2<T> {
        Affine2::scale(factor, factor)
    }
}

/// Named constructor for 2D rotation strategies.
#[derive(Debug, Default, Clone, Copy)]
pub struct Rotate;

impl Rotate {
    /// Rotate counter-clockwise by radians around the coordinate origin.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn radians(angle: f64) -> Affine2<f64> {
        Affine2::rotation(angle)
    }

    /// Rotate counter-clockwise by degrees around the coordinate origin.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn degrees(angle: f64) -> Affine2<f64> {
        Self::radians(angle.to_radians())
    }

    /// Rotate counter-clockwise by radians around `origin`.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn around<P>(angle: f64, origin: &P) -> Affine2<f64>
    where
        P: Point<Scalar = f64>,
    {
        let x = origin.get::<0>();
        let y = origin.get::<1>();
        Affine2::translation(x, y)
            .then(Affine2::rotation(angle))
            .then(Affine2::translation(-x, -y))
    }
}

/// Named constructor for 2D shear/skew strategies.
#[derive(Debug, Default, Clone, Copy)]
pub struct Skew;

impl Skew {
    /// Apply direct x- and y-axis shear factors.
    ///
    /// The resulting mapping is `x' = x + x_shear*y` and
    /// `y' = y_shear*x + y`.
    #[must_use]
    pub fn by(x_shear: f64, y_shear: f64) -> Affine2<f64> {
        Affine2 {
            m: [1.0, x_shear, 0.0, y_shear, 1.0, 0.0, 0.0, 0.0, 1.0],
            _t: PhantomData,
        }
    }

    /// Apply shears specified as angles in radians.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn radians(x_angle: f64, y_angle: f64) -> Affine2<f64> {
        Self::by(x_angle.tan(), y_angle.tan())
    }

    /// Apply shears specified as angles in degrees.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn degrees(x_angle: f64, y_angle: f64) -> Affine2<f64> {
        Self::radians(x_angle.to_radians(), y_angle.to_radians())
    }
}

/// 3×3 affine matrix in homogeneous 2D coordinates.
///
/// Mirrors `boost::geometry::strategy::transform::matrix_transformer<T, 2, 2>`
/// from `matrix_transformers.hpp`.
#[derive(Debug, Clone, Copy)]
pub struct Affine2<T: CoordinateScalar> {
    /// Row-major: `[a, b, tx, c, d, ty, 0, 0, 1]`.
    pub m: [T; 9],
    _t: PhantomData<T>,
}

impl<T: CoordinateScalar> Affine2<T> {
    /// The identity transform.
    #[must_use]
    pub fn identity() -> Self {
        Self {
            m: [
                T::ONE,
                T::ZERO,
                T::ZERO,
                T::ZERO,
                T::ONE,
                T::ZERO,
                T::ZERO,
                T::ZERO,
                T::ONE,
            ],
            _t: PhantomData,
        }
    }

    /// Translation by `(tx, ty)`.
    #[must_use]
    pub fn translation(tx: T, ty: T) -> Self {
        let mut m = Self::identity();
        m.m[2] = tx;
        m.m[5] = ty;
        m
    }

    /// Non-uniform scale by `(sx, sy)`.
    #[must_use]
    pub fn scale(sx: T, sy: T) -> Self {
        let mut m = Self::identity();
        m.m[0] = sx;
        m.m[4] = sy;
        m
    }

    /// `self ∘ other` — apply `other` first, then `self`
    /// (standard matrix composition `self * other`).
    #[must_use]
    pub fn then(self, other: Self) -> Self {
        // The point is transformed by `self` after `other`, i.e.
        // `result = self * other` as 3×3 matrices.
        let a = &self.m;
        let b = &other.m;
        let mut out = [T::ZERO; 9];
        for (r, row) in out.chunks_mut(3).enumerate() {
            for (c, cell) in row.iter_mut().enumerate() {
                *cell = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
            }
        }
        Self {
            m: out,
            _t: PhantomData,
        }
    }
}

impl Affine2<f64> {
    /// Rotation by `angle_rad` radians counter-clockwise. Only
    /// available for `f64` (needs `sin`/`cos`, which the scalar trait
    /// does not yet expose generically).
    ///
    /// `std`-gated: `f64::sin_cos` is not shimmed by `geometry-coords`
    /// under `libm` (only `sqrt`/`abs` are), so this constructor is
    /// unavailable in a `no_std` build.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn rotation(angle_rad: f64) -> Self {
        let (s, c) = angle_rad.sin_cos();
        let mut m = Self::identity();
        m.m[0] = c;
        m.m[1] = -s;
        m.m[3] = s;
        m.m[4] = c;
        m
    }
}

impl<T, P> TransformStrategy<P> for Affine2<T>
where
    T: CoordinateScalar,
    P: Point<Scalar = T>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = Point2D<T, Cartesian>;

    #[inline]
    fn transform(&self, src: &P) -> Self::Output {
        let x = src.get::<0>();
        let y = src.get::<1>();
        let nx = self.m[0] * x + self.m[1] * y + self.m[2];
        let ny = self.m[3] * x + self.m[4] * y + self.m[5];
        Point2D::new(nx, ny)
    }
}

/// 4×4 affine matrix in homogeneous 3D coordinates. Same shape as
/// [`Affine2`], one dimension up.
#[derive(Debug, Clone, Copy)]
pub struct Affine3<T: CoordinateScalar> {
    /// Row-major 4×4.
    pub m: [T; 16],
    _t: PhantomData<T>,
}

impl<T: CoordinateScalar> Affine3<T> {
    /// The identity transform.
    #[must_use]
    pub fn identity() -> Self {
        let mut m = [T::ZERO; 16];
        m[0] = T::ONE;
        m[5] = T::ONE;
        m[10] = T::ONE;
        m[15] = T::ONE;
        Self { m, _t: PhantomData }
    }

    /// Translation by `(tx, ty, tz)`.
    #[must_use]
    pub fn translation(tx: T, ty: T, tz: T) -> Self {
        let mut m = Self::identity();
        m.m[3] = tx;
        m.m[7] = ty;
        m.m[11] = tz;
        m
    }

    /// Non-uniform scale by `(sx, sy, sz)`.
    #[must_use]
    pub fn scale(sx: T, sy: T, sz: T) -> Self {
        let mut m = Self::identity();
        m.m[0] = sx;
        m.m[5] = sy;
        m.m[10] = sz;
        m
    }
}

impl<T, P> TransformStrategy<P> for Affine3<T>
where
    T: CoordinateScalar,
    P: Point<Scalar = T>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Output = Point3D<T, Cartesian>;

    #[inline]
    fn transform(&self, src: &P) -> Self::Output {
        let x = src.get::<0>();
        let y = src.get::<1>();
        let z = src.get::<2>();
        let m = &self.m;
        let nx = m[0] * x + m[1] * y + m[2] * z + m[3];
        let ny = m[4] * x + m[5] * y + m[6] * z + m[7];
        let nz = m[8] * x + m[9] * y + m[10] * z + m[11];
        let mut out = Point3D::<T, Cartesian>::default();
        out.set::<0>(nx);
        out.set::<1>(ny);
        out.set::<2>(nz);
        out
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Affine outputs of integer inputs are exact."
)]
mod tests {
    //! Reference behaviour from
    //! `boost/geometry/test/algorithms/transform.cpp`: translation and
    //! scale map a point to the expected coordinates.

    use super::{Affine2, Rotate, Scale, Skew, TransformStrategy, Translate};
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;
    use geometry_trait::Point as _;

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn translation_shifts_point() {
        let t = Affine2::translation(3.0, 4.0);
        let out = t.transform(&P::new(1.0, 1.0));
        assert_eq!(out.get::<0>(), 4.0);
        assert_eq!(out.get::<1>(), 5.0);
    }

    #[test]
    fn scale_multiplies_point() {
        let s = Affine2::scale(2.0, 3.0);
        let out = s.transform(&P::new(1.0, 1.0));
        assert_eq!(out.get::<0>(), 2.0);
        assert_eq!(out.get::<1>(), 3.0);
    }

    #[test]
    fn rotation_by_half_pi_maps_x_to_y() {
        let r = Affine2::rotation(core::f64::consts::FRAC_PI_2);
        let out = r.transform(&P::new(1.0, 0.0));
        assert!((out.get::<0>() - 0.0).abs() < 1e-12);
        assert!((out.get::<1>() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn then_composes_translate_after_scale() {
        // Apply scale first, then translate: (1,1) → scale×2 → (2,2)
        // → translate(1,1) → (3,3).
        let combined = Affine2::translation(1.0, 1.0).then(Affine2::scale(2.0, 2.0));
        let out = combined.transform(&P::new(1.0, 1.0));
        assert_eq!(out.get::<0>(), 3.0);
        assert_eq!(out.get::<1>(), 3.0);
    }

    #[test]
    fn named_constructors_build_affine_strategies() {
        assert_eq!(
            Translate::by(2.0, 3.0).transform(&P::new(1.0, 1.0)),
            P::new(3.0, 4.0)
        );
        assert_eq!(
            Scale::uniform(2.0).transform(&P::new(1.0, 3.0)),
            P::new(2.0, 6.0)
        );
        assert_eq!(
            Skew::by(2.0, 0.0).transform(&P::new(1.0, 3.0)),
            P::new(7.0, 3.0)
        );
        let rotated = Rotate::around(core::f64::consts::FRAC_PI_2, &P::new(1.0, 1.0))
            .transform(&P::new(2.0, 1.0));
        assert!((rotated.get::<0>() - 1.0).abs() < 1e-12);
        assert!((rotated.get::<1>() - 2.0).abs() < 1e-12);
    }
}
