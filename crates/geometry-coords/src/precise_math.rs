//! Adaptive-precision floating-point expansion arithmetic.
//!
//! Mirrors `boost::geometry::detail::precise_math` from
//! `util/precise_math.hpp:35-635`. The algorithms implement the error-free
//! transforms and exact-sign 2D orientation/in-circle determinants described
//! by Shewchuk, using fixed stack storage so the module remains `no_std` and
//! allocation-free.

const EXPANSION_CAPACITY: usize = 256;

/// Add `a + b`, returning `[rounded_sum, exact_roundoff]`.
///
/// Mirrors `precise_math::fast_two_sum` from
/// `util/precise_math.hpp:39-51` (Theorem 6). Correctness requires
/// `|a| >= |b|`.
#[inline]
#[must_use]
pub fn fast_two_sum(a: f64, b: f64) -> [f64; 2] {
    let x = a + b;
    let b_virtual = x - a;
    [x, b - b_virtual]
}

/// Add `a + b`, returning `[rounded_sum, exact_roundoff]` without a
/// magnitude precondition.
///
/// Mirrors `precise_math::two_sum` from
/// `util/precise_math.hpp:53-69` (Theorem 7).
#[inline]
#[must_use]
pub fn two_sum(a: f64, b: f64) -> [f64; 2] {
    let x = a + b;
    let b_virtual = x - a;
    let a_virtual = x - b_virtual;
    let b_roundoff = b - b_virtual;
    let a_roundoff = a - a_virtual;
    [x, a_roundoff + b_roundoff]
}

/// Return the exact roundoff tail of the already-rounded `x = a - b`.
///
/// Mirrors `precise_math::two_diff_tail` from
/// `util/precise_math.hpp:71-84`.
#[inline]
#[must_use]
pub fn two_diff_tail(a: f64, b: f64, x: f64) -> f64 {
    let b_virtual = a - x;
    let a_virtual = x + b_virtual;
    let b_roundoff = b_virtual - b;
    let a_roundoff = a - a_virtual;
    a_roundoff + b_roundoff
}

/// Subtract `a - b`, returning `[rounded_difference, exact_roundoff]`.
///
/// Mirrors `precise_math::two_diff` from
/// `util/precise_math.hpp:86-98`.
#[inline]
#[must_use]
pub fn two_diff(a: f64, b: f64) -> [f64; 2] {
    let x = a - b;
    [x, two_diff_tail(a, b, x)]
}

/// Return the exact roundoff tail of the already-rounded `x = a * b`.
///
/// Mirrors `precise_math::two_product_tail` from
/// `util/precise_math.hpp:100-111` (Theorem 18). IEEE-754 fused
/// multiply-add supplies the single-rounding operation used by Boost's
/// `std::fma` implementation.
#[inline]
#[must_use]
pub fn two_product_tail(a: f64, b: f64, x: f64) -> f64 {
    a.mul_add(b, -x)
}

/// Multiply `a * b`, returning `[rounded_product, exact_roundoff]`.
///
/// Mirrors `precise_math::two_product` from
/// `util/precise_math.hpp:113-126`.
#[inline]
#[must_use]
pub fn two_product(a: f64, b: f64) -> [f64; 2] {
    let x = a * b;
    [x, two_product_tail(a, b, x)]
}

/// Subtract two two-component expansions.
///
/// Inputs use Boost's `[large, small]` ordering; the four output components
/// are ascending in magnitude. Mirrors `two_two_expansion_diff` from
/// `util/precise_math.hpp:128-151`.
#[inline]
#[must_use]
pub fn two_two_expansion_diff(a: [f64; 2], b: [f64; 2]) -> [f64; 4] {
    let mut h = [0.0; 4];
    let mut qh = two_diff(a[1], b[1]);
    h[0] = qh[1];
    qh = two_sum(a[0], qh[0]);
    let j = qh[0];
    qh = two_diff(qh[1], b[0]);
    h[1] = qh[1];
    qh = two_sum(j, qh[0]);
    h[2] = qh[1];
    h[3] = qh[0];
    h
}

/// Merge two non-overlapping, ascending-magnitude expansions, eliminating
/// zero components.
///
/// Returns the initialized prefix length of `output`. Mirrors
/// `fast_expansion_sum_zeroelim` from
/// `util/precise_math.hpp:153-230` (Theorem 13).
///
/// # Panics
///
/// Panics if `output` cannot hold `left.len() + right.len()` components.
pub fn fast_expansion_sum_zeroelim(left: &[f64], right: &[f64], output: &mut [f64]) -> usize {
    assert!(output.len() >= left.len() + right.len());
    if left.is_empty() {
        return copy_nonzero(right, output);
    }
    if right.is_empty() {
        return copy_nonzero(left, output);
    }

    let mut left_index = 0;
    let mut right_index = 0;
    let mut output_index = 0;
    let mut q = if right[0].abs() > left[0].abs() {
        left_index += 1;
        left[0]
    } else {
        right_index += 1;
        right[0]
    };

    if left_index < left.len() && right_index < right.len() {
        let pair = if right[right_index].abs() > left[left_index].abs() {
            let pair = fast_two_sum(left[left_index], q);
            left_index += 1;
            pair
        } else {
            let pair = fast_two_sum(right[right_index], q);
            right_index += 1;
            pair
        };
        q = pair[0];
        if pair[1] != 0.0 {
            output[output_index] = pair[1];
            output_index += 1;
        }

        while left_index < left.len() && right_index < right.len() {
            let pair = if right[right_index].abs() > left[left_index].abs() {
                let pair = two_sum(q, left[left_index]);
                left_index += 1;
                pair
            } else {
                let pair = two_sum(q, right[right_index]);
                right_index += 1;
                pair
            };
            q = pair[0];
            if pair[1] != 0.0 {
                output[output_index] = pair[1];
                output_index += 1;
            }
        }
    }

    while left_index < left.len() {
        let pair = two_sum(q, left[left_index]);
        left_index += 1;
        q = pair[0];
        if pair[1] != 0.0 {
            output[output_index] = pair[1];
            output_index += 1;
        }
    }
    while right_index < right.len() {
        let pair = two_sum(q, right[right_index]);
        right_index += 1;
        q = pair[0];
        if pair[1] != 0.0 {
            output[output_index] = pair[1];
            output_index += 1;
        }
    }
    if q != 0.0 || output_index == 0 {
        output[output_index] = q;
        output_index += 1;
    }
    output_index
}

/// Multiply an ascending-magnitude expansion by one scalar, eliminating zero
/// components.
///
/// Returns the initialized prefix length of `output`. Mirrors
/// `scale_expansion_zeroelim` from
/// `util/precise_math.hpp:232-278` (Theorem 19).
///
/// # Panics
///
/// Panics if `output` has fewer than `2 * expansion.len()` slots or if the
/// input expansion is empty.
pub fn scale_expansion_zeroelim(expansion: &[f64], scalar: f64, output: &mut [f64]) -> usize {
    assert!(!expansion.is_empty());
    assert!(output.len() >= 2 * expansion.len());
    let mut qh = two_product(expansion[0], scalar);
    let mut output_index = 0;
    if qh[1] != 0.0 {
        output[output_index] = qh[1];
        output_index += 1;
    }
    for &component in &expansion[1..] {
        let product = two_product(component, scalar);
        qh = two_sum(qh[0], product[1]);
        if qh[1] != 0.0 {
            output[output_index] = qh[1];
            output_index += 1;
        }
        qh = fast_two_sum(product[0], qh[0]);
        if qh[1] != 0.0 {
            output[output_index] = qh[1];
            output_index += 1;
        }
    }
    if qh[0] != 0.0 || output_index == 0 {
        output[output_index] = qh[0];
        output_index += 1;
    }
    output_index
}

fn copy_nonzero(input: &[f64], output: &mut [f64]) -> usize {
    let mut length = 0;
    for &component in input {
        if component != 0.0 {
            output[length] = component;
            length += 1;
        }
    }
    if length == 0 {
        output[0] = 0.0;
        1
    } else {
        length
    }
}

#[derive(Clone, Copy)]
struct Expansion {
    terms: [f64; EXPANSION_CAPACITY],
    length: usize,
}

impl Expansion {
    fn zero() -> Self {
        let mut result = Self {
            terms: [0.0; EXPANSION_CAPACITY],
            length: 1,
        };
        result.terms[0] = 0.0;
        result
    }

    fn difference(left: f64, right: f64) -> Self {
        let [head, tail] = two_diff(left, right);
        let mut result = Self::zero();
        result.length = 0;
        if tail != 0.0 {
            result.terms[result.length] = tail;
            result.length += 1;
        }
        if head != 0.0 || result.length == 0 {
            result.terms[result.length] = head;
            result.length += 1;
        }
        result
    }

    fn negate(mut self) -> Self {
        for term in &mut self.terms[..self.length] {
            *term = -*term;
        }
        self
    }

    fn add(self, other: Self) -> Self {
        let mut result = Self::zero();
        result.length = fast_expansion_sum_zeroelim(
            &self.terms[..self.length],
            &other.terms[..other.length],
            &mut result.terms,
        );
        result
    }

    fn scale(self, scalar: f64) -> Self {
        let mut result = Self::zero();
        result.length =
            scale_expansion_zeroelim(&self.terms[..self.length], scalar, &mut result.terms);
        result
    }

    fn multiply(self, other: Self) -> Self {
        let mut result = Self::zero();
        for &component in &other.terms[..other.length] {
            result = result.add(self.scale(component));
        }
        result
    }

    fn square(self) -> Self {
        self.multiply(self)
    }

    fn most_significant(self) -> f64 {
        self.terms[self.length - 1]
    }
}

/// Adaptive, exact-sign orientation determinant for three 2D points.
///
/// Positive means counter-clockwise/left, negative means clockwise/right,
/// and zero means collinear. Mirrors `precise_math::orient2d` and
/// `orient2dtail` from `util/precise_math.hpp:287-384`. The common path uses
/// Boost's error bound; uncertain cases are completed with the same
/// error-free expansion primitives.
#[inline]
#[must_use]
pub fn orient2d(p1: [f64; 2], p2: [f64; 2], p3: [f64; 2]) -> f64 {
    let t1 = p1[0] - p3[0];
    let t2 = p2[1] - p3[1];
    let t3 = p1[1] - p3[1];
    let t4 = p2[0] - p3[0];
    let diagonal = t1 * t2;
    let antidiagonal = t3 * t4;
    let determinant = diagonal - antidiagonal;
    let magnitude = diagonal.abs() + antidiagonal.abs();
    let bound = (1.5 + 4.0 * f64::EPSILON) * f64::EPSILON * magnitude;
    if determinant.abs() >= bound
        || (diagonal > 0.0 && antidiagonal <= 0.0)
        || (diagonal < 0.0 && antidiagonal >= 0.0)
    {
        return determinant;
    }

    let acx = Expansion::difference(p1[0], p3[0]);
    let bcy = Expansion::difference(p2[1], p3[1]);
    let acy = Expansion::difference(p1[1], p3[1]);
    let bcx = Expansion::difference(p2[0], p3[0]);
    acx.multiply(bcy)
        .add(acy.multiply(bcx).negate())
        .most_significant()
}

/// Adaptive, exact-sign in-circle determinant for four 2D points.
///
/// For counter-clockwise `p1,p2,p3`, positive means `p4` is inside the
/// circumcircle, negative means outside, and zero means cocircular. Mirrors
/// `precise_math::incircle` from `util/precise_math.hpp:386-632`; uncertain
/// cases evaluate the translated determinant with exact expansions.
#[inline]
#[must_use]
pub fn incircle(p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], p4: [f64; 2]) -> f64 {
    let a11 = p1[0] - p4[0];
    let a21 = p2[0] - p4[0];
    let a31 = p3[0] - p4[0];
    let a12 = p1[1] - p4[1];
    let a22 = p2[1] - p4[1];
    let a32 = p3[1] - p4[1];
    let a21_a32 = a21 * a32;
    let a31_a22 = a31 * a22;
    let a31_a12 = a31 * a12;
    let a11_a32 = a11 * a32;
    let a11_a22 = a11 * a22;
    let a21_a12 = a21 * a12;
    let a13 = a11 * a11 + a12 * a12;
    let a23 = a21 * a21 + a22 * a22;
    let a33 = a31 * a31 + a32 * a32;
    let determinant =
        a13 * (a21_a32 - a31_a22) + a23 * (a31_a12 - a11_a32) + a33 * (a11_a22 - a21_a12);
    let magnitude = (a21_a32.abs() + a31_a22.abs()) * a13
        + (a31_a12.abs() + a11_a32.abs()) * a23
        + (a11_a22.abs() + a21_a12.abs()) * a33;
    let bound = (5.0 + 24.0 * f64::EPSILON) * f64::EPSILON * magnitude;
    if determinant.abs() > bound {
        return determinant;
    }

    let adx = Expansion::difference(p1[0], p4[0]);
    let bdx = Expansion::difference(p2[0], p4[0]);
    let cdx = Expansion::difference(p3[0], p4[0]);
    let ady = Expansion::difference(p1[1], p4[1]);
    let bdy = Expansion::difference(p2[1], p4[1]);
    let cdy = Expansion::difference(p3[1], p4[1]);
    let alift = adx.square().add(ady.square());
    let blift = bdx.square().add(bdy.square());
    let clift = cdx.square().add(cdy.square());
    let bc = bdx.multiply(cdy).add(cdx.multiply(bdy).negate());
    let ca = cdx.multiply(ady).add(adx.multiply(cdy).negate());
    let ab = adx.multiply(bdy).add(bdx.multiply(ady).negate());
    alift
        .multiply(bc)
        .add(blift.multiply(ca))
        .add(clift.multiply(ab))
        .most_significant()
}
