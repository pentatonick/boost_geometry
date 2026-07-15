//! Public-facade parity tests for adaptive-precision arithmetic.

#![allow(
    clippy::float_cmp,
    reason = "error-free transform witnesses require the exact decomposed floating-point values"
)]

use boost_geometry::coords::precise_math::{
    fast_expansion_sum_zeroelim, incircle, orient2d, scale_expansion_zeroelim, two_product,
    two_sum, two_two_expansion_diff,
};

/// `util/precise_math.hpp:42-85` — error-free sums and products preserve the
/// roundoff component that ordinary floating-point arithmetic discards.
#[test]
fn error_free_transforms_preserve_roundoff() {
    let [sum, tail] = two_sum(1.0e16, 1.0);
    assert_eq!(sum, 1.0e16);
    assert_eq!(tail, 1.0);

    let [product, product_tail] = two_product(1.0e16 + 2.0, 1.0e-16);
    assert_eq!(product + product_tail, (1.0e16 + 2.0) * 1.0e-16);
}

/// `util/precise_math.hpp:220-384` — adaptive orientation recovers a sign
/// after the ordinary determinant underflows/cancels to zero.
#[test]
fn adaptive_orientation_recovers_tiny_offset() {
    let tiny = f64::MIN_POSITIVE;
    let from = [-1.0, -1.0];
    let to = [1.0, 1.0];
    assert!(orient2d(from, to, [-tiny, tiny]) > 0.0);
    assert!(orient2d(from, to, [tiny, -tiny]) < 0.0);
}

/// `util/precise_math.hpp:386-632` — the in-circle determinant has the C++
/// sign convention for inside, cocircular, and outside points.
#[test]
fn adaptive_incircle_uses_reference_sign_convention() {
    let a = [1.0, 0.0];
    let b = [0.0, 1.0];
    let c = [-1.0, 0.0];
    assert!(incircle(a, b, c, [0.0, 0.0]) > 0.0);
    assert_eq!(incircle(a, b, c, [0.0, -1.0]), 0.0);
    assert!(incircle(a, b, c, [2.0, 2.0]) < 0.0);
}

/// `util/precise_math.hpp:128-230` — the public expansion primitives retain
/// exact totals while eliminating zero components and handling empty inputs.
#[test]
fn expansion_difference_and_zero_eliminating_merge_cover_edge_inputs() {
    let expansion = two_two_expansion_diff([1.0e16, 1.0], [1.0e16, -1.0]);
    assert_eq!(expansion.iter().sum::<f64>(), 2.0);

    let mut output = [f64::NAN; 8];
    let length = fast_expansion_sum_zeroelim(&[], &[0.0, 1.0, 0.0], &mut output);
    assert_eq!(length, 1);
    assert_eq!(output[0], 1.0);

    let length = fast_expansion_sum_zeroelim(&[0.0, 0.0], &[], &mut output);
    assert_eq!(length, 1);
    assert_eq!(output[0], 0.0);

    let left = [1.0, 1.0e16];
    let right = [0.5];
    let length = fast_expansion_sum_zeroelim(&left, &right, &mut output);
    assert!(length >= 2);
    assert_eq!(output[..length].iter().sum::<f64>(), 1.0e16 + 2.0);

    let factor = 1.0e16 + 2.0;
    let length = scale_expansion_zeroelim(&[factor, -factor], 1.0e-16, &mut output);
    assert_eq!(length, 2);
    assert_ne!(output[0], 0.0);
    assert_eq!(output[0], -output[1]);
}
