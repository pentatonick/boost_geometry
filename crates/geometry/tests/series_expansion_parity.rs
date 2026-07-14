//! Public-facade tests for eighth-order geodesic series coefficients.

#![allow(
    clippy::cast_precision_loss,
    clippy::similar_names,
    reason = "small Fourier indices and C1/C1p names mirror the cited Boost series"
)]

use boost_geometry::coords::series_expansion::{
    coefficients_a3, coefficients_c1, coefficients_c1p, coefficients_c2, coefficients_c3,
    evaluate_a1, evaluate_a2, sin_cos_series,
};

/// `test/formulas/direct.cpp:99-106` exercises the coefficient families
/// through the Karney direct formula. This witness checks the generated
/// eighth-order values against the expressions in `util/series_expansion.hpp`.
#[test]
fn eighth_order_coefficient_families_match_generated_expressions() {
    let epsilon = 0.01;
    let epsilon2 = epsilon * epsilon;
    let c1 = coefficients_c1(epsilon);
    let c1p = coefficients_c1p(epsilon);
    let c2 = coefficients_c2(epsilon);
    assert!(!c1.is_empty());

    let expected_c1_1 =
        epsilon * (epsilon2 * (epsilon2 * (19.0 * epsilon2 - 64.0) + 384.0) - 1024.0) / 2048.0;
    let expected_c1p_1 = epsilon
        * (epsilon2 * ((9840.0 - 4879.0 * epsilon2) * epsilon2 - 20_736.0) + 36_864.0)
        / 73_728.0;
    let expected_c2_1 =
        epsilon * (epsilon2 * (epsilon2 * (41.0 * epsilon2 + 64.0) + 128.0) + 1024.0) / 2048.0;
    assert!((c1[1] - expected_c1_1).abs() < 1e-18);
    assert!((c1p[1] - expected_c1p_1).abs() < 1e-18);
    assert!((c2[1] - expected_c2_1).abs() < 1e-18);

    assert!(evaluate_a1(epsilon) > 0.0);
    assert!(evaluate_a2(epsilon) < 0.0);
    assert_eq!(coefficients_a3(1.0 / 300.0).len(), 8);
    assert_eq!(coefficients_c3(1.0 / 300.0, epsilon).len(), 8);
}

/// `test/formulas/direct.cpp:104-106` uses `sin_cos_series` inside Karney's
/// longitude correction. Clenshaw evaluation must equal the literal Fourier
/// sum represented by the same public coefficient sequence.
#[test]
fn clenshaw_evaluation_matches_literal_fourier_sum() {
    let coefficients = coefficients_c1(0.02);
    let angle: f64 = 0.37;
    let direct: f64 = coefficients
        .as_slice()
        .iter()
        .enumerate()
        .skip(1)
        .map(|(index, coefficient)| coefficient * (2.0 * index as f64 * angle).sin())
        .sum();
    let clenshaw = sin_cos_series(angle.sin(), angle.cos(), &coefficients);
    assert!((clenshaw - direct).abs() < 1e-16);
}
