//! Eighth-order coefficient tables for Karney geodesic series.
//!
//! Mirrors `boost::geometry::series_expansion` from
//! `util/series_expansion.hpp:35-758`. Boost makes the order a template
//! parameter from 0 through 8; the Rust port exposes the most accurate
//! generated order (8), which is the order used by the full-accuracy Karney
//! formulas, and avoids const-generic arithmetic in public array lengths.

use core::ops::Index;

/// A fixed-capacity sequence of geodesic-series coefficients.
///
/// Mirrors the `coeffs_C1`, `coeffs_C1p`, `coeffs_C2`, `coeffs_C3`, and
/// `coeffs_A3` containers at `util/series_expansion.hpp:702-756` for series
/// order 8.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeriesCoefficients {
    values: [f64; 9],
    length: usize,
}

impl SeriesCoefficients {
    fn with_length(length: usize) -> Self {
        Self {
            values: [0.0; 9],
            length,
        }
    }

    /// Return the initialized coefficient sequence.
    #[inline]
    #[must_use]
    pub fn as_slice(&self) -> &[f64] {
        &self.values[..self.length]
    }

    /// Return the coefficient count.
    #[inline]
    #[must_use]
    pub const fn len(&self) -> usize {
        self.length
    }

    /// Return whether the sequence is empty.
    #[inline]
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.length == 0
    }
}

impl Index<usize> for SeriesCoefficients {
    type Output = f64;

    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        &self.as_slice()[index]
    }
}

/// Evaluate the order-8 `A1 - 1` scale factor.
///
/// Mirrors `evaluate_A1<8>` from `util/series_expansion.hpp:65-96`.
#[inline]
#[must_use]
pub fn evaluate_a1(epsilon: f64) -> f64 {
    let epsilon2 = epsilon * epsilon;
    let t =
        epsilon2 * (epsilon2 * (epsilon2 * (25.0 * epsilon2 + 64.0) + 256.0) + 4096.0) / 16_384.0;
    (t + epsilon) / (1.0 - epsilon)
}

/// Evaluate the order-8 `A2 - 1` scale factor.
///
/// Mirrors `evaluate_A2<8>` from `util/series_expansion.hpp:111-142`.
#[inline]
#[must_use]
pub fn evaluate_a2(epsilon: f64) -> f64 {
    let epsilon2 = epsilon * epsilon;
    let t = epsilon2 * (epsilon2 * ((-375.0 * epsilon2 - 704.0) * epsilon2 - 1792.0) - 12_288.0)
        / 16_384.0;
    (t - epsilon) / (1.0 + epsilon)
}

/// Generate the order-8 `A3` polynomial coefficients.
///
/// Mirrors the default arm of `evaluate_coeffs_A3` from
/// `util/series_expansion.hpp:157-210`.
#[must_use]
pub fn coefficients_a3(n: f64) -> SeriesCoefficients {
    let mut c = SeriesCoefficients::with_length(8);
    c.values[0] = 1.0;
    c.values[1] = (n - 1.0) / 2.0;
    c.values[2] = (n * (3.0 * n - 1.0) - 2.0) / 8.0;
    c.values[3] = (n * (n * (5.0 * n - 1.0) - 3.0) - 1.0) / 16.0;
    c.values[4] = (n * ((-5.0 * n - 20.0) * n - 4.0) - 6.0) / 128.0;
    c.values[5] = ((-5.0 * n - 10.0) * n - 6.0) / 256.0;
    c.values[6] = (-15.0 * n - 20.0) / 1024.0;
    c.values[7] = -25.0 / 2048.0;
    c
}

/// Generate the order-8 Fourier coefficients `C1[1..=8]`.
///
/// Mirrors the default arm of `evaluate_coeffs_C1` from
/// `util/series_expansion.hpp:227-317`. Index zero is intentionally zero,
/// matching Boost's coefficient container.
#[must_use]
pub fn coefficients_c1(epsilon: f64) -> SeriesCoefficients {
    let epsilon2 = epsilon * epsilon;
    let mut d = epsilon;
    let mut c = SeriesCoefficients::with_length(9);
    c.values[1] = d * (epsilon2 * (epsilon2 * (19.0 * epsilon2 - 64.0) + 384.0) - 1024.0) / 2048.0;
    d *= epsilon;
    c.values[2] = d * (epsilon2 * (epsilon2 * (7.0 * epsilon2 - 18.0) + 128.0) - 256.0) / 4096.0;
    d *= epsilon;
    c.values[3] = d * ((72.0 - 9.0 * epsilon2) * epsilon2 - 128.0) / 6144.0;
    d *= epsilon;
    c.values[4] = d * ((96.0 - 11.0 * epsilon2) * epsilon2 - 160.0) / 16_384.0;
    d *= epsilon;
    c.values[5] = d * (35.0 * epsilon2 - 56.0) / 10_240.0;
    d *= epsilon;
    c.values[6] = d * (9.0 * epsilon2 - 14.0) / 4096.0;
    d *= epsilon;
    c.values[7] = -33.0 * d / 14_336.0;
    d *= epsilon;
    c.values[8] = -429.0 * d / 262_144.0;
    c
}

/// Generate the order-8 reverted Fourier coefficients `C1p[1..=8]`.
///
/// Mirrors the default arm of `evaluate_coeffs_C1p` from
/// `util/series_expansion.hpp:327-417`.
#[must_use]
pub fn coefficients_c1p(epsilon: f64) -> SeriesCoefficients {
    let epsilon2 = epsilon * epsilon;
    let mut d = epsilon;
    let mut c = SeriesCoefficients::with_length(9);
    c.values[1] =
        d * (epsilon2 * ((9840.0 - 4879.0 * epsilon2) * epsilon2 - 20_736.0) + 36_864.0) / 73_728.0;
    d *= epsilon;
    c.values[2] = d
        * (epsilon2 * ((120_150.0 - 86_171.0 * epsilon2) * epsilon2 - 142_080.0) + 115_200.0)
        / 368_640.0;
    d *= epsilon;
    c.values[3] = d * (epsilon2 * (8703.0 * epsilon2 - 7200.0) + 3712.0) / 12_288.0;
    d *= epsilon;
    c.values[4] = d * (epsilon2 * (1_082_857.0 * epsilon2 - 688_608.0) + 258_720.0) / 737_280.0;
    d *= epsilon;
    c.values[5] = d * (41_604.0 - 141_115.0 * epsilon2) / 92_160.0;
    d *= epsilon;
    c.values[6] = d * (533_134.0 - 2_200_311.0 * epsilon2) / 860_160.0;
    d *= epsilon;
    c.values[7] = 459_485.0 * d / 516_096.0;
    d *= epsilon;
    c.values[8] = 109_167_851.0 * d / 82_575_360.0;
    c
}

/// Generate the order-8 Fourier coefficients `C2[1..=8]`.
///
/// Mirrors the default arm of `evaluate_coeffs_C2` from
/// `util/series_expansion.hpp:427-517`.
#[must_use]
pub fn coefficients_c2(epsilon: f64) -> SeriesCoefficients {
    let epsilon2 = epsilon * epsilon;
    let mut d = epsilon;
    let mut c = SeriesCoefficients::with_length(9);
    c.values[1] = d * (epsilon2 * (epsilon2 * (41.0 * epsilon2 + 64.0) + 128.0) + 1024.0) / 2048.0;
    d *= epsilon;
    c.values[2] = d * (epsilon2 * (epsilon2 * (47.0 * epsilon2 + 70.0) + 128.0) + 768.0) / 4096.0;
    d *= epsilon;
    c.values[3] = d * (epsilon2 * (69.0 * epsilon2 + 120.0) + 640.0) / 6144.0;
    d *= epsilon;
    c.values[4] = d * (epsilon2 * (133.0 * epsilon2 + 224.0) + 1120.0) / 16_384.0;
    d *= epsilon;
    c.values[5] = d * (105.0 * epsilon2 + 504.0) / 10_240.0;
    d *= epsilon;
    c.values[6] = d * (33.0 * epsilon2 + 154.0) / 4096.0;
    d *= epsilon;
    c.values[7] = 429.0 * d / 14_336.0;
    d *= epsilon;
    c.values[8] = 6435.0 * d / 262_144.0;
    c
}

/// Generate the order-8 Fourier coefficients `C3[1..=7]`.
///
/// Mirrors `evaluate_coeffs_C3x<8>` and `evaluate_coeffs_C3` from
/// `util/series_expansion.hpp:527-663`. Index zero is zero.
#[must_use]
pub fn coefficients_c3(n: f64, epsilon: f64) -> SeriesCoefficients {
    let n2 = n * n;
    let mut x = [0.0; 28];
    x[0] = (1.0 - n) / 4.0;
    x[1] = (1.0 - n2) / 8.0;
    x[2] = (n * ((-5.0 * n - 1.0) * n + 3.0) + 3.0) / 64.0;
    x[3] = (n * ((2.0 - 2.0 * n) * n + 2.0) + 5.0) / 128.0;
    x[4] = (n * (3.0 * n + 11.0) + 12.0) / 512.0;
    x[5] = (10.0 * n + 21.0) / 1024.0;
    x[6] = 243.0 / 16_384.0;
    x[7] = ((n - 3.0) * n + 2.0) / 32.0;
    x[8] = (n * (n * (2.0 * n - 3.0) - 2.0) + 3.0) / 64.0;
    x[9] = (n * ((-6.0 * n - 9.0) * n + 2.0) + 6.0) / 256.0;
    x[10] = ((1.0 - 2.0 * n) * n + 5.0) / 256.0;
    x[11] = (69.0 * n + 108.0) / 8192.0;
    x[12] = 187.0 / 16_384.0;
    x[13] = (n * ((5.0 - n) * n - 9.0) + 5.0) / 192.0;
    x[14] = (n * (n * (10.0 * n - 6.0) - 10.0) + 9.0) / 384.0;
    x[15] = ((-77.0 * n - 8.0) * n + 42.0) / 3072.0;
    x[16] = (12.0 - n) / 1024.0;
    x[17] = 139.0 / 16_384.0;
    x[18] = (n * ((20.0 - 7.0 * n) * n - 28.0) + 14.0) / 1024.0;
    x[19] = ((-7.0 * n - 40.0) * n + 28.0) / 2048.0;
    x[20] = (72.0 - 43.0 * n) / 8192.0;
    x[21] = 127.0 / 16_384.0;
    x[22] = (n * (75.0 * n - 90.0) + 42.0) / 5120.0;
    x[23] = (9.0 - 15.0 * n) / 1024.0;
    x[24] = 99.0 / 16_384.0;
    x[25] = (44.0 - 99.0 * n) / 8192.0;
    x[26] = 99.0 / 16_384.0;
    x[27] = 429.0 / 114_688.0;

    let mut coefficients = SeriesCoefficients::with_length(8);
    let mut multiplier = 1.0;
    let mut offset = 0;
    for index in 1..8 {
        let polynomial_length = 8 - index;
        multiplier *= epsilon;
        coefficients.values[index] =
            multiplier * horner(epsilon, &x[offset..offset + polynomial_length]);
        offset += polynomial_length;
    }
    coefficients
}

fn horner(value: f64, coefficients: &[f64]) -> f64 {
    coefficients
        .iter()
        .rev()
        .fold(0.0, |result, coefficient| result * value + coefficient)
}

/// Evaluate `Σ c[i] sin(2 i x)` using Clenshaw summation.
///
/// `sin_x` and `cos_x` are supplied separately because geodesic formulas
/// already have both. Mirrors `sin_cos_series` from
/// `util/series_expansion.hpp:673-692`.
#[inline]
#[must_use]
pub fn sin_cos_series(sin_x: f64, cos_x: f64, coefficients: &SeriesCoefficients) -> f64 {
    let mut n = coefficients.len() - 1;
    let mut index = n + 1;
    let recurrence = 2.0 * (cos_x - sin_x) * (cos_x + sin_x);
    let mut k0 = if n & 1 != 0 {
        index -= 1;
        coefficients[index]
    } else {
        0.0
    };
    let mut k1 = 0.0;
    n /= 2;
    while n != 0 {
        index -= 1;
        k1 = recurrence * k0 - k1 + coefficients[index];
        index -= 1;
        k0 = recurrence * k1 - k0 + coefficients[index];
        n -= 1;
    }
    2.0 * sin_x * cos_x * k0
}
