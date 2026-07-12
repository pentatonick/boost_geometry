//! Cross-kind mismatch error for the `_dyn` algorithm wrappers.
//!
//! Mirrors no single Boost type — Boost would static-assert at
//! compile time. The Rust port surfaces the same error as a
//! `Result::Err` once the dispatch is dynamic.

use core::fmt;

use geometry_model::DynKind;

/// Returned by `_dyn` wrappers when the input combination has no
/// static algorithm impl.
///
/// Example: `distance_dyn(&Polygon, &Polygon)` — v1 has no
/// polygon-to-polygon distance impl yet. The wrapper returns
/// `Err(DynKindMismatch { got: [Polygon, Polygon], expected: &[...] })`
/// rather than panicking.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DynKindMismatch {
    /// The actual kinds of the inputs, in argument order.
    pub got: alloc::vec::Vec<DynKind>,
    /// The kind combinations the algorithm supports. Each inner slice
    /// is one supported argument list.
    pub expected: &'static [&'static [DynKind]],
}

impl fmt::Display for DynKindMismatch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "dynamic-geometry algorithm received {:?}; supported combinations: {:?}",
            self.got, self.expected,
        )
    }
}

#[cfg(feature = "std")]
impl std::error::Error for DynKindMismatch {}
