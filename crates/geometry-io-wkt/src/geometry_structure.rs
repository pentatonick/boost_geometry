//! Structural requirements of the `PostGIS` XY text grammar.

use geometry_trait::Point;

/// Maximum accepted geometry record depth for both text directions.
pub(crate) const MAX_DEPTH: usize = 128;

/// A geometry cannot be represented by the supported text grammar.
///
/// These are ingestion checks, not topology validation.
///
/// ```
/// use geometry_io_wkt::GeometryStructureError;
/// let error = GeometryStructureError::UnclosedRing;
/// assert_eq!(error.to_string(), "polygon ring is not closed");
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryStructureError {
    /// A nonempty line or ring has too few positions.
    TooFewPoints {
        /// Required position count, including ring closure.
        minimum: usize,
        /// Supplied position count.
        actual: usize,
    },
    /// First and last coordinates differ under text-ingest closure rules.
    UnclosedRing,
    /// Interior rings exist without an exterior.
    MissingExterior,
    /// An interior ring has no positions.
    EmptyInterior,
}

impl core::fmt::Display for GeometryStructureError {
    fn fmt(&self, out: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooFewPoints { minimum, actual } => {
                write!(out, "expected at least {minimum} points, found {actual}")
            }
            Self::UnclosedRing => out.write_str("polygon ring is not closed"),
            Self::MissingExterior => out.write_str("polygon has holes without an exterior"),
            Self::EmptyInterior => out.write_str("polygon has an empty interior ring"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for GeometryStructureError {}

pub(crate) fn linestring_count(count: usize) -> Result<(), GeometryStructureError> {
    if count == 1 {
        Err(GeometryStructureError::TooFewPoints {
            minimum: 2,
            actual: count,
        })
    } else {
        Ok(())
    }
}

/// Text maps all NaNs to one spelling but preserves the sign of finite zero.
fn same_ordinate(a: f64, b: f64) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

pub(crate) fn ring<'a, P: Point<Scalar = f64> + 'a>(
    points: impl Iterator<Item = &'a P>,
) -> Result<(), GeometryStructureError> {
    let mut points = points;
    let Some(first) = points.next() else {
        return Err(GeometryStructureError::TooFewPoints {
            minimum: 4,
            actual: 0,
        });
    };
    let mut last = first;
    let mut count = 1;
    for point in points {
        last = point;
        count += 1;
    }
    if count < 4 {
        return Err(GeometryStructureError::TooFewPoints {
            minimum: 4,
            actual: count,
        });
    }
    if !same_ordinate(first.get::<0>(), last.get::<0>())
        || !same_ordinate(first.get::<1>(), last.get::<1>())
    {
        return Err(GeometryStructureError::UnclosedRing);
    }
    Ok(())
}
