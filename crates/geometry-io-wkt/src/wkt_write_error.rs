//! Errors from checked XY text serialization.

use crate::geometry_structure::GeometryStructureError;

/// A geometry or output sink prevented WKT serialization.
///
/// ```
/// use geometry_io_wkt::WktWriteError;
/// assert_eq!(WktWriteError::InfiniteCoordinate.to_string(), "infinity has no PostGIS text spelling");
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WktWriteError {
    /// The point type is not XY, even if the geometry is empty.
    UnsupportedDimension {
        /// Declared number of ordinates.
        dimensions: usize,
    },
    /// Infinity has no accepted text token in the target profile.
    InfiniteCoordinate,
    /// A count or ring structure violates the ingestion grammar.
    InvalidGeometry(GeometryStructureError),
    /// The geometry exceeds the supported text nesting depth.
    NestingTooDeep,
    /// The output sink, or an external writer, returned an error.
    Sink(core::fmt::Error),
}

impl From<core::fmt::Error> for WktWriteError {
    fn from(error: core::fmt::Error) -> Self {
        Self::Sink(error)
    }
}

impl From<GeometryStructureError> for WktWriteError {
    fn from(error: GeometryStructureError) -> Self {
        Self::InvalidGeometry(error)
    }
}

impl core::fmt::Display for WktWriteError {
    fn fmt(&self, out: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnsupportedDimension { dimensions } => write!(
                out,
                "expected XY point coordinates, found {dimensions} ordinates"
            ),
            Self::NestingTooDeep => out.write_str("WKT nesting exceeds the supported depth"),
            Self::InfiniteCoordinate => out.write_str("infinity has no PostGIS text spelling"),
            Self::InvalidGeometry(error) => write!(out, "invalid WKT geometry: {error}"),
            Self::Sink(error) => write!(out, "WKT output failed: {error}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for WktWriteError {}
