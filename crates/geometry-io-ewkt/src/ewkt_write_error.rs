//! Checked EWKT serialization failures.

use geometry_io_wkt::WktWriteError;

/// The SRID, geometry or sink prevented EWKT serialization.
///
/// ```
/// use geometry_io_ewkt::EwktWriteError;
/// let error = EwktWriteError::SridOutOfRange { srid: 1_000_000 };
/// assert!(error.to_string().contains("1000000"));
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EwktWriteError {
    /// A supplied opaque SRID would be changed by `PostGIS` ingestion.
    SridOutOfRange {
        /// The caller's unchanged SRID value.
        srid: u32,
    },
    /// The WKT body could not be encoded.
    Wkt(WktWriteError),
    /// The sink refused the SRID prefix.
    Sink(core::fmt::Error),
}

impl From<WktWriteError> for EwktWriteError {
    fn from(error: WktWriteError) -> Self {
        Self::Wkt(error)
    }
}

impl From<core::fmt::Error> for EwktWriteError {
    fn from(error: core::fmt::Error) -> Self {
        Self::Sink(error)
    }
}

impl core::fmt::Display for EwktWriteError {
    fn fmt(&self, out: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::SridOutOfRange { srid } => write!(
                out,
                "SRID {srid} is outside PostGIS's 0..=999999 text range"
            ),
            Self::Wkt(error) => write!(out, "invalid WKT output: {error}"),
            Self::Sink(error) => write!(out, "EWKT prefix output failed: {error}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EwktWriteError {}
