//! Failures while reading the `PostGIS` XY text profile.

use crate::geometry_structure::GeometryStructureError;
use alloc::string::String;

/// Everything that can go wrong reading WKT.
///
/// Covers the lexer's character-level failures plus the parser's
/// token-level and type-level failures, so a single error type flows
/// through the whole read path (mirroring how
/// `boost/geometry/io/wkt/read.hpp` throws a single
/// `read_wkt_exception`).
#[derive(Debug, Clone, PartialEq)]
pub enum WktError {
    /// A character that cannot begin any lexeme, at byte offset `pos`.
    UnexpectedChar {
        /// Byte offset of the offending character in the input.
        pos: usize,
        /// The offending character.
        ch: char,
    },
    /// A token that does not fit the grammar at this point.
    UnexpectedToken {
        /// A human-readable description of what the parser wanted.
        expected: &'static str,
        /// A `Debug`-style rendering of the token actually found.
        found: String,
    },
    /// Input ended while the parser still needed more tokens.
    UnexpectedEof,
    /// A numeric lexeme that failed to parse as `f64`.
    InvalidNumber(String),
    /// A syntactically valid number exceeds finite f64 range.
    NumberOutOfRange {
        /// Byte offset of the number in the original input.
        pos: usize,
        /// The offending numeric spelling.
        literal: String,
    },
    /// A leading keyword that is not a known OGC geometry type.
    UnknownGeometryType(String),
    /// A typed-parse convenience function was handed WKT of the wrong
    /// kind (e.g. [`crate::parse_point`] on a `LINESTRING`).
    TypeMismatch {
        /// The kind the caller asked for.
        expected: &'static str,
        /// The kind actually present in the input.
        found: &'static str,
    },
    /// Nested `GEOMETRYCOLLECTION`s exceeded the reader's recursion limit.
    /// Rejecting deep nesting keeps a hostile string from overflowing the
    /// native stack (an uncatchable process abort).
    NestingTooDeep,
    /// A Z, M or ZM qualifier is unsupported by the XY reader.
    UnsupportedDimension {
        /// Original input byte offset.
        pos: usize,
        /// Qualifier spelling.
        qualifier: String,
    },
    /// A point has too few or too many ordinates. For excess input,
    /// `found` is the first excessive count rather than scanning the rest.
    CoordinateCount {
        /// Original input byte offset.
        pos: usize,
        /// Required count.
        expected: usize,
        /// Observed count.
        found: usize,
    },
    /// A line/ring structure fails the text grammar's ingestion checks.
    InvalidGeometry {
        /// Original input byte offset.
        pos: usize,
        /// Structural cause.
        reason: GeometryStructureError,
    },
    /// The requested legacy result cannot hold an empty point.
    EmptyPoint(geometry_model::EmptyPointError),
}

impl core::fmt::Display for WktError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WktError::UnexpectedChar { pos, ch } => {
                write!(f, "unexpected character {ch:?} at byte {pos}")
            }
            WktError::UnexpectedToken { expected, found } => {
                write!(f, "expected {expected}, found {found}")
            }
            WktError::UnexpectedEof => f.write_str("unexpected end of input"),
            WktError::InvalidNumber(s) => write!(f, "invalid number {s:?}"),
            WktError::NumberOutOfRange { pos, literal } => write!(
                f,
                "number {literal:?} at byte {pos} exceeds finite f64 range"
            ),
            WktError::UnknownGeometryType(s) => write!(f, "unknown geometry type {s:?}"),
            WktError::TypeMismatch { expected, found } => {
                write!(f, "type mismatch: expected {expected}, found {found}")
            }
            WktError::UnsupportedDimension { pos, qualifier } => write!(
                f,
                "unsupported {qualifier} dimension at byte {pos}: this reader is XY only"
            ),
            WktError::CoordinateCount {
                pos,
                expected,
                found,
            } => write!(
                f,
                "expected {expected} ordinates at byte {pos}, found {found}"
            ),
            WktError::InvalidGeometry { pos, reason } => {
                write!(f, "invalid geometry at byte {pos}: {reason}")
            }
            WktError::EmptyPoint(error) => error.fmt(f),
            WktError::NestingTooDeep => {
                f.write_str("WKT nesting too deep; exceeded the reader's recursion limit")
            }
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for WktError {}

impl WktError {
    /// Rebase a body error onto the containing input. Offsets saturate on overflow.
    ///
    /// ```
    /// use geometry_io_wkt::WktError;
    /// let error = WktError::UnexpectedChar { pos: 2, ch: '?' }.with_offset(10);
    /// assert_eq!(error, WktError::UnexpectedChar { pos: 12, ch: '?' });
    /// ```
    #[must_use]
    pub fn with_offset(mut self, offset: usize) -> Self {
        match &mut self {
            Self::UnexpectedChar { pos, .. }
            | Self::NumberOutOfRange { pos, .. }
            | Self::UnsupportedDimension { pos, .. }
            | Self::CoordinateCount { pos, .. }
            | Self::InvalidGeometry { pos, .. } => *pos = pos.saturating_add(offset),
            _ => {}
        }
        self
    }
}
