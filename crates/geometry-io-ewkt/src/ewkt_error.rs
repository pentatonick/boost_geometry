//! Everything that can go wrong reading EWKT.
//!
//! Hoisted to its own file because two sibling parts produce it: the
//! prefix scanner builds [`EwktError::InvalidSrid`], and `ewkt.rs` builds
//! [`EwktError::Wkt`] after rebasing the WKT crate's byte offsets.

use geometry_io_wkt::WktError;

/// Everything that can go wrong reading EWKT.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::{EwktError, from_ewkt};
///
/// let e = from_ewkt("SRID=-1;POINT(1 2)").unwrap_err();
/// assert_eq!(
///     e,
///     EwktError::InvalidSrid {
///         reason: "sign not allowed",
///         pos: 5,
///     }
/// );
/// assert_eq!(e.to_string(), "invalid SRID prefix: sign not allowed at byte 5");
/// ```
#[derive(Debug, Clone, PartialEq)]
pub enum EwktError {
    /// After leading whitespace the input begins with a run of ASCII
    /// letters that uppercases to `SRID`, but the prefix is malformed.
    InvalidSrid {
        /// Why the scanner stopped at `pos`: one of
        /// "expected '='", "expected digits", "sign not allowed",
        /// "value exceeds u32", "expected ';'".
        reason: &'static str,
        /// Byte offset, in the caller's original string, of the first
        /// byte that does not fit the prefix grammar or its `u32` bound,
        /// or `input.len()` when the input ended inside the prefix.
        pos: usize,
    },
    /// The geometry body failed to parse as WKT. Positions index the
    /// caller's original string; an `UnexpectedToken.found` payload
    /// names the token as the WKT crate saw it *after* the glued
    /// dimension-suffix normalisation (so a stray `POINTM` is reported
    /// as `POINT`).
    Wkt(WktError),
}

impl core::fmt::Display for EwktError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EwktError::InvalidSrid { reason, pos } => {
                write!(f, "invalid SRID prefix: {reason} at byte {pos}")
            }
            EwktError::Wkt(e) => write!(f, "invalid WKT body: {e}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EwktError {}

#[cfg(test)]
mod tests {
    use alloc::format;

    use geometry_io_wkt::WktError;

    use super::EwktError;

    #[test]
    fn invalid_srid_display() {
        let e = EwktError::InvalidSrid {
            reason: "expected ';'",
            pos: 9,
        };
        assert_eq!(
            format!("{e}"),
            "invalid SRID prefix: expected ';' at byte 9"
        );
    }

    #[test]
    fn wkt_display_includes_the_inner_message() {
        let e = EwktError::Wkt(WktError::UnexpectedEof);
        assert_eq!(format!("{e}"), "invalid WKT body: unexpected end of input");
    }
}
