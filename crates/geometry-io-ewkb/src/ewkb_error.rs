//! What can go wrong reading EWKB.

use geometry_io_wkb::WkbError;

/// Everything that can go wrong reading EWKB.
///
/// ```
/// use geometry_io_ewkb::EwkbError;
///
/// let e = EwkbError::DimensionFlag {
///     type_word: 0x8000_0001,
/// };
/// assert_eq!(
///     e.to_string(),
///     "type word 0x80000001 sets the Z or M flag: this reader is 2D only"
/// );
/// ```
///
/// No `Eq`: the [`WkbError`] payload derives only `Debug, Clone,
/// PartialEq`, so an `Eq` derive would not compile.
#[derive(Debug, Clone, PartialEq)]
pub enum EwkbError {
    /// The outermost type word set the bounding-box flag
    /// (`0x1000_0000`), which prefixes the body with a box whose width
    /// depends on the Z/M flags this reader already refuses.
    ///
    /// `PostGIS`'s own writer never emits this flag, so a record
    /// carrying it did not come from `PostGIS`.
    BoundingBoxFlag {
        /// The whole 32-bit type word, for diagnosis.
        type_word: u32,
    },
    /// The outermost type word set the SRID flag but fewer than four
    /// bytes followed it.
    TruncatedSrid {
        /// The whole 32-bit type word, for diagnosis.
        type_word: u32,
    },
    /// The outermost type word set the `Z` or `M` flag. This is a
    /// strictly-2D reader and rejects higher dimensions rather than
    /// silently dropping ordinates.
    DimensionFlag {
        /// The whole 32-bit type word, for diagnosis.
        type_word: u32,
    },
    /// A hex string held a byte that is not a hex digit, or had an odd
    /// length so that its final digit has no pair.
    InvalidHex {
        /// Byte offset of the offending digit. For an odd-length input
        /// this is the index of the unpaired final digit.
        index: usize,
    },
    /// The record's OGC-shaped bytes failed to parse.
    ///
    /// Where the inner error carries a `type_word`, it is the word
    /// **after** the SRID bit was cleared — what the WKB reader
    /// actually saw. For example, a wire word of `0x2800_0001` is
    /// reported as `0x0800_0001` after its SRID flag is removed.
    Wkb(WkbError),
}

impl From<WkbError> for EwkbError {
    fn from(error: WkbError) -> Self {
        Self::Wkb(error)
    }
}

impl core::fmt::Display for EwkbError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EwkbError::BoundingBoxFlag { type_word } => write!(
                f,
                "type word {type_word:#010x} sets the bounding-box flag: \
                 this reader does not skip a bounding box"
            ),
            EwkbError::TruncatedSrid { type_word } => write!(
                f,
                "type word {type_word:#010x} sets the SRID flag but fewer \
                 than four bytes follow it"
            ),
            EwkbError::DimensionFlag { type_word } => write!(
                f,
                "type word {type_word:#010x} sets the Z or M flag: \
                 this reader is 2D only"
            ),
            EwkbError::InvalidHex { index } => {
                write!(f, "invalid hex input at byte {index}")
            }
            EwkbError::Wkb(e) => write!(f, "invalid WKB body: {e}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EwkbError {} // source() stays None
