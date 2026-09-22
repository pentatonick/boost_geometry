//! WKB byte-order, error type, and the low-level byte cursor.
//!
//! Mirrors the header layout of OGC Simple Feature Access 06-103r4 §8.2:
//! every WKB record opens with a one-byte endianness flag followed by a
//! 32-bit geometry-type tag read *in that byte order*. Boost ships no
//! WKB reader, so there is no C++ counterpart to mirror; the shapes here
//! follow the OGC spec directly. The [`Cursor`] is the shared, panic-free
//! byte reader every `parse`/`write` step drives.
//!
//! Reference: OGC 06-103r4 §8.2 (Well-Known Binary representation).

/// Bytes in a WKB record header: a one-byte order flag plus a 32-bit
/// type word (OGC 06-103r4 §8.2.3-8.2.4). The aggregate that owns the
/// header owns its width; `parse` and `write` both point here rather
/// than spelling `5` a second and third time.
pub(crate) const RECORD_HEADER_LEN: usize = 5;
/// Bytes in the 32-bit type word — the other half of
/// [`RECORD_HEADER_LEN`], the first being the order flag.
pub(crate) const TYPE_WORD_LEN: usize = 4;
const _: () = assert!(RECORD_HEADER_LEN == 1 + TYPE_WORD_LEN);

/// The two byte orders a WKB record may declare.
///
/// The leading byte of every WKB record is `0x00` for big-endian
/// (network byte order) or `0x01` for little-endian, per OGC 06-103r4
/// §8.2.3. Every multi-byte scalar in that record is then read/written
/// in the declared order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ByteOrder {
    /// `0x01` — least-significant byte first.
    LittleEndian,
    /// `0x00` — most-significant byte first (XDR / network order).
    BigEndian,
}

impl ByteOrder {
    /// The order a WKB record's leading flag byte declares, or `None`
    /// when the byte is neither `0x00` nor `0x01` (OGC 06-103r4 §8.2.3).
    ///
    /// ```
    /// use geometry_io_wkb::ByteOrder;
    ///
    /// assert_eq!(ByteOrder::from_flag(0x01), Some(ByteOrder::LittleEndian));
    /// assert_eq!(ByteOrder::from_flag(0x00), Some(ByteOrder::BigEndian));
    /// assert_eq!(ByteOrder::from_flag(0x02), None);
    /// ```
    #[must_use]
    pub const fn from_flag(byte: u8) -> Option<Self> {
        match byte {
            0x00 => Some(Self::BigEndian),
            0x01 => Some(Self::LittleEndian),
            _ => None,
        }
    }

    /// The flag byte that declares this order.
    ///
    /// Crate-internal: no decision authorised it as public API, and no
    /// consumer needs it — a reader gets the order from
    /// [`ByteOrder::from_flag`], and a writer gets the flag byte from
    /// the record `write_wkb` already produced.
    pub(crate) const fn flag(self) -> u8 {
        match self {
            Self::LittleEndian => 0x01,
            Self::BigEndian => 0x00,
        }
    }

    /// Decode a WKB `uint32` held in this order.
    ///
    /// ```
    /// use geometry_io_wkb::ByteOrder;
    ///
    /// assert_eq!(ByteOrder::LittleEndian.read_u32([0x01, 0, 0, 0]), 1);
    /// assert_eq!(ByteOrder::BigEndian.read_u32([0, 0, 0, 0x01]), 1);
    /// ```
    #[must_use]
    pub const fn read_u32(self, bytes: [u8; 4]) -> u32 {
        match self {
            Self::LittleEndian => u32::from_le_bytes(bytes),
            Self::BigEndian => u32::from_be_bytes(bytes),
        }
    }

    /// Encode a WKB `uint32` in this order.
    ///
    /// ```
    /// use geometry_io_wkb::ByteOrder;
    ///
    /// assert_eq!(ByteOrder::LittleEndian.to_bytes(1), [0x01, 0, 0, 0]);
    /// assert_eq!(ByteOrder::BigEndian.read_u32(ByteOrder::BigEndian.to_bytes(4326)), 4326);
    /// ```
    #[must_use]
    pub const fn to_bytes(self, value: u32) -> [u8; 4] {
        match self {
            Self::LittleEndian => value.to_le_bytes(),
            Self::BigEndian => value.to_be_bytes(),
        }
    }

    /// Decode a WKB `float64` held in this order.
    ///
    /// Crate-internal, deliberately: a public `f64` **decoder** with no
    /// matching encoder would be an incoherent surface, and nothing
    /// outside this crate needs either.
    pub(crate) const fn read_f64(self, bytes: [u8; 8]) -> f64 {
        match self {
            Self::LittleEndian => f64::from_le_bytes(bytes),
            Self::BigEndian => f64::from_be_bytes(bytes),
        }
    }
}

/// A decoded WKB header with a borrowed slice of the remaining record.
///
/// The type word is preserved without interpretation so dialect readers
/// can handle their flags before parsing the body.
///
/// ```
/// use geometry_io_wkb::{ByteOrder, WkbHeader, split_header};
///
/// let bytes = [1, 3, 0, 0, 0, 0, 0, 0, 0];
/// let header: WkbHeader<'_> = split_header(&bytes).unwrap();
/// assert_eq!(header.byte_order, ByteOrder::LittleEndian);
/// assert_eq!(header.type_word, 3);
/// assert_eq!(header.body, &[0, 0, 0, 0]);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WkbHeader<'a> {
    /// The byte order declared by the record.
    pub byte_order: ByteOrder,
    /// The uninterpreted 32-bit geometry type word.
    pub type_word: u32,
    /// All bytes after the five-byte header, borrowed from the input.
    pub body: &'a [u8],
}

/// Read the byte order and raw type word, borrowing the remaining bytes.
///
/// No body or type-code validation is performed. A dialect reader may
/// consume additional fields before calling [`crate::from_wkb_parts`].
///
/// # Errors
///
/// Returns [`WkbError::UnexpectedEof`] for a truncated header or
/// [`WkbError::InvalidByteOrder`] for an invalid first byte. The order
/// flag is checked first, including when the rest of the header is missing.
///
/// ```
/// use geometry_io_wkb::{WkbError, split_header};
///
/// assert_eq!(split_header(&[]), Err(WkbError::UnexpectedEof));
/// assert_eq!(split_header(&[2]), Err(WkbError::InvalidByteOrder(2)));
/// assert_eq!(split_header(&[1, 1, 0, 0, 0]).unwrap().type_word, 1);
/// ```
pub fn split_header(bytes: &[u8]) -> Result<WkbHeader<'_>, WkbError> {
    Cursor::new(bytes).read_header()
}

/// Everything that can go wrong reading WKB.
///
/// Covers the cursor's bounds failures plus the parser's header-level
/// failures, so a single error type flows through the whole read path
/// (mirroring how [`crate::from_wkb`] surfaces one error kind, the way
/// the sibling WKT reader funnels through `WktError`).
#[derive(Debug, Clone, PartialEq)]
pub enum WkbError {
    /// The cursor ran off the end of the buffer while a read was still
    /// in progress.
    UnexpectedEof,
    /// The leading byte-order flag was neither `0x00` nor `0x01`.
    InvalidByteOrder(u8),
    /// The 32-bit geometry-type tag is not one of the seven OGC base
    /// codes (`1`..=`7`).
    UnknownGeometryType(u32),
    /// The geometry-type tag carried a `Z`, `M`, or `ZM` dimension flag
    /// — the EWKB high bits `0x8000_0000` / `0x4000_0000`, or the ISO
    /// SQL/MM `1000` / `2000` / `3000` ranges. This is a strictly-2D
    /// port and rejects higher dimensions rather than silently dropping
    /// ordinates.
    HigherDimension {
        /// The whole 32-bit type word, for diagnosis.
        type_word: u32,
    },
    /// The geometry-type tag carried the EWKB SRID flag
    /// (`0x2000_0000`), which prefixes the body with four bytes this
    /// OGC reader does not consume. Nothing about the dimension is
    /// wrong; the buffer is EWKB, not WKB.
    UnexpectedSridFlag {
        /// The whole 32-bit type word, for diagnosis.
        type_word: u32,
    },
    /// The geometry-type tag is above the 2D range but is neither a
    /// dimension encoding nor an SRID flag — an EWKB bounding-box bit,
    /// or an undefined high bit. The reader cannot classify it, which
    /// is a different fact from "this is 3D".
    UnrecognisedTypeWord {
        /// The whole 32-bit type word, for diagnosis.
        type_word: u32,
    },
    /// The top-level geometry was parsed successfully but bytes remained
    /// in the buffer afterwards.
    TrailingBytes,
    /// The multi / collection nesting exceeded the reader's recursion
    /// limit. Rejecting deep nesting keeps a hostile buffer from
    /// overflowing the native stack (an uncatchable process abort).
    NestingTooDeep,
    /// A multi-geometry member record parsed successfully but has the
    /// wrong kind — e.g. a `MultiPoint` whose member is a
    /// `LineString`. Both codes are OGC base type codes (`1`..=`7`).
    MismatchedMemberType {
        /// The member code the container requires.
        expected: u32,
        /// The member code actually found.
        found: u32,
    },
}

impl core::fmt::Display for WkbError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WkbError::UnexpectedEof => f.write_str("unexpected end of WKB input"),
            WkbError::InvalidByteOrder(b) => {
                write!(
                    f,
                    "invalid byte-order flag {b:#04x} (expected 0x00 or 0x01)"
                )
            }
            WkbError::UnknownGeometryType(t) => write!(f, "unknown WKB geometry type {t}"),
            WkbError::HigherDimension { type_word } => write!(
                f,
                "type word {type_word:#010x} carries a Z/M dimension; this reader is 2D only"
            ),
            WkbError::UnexpectedSridFlag { type_word } => write!(
                f,
                "type word {type_word:#010x} sets the EWKB SRID flag; this is an OGC WKB reader"
            ),
            WkbError::UnrecognisedTypeWord { type_word } => write!(
                f,
                "type word {type_word:#010x} is above the 2D range and is neither a dimension nor an SRID flag"
            ),
            WkbError::TrailingBytes => f.write_str("trailing bytes after WKB geometry"),
            WkbError::NestingTooDeep => {
                f.write_str("WKB nesting too deep; exceeded the reader's recursion limit")
            }
            WkbError::MismatchedMemberType { expected, found } => write!(
                f,
                "WKB multi-geometry member has type {found}, expected {expected}"
            ),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for WkbError {}

/// A panic-free cursor over a WKB byte buffer.
///
/// Every read is bounds-checked and yields [`WkbError::UnexpectedEof`]
/// rather than indexing out of range — the crate forbids `unsafe`, so
/// all slice access flows through these methods. Multi-byte reads take
/// the [`ByteOrder`] of the enclosing record and use
/// `u32::from_le_bytes` / `from_be_bytes` (and the `f64` equivalents),
/// so no external `byteorder` dependency is needed.
pub(crate) struct Cursor<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    /// Wrap a byte slice at position zero.
    pub(crate) fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    /// `true` once every byte has been consumed. Used by
    /// [`crate::from_wkb`] to detect [`WkbError::TrailingBytes`].
    pub(crate) fn is_empty(&self) -> bool {
        self.pos >= self.bytes.len()
    }

    /// Bytes left to read. Used to bound speculative `Vec::with_capacity`
    /// reservations against a possibly-hostile element count so a
    /// corrupt/malicious buffer cannot request a huge allocation before a
    /// single element is read.
    pub(crate) fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.pos)
    }

    /// Read `len` bytes as one borrowed slice, advancing the cursor.
    /// Point runs use this to validate their complete fixed-width body
    /// once instead of repeating a bounds check for every ordinate.
    pub(crate) fn read_slice(&mut self, len: usize) -> Result<&'a [u8], WkbError> {
        let end = self.pos.checked_add(len).ok_or(WkbError::UnexpectedEof)?;
        let slice = self
            .bytes
            .get(self.pos..end)
            .ok_or(WkbError::UnexpectedEof)?;
        self.pos = end;
        Ok(slice)
    }

    /// Read the next `N` bytes as a fixed-size array, advancing the
    /// cursor. Fails with [`WkbError::UnexpectedEof`] if fewer than `N`
    /// bytes remain.
    fn read_array<const N: usize>(&mut self) -> Result<[u8; N], WkbError> {
        let slice = self.read_slice(N)?;
        let mut buf = [0u8; N];
        buf.copy_from_slice(slice);
        Ok(buf)
    }

    /// Read one byte, advancing the cursor.
    ///
    /// # Errors
    ///
    /// [`WkbError::UnexpectedEof`] if the buffer is exhausted.
    pub(crate) fn read_u8(&mut self) -> Result<u8, WkbError> {
        Ok(self.read_array::<1>()?[0])
    }

    /// Read a 32-bit unsigned integer in the given byte order.
    ///
    /// # Errors
    ///
    /// [`WkbError::UnexpectedEof`] if fewer than four bytes remain.
    pub(crate) fn read_u32(&mut self, order: ByteOrder) -> Result<u32, WkbError> {
        Ok(order.read_u32(self.read_array::<4>()?))
    }

    /// Read a 64-bit IEEE-754 float in the given byte order.
    ///
    /// # Errors
    ///
    /// [`WkbError::UnexpectedEof`] if fewer than eight bytes remain.
    pub(crate) fn read_f64(&mut self, order: ByteOrder) -> Result<f64, WkbError> {
        Ok(order.read_f64(self.read_array::<8>()?))
    }

    /// Consume one header, leaving the cursor at its body.
    pub(crate) fn read_header(&mut self) -> Result<WkbHeader<'a>, WkbError> {
        let byte_order = self.read_byte_order()?;
        let type_word = self.read_u32(byte_order)?;
        Ok(WkbHeader {
            byte_order,
            type_word,
            body: &self.bytes[self.pos..],
        })
    }

    /// Read the one-byte endianness flag that opens a WKB record
    /// (OGC 06-103r4 §8.2.3): `0x00` → big-endian, `0x01` → little.
    ///
    /// # Errors
    ///
    /// [`WkbError::UnexpectedEof`] at end of input, or
    /// [`WkbError::InvalidByteOrder`] for any other flag byte.
    pub(crate) fn read_byte_order(&mut self) -> Result<ByteOrder, WkbError> {
        let flag = self.read_u8()?;
        ByteOrder::from_flag(flag).ok_or(WkbError::InvalidByteOrder(flag))
    }
}

#[cfg(test)]
mod tests {
    //! Cursor-level witnesses against hand-crafted bytes, per OGC
    //! 06-103r4 §8.2.

    use super::*;

    #[test]
    fn flag_round_trips_both_orders() {
        assert_eq!(ByteOrder::LittleEndian.flag(), 0x01);
        assert_eq!(ByteOrder::BigEndian.flag(), 0x00);
        for o in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
            assert_eq!(ByteOrder::from_flag(o.flag()), Some(o));
        }
    }

    #[test]
    #[allow(
        clippy::float_cmp,
        reason = "the value comes from an exact byte literal, not from arithmetic"
    )]
    fn read_f64_decodes_in_both_orders() {
        assert_eq!(ByteOrder::LittleEndian.read_f64(1.5f64.to_le_bytes()), 1.5);
        assert_eq!(ByteOrder::BigEndian.read_f64(1.5f64.to_be_bytes()), 1.5);
    }

    #[test]
    fn to_bytes_is_byte_exact_in_both_orders() {
        // Asserted at byte level, not by round-trip: a round-trip would
        // pass even if both directions were swapped together.
        assert_eq!(ByteOrder::LittleEndian.to_bytes(1), [0x01, 0, 0, 0]);
        assert_eq!(ByteOrder::BigEndian.to_bytes(1), [0, 0, 0, 0x01]);
        assert_eq!(
            ByteOrder::BigEndian.to_bytes(4326),
            [0x00, 0x00, 0x10, 0xE6]
        );
    }

    #[test]
    fn reads_le_u32() {
        // 0x0000_0001 little-endian.
        let mut c = Cursor::new(&[0x01, 0x00, 0x00, 0x00]);
        assert_eq!(c.read_u32(ByteOrder::LittleEndian).unwrap(), 1);
    }

    #[test]
    fn reads_be_u32() {
        // 0x0000_0001 big-endian.
        let mut c = Cursor::new(&[0x00, 0x00, 0x00, 0x01]);
        assert_eq!(c.read_u32(ByteOrder::BigEndian).unwrap(), 1);
    }

    #[test]
    fn reads_byte_order_flags() {
        assert_eq!(
            Cursor::new(&[0x00]).read_byte_order().unwrap(),
            ByteOrder::BigEndian
        );
        assert_eq!(
            Cursor::new(&[0x01]).read_byte_order().unwrap(),
            ByteOrder::LittleEndian
        );
    }

    #[test]
    fn invalid_byte_order_is_rejected() {
        let err = Cursor::new(&[0x02]).read_byte_order().unwrap_err();
        assert_eq!(err, WkbError::InvalidByteOrder(0x02));
    }

    #[test]
    fn short_buffer_is_eof() {
        let err = Cursor::new(&[0x00, 0x00])
            .read_u32(ByteOrder::LittleEndian)
            .unwrap_err();
        assert_eq!(err, WkbError::UnexpectedEof);
    }

    /// Every `WkbError` variant renders a distinct, descriptive message
    /// through its `Display` impl, including the embedded byte/type/code
    /// values.
    #[test]
    fn every_error_variant_displays_descriptively() {
        extern crate alloc;
        use alloc::format;

        assert_eq!(
            format!("{}", WkbError::UnexpectedEof),
            "unexpected end of WKB input"
        );
        assert_eq!(
            format!("{}", WkbError::InvalidByteOrder(0x02)),
            "invalid byte-order flag 0x02 (expected 0x00 or 0x01)"
        );
        assert_eq!(
            format!("{}", WkbError::UnknownGeometryType(9)),
            "unknown WKB geometry type 9"
        );
        assert!(
            format!(
                "{}",
                WkbError::HigherDimension {
                    type_word: 0x8000_0001
                }
            )
            .contains("2D only"),
            "dimension message"
        );
        assert_eq!(
            format!(
                "{}",
                WkbError::UnexpectedSridFlag {
                    type_word: 0x2000_0001
                }
            ),
            "type word 0x20000001 sets the EWKB SRID flag; this is an OGC WKB reader"
        );
        assert!(
            format!(
                "{}",
                WkbError::UnrecognisedTypeWord {
                    type_word: 0x1000_0001
                }
            )
            .contains("neither a dimension nor an SRID flag"),
            "unrecognised message"
        );
        assert_eq!(
            format!("{}", WkbError::TrailingBytes),
            "trailing bytes after WKB geometry"
        );
        assert!(
            format!("{}", WkbError::NestingTooDeep).contains("nesting too deep"),
            "nesting message"
        );
        assert_eq!(
            format!(
                "{}",
                WkbError::MismatchedMemberType {
                    expected: 1,
                    found: 2
                }
            ),
            "WKB multi-geometry member has type 2, expected 1"
        );
    }
}
