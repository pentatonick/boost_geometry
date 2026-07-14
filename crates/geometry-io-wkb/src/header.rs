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
    /// (high bits `0x8000_0000` / `0x4000_0000`, or the ISO `1000`+
    /// ranges). This is a strictly-2D port and rejects higher
    /// dimensions rather than silently dropping ordinates.
    UnsupportedDimension,
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
            WkbError::UnsupportedDimension => {
                f.write_str("unsupported WKB dimension (Z/M ordinates); this reader is 2D only")
            }
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

    /// Read the next `N` bytes as a fixed-size array, advancing the
    /// cursor. Fails with [`WkbError::UnexpectedEof`] if fewer than `N`
    /// bytes remain.
    fn read_array<const N: usize>(&mut self) -> Result<[u8; N], WkbError> {
        let end = self.pos.checked_add(N).ok_or(WkbError::UnexpectedEof)?;
        let slice = self
            .bytes
            .get(self.pos..end)
            .ok_or(WkbError::UnexpectedEof)?;
        let mut buf = [0u8; N];
        buf.copy_from_slice(slice);
        self.pos = end;
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
        let b = self.read_array::<4>()?;
        Ok(match order {
            ByteOrder::LittleEndian => u32::from_le_bytes(b),
            ByteOrder::BigEndian => u32::from_be_bytes(b),
        })
    }

    /// Read a 64-bit IEEE-754 float in the given byte order.
    ///
    /// # Errors
    ///
    /// [`WkbError::UnexpectedEof`] if fewer than eight bytes remain.
    pub(crate) fn read_f64(&mut self, order: ByteOrder) -> Result<f64, WkbError> {
        let b = self.read_array::<8>()?;
        Ok(match order {
            ByteOrder::LittleEndian => f64::from_le_bytes(b),
            ByteOrder::BigEndian => f64::from_be_bytes(b),
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
        match self.read_u8()? {
            0x00 => Ok(ByteOrder::BigEndian),
            0x01 => Ok(ByteOrder::LittleEndian),
            other => Err(WkbError::InvalidByteOrder(other)),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Cursor-level witnesses against hand-crafted bytes, per OGC
    //! 06-103r4 §8.2.

    use super::*;

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
            format!("{}", WkbError::UnsupportedDimension).contains("2D only"),
            "dimension message"
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
