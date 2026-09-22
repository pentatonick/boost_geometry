//! Hex-encoded EWKB: the form `PostGIS` gives a `geometry` column when
//! it is read as text.
//!
//! `PostGIS` text output: `LWGEOM_out` returns
//! `lwgeom_to_hexwkb_buffer(lwgeom, WKB_EXTENDED)`, whose digits come
//! from `static char *hexchr = "0123456789ABCDEF"` — so `PostGIS` emits
//! **uppercase**, and there is **no `SRID=` prefix**. The `SRID=` form
//! is an *input* spelling of `LWGEOM_in`; hex EWKB is the plain hex of
//! the EWKB bytes and nothing else.
//!
//! This module owns the hex alphabet and nothing else. It adds no
//! dependency: the codec is a sixteen-byte table one way and a nibble
//! match the other.

use alloc::string::String;
use alloc::vec::Vec;

use geometry_cs::Cartesian;
use geometry_io_wkb::{ByteOrder, WriteWkb};
use geometry_model::DynGeometry;
use geometry_srid::Srid;
use geometry_trait::Geometry;

use crate::ewkb::{Ewkb, from_ewkb, to_ewkb};
use crate::ewkb_error::EwkbError;

/// The digits `PostGIS` emits, in `PostGIS`'s order.
const UPPER: &[u8; 16] = b"0123456789ABCDEF";

/// The value of one hex digit, or `None` if it is not one.
///
/// Accepts both cases: `PostGIS`'s text output is uppercase, but SQL's
/// `encode(…, 'hex')` is lowercase, so a user pasting from either is
/// served.
const fn digit(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

/// Decode a hex string into the bytes it spells.
///
/// # Errors
///
/// [`EwkbError::InvalidHex`] naming the offending **byte** offset: the
/// first non-hex byte, or — when every byte is a hex digit — the
/// unpaired final digit of an odd-length input. The offset is always a
/// `char` boundary, because the first byte of any non-ASCII character
/// is itself not a hex digit and so is what the scan reports.
fn decode(s: &str) -> Result<Vec<u8>, EwkbError> {
    let src = s.as_bytes();
    // A bad digit is reported before an odd length, so the index always
    // names a byte that is actually wrong. Checking the parity first
    // would blame a valid digit: `"0Z1"` would point at the `1`.
    if let Some(index) = src.iter().position(|&b| digit(b).is_none()) {
        return Err(EwkbError::InvalidHex { index });
    }
    if src.len() % 2 != 0 {
        return Err(EwkbError::InvalidHex {
            index: src.len() - 1,
        });
    }
    let mut out = Vec::with_capacity(src.len() / 2);
    for pair in src.chunks_exact(2) {
        // Both digits were validated by the scan above.
        let (hi, lo) = (digit(pair[0]), digit(pair[1]));
        match (hi, lo) {
            (Some(hi), Some(lo)) => out.push((hi << 4) | lo),
            _ => unreachable!("every byte passed the scan above"),
        }
    }
    Ok(out)
}

/// Encode bytes as uppercase hex.
fn encode(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for &b in bytes {
        out.push(UPPER[usize::from(b >> 4)] as char);
        out.push(UPPER[usize::from(b & 0x0F)] as char);
    }
    out
}

/// Read hex-encoded EWKB — the text form of a `PostGIS` `geometry`
/// column.
///
/// Accepts upper- and lower-case digits. There is no `SRID=` prefix to
/// strip: that spelling belongs to EWKT, not to hex EWKB.
///
/// # Errors
///
/// [`EwkbError::InvalidHex`] if the input is not an even-length run of
/// hex digits, and otherwise whatever [`from_ewkb`] returns for the
/// bytes it spells.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkb::{Srid, from_ewkb_hex};
///
/// // POINT(1 2) with SRID 4326, as PostGIS prints it.
/// let s = "0101000020E6100000000000000000F03F0000000000000040";
/// assert_eq!(from_ewkb_hex(s).unwrap().srid, Some(Srid::new(4326)));
/// // Lowercase is accepted too.
/// assert_eq!(from_ewkb_hex(&s.to_lowercase()).unwrap().srid, Some(Srid::new(4326)));
/// ```
pub fn from_ewkb_hex(s: &str) -> Result<Ewkb<DynGeometry<f64, Cartesian>>, EwkbError> {
    from_ewkb(&decode(s)?)
}

/// Write a geometry as hex-encoded EWKB, the way `PostGIS` prints it.
///
/// Uppercase, no prefix.
///
/// # Panics
///
/// Propagates the writer's panic contracts from [`to_ewkb`]. Allocation
/// also panics if the hex output exceeds the supported capacity.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_ewkb::{ByteOrder, Srid, to_ewkb_hex};
/// use geometry_model::Point2D;
///
/// let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
/// assert_eq!(
///     to_ewkb_hex(&p, Some(Srid::new(4326)), ByteOrder::LittleEndian),
///     "0101000020E6100000000000000000F03F0000000000000040",
/// );
/// ```
#[must_use]
pub fn to_ewkb_hex<G: Geometry + WriteWkb>(g: &G, srid: Option<Srid>, order: ByteOrder) -> String {
    encode(&to_ewkb(g, srid, order))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odd_length_names_the_unpaired_digit() {
        assert_eq!(decode("ABC"), Err(EwkbError::InvalidHex { index: 2 }));
    }

    #[test]
    fn non_hex_names_its_own_offset() {
        assert_eq!(decode("00GG"), Err(EwkbError::InvalidHex { index: 2 }));
        assert_eq!(decode("0G"), Err(EwkbError::InvalidHex { index: 1 }));
    }

    #[test]
    fn a_bad_digit_outranks_an_odd_length() {
        // Byte 2 is `1`, a perfectly good digit; the offender is at 1.
        assert_eq!(decode("0Z1"), Err(EwkbError::InvalidHex { index: 1 }));
    }

    #[test]
    fn the_offset_is_always_a_char_boundary() {
        // `é` is two bytes; the reported index is its first, so a
        // caller slicing `&s[..index]` cannot panic.
        let s = "0é";
        let Err(EwkbError::InvalidHex { index }) = decode(s) else {
            panic!("expected InvalidHex")
        };
        assert_eq!(index, 1);
        assert!(s.is_char_boundary(index));
    }

    #[test]
    fn empty_decodes_to_empty() {
        assert_eq!(decode(""), Ok(Vec::new()));
        assert_eq!(encode(&[]), "");
    }

    #[test]
    fn encode_is_uppercase_per_row_a11() {
        assert_eq!(encode(&[0x00, 0x0F, 0xAB, 0xFF]), "000FABFF");
    }

    #[test]
    fn round_trips_both_cases() {
        let bytes = [0x01u8, 0xE6, 0x10, 0xAB, 0xFF, 0x00];
        let up = encode(&bytes);
        assert_eq!(decode(&up).unwrap(), bytes);
        assert_eq!(decode(&up.to_lowercase()).unwrap(), bytes);
    }
}
