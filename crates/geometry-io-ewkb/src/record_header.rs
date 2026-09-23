//! EWKB's flag decisions and optional SRID field.
//!
//! WKB owns the record header layout and geometry body. This adapter
//! consumes or inserts an SRID between them without copying the body.

use alloc::vec::Vec;

use geometry_io_wkb::{WkbHeader, split_header};
use geometry_srid::Srid;

use crate::ewkb_error::EwkbError;

/// EWKB's `Z` dimension bit.
const EWKB_Z: u32 = 0x8000_0000;
/// EWKB's `M` dimension bit.
const EWKB_M: u32 = 0x4000_0000;
/// EWKB's SRID-present bit.
const EWKB_SRID: u32 = 0x2000_0000;
/// EWKB's unsupported bounding-box bit.
const EWKB_BBOX: u32 = 0x1000_0000;

/// Bytes in the EWKB SRID field.
const SRID_LEN: usize = 4;

/// The normalized WKB header/body and the SRID removed from them.
pub(crate) struct Header<'a> {
    pub(crate) wkb: WkbHeader<'a>,
    pub(crate) srid: Option<Srid>,
}

/// Reject BBOX, then Z/M, before reading any optional SRID.
pub(crate) fn read(bytes: &[u8]) -> Result<Header<'_>, EwkbError> {
    let mut wkb = split_header(bytes)?;
    let tag = wkb.type_word;
    if tag & EWKB_BBOX != 0 {
        return Err(EwkbError::BoundingBoxFlag { type_word: tag });
    }
    if tag & (EWKB_Z | EWKB_M) != 0 {
        return Err(EwkbError::DimensionFlag { type_word: tag });
    }
    if tag & EWKB_SRID == 0 {
        return Ok(Header { wkb, srid: None });
    }
    let (srid_bytes, body) = wkb
        .body
        .split_first_chunk::<SRID_LEN>()
        .ok_or(EwkbError::TruncatedSrid { type_word: tag })?;
    let srid = Srid::new(wkb.byte_order.read_u32(*srid_bytes));
    // Preserve every other bit for the WKB reader to validate.
    wkb.type_word &= !EWKB_SRID;
    wkb.body = body;
    Ok(Header {
        wkb,
        srid: Some(srid),
    })
}

/// Build an EWKB record carrying `srid`.
///
/// Reserves an output buffer from the length hint and rotates only its
/// nine-byte prefix. The geometry body is neither moved nor copied.
/// The SRID uses the byte order of the record the closure wrote.
///
/// # Panics
///
/// Panics if `write_ogc_record` does not append a valid WKB header —
/// a violation of `WriteWkb`'s append contract.
pub(crate) fn record_with_srid(
    record_hint: usize,
    srid: Srid,
    write_ogc_record: impl FnOnce(&mut Vec<u8>),
) -> Vec<u8> {
    let mut out = Vec::with_capacity(record_hint.saturating_add(SRID_LEN));
    out.extend_from_slice(&[0u8; SRID_LEN]);
    write_ogc_record(&mut out);

    let header = split_header(&out[SRID_LEN..])
        .expect("write_ogc_record appends a complete WKB record: a valid 5-byte header at minimum");
    let header_len = out.len() - SRID_LEN - header.body.len();
    let order = header.byte_order;
    let tag = header.type_word;

    let head = &mut out[..SRID_LEN + header_len];
    // [four placeholders][order][type] -> [order][type][SRID space].
    head.rotate_left(SRID_LEN);
    head[1..header_len].copy_from_slice(&order.to_bytes(tag | EWKB_SRID));
    head[header_len..].copy_from_slice(&order.to_bytes(srid.get()));
    out
}
