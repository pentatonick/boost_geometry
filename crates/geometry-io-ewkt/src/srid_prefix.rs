//! The `PostGIS` SRID envelope: checked signed input and bounded output.

use crate::ewkt_error::EwktError;
use crate::ewkt_write_error::EwktWriteError;
use geometry_io_wkt::trim_wkt_start;
use geometry_srid::Srid;

const SRID_MAXIMUM: u32 = 999_999;

#[derive(Debug, PartialEq)]
pub(crate) struct Scanned {
    pub(crate) srid: Option<Srid>,
    pub(crate) body_start: usize,
}

fn invalid(reason: &'static str, pos: usize) -> EwktError {
    EwktError::InvalidSrid { reason, pos }
}

pub(crate) fn scan(input: &str) -> Result<Scanned, EwktError> {
    let bytes = input.as_bytes();
    let start = input.len() - trim_wkt_start(input).len();
    let mut pos = start;
    while bytes.get(pos).is_some_and(u8::is_ascii_alphabetic) {
        pos += 1;
    }
    if !input[start..pos].eq_ignore_ascii_case("SRID") {
        return Ok(Scanned {
            srid: None,
            body_start: 0,
        });
    }
    if bytes.get(pos) != Some(&b'=') {
        return Err(invalid("expected '='", pos));
    }
    pos += 1;
    if bytes.get(pos) == Some(&b'+') {
        return Err(invalid("leading '+' not allowed", pos));
    }
    let negative = bytes.get(pos) == Some(&b'-');
    if negative {
        pos += 1;
    }
    if !bytes.get(pos).is_some_and(u8::is_ascii_digit) {
        return Err(invalid("expected digits", pos));
    }
    let limit = if negative {
        2_147_483_648
    } else {
        2_147_483_647
    };
    let mut value = 0_u32;
    while let Some(&byte) = bytes.get(pos).filter(|b| b.is_ascii_digit()) {
        value = value
            .checked_mul(10)
            .and_then(|v| v.checked_add(u32::from(byte - b'0')))
            .filter(|v| *v <= limit)
            .ok_or_else(|| invalid("value exceeds i32", pos))?;
        pos += 1;
    }
    pos = input.len() - trim_wkt_start(&input[pos..]).len();
    if bytes.get(pos) != Some(&b';') {
        return Err(invalid("expected ';'", pos));
    }
    pos += 1;
    let value = if negative {
        0
    } else if value > SRID_MAXIMUM {
        999_000 + value % 999
    } else {
        value
    };
    Ok(Scanned {
        srid: Some(Srid::new(value)),
        body_start: pos,
    })
}

pub(crate) fn write<W: core::fmt::Write + ?Sized>(
    srid: Option<Srid>,
    out: &mut W,
) -> Result<(), EwktWriteError> {
    if let Some(srid) = srid {
        if srid.get() > SRID_MAXIMUM {
            return Err(EwktWriteError::SridOutOfRange { srid: srid.get() });
        }
        write!(out, "SRID={srid};")?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{Scanned, scan};
    use crate::ewkt_error::EwktError;
    use geometry_srid::Srid;

    /// The scanner's error for `input`, which must be claimed.
    fn err(input: &str) -> EwktError {
        scan(input).expect_err("the input is claimed and malformed")
    }

    /// The expected `InvalidSrid` value.
    fn invalid(reason: &'static str, pos: usize) -> EwktError {
        EwktError::InvalidSrid { reason, pos }
    }

    /// The expected `Scanned` for a claimed prefix.
    fn claimed(code: u32, body_start: usize) -> Scanned {
        Scanned {
            srid: Some(Srid::new(code)),
            body_start,
        }
    }

    /// The expected `Scanned` when nothing is claimed.
    fn unclaimed() -> Scanned {
        Scanned {
            srid: None,
            body_start: 0,
        }
    }

    #[test]
    fn canonical_prefix_and_bare_body() {
        assert_eq!(scan("SRID=4326;POINT(1 2)"), Ok(claimed(4326, 10)));
        assert_eq!(scan("POINT(1 2)"), Ok(unclaimed()));
    }

    // --- error rows of the behaviour matrix ---

    #[test]
    fn digit_glued_to_the_keyword() {
        assert_eq!(err("SRID1=1;POINT(1 2)"), invalid("expected '='", 4));
    }

    #[test]
    fn no_digits_at_all() {
        assert_eq!(err("SRID=;POINT(1 2)"), invalid("expected digits", 5));
    }

    #[test]
    fn minus_sign() {
        assert_eq!(scan("SRID=-1;POINT(1 2)"), Ok(claimed(0, 8)));
    }

    #[test]
    fn plus_sign() {
        assert_eq!(
            err("SRID=+1;POINT(1 2)"),
            invalid("leading '+' not allowed", 5)
        );
    }

    #[test]
    fn space_before_the_equals() {
        assert_eq!(err("SRID = 4326;POINT(1 2)"), invalid("expected '='", 4));
    }

    #[test]
    fn space_after_the_equals() {
        assert_eq!(err("SRID= 4326;POINT(1 2)"), invalid("expected digits", 5));
    }

    #[test]
    fn space_before_the_semicolon() {
        assert_eq!(scan("SRID=4326 ;POINT(1 2)"), Ok(claimed(4326, 11)));
    }

    #[test]
    fn no_equals_at_all() {
        assert_eq!(err("SRID 4326;POINT(1 2)"), invalid("expected '='", 4));
    }

    #[test]
    fn no_semicolon_at_all() {
        assert_eq!(err("SRID=4326 POINT(1 2)"), invalid("expected ';'", 10));
    }

    #[test]
    fn input_ends_before_the_semicolon() {
        assert_eq!(err("SRID=4326"), invalid("expected ';'", 9));
    }

    #[test]
    fn input_ends_before_the_digits() {
        assert_eq!(err("SRID="), invalid("expected digits", 5));
    }

    #[test]
    fn input_ends_after_the_keyword() {
        assert_eq!(err("SRID"), invalid("expected '='", 4));
    }

    #[test]
    fn one_past_the_upper_bound() {
        assert_eq!(
            err("SRID=2147483648;POINT(1 2)"),
            invalid("value exceeds i32", 14)
        );
    }

    #[test]
    fn far_past_the_upper_bound() {
        assert_eq!(
            err("SRID=99999999999;POINT(1 2)"),
            invalid("value exceeds i32", 14)
        );
    }

    // --- success rows ---

    #[test]
    fn lower_case_keyword() {
        assert_eq!(scan("srid=4326;point(1 2)"), Ok(claimed(4326, 10)));
    }

    #[test]
    fn zero_is_claimed() {
        assert_eq!(
            scan("SRID=0;POINT(1 2)"),
            Ok(Scanned {
                srid: Some(Srid::UNKNOWN),
                body_start: 7,
            })
        );
    }

    #[test]
    fn leading_zeros_are_accepted() {
        assert_eq!(scan("SRID=0004326;POINT(1 2)"), Ok(claimed(4326, 13)));
    }

    #[test]
    fn upper_bound_is_accepted() {
        assert_eq!(scan("SRID=2147483647;POINT(1 2)"), Ok(claimed(999_280, 16)));
    }

    #[test]
    fn ascii_whitespace_is_skipped() {
        assert_eq!(scan("  SRID=4326;  POINT(1 2)"), Ok(claimed(4326, 12)));
    }

    #[test]
    fn non_ascii_whitespace_is_not_skipped() {
        assert_eq!(scan("\u{a0}SRID=4326;POINT(1 2)"), Ok(unclaimed()));
    }

    #[test]
    fn vertical_tab_is_not_skipped() {
        assert_eq!(scan("\x0bSRID=1;POINT(1 2)"), Ok(unclaimed()));
    }

    /// Non-ASCII characters prevent the prefix from being claimed.
    #[test]
    fn non_ascii_non_whitespace_is_not_skipped() {
        assert_eq!(scan("\u{e9}SRID=4326;POINT(1 2)"), Ok(unclaimed()));
        assert_eq!(scan("\u{4e2d}SRID=4326;POINT(1 2)"), Ok(unclaimed()));
    }
}
