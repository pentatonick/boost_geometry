//! The `SRID=` prefix scanner.
//!
//! Grammar (`PostGIS` manual §4.2.1; the reader is
//! `liblwgeom/lwin_wkt_lex.l`):
//!
//! ```text
//! ewkt        := ws* srid_prefix body | body
//! srid_prefix := "SRID" "=" digits ";"   ; "SRID" case-insensitive; no whitespace inside
//! digits      := [0-9]+                  ; value must fit u32
//! body        := <anything; handed to geometry-io-wkt after normalisation>
//! ```
//!
//! **Claiming rule.** Leading whitespace is skipped with the WKT lexer's
//! own two-tier rule — `u8::is_ascii_whitespace` for an ASCII byte,
//! `char::is_whitespace` for a non-ASCII character — so the scanner
//! accepts exactly the leading whitespace the lexer would. If the maximal
//! run of ASCII letters at the cursor uppercases to exactly `SRID`, the
//! scanner *claims* the input and every deviation from the grammar is
//! [`EwktError::InvalidSrid`]. Otherwise it claims nothing, `body_start`
//! is 0, and the whole string — leading whitespace included — is the
//! body.
//!
//! `PostGIS` permits whitespace around the `;` (the prefix is two tokens,
//! `SRID=-?[0-9]+` and `\;`, joined by a grammar rule) and admits a minus
//! sign in the digits. This scanner is deliberately narrower: no
//! whitespace anywhere inside the prefix, and no sign.

use crate::ewkt_error::EwktError;
use crate::srid::Srid;

/// What the scanner found at the head of the input.
#[derive(Debug, PartialEq)]
pub(crate) struct Scanned {
    /// The prefix's spatial-reference id, or `None` when the input
    /// carried no prefix.
    pub(crate) srid: Option<Srid>,
    /// Byte offset at which the geometry body begins: the byte after
    /// `;`, or 0 when nothing was claimed.
    pub(crate) body_start: usize,
}

/// Build an `InvalidSrid` for `reason` at `pos`.
///
/// The variant carries a position and never a copy of the offending
/// text, so a hostile megabyte after `SRID=` is not duplicated.
fn invalid(reason: &'static str, pos: usize) -> EwktError {
    EwktError::InvalidSrid { reason, pos }
}

/// Scan an optional `SRID=<digits>;` prefix off the head of `input`.
///
/// # Errors
///
/// [`EwktError::InvalidSrid`] when the input is claimed (its leading
/// letter run uppercases to `SRID`) but deviates from the grammar above,
/// or when the digit run exceeds `u32`.
pub(crate) fn scan(input: &str) -> Result<Scanned, EwktError> {
    let bytes = input.as_bytes();

    let mut cursor = 0;
    while let Some(&byte) = bytes.get(cursor) {
        if byte.is_ascii_whitespace() {
            cursor += 1;
        } else if byte.is_ascii() {
            break;
        } else {
            let ch = input[cursor..]
                .chars()
                .next()
                .expect("cursor is inside the input");
            if ch.is_whitespace() {
                cursor += ch.len_utf8();
            } else {
                break;
            }
        }
    }

    let mut pos = cursor;
    while bytes.get(pos).is_some_and(u8::is_ascii_alphabetic) {
        pos += 1;
    }
    if !input[cursor..pos].eq_ignore_ascii_case("SRID") {
        return Ok(Scanned {
            srid: None,
            body_start: 0,
        });
    }

    if bytes.get(pos) == Some(&b'=') {
        pos += 1;
    } else {
        return Err(invalid("expected '='", pos));
    }

    match bytes.get(pos) {
        Some(b'+' | b'-') => return Err(invalid("sign not allowed", pos)),
        Some(byte) if byte.is_ascii_digit() => {}
        _ => return Err(invalid("expected digits", pos)),
    }

    let mut value: u32 = 0;
    while let Some(&byte) = bytes.get(pos) {
        if !byte.is_ascii_digit() {
            break;
        }
        value = value
            .checked_mul(10)
            .and_then(|scaled| scaled.checked_add(u32::from(byte - b'0')))
            .ok_or_else(|| invalid("value exceeds u32", pos))?;
        pos += 1;
    }

    if bytes.get(pos) == Some(&b';') {
        pos += 1;
    } else {
        return Err(invalid("expected ';'", pos));
    }

    Ok(Scanned {
        srid: Some(Srid::new(value)),
        body_start: pos,
    })
}

#[cfg(test)]
mod tests {
    use super::{Scanned, scan};
    use crate::ewkt_error::EwktError;
    use crate::srid::Srid;

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
        assert_eq!(err("SRID=-1;POINT(1 2)"), invalid("sign not allowed", 5));
    }

    #[test]
    fn plus_sign() {
        assert_eq!(err("SRID=+1;POINT(1 2)"), invalid("sign not allowed", 5));
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
        assert_eq!(err("SRID=4326 ;POINT(1 2)"), invalid("expected ';'", 9));
    }

    #[test]
    fn no_equals_at_all() {
        assert_eq!(err("SRID 4326;POINT(1 2)"), invalid("expected '='", 4));
    }

    #[test]
    fn no_semicolon_at_all() {
        assert_eq!(err("SRID=4326 POINT(1 2)"), invalid("expected ';'", 9));
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
            err("SRID=4294967296;POINT(1 2)"),
            invalid("value exceeds u32", 14)
        );
    }

    #[test]
    fn far_past_the_upper_bound() {
        assert_eq!(
            err("SRID=99999999999;POINT(1 2)"),
            invalid("value exceeds u32", 14)
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
        assert_eq!(
            scan("SRID=4294967295;POINT(1 2)"),
            Ok(claimed(u32::MAX, 16))
        );
    }

    #[test]
    fn ascii_whitespace_is_skipped() {
        assert_eq!(scan("  SRID=4326;  POINT(1 2)"), Ok(claimed(4326, 12)));
    }

    #[test]
    fn non_ascii_whitespace_is_skipped() {
        assert_eq!(scan("\u{a0}SRID=4326;POINT(1 2)"), Ok(claimed(4326, 12)));
    }

    #[test]
    fn vertical_tab_is_not_skipped() {
        assert_eq!(scan("\x0bSRID=1;POINT(1 2)"), Ok(unclaimed()));
    }
}
