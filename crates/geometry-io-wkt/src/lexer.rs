//! The WKT tokenizer and its error type.
//!
//! Mirrors the `tokenizer` block in `boost/geometry/io/wkt/read.hpp`
//! (Boost drives the read with `boost::tokenizer` over a small set of
//! separators). This port hand-rolls the scan so it can classify each
//! lexeme into a [`Token`] up front — identifiers (uppercased keywords
//! such as `POINT`, `LINESTRING`, the OGC `Z`/`M`/`ZM` dimension
//! suffixes, and `EMPTY`), signed decimal / E-notation numbers, and the
//! `(`, `)`, `,` punctuation the grammar uses.
//!
//! Reference: OGC Simple Feature Access Part 1 §7 for the WKT grammar,
//! and `boost/geometry/io/wkt/read.hpp` for the C++ tokenizer.

use alloc::string::{String, ToString};
use alloc::vec::Vec;

/// One lexeme of a WKT string.
///
/// Mirrors the token classes the `boost::tokenizer` in
/// `boost/geometry/io/wkt/read.hpp` separates on: a keyword/identifier,
/// a number, the three punctuation marks, and end-of-input. `EMPTY` is
/// broken out as its own token (rather than folded into `Ident`) so the
/// parser can branch on `<TYPE> EMPTY` without re-comparing strings.
#[derive(Debug, Clone, PartialEq)]
pub enum Token {
    /// An uppercased keyword — a geometry type (`POINT`, `LINESTRING`,
    /// …) or an OGC dimension suffix (`Z`, `M`, `ZM`).
    Ident(String),
    /// A numeric literal, already parsed to `f64` (signed, decimal,
    /// E-notation).
    Number(f64),
    /// `(`
    LeftParen,
    /// `)`
    RightParen,
    /// `,`
    Comma,
    /// The `EMPTY` keyword.
    Empty,
    /// End of input.
    Eof,
}

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
            WktError::UnknownGeometryType(s) => write!(f, "unknown geometry type {s:?}"),
            WktError::TypeMismatch { expected, found } => {
                write!(f, "type mismatch: expected {expected}, found {found}")
            }
            WktError::NestingTooDeep => {
                f.write_str("WKT nesting too deep; exceeded the reader's recursion limit")
            }
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for WktError {}

/// Split a WKT string into its token stream.
///
/// Whitespace separates tokens and is otherwise discarded. Identifiers
/// (ASCII letters) are uppercased so `Point`, `POINT`, and `point` all
/// lex to the same keyword — matching the case-insensitivity Boost gets
/// from comparing against upper-cased keyword tables in
/// `boost/geometry/io/wkt/read.hpp`. Numbers accept an optional sign, a
/// decimal point, and an `e`/`E` exponent (e.g. `1.5e-3`). The stream
/// always ends with a single [`Token::Eof`].
///
/// # Errors
///
/// Returns [`WktError::UnexpectedChar`] for a character that cannot
/// start any lexeme, and [`WktError::InvalidNumber`] for a numeric
/// lexeme that fails to parse as `f64`.
pub(crate) fn tokenize(input: &str) -> Result<Vec<Token>, WktError> {
    let mut tokens = Vec::new();
    // Index over `char_indices` so error positions are byte offsets and
    // multi-byte characters are handled correctly.
    let mut chars = input.char_indices().peekable();

    while let Some(&(pos, ch)) = chars.peek() {
        if ch.is_whitespace() {
            chars.next();
        } else if ch == '(' {
            chars.next();
            tokens.push(Token::LeftParen);
        } else if ch == ')' {
            chars.next();
            tokens.push(Token::RightParen);
        } else if ch == ',' {
            chars.next();
            tokens.push(Token::Comma);
        } else if ch.is_ascii_alphabetic() {
            let mut word = String::new();
            while let Some(&(_, c)) = chars.peek() {
                if c.is_ascii_alphabetic() {
                    word.push(c.to_ascii_uppercase());
                    chars.next();
                } else {
                    break;
                }
            }
            if word == "EMPTY" {
                tokens.push(Token::Empty);
            } else {
                tokens.push(Token::Ident(word));
            }
        } else if ch == '+' || ch == '-' || ch == '.' || ch.is_ascii_digit() {
            let start = pos;
            let mut end = pos + ch.len_utf8();
            chars.next();
            while let Some(&(p, c)) = chars.peek() {
                if c.is_ascii_digit() || c == '.' || c == '+' || c == '-' || c == 'e' || c == 'E' {
                    end = p + c.len_utf8();
                    chars.next();
                } else {
                    break;
                }
            }
            let slice = &input[start..end];
            let value: f64 = slice
                .parse()
                .map_err(|_| WktError::InvalidNumber(slice.to_string()))?;
            tokens.push(Token::Number(value));
        } else {
            return Err(WktError::UnexpectedChar { pos, ch });
        }
    }

    tokens.push(Token::Eof);
    Ok(tokens)
}

#[cfg(test)]
mod tests {
    //! One case per token kind plus a malformed-input fixture per lexer
    //! error category. Mirrors the tokenizer coverage in
    //! `boost/geometry/test/io/wkt/wkt.cpp`.
    #![allow(
        clippy::float_cmp,
        reason = "number tokens come from exact integer / short-decimal WKT literals"
    )]

    use super::{Token, WktError, tokenize};
    use alloc::vec;

    #[test]
    fn each_token_kind() {
        let toks = tokenize("POINT ( 1 , 2 )").unwrap();
        assert_eq!(
            toks,
            vec![
                Token::Ident("POINT".into()),
                Token::LeftParen,
                Token::Number(1.0),
                Token::Comma,
                Token::Number(2.0),
                Token::RightParen,
                Token::Eof,
            ]
        );
    }

    #[test]
    fn identifiers_are_uppercased() {
        let toks = tokenize("LineString").unwrap();
        assert_eq!(toks, vec![Token::Ident("LINESTRING".into()), Token::Eof]);
    }

    #[test]
    fn empty_is_its_own_token() {
        let toks = tokenize("POINT EMPTY").unwrap();
        assert_eq!(
            toks,
            vec![Token::Ident("POINT".into()), Token::Empty, Token::Eof]
        );
    }

    #[test]
    fn dimension_suffix_lexes_as_ident() {
        let toks = tokenize("POINT ZM").unwrap();
        assert_eq!(
            toks,
            vec![
                Token::Ident("POINT".into()),
                Token::Ident("ZM".into()),
                Token::Eof,
            ]
        );
    }

    #[test]
    fn number_e_notation() {
        let toks = tokenize("1.5e-3").unwrap();
        assert_eq!(toks, vec![Token::Number(0.0015), Token::Eof]);
    }

    #[test]
    fn number_signed_and_decimal() {
        let toks = tokenize("-10 20.5 +3").unwrap();
        assert_eq!(
            toks,
            vec![
                Token::Number(-10.0),
                Token::Number(20.5),
                Token::Number(3.0),
                Token::Eof,
            ]
        );
    }

    #[test]
    fn malformed_char_reports_position() {
        let err = tokenize("POINT (1 @)").unwrap_err();
        assert_eq!(err, WktError::UnexpectedChar { pos: 9, ch: '@' });
    }

    #[test]
    fn malformed_number_reports_slice() {
        let err = tokenize("1.2.3").unwrap_err();
        assert_eq!(err, WktError::InvalidNumber("1.2.3".into()));
    }
}
