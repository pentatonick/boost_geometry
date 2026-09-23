//! Tokenization for the `PostGIS`-compatible XY text profile.

use crate::wkt_error::WktError;
use alloc::string::{String, ToString};
#[cfg(test)]
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

/// One-token-at-a-time scanning preserves positions in the original input.
pub(crate) struct Lexer<'a> {
    input: &'a str,
    pos: usize,
    token_start: usize,
}

impl<'a> Lexer<'a> {
    pub(crate) fn new(input: &'a str) -> Self {
        Self {
            input,
            pos: 0,
            token_start: 0,
        }
    }

    pub(crate) fn token_start(&self) -> usize {
        self.token_start
    }

    pub(crate) fn next_token(&mut self) -> Result<Token, WktError> {
        let bytes = self.input.as_bytes();
        self.pos =
            self.input.len() - crate::whitespace::trim_wkt_start(&self.input[self.pos..]).len();
        self.token_start = self.pos;
        let Some(&byte) = bytes.get(self.pos) else {
            return Ok(Token::Eof);
        };
        match byte {
            b'(' => {
                self.pos += 1;
                Ok(Token::LeftParen)
            }
            b')' => {
                self.pos += 1;
                Ok(Token::RightParen)
            }
            b',' => {
                self.pos += 1;
                Ok(Token::Comma)
            }
            b'+' | b'-' | b'.' | b'0'..=b'9' => self.number(),
            b'A'..=b'Z' | b'a'..=b'z' => self.word(),
            _ => Err(self.unexpected_char()),
        }
    }

    fn starts_with(&self, word: &str) -> bool {
        self.input.as_bytes()[self.pos..]
            .get(..word.len())
            .is_some_and(|prefix| prefix.eq_ignore_ascii_case(word.as_bytes()))
    }

    fn word(&mut self) -> Result<Token, WktError> {
        if self.starts_with("NAN") {
            self.pos += 3;
            self.number_end()?;
            return Ok(Token::Number(f64::NAN));
        }
        // PostGIS recognizes literal tokens even without an intervening space:
        // POINTEMPTY and POINTZM are POINT + EMPTY and POINT + ZM, respectively.
        for word in [
            "GEOMETRYCOLLECTION",
            "MULTILINESTRING",
            "MULTIPOLYGON",
            "MULTIPOINT",
            "LINESTRING",
            "POLYGON",
            "POINT",
            "EMPTY",
            "ZM",
            "Z",
            "M",
        ] {
            if self.starts_with(word) {
                self.pos += word.len();
                return Ok(if word == "EMPTY" {
                    Token::Empty
                } else {
                    Token::Ident(word.into())
                });
            }
        }
        let start = self.pos;
        while self
            .input
            .as_bytes()
            .get(self.pos)
            .is_some_and(u8::is_ascii_alphabetic)
        {
            self.pos += 1;
        }
        Ok(Token::Ident(
            self.input[start..self.pos].to_ascii_uppercase(),
        ))
    }

    fn unexpected_char(&self) -> WktError {
        WktError::UnexpectedChar {
            pos: self.pos,
            ch: self.input[self.pos..]
                .chars()
                .next()
                .expect("position is inside input"),
        }
    }

    fn number_end(&self) -> Result<(), WktError> {
        match self.input.as_bytes().get(self.pos) {
            None | Some(b',' | b')') => Ok(()),
            Some(_)
                if crate::whitespace::trim_wkt_start(&self.input[self.pos..]).len()
                    < self.input.len() - self.pos =>
            {
                Ok(())
            }
            _ => Err(self.unexpected_char()),
        }
    }

    fn number(&mut self) -> Result<Token, WktError> {
        let start = self.pos;
        while self
            .input
            .as_bytes()
            .get(self.pos)
            .is_some_and(|b| b.is_ascii_digit() || matches!(b, b'.' | b'+' | b'-' | b'e' | b'E'))
        {
            self.pos += 1;
        }
        let literal = &self.input[start..self.pos];
        if !decimal_literal(literal) {
            return Err(WktError::InvalidNumber(literal.to_string()));
        }
        self.number_end()?;
        let unsigned = literal.strip_prefix('-').unwrap_or(literal);
        // The integer fast path preserves signed zero and avoids float parsing
        // for the ordinary small integer coordinates common in geometry data.
        if let Ok(integer) = unsigned.parse::<u64>() {
            #[allow(
                clippy::cast_precision_loss,
                reason = "IEEE-754 integer rounding matches decimal parsing"
            )]
            let value = integer as f64;
            return Ok(Token::Number(if literal.starts_with('-') {
                -value
            } else {
                value
            }));
        }
        let value: f64 = literal
            .parse()
            .map_err(|_| WktError::InvalidNumber(literal.to_string()))?;
        if value.is_infinite() {
            return Err(WktError::NumberOutOfRange {
                pos: start,
                literal: literal.to_string(),
            });
        }
        Ok(Token::Number(value))
    }
}

/// `PostGIS` decimal spelling: minus is allowed, plus only in an exponent;
/// a trailing decimal point is allowed only without an exponent.
fn decimal_literal(input: &str) -> bool {
    let bytes = input.strip_prefix('-').unwrap_or(input).as_bytes();
    let mut pos = 0;
    while bytes.get(pos).is_some_and(u8::is_ascii_digit) {
        pos += 1;
    }
    let integer_digits = pos;
    let mut fraction_digits = 0;
    let has_dot = bytes.get(pos) == Some(&b'.');
    if has_dot {
        pos += 1;
        let start = pos;
        while bytes.get(pos).is_some_and(u8::is_ascii_digit) {
            pos += 1;
        }
        fraction_digits = pos - start;
    }
    if integer_digits + fraction_digits == 0 {
        return false;
    }
    if matches!(bytes.get(pos), Some(b'e' | b'E')) {
        if has_dot && fraction_digits == 0 {
            return false;
        }
        pos += 1;
        if matches!(bytes.get(pos), Some(b'+' | b'-')) {
            pos += 1;
        }
        let start = pos;
        while bytes.get(pos).is_some_and(u8::is_ascii_digit) {
            pos += 1;
        }
        if start == pos {
            return false;
        }
    }
    pos == bytes.len()
}

#[cfg(test)]
pub(crate) fn tokenize(input: &str) -> Result<Vec<Token>, WktError> {
    let mut lexer = Lexer::new(input);
    let mut tokens = Vec::new();
    loop {
        let token = lexer.next_token()?;
        let done = token == Token::Eof;
        tokens.push(token);
        if done {
            return Ok(tokens);
        }
    }
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
        let toks = tokenize("-10 20.5 3").unwrap();
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
    fn integral_fast_path_matches_standard_float_rounding() {
        for literal in [
            "0",
            "-0",
            "9007199254740993",
            "18446744073709551615",
            "-18446744073709551615",
        ] {
            let expected = literal.parse::<f64>().unwrap();
            let tokens = tokenize(literal).unwrap();
            assert!(
                matches!(&tokens[0], Token::Number(actual) if actual.to_bits() == expected.to_bits()),
                "literal {literal}: expected number token {expected:?}"
            );
        }
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

    /// An exponent too large for `f64` is rejected rather than silently
    /// becoming an infinity. `str::parse` returns `Ok(inf)` for these, so
    /// without the finiteness check `POINT(1e400 1)` would enter the
    /// model as `+inf` and come back out as the unparseable
    /// `POINT(inf 1)`.
    #[test]
    fn overflowing_exponent_is_rejected_not_infinity() {
        for literal in ["1e400", "-1e400", "1.5e309"] {
            assert_eq!(
                tokenize(literal).unwrap_err(),
                WktError::NumberOutOfRange {
                    pos: 0,
                    literal: literal.into()
                },
                "literal {literal}"
            );
        }
    }

    /// Underflow is not an error: it rounds to a finite zero, which WKT
    /// can spell and which round-trips.
    #[test]
    fn underflowing_exponent_is_a_finite_zero() {
        let tokens = tokenize("1e-400").unwrap();
        assert!(matches!(tokens[0], Token::Number(v) if v == 0.0));
    }

    /// Every `WktError` variant renders a distinct message through its
    /// `Display` impl, embedding its payload.
    #[test]
    fn every_error_variant_displays_descriptively() {
        use alloc::format;

        assert_eq!(
            format!("{}", WktError::UnexpectedChar { pos: 9, ch: '@' }),
            "unexpected character '@' at byte 9"
        );
        assert_eq!(
            format!(
                "{}",
                WktError::UnexpectedToken {
                    expected: "'('",
                    found: "Comma".into()
                }
            ),
            "expected '(', found Comma"
        );
        assert_eq!(
            format!("{}", WktError::UnexpectedEof),
            "unexpected end of input"
        );
        assert_eq!(
            format!("{}", WktError::InvalidNumber("1.2.3".into())),
            "invalid number \"1.2.3\""
        );
        assert_eq!(
            format!("{}", WktError::UnknownGeometryType("TRIANGLE".into())),
            "unknown geometry type \"TRIANGLE\""
        );
        assert_eq!(
            format!(
                "{}",
                WktError::TypeMismatch {
                    expected: "POINT",
                    found: "LINESTRING"
                }
            ),
            "type mismatch: expected POINT, found LINESTRING"
        );
        assert!(
            format!("{}", WktError::NestingTooDeep).contains("nesting too deep"),
            "nesting message"
        );
    }
}
