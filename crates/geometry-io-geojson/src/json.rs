//! A minimal, self-contained JSON value model, parser, and error type.
//!
//! RFC 7946 `GeoJSON` geometry objects use only a tiny slice of JSON:
//! objects (`{ "type": …, "coordinates": … }`, plus `"geometries"` for a
//! `GeometryCollection`), string keys and type names, `f64` coordinate
//! numbers, and arbitrarily nested arrays. This module supplies exactly
//! that — a hand-written recursive-descent [`parse_json`] over a small
//! [`JsonValue`] enum — so the crate carries **no external JSON
//! dependency**. It is deliberately not a general-purpose JSON library;
//! it handles the grammar RFC 7946 §3 needs and nothing more.
//!
//! Reference: RFC 7946 (the `GeoJSON` media type) §3 (geometry objects)
//! and RFC 8259 (JSON) §2–§9 for the value grammar.

use alloc::string::{String, ToString};
use alloc::vec::Vec;

/// Everything that can go wrong reading `GeoJSON`.
///
/// Covers the JSON layer's character-level failures ([`GeoJsonError::Json`],
/// [`GeoJsonError::UnexpectedEof`]) and the `GeoJSON` layer's structural
/// failures (a missing/invalid `"type"`, an unknown or unsupported kind,
/// malformed `"coordinates"`), so a single error type flows through the
/// whole read path — mirroring the single-error-type shape of the sibling
/// WKT crate's `WktError`.
#[derive(Debug, Clone, PartialEq)]
pub enum GeoJsonError {
    /// A JSON syntax error, with a human-readable description of what went
    /// wrong (unexpected character, bad escape, malformed number, …).
    Json(String),
    /// Input ended while the parser still needed more.
    UnexpectedEof,
    /// A `GeoJSON` object had no string `"type"` member (RFC 7946 §3
    /// requires one).
    ExpectedType,
    /// The `"type"` was not a recognised RFC 7946 geometry kind.
    UnknownGeometryType(String),
    /// A `"coordinates"` (or `"geometries"`) member was missing or did
    /// not have the shape the declared kind requires.
    MalformedCoordinates,
    /// A recognised `GeoJSON` object kind that this reader does not support
    /// — a `Feature` or `FeatureCollection` wrapper (RFC 7946 §3.2/§3.3).
    /// Only bare geometry objects are accepted.
    UnsupportedType(String),
}

impl core::fmt::Display for GeoJsonError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            GeoJsonError::Json(msg) => write!(f, "invalid JSON: {msg}"),
            GeoJsonError::UnexpectedEof => f.write_str("unexpected end of input"),
            GeoJsonError::ExpectedType => f.write_str("missing GeoJSON \"type\" member"),
            GeoJsonError::UnknownGeometryType(s) => {
                write!(f, "unknown GeoJSON geometry type {s:?}")
            }
            GeoJsonError::MalformedCoordinates => f.write_str("malformed or missing coordinates"),
            GeoJsonError::UnsupportedType(s) => write!(f, "unsupported GeoJSON type {s:?}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for GeoJsonError {}

/// A parsed JSON value — the smallest tree that covers RFC 7946 geometry
/// objects.
///
/// Hidden from the public docs: it is a `pub(crate)` implementation
/// detail of [`crate::from_geojson`]. `Object` preserves insertion order
/// (a `Vec` of pairs, not a map) because `GeoJSON` never depends on key
/// ordering and the member count is tiny.
#[doc(hidden)]
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum JsonValue {
    /// JSON `null`.
    Null,
    /// JSON `true` / `false`.
    Bool(bool),
    /// A JSON number, held as `f64` (`GeoJSON` coordinates are `f64`).
    Number(f64),
    /// A JSON string with escapes already decoded.
    Str(String),
    /// A JSON array.
    Array(Vec<JsonValue>),
    /// A JSON object as insertion-ordered `(key, value)` pairs.
    Object(Vec<(String, JsonValue)>),
}

impl JsonValue {
    /// Borrow this value as a `&str` if it is a [`JsonValue::Str`].
    pub(crate) fn as_str(&self) -> Option<&str> {
        match self {
            JsonValue::Str(s) => Some(s),
            _ => None,
        }
    }

    /// Read this value as an `f64` if it is a [`JsonValue::Number`].
    pub(crate) fn as_f64(&self) -> Option<f64> {
        match self {
            JsonValue::Number(n) => Some(*n),
            _ => None,
        }
    }

    /// Borrow this value's elements if it is a [`JsonValue::Array`].
    pub(crate) fn as_array(&self) -> Option<&[JsonValue]> {
        match self {
            JsonValue::Array(a) => Some(a),
            _ => None,
        }
    }

    /// Look up an object member by key, if this value is a
    /// [`JsonValue::Object`]. Returns the first match (`GeoJSON` objects do
    /// not carry duplicate keys).
    pub(crate) fn get(&self, key: &str) -> Option<&JsonValue> {
        match self {
            JsonValue::Object(members) => members.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }
}

/// Parse a JSON document into a [`JsonValue`].
///
/// A hand-written recursive-descent parser over the RFC 8259 grammar,
/// restricted to what `GeoJSON` needs: whitespace, `null`/`true`/`false`,
/// signed decimal / E-notation numbers, strings (with the `\" \\ \/ \n
/// \t` escapes), arrays, and objects. Trailing non-whitespace after a
/// complete value is rejected.
///
/// # Errors
///
/// Returns [`GeoJsonError::Json`] for a syntax error and
/// [`GeoJsonError::UnexpectedEof`] when the input ends mid-value.
pub(crate) fn parse_json(input: &str) -> Result<JsonValue, GeoJsonError> {
    let mut p = JsonParser {
        bytes: input.as_bytes(),
        pos: 0,
    };
    p.skip_ws();
    let value = p.parse_value(0)?;
    p.skip_ws();
    if p.pos != p.bytes.len() {
        return Err(GeoJsonError::Json(
            "trailing characters after value".to_string(),
        ));
    }
    Ok(value)
}

/// Maximum array/object nesting depth accepted while parsing. Recursive
/// descent over adversarial input (e.g. `[[[[…]]]]` tens of thousands
/// deep) would otherwise overflow the native stack and **abort the
/// process** — a stack overflow is not catchable by `catch_unwind`. A
/// bounded depth turns that denial-of-service into a normal recoverable
/// error. `128` matches the default recursion limit `serde_json` enforces
/// for the same reason (`serde_json::de::Deserializer::RECURSION_LIMIT`);
/// real `GeoJSON` nests only a handful of levels deep (a `MultiPolygon`
/// inside a `GeometryCollection` is nowhere near this).
const MAX_DEPTH: usize = 128;

/// A byte cursor over the JSON input. The `GeoJSON` grammar is ASCII apart
/// from string contents, and strings are handled char-by-char, so a
/// `&[u8]` cursor is sufficient and keeps the scan simple.
struct JsonParser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl JsonParser<'_> {
    /// Peek at the current byte without consuming it.
    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    /// Advance past ASCII JSON whitespace.
    fn skip_ws(&mut self) {
        while let Some(b) = self.peek() {
            if b == b' ' || b == b'\t' || b == b'\n' || b == b'\r' {
                self.pos += 1;
            } else {
                break;
            }
        }
    }

    /// Consume the literal `word`, or fail with a syntax error.
    fn expect_literal(&mut self, word: &str, value: JsonValue) -> Result<JsonValue, GeoJsonError> {
        let w = word.as_bytes();
        if self.bytes[self.pos..].starts_with(w) {
            self.pos += w.len();
            Ok(value)
        } else {
            Err(GeoJsonError::Json(alloc::format!("expected `{word}`")))
        }
    }

    /// Parse any JSON value at the current position. `depth` is the
    /// current array/object nesting level; it is checked against
    /// [`MAX_DEPTH`] before descending so adversarial deep nesting fails
    /// with a recoverable error instead of overflowing the stack.
    fn parse_value(&mut self, depth: usize) -> Result<JsonValue, GeoJsonError> {
        match self.peek() {
            None => Err(GeoJsonError::UnexpectedEof),
            Some(b'{') => self.parse_object(depth),
            Some(b'[') => self.parse_array(depth),
            Some(b'"') => Ok(JsonValue::Str(self.parse_string()?)),
            Some(b't') => self.expect_literal("true", JsonValue::Bool(true)),
            Some(b'f') => self.expect_literal("false", JsonValue::Bool(false)),
            Some(b'n') => self.expect_literal("null", JsonValue::Null),
            Some(b) if b == b'-' || b.is_ascii_digit() => self.parse_number(),
            Some(b) => Err(GeoJsonError::Json(alloc::format!(
                "unexpected character {:?}",
                b as char
            ))),
        }
    }

    /// Parse an object: `{ "k": v, … }`.
    fn parse_object(&mut self, depth: usize) -> Result<JsonValue, GeoJsonError> {
        if depth >= MAX_DEPTH {
            return Err(GeoJsonError::Json("nesting too deep".to_string()));
        }
        self.pos += 1; // consume '{'
        let mut members = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b'}') {
            self.pos += 1;
            return Ok(JsonValue::Object(members));
        }
        loop {
            self.skip_ws();
            if self.peek() != Some(b'"') {
                return Err(GeoJsonError::Json("expected string key".to_string()));
            }
            let key = self.parse_string()?;
            self.skip_ws();
            if self.peek() != Some(b':') {
                return Err(GeoJsonError::Json("expected `:` after key".to_string()));
            }
            self.pos += 1;
            self.skip_ws();
            let value = self.parse_value(depth + 1)?;
            members.push((key, value));
            self.skip_ws();
            match self.peek() {
                Some(b',') => self.pos += 1,
                Some(b'}') => {
                    self.pos += 1;
                    return Ok(JsonValue::Object(members));
                }
                Some(_) => {
                    return Err(GeoJsonError::Json("expected `,` or `}`".to_string()));
                }
                None => return Err(GeoJsonError::UnexpectedEof),
            }
        }
    }

    /// Parse an array: `[ v, … ]`.
    fn parse_array(&mut self, depth: usize) -> Result<JsonValue, GeoJsonError> {
        if depth >= MAX_DEPTH {
            return Err(GeoJsonError::Json("nesting too deep".to_string()));
        }
        self.pos += 1; // consume '['
        let mut items = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b']') {
            self.pos += 1;
            return Ok(JsonValue::Array(items));
        }
        loop {
            self.skip_ws();
            items.push(self.parse_value(depth + 1)?);
            self.skip_ws();
            match self.peek() {
                Some(b',') => self.pos += 1,
                Some(b']') => {
                    self.pos += 1;
                    return Ok(JsonValue::Array(items));
                }
                Some(_) => {
                    return Err(GeoJsonError::Json("expected `,` or `]`".to_string()));
                }
                None => return Err(GeoJsonError::UnexpectedEof),
            }
        }
    }

    /// Parse a string literal, decoding the `\" \\ \/ \n \t \r \b \f`
    /// escapes. `\uXXXX` is not needed by `GeoJSON` keys/type names and is
    /// rejected.
    fn parse_string(&mut self) -> Result<String, GeoJsonError> {
        self.pos += 1; // consume opening '"'
        let mut out = String::new();
        loop {
            match self.peek() {
                None => return Err(GeoJsonError::UnexpectedEof),
                Some(b'"') => {
                    self.pos += 1;
                    return Ok(out);
                }
                Some(b'\\') => {
                    self.pos += 1;
                    match self.peek() {
                        None => return Err(GeoJsonError::UnexpectedEof),
                        Some(b'"') => out.push('"'),
                        Some(b'\\') => out.push('\\'),
                        Some(b'/') => out.push('/'),
                        Some(b'n') => out.push('\n'),
                        Some(b't') => out.push('\t'),
                        Some(b'r') => out.push('\r'),
                        Some(b'b') => out.push('\u{0008}'),
                        Some(b'f') => out.push('\u{000C}'),
                        Some(other) => {
                            return Err(GeoJsonError::Json(alloc::format!(
                                "unsupported escape \\{:?}",
                                other as char
                            )));
                        }
                    }
                    self.pos += 1;
                }
                Some(_) => {
                    // Copy one whole UTF-8 character so multi-byte
                    // sequences in string contents survive intact.
                    let rest = &self.bytes[self.pos..];
                    let ch_len = utf8_char_len(rest[0]);
                    let slice = rest.get(..ch_len).ok_or(GeoJsonError::UnexpectedEof)?;
                    let s = core::str::from_utf8(slice)
                        .map_err(|_| GeoJsonError::Json("invalid UTF-8 in string".to_string()))?;
                    out.push_str(s);
                    self.pos += ch_len;
                }
            }
        }
    }

    /// Parse a number: optional sign, integer part, optional fraction,
    /// optional `e`/`E` exponent. The lexeme is handed to Rust's `f64`
    /// parser.
    fn parse_number(&mut self) -> Result<JsonValue, GeoJsonError> {
        let start = self.pos;
        if self.peek() == Some(b'-') {
            self.pos += 1;
        }
        while let Some(b) = self.peek() {
            if b.is_ascii_digit() || b == b'.' || b == b'e' || b == b'E' || b == b'+' || b == b'-' {
                self.pos += 1;
            } else {
                break;
            }
        }
        let slice = &self.bytes[start..self.pos];
        let text = core::str::from_utf8(slice)
            .map_err(|_| GeoJsonError::Json("invalid number".to_string()))?;
        text.parse::<f64>()
            .map(JsonValue::Number)
            .map_err(|_| GeoJsonError::Json(alloc::format!("invalid number {text:?}")))
    }
}

/// Byte length of the UTF-8 character that starts with `first`.
fn utf8_char_len(first: u8) -> usize {
    if first < 0x80 {
        1
    } else if first < 0xE0 {
        2
    } else if first < 0xF0 {
        3
    } else {
        4
    }
}

#[cfg(test)]
mod tests {
    //! Round-trips a small object, nested arrays, escapes, and the
    //! numeric forms; plus one malformed fixture per JSON error category.
    #![allow(
        clippy::float_cmp,
        reason = "number literals in these fixtures are exact"
    )]

    use super::{GeoJsonError, JsonValue, parse_json};
    use alloc::string::ToString;

    #[test]
    fn parses_object_with_array_and_string() {
        let v = parse_json(r#"{"a":[1,2.5,-3e2],"b":"x"}"#).unwrap();
        let arr = v.get("a").unwrap().as_array().unwrap();
        assert_eq!(arr.len(), 3);
        assert_eq!(arr[0].as_f64(), Some(1.0));
        assert_eq!(arr[1].as_f64(), Some(2.5));
        assert_eq!(arr[2].as_f64(), Some(-300.0));
        assert_eq!(v.get("b").unwrap().as_str(), Some("x"));
    }

    #[test]
    fn parses_deeply_nested_arrays() {
        let v = parse_json("[[[1,2]],[[3,4]]]").unwrap();
        let outer = v.as_array().unwrap();
        assert_eq!(outer.len(), 2);
        let inner = outer[1].as_array().unwrap()[0].as_array().unwrap();
        assert_eq!(inner[0].as_f64(), Some(3.0));
        assert_eq!(inner[1].as_f64(), Some(4.0));
    }

    #[test]
    fn decodes_string_escapes() {
        let v = parse_json(r#""a\"b\\c\/d\ne""#).unwrap();
        assert_eq!(v.as_str(), Some("a\"b\\c/d\ne"));
    }

    #[test]
    fn parses_keywords() {
        assert_eq!(parse_json("true").unwrap(), JsonValue::Bool(true));
        assert_eq!(parse_json("false").unwrap(), JsonValue::Bool(false));
        assert_eq!(parse_json("null").unwrap(), JsonValue::Null);
    }

    #[test]
    fn rejects_trailing_characters() {
        let err = parse_json("{} junk").unwrap_err();
        assert_eq!(
            err,
            GeoJsonError::Json("trailing characters after value".to_string())
        );
    }

    #[test]
    fn reports_eof_on_truncated_input() {
        assert_eq!(
            parse_json("[1, 2").unwrap_err(),
            GeoJsonError::UnexpectedEof
        );
    }

    #[test]
    fn rejects_malformed_number() {
        assert!(matches!(
            parse_json("1.2.3").unwrap_err(),
            GeoJsonError::Json(_)
        ));
    }

    #[test]
    fn rejects_deeply_nested_input_without_overflow() {
        // Regression: recursive descent over adversarial deep nesting must
        // fail with a recoverable error, NOT overflow the native stack
        // (which aborts the process uncatchably). 100k-deep arrays and
        // objects both used to `SIGABRT`; now they return a `Json` error.
        let deep_arrays = "[".repeat(100_000);
        assert!(matches!(
            parse_json(&deep_arrays).unwrap_err(),
            GeoJsonError::Json(_)
        ));

        let mut deep_objects = String::new();
        for _ in 0..100_000 {
            deep_objects.push_str("{\"a\":");
        }
        assert!(matches!(
            parse_json(&deep_objects).unwrap_err(),
            GeoJsonError::Json(_)
        ));

        // A modest, legitimate nesting depth still parses fine.
        let ok = "[".repeat(64) + "1" + &"]".repeat(64);
        assert!(parse_json(&ok).is_ok());
    }
}
