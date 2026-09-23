//! Whitespace admitted by the `PostGIS` text grammar.

/// Remove leading space, tab, LF and CR without accepting Unicode whitespace.
///
/// Dialect envelopes use the same rule as the WKT body lexer.
///
/// ```
/// use geometry_io_wkt::trim_wkt_start;
/// assert_eq!(trim_wkt_start(" \tPOINT(1 2)"), "POINT(1 2)");
/// assert_eq!(trim_wkt_start("\u{a0}POINT(1 2)"), "\u{a0}POINT(1 2)");
/// ```
#[must_use]
pub fn trim_wkt_start(input: &str) -> &str {
    input.trim_start_matches([' ', '\t', '\n', '\r'])
}
