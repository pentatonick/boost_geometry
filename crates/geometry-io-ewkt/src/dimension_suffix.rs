//! The glued dimension-suffix normaliser.
//!
//! `ST_AsEWKT` glues only `M` (`POINTM(1 2 3)`), but `PostGIS` reads the
//! glued `Z`, `M`, and `ZM` spellings as readily as the OGC-spaced
//! `POINT Z` forms, because there the suffix is its own token. The WKT
//! crate's lexer, by contrast, produces `Ident("POINTM")` and its parser
//! reports `UnknownGeometryType("POINTM")`.
//!
//! So the body is walked once before it is delegated, and every maximal
//! run of ASCII letters that uppercases to one of the seven geometry
//! keywords immediately followed by `Z`, `M`, or `ZM` has those trailing
//! suffix bytes overwritten with ASCII spaces: `POINTM(1 2 3)` becomes
//! `POINT (1 2 3)`, `POINTZM(1 2 3 4)` becomes `POINT  (1 2 3 4)`.
//!
//! Overwriting rather than inserting keeps the output the same byte
//! length as the input, which is what makes the error-offset rebasing in
//! `ewkt.rs` exact. Nothing is lost: the WKT crate reads only the first
//! two ordinates of every coordinate and discards the rest whether or not
//! a dimension word is present, so `POINT (1 2 3)` and `POINT M (1 2 3)`
//! parse to the same value.
//!
//! The rewrite is safe because it is anchored on the whole run. WKT has
//! no string literals, comments, or escapes; the only letters the lexer
//! folds into a non-`Ident` lexeme are the exponent markers `e`/`E`
//! inside a number and the word `EMPTY`, and neither can form a
//! keyword-plus-suffix run (`EMPTYM` is left alone and rejected). Every
//! other letter run is an `Ident`, which the WKT parser accepts only in
//! the keyword slot or as a free-standing `Z`/`M`/`ZM` suffix — so a run
//! this pass rewrites can only ever have been an unknown keyword the WKT
//! crate would reject. The rewrite can turn a rejection into an
//! acceptance, never change an accepted parse. `POINTMM`, `MPOINT`,
//! `POINTX`, `POINTZZ`, and a free-standing `ZM` are therefore untouched.
//!
//! One known widening: `POINTZ M (1 2 3 4)` becomes `POINT  M (1 2 3 4)`,
//! which is valid WKT. `PostGIS` rejects the original (one dimension
//! token per grammar rule) and so does the WKT crate; it is accepted here
//! in exchange for a normaliser with no lookahead, and is harmless in 2D.

use alloc::borrow::Cow;
use alloc::string::String;
use alloc::vec::Vec;

/// The seven OGC geometry keywords each followed by `Z`, `M`, or `ZM`,
/// paired with the byte length of the suffix to blank.
const GLUED_SPELLINGS: [(&str, usize); 21] = [
    ("POINTZ", 1),
    ("POINTM", 1),
    ("POINTZM", 2),
    ("LINESTRINGZ", 1),
    ("LINESTRINGM", 1),
    ("LINESTRINGZM", 2),
    ("POLYGONZ", 1),
    ("POLYGONM", 1),
    ("POLYGONZM", 2),
    ("MULTIPOINTZ", 1),
    ("MULTIPOINTM", 1),
    ("MULTIPOINTZM", 2),
    ("MULTILINESTRINGZ", 1),
    ("MULTILINESTRINGM", 1),
    ("MULTILINESTRINGZM", 2),
    ("MULTIPOLYGONZ", 1),
    ("MULTIPOLYGONM", 1),
    ("MULTIPOLYGONZM", 2),
    ("GEOMETRYCOLLECTIONZ", 1),
    ("GEOMETRYCOLLECTIONM", 1),
    ("GEOMETRYCOLLECTIONZM", 2),
];

/// The suffix byte length to blank if `run` is one of the 21 glued
/// spellings, matched case-insensitively against the whole run.
fn glued_suffix_len(run: &str) -> Option<usize> {
    GLUED_SPELLINGS.iter().find_map(|&(spelling, suffix_len)| {
        run.eq_ignore_ascii_case(spelling).then_some(suffix_len)
    })
}

/// Blank the glued dimension suffix of every geometry keyword in `body`.
///
/// The result always has the same byte length as `body`, and is borrowed
/// — no allocation at all — when `body` contains no glued suffix, which
/// is the overwhelmingly common case.
pub(crate) fn normalise(body: &str) -> Cow<'_, str> {
    let bytes = body.as_bytes();
    let mut rewritten: Option<Vec<u8>> = None;
    let mut pos = 0;

    while pos < bytes.len() {
        if !bytes[pos].is_ascii_alphabetic() {
            pos += 1;
            continue;
        }
        let start = pos;
        while bytes.get(pos).is_some_and(u8::is_ascii_alphabetic) {
            pos += 1;
        }
        if let Some(suffix_len) = glued_suffix_len(&body[start..pos]) {
            let target = rewritten.get_or_insert_with(|| bytes.to_vec());
            for byte in &mut target[pos - suffix_len..pos] {
                *byte = b' ';
            }
        }
    }

    match rewritten {
        None => Cow::Borrowed(body),
        Some(bytes) => Cow::Owned(
            String::from_utf8(bytes).expect("only ASCII bytes were replaced by ASCII spaces"),
        ),
    }
}

#[cfg(test)]
mod tests {
    use alloc::borrow::Cow;

    use super::normalise;

    /// Assert the rewrite and that it preserved every byte offset.
    fn assert_rewrite(input: &str, expected: &str) {
        let out = normalise(input);
        assert_eq!(out, expected, "rewrite of {input:?}");
        assert_eq!(out.len(), input.len(), "length changed for {input:?}");
    }

    /// Assert the run is not a glued spelling, so nothing is copied.
    fn assert_untouched(input: &str) {
        let out = normalise(input);
        assert_eq!(out, input, "rewrite of {input:?}");
        assert_eq!(out.len(), input.len(), "length changed for {input:?}");
        assert!(
            matches!(out, Cow::Borrowed(_)),
            "{input:?} allocated needlessly"
        );
    }

    #[test]
    fn point_suffixes() {
        assert_rewrite("POINTZ(1 2 3)", "POINT (1 2 3)");
        assert_rewrite("POINTM(1 2 3)", "POINT (1 2 3)");
        assert_rewrite("POINTZM(1 2 3 4)", "POINT  (1 2 3 4)");
    }

    #[test]
    fn linestring_suffixes() {
        assert_rewrite("LINESTRINGZ(1 2 3)", "LINESTRING (1 2 3)");
        assert_rewrite("LINESTRINGM(1 2 3)", "LINESTRING (1 2 3)");
        assert_rewrite("LINESTRINGZM(1 2 3 4)", "LINESTRING  (1 2 3 4)");
    }

    #[test]
    fn polygon_suffixes() {
        assert_rewrite("POLYGONZ((1 2 3))", "POLYGON ((1 2 3))");
        assert_rewrite("POLYGONM((1 2 3))", "POLYGON ((1 2 3))");
        assert_rewrite("POLYGONZM((1 2 3 4))", "POLYGON  ((1 2 3 4))");
    }

    #[test]
    fn multi_point_suffixes() {
        assert_rewrite("MULTIPOINTZ(1 2 3)", "MULTIPOINT (1 2 3)");
        assert_rewrite("MULTIPOINTM(1 2 3)", "MULTIPOINT (1 2 3)");
        assert_rewrite("MULTIPOINTZM(1 2 3 4)", "MULTIPOINT  (1 2 3 4)");
    }

    #[test]
    fn multi_linestring_suffixes() {
        assert_rewrite("MULTILINESTRINGZ((1 2 3))", "MULTILINESTRING ((1 2 3))");
        assert_rewrite("MULTILINESTRINGM((1 2 3))", "MULTILINESTRING ((1 2 3))");
        assert_rewrite(
            "MULTILINESTRINGZM((1 2 3 4))",
            "MULTILINESTRING  ((1 2 3 4))",
        );
    }

    #[test]
    fn multi_polygon_suffixes() {
        assert_rewrite("MULTIPOLYGONZ(((1 2 3)))", "MULTIPOLYGON (((1 2 3)))");
        assert_rewrite("MULTIPOLYGONM(((1 2 3)))", "MULTIPOLYGON (((1 2 3)))");
        assert_rewrite("MULTIPOLYGONZM(((1 2 3 4)))", "MULTIPOLYGON  (((1 2 3 4)))");
    }

    #[test]
    fn geometry_collection_suffixes() {
        assert_rewrite(
            "GEOMETRYCOLLECTIONZ(POINT(1 2 3))",
            "GEOMETRYCOLLECTION (POINT(1 2 3))",
        );
        assert_rewrite(
            "GEOMETRYCOLLECTIONM(POINT(1 2 3))",
            "GEOMETRYCOLLECTION (POINT(1 2 3))",
        );
        assert_rewrite(
            "GEOMETRYCOLLECTIONZM(POINT(1 2 3 4))",
            "GEOMETRYCOLLECTION  (POINT(1 2 3 4))",
        );
    }

    #[test]
    fn nested_runs_are_rewritten() {
        assert_rewrite(
            "GEOMETRYCOLLECTIONM(POINTM(1 2 3))",
            "GEOMETRYCOLLECTION (POINT (1 2 3))",
        );
    }

    #[test]
    fn case_is_folded_like_the_lexer() {
        assert_rewrite("pointm(1 2 3)", "point (1 2 3)");
    }

    #[test]
    fn runs_left_alone() {
        assert_untouched("POINTMM(1 2 3)");
        assert_untouched("MPOINT(1 2)");
        assert_untouched("POINTX(1 2)");
        assert_untouched("POINTZZ(1 2 3)");
        assert_untouched("EMPTYM");
        assert_untouched("ZM");
        assert_untouched("POINT M (1 2 3)");
    }

    #[test]
    fn non_ascii_bytes_keep_their_offset() {
        let input = "POINTM(1 é)";
        assert_rewrite(input, "POINT (1 é)");
        assert_eq!(input.find('é'), Some(9));
        assert_eq!(normalise(input).find('é'), Some(9));
    }

    #[test]
    fn plain_wkt_is_borrowed() {
        assert!(matches!(normalise("POINT(1 2)"), Cow::Borrowed(_)));
    }
}
