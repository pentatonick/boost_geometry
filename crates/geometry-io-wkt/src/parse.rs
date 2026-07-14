//! The WKT recursive-descent parser and the typed-parse conveniences.
//!
//! Mirrors `boost/geometry/io/wkt/read.hpp` — the C++ side dispatches on
//! the leading keyword to a per-kind reader (`point_parser`,
//! `linestring_parser`, `polygon_parser`, and the multi variants) that
//! walks the same parenthesised coordinate grammar. This port emits a
//! [`DynGeometry`] because WKT is heterogeneous by construction (a
//! `GEOMETRYCOLLECTION` mixes kinds).
//!
//! Reference: OGC Simple Feature Access Part 1 §7 (grammar) and §6.1.10
//! (worked examples).

use alloc::format;
use alloc::vec::Vec;

use geometry_cs::Cartesian;
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};

use crate::lexer::{Lexer, Token, WktError};

/// A concrete 2D Cartesian point — the coordinate type every parsed
/// geometry is built from.
type Pt = Point2D<f64, Cartesian>;

/// Maximum `GEOMETRYCOLLECTION` nesting depth accepted while parsing.
/// `parse_collection_body` recurses through `parse_geometry` per member,
/// so an adversarial string of tens of thousands of nested
/// `GEOMETRYCOLLECTION(` headers would otherwise overflow the native
/// stack and **abort the process** (a stack overflow is not catchable).
/// A bounded depth turns that denial-of-service into a recoverable
/// [`WktError::NestingTooDeep`]. `128` mirrors the sibling WKB / `GeoJSON`
/// readers' cap; real WKT nests only a few levels deep.
const MAX_DEPTH: usize = 128;

/// A one-token lookahead cursor over the input. Every `parse_*` method
/// advances it, and tokens are scanned only as the grammar consumes them.
struct Parser<'a> {
    lexer: Lexer<'a>,
    current: Token,
}

impl<'a> Parser<'a> {
    fn new(input: &'a str) -> Result<Self, WktError> {
        let mut lexer = Lexer::new(input);
        let current = lexer.next_token()?;
        Ok(Self { lexer, current })
    }

    /// Borrow the current token without consuming it.
    fn peek(&self) -> &Token {
        &self.current
    }

    /// Consume and return the current token. The cursor is strictly
    /// monotonic, so the vacated slot is never read again; `Eof` is the
    /// placeholder because a stray re-read then fails exactly like
    /// running off the end of the stream.
    fn next(&mut self) -> Result<Token, WktError> {
        let next = self.lexer.next_token()?;
        Ok(core::mem::replace(&mut self.current, next))
    }

    fn advance(&mut self) -> Result<(), WktError> {
        self.next().map(drop)
    }

    /// Consume a `(`, or fail.
    fn expect_left_paren(&mut self) -> Result<(), WktError> {
        match self.next()? {
            Token::LeftParen => Ok(()),
            other => Err(WktError::UnexpectedToken {
                expected: "'('",
                found: format!("{other:?}"),
            }),
        }
    }

    /// Consume a `)`, or fail.
    fn expect_right_paren(&mut self) -> Result<(), WktError> {
        match self.next()? {
            Token::RightParen => Ok(()),
            other => Err(WktError::UnexpectedToken {
                expected: "')'",
                found: format!("{other:?}"),
            }),
        }
    }

    /// Consume one `f64` numeric token, or fail.
    fn expect_number(&mut self) -> Result<f64, WktError> {
        match self.next()? {
            Token::Number(n) => Ok(n),
            Token::Eof => Err(WktError::UnexpectedEof),
            other => Err(WktError::UnexpectedToken {
                expected: "number",
                found: format!("{other:?}"),
            }),
        }
    }

    /// Skip the optional OGC dimension suffix (`Z`, `M`, or `ZM`) that
    /// may follow the type keyword. This 2D port ignores the extra
    /// ordinates; the suffix is consumed so it does not confuse the
    /// coordinate scan. Mirrors Boost's handling of the dimension in
    /// `boost/geometry/io/wkt/read.hpp` (it likewise reads only the
    /// coordinates its point type declares).
    fn skip_dimension_suffix(&mut self) -> Result<(), WktError> {
        if let Token::Ident(word) = self.peek() {
            if word == "Z" || word == "M" || word == "ZM" {
                self.advance()?;
            }
        }
        Ok(())
    }

    /// Read exactly two ordinates into a 2D point. Any further ordinates
    /// (from a `Z`/`M` input) are consumed and discarded.
    fn parse_point_coords(&mut self) -> Result<Pt, WktError> {
        let x = self.expect_number()?;
        let y = self.expect_number()?;
        // Discard trailing Z / M ordinates for this 2D port.
        while let Token::Number(_) = self.peek() {
            self.advance()?;
        }
        Ok(Point2D::new(x, y))
    }

    /// Parse a parenthesised, comma-separated list of coordinate pairs:
    /// `(x y, x y, …)`. Used by `LINESTRING`, each ring of a `POLYGON`,
    /// and the bare-coordinate form of `MULTIPOINT`.
    fn parse_coord_list(&mut self) -> Result<Vec<Pt>, WktError> {
        self.expect_left_paren()?;
        let mut pts = Vec::new();
        loop {
            pts.push(self.parse_point_coords()?);
            match self.peek() {
                Token::Comma => {
                    self.advance()?;
                }
                _ => break,
            }
        }
        self.expect_right_paren()?;
        Ok(pts)
    }

    /// Parse a parenthesised, comma-separated list of coordinate lists:
    /// `((…), (…), …)`. Used by `POLYGON` (outer ring + holes) and
    /// `MULTILINESTRING`.
    fn parse_coord_list_list(&mut self) -> Result<Vec<Vec<Pt>>, WktError> {
        self.expect_left_paren()?;
        let mut rings = Vec::new();
        loop {
            rings.push(self.parse_coord_list()?);
            match self.peek() {
                Token::Comma => {
                    self.advance()?;
                }
                _ => break,
            }
        }
        self.expect_right_paren()?;
        Ok(rings)
    }

    /// `POINT` body: `(x y)`. `POINT EMPTY` is rejected — see the crate
    /// docs — because a 2D `Point` cannot represent "no coordinate".
    fn parse_point_body(&mut self) -> Result<DynGeometry<f64, Cartesian>, WktError> {
        if let Token::Empty = self.peek() {
            return Err(WktError::TypeMismatch {
                expected: "POINT with coordinates",
                found: "POINT EMPTY",
            });
        }
        self.expect_left_paren()?;
        let p = self.parse_point_coords()?;
        self.expect_right_paren()?;
        Ok(DynGeometry::Point(p))
    }

    /// `LINESTRING` body: `(x y, …)` or `EMPTY`.
    fn parse_linestring_body(&mut self) -> Result<DynGeometry<f64, Cartesian>, WktError> {
        if let Token::Empty = self.peek() {
            self.advance()?;
            return Ok(DynGeometry::LineString(Linestring(Vec::new())));
        }
        let pts = self.parse_coord_list()?;
        Ok(DynGeometry::LineString(Linestring(pts)))
    }

    /// `POLYGON` body: `((outer), (hole1), …)` or `EMPTY`. The first
    /// ring is the exterior; the rest are holes.
    fn parse_polygon_body(&mut self) -> Result<DynGeometry<f64, Cartesian>, WktError> {
        Ok(DynGeometry::Polygon(self.parse_polygon_value()?))
    }

    /// The shared `POLYGON` value builder, reused by `MULTIPOLYGON`.
    fn parse_polygon_value(&mut self) -> Result<Polygon<Pt>, WktError> {
        if let Token::Empty = self.peek() {
            self.advance()?;
            return Ok(Polygon::new(Ring::new()));
        }
        let mut rings = self.parse_coord_list_list()?.into_iter();
        let outer = rings.next().map_or_else(Ring::new, Ring::from_vec);
        let inners: Vec<Ring<Pt>> = rings.map(Ring::from_vec).collect();
        Ok(Polygon::with_inners(outer, inners))
    }

    /// `MULTIPOINT` body. OGC allows both the bare form
    /// `MULTIPOINT (1 1, 2 2)` and the parenthesised form
    /// `MULTIPOINT ((1 1), (2 2))`; this reader accepts either by
    /// peeking past the outer `(` for a nested `(`.
    fn parse_multipoint_body(&mut self) -> Result<DynGeometry<f64, Cartesian>, WktError> {
        if let Token::Empty = self.peek() {
            self.advance()?;
            return Ok(DynGeometry::MultiPoint(MultiPoint(Vec::new())));
        }
        self.expect_left_paren()?;
        let mut pts = Vec::new();
        loop {
            if let Token::LeftParen = self.peek() {
                // Parenthesised member: `(x y)`.
                self.advance()?;
                pts.push(self.parse_point_coords()?);
                self.expect_right_paren()?;
            } else {
                // Bare member: `x y`.
                pts.push(self.parse_point_coords()?);
            }
            match self.peek() {
                Token::Comma => {
                    self.advance()?;
                }
                _ => break,
            }
        }
        self.expect_right_paren()?;
        Ok(DynGeometry::MultiPoint(MultiPoint(pts)))
    }

    /// `MULTILINESTRING` body: `((x y, …), …)` or `EMPTY`.
    fn parse_multilinestring_body(&mut self) -> Result<DynGeometry<f64, Cartesian>, WktError> {
        if let Token::Empty = self.peek() {
            self.advance()?;
            return Ok(DynGeometry::MultiLineString(MultiLinestring(Vec::new())));
        }
        let lists = self.parse_coord_list_list()?;
        let lines: Vec<Linestring<Pt>> = lists.into_iter().map(Linestring).collect();
        Ok(DynGeometry::MultiLineString(MultiLinestring(lines)))
    }

    /// `MULTIPOLYGON` body: `(((ring), …), …)` or `EMPTY`.
    fn parse_multipolygon_body(&mut self) -> Result<DynGeometry<f64, Cartesian>, WktError> {
        if let Token::Empty = self.peek() {
            self.advance()?;
            return Ok(DynGeometry::MultiPolygon(MultiPolygon(Vec::new())));
        }
        self.expect_left_paren()?;
        let mut polys = Vec::new();
        loop {
            polys.push(self.parse_polygon_value()?);
            match self.peek() {
                Token::Comma => {
                    self.advance()?;
                }
                _ => break,
            }
        }
        self.expect_right_paren()?;
        Ok(DynGeometry::MultiPolygon(MultiPolygon(polys)))
    }

    /// `GEOMETRYCOLLECTION` body: `(<geometry>, …)` or `EMPTY`. Recurses
    /// into [`Parser::parse_geometry`] for each member, carrying `depth`
    /// so nested collections stay bounded by [`MAX_DEPTH`].
    fn parse_collection_body(
        &mut self,
        depth: usize,
    ) -> Result<DynGeometry<f64, Cartesian>, WktError> {
        if let Token::Empty = self.peek() {
            self.advance()?;
            return Ok(DynGeometry::GeometryCollection(Vec::new()));
        }
        self.expect_left_paren()?;
        let mut items = Vec::new();
        loop {
            items.push(self.parse_geometry(depth + 1)?);
            match self.peek() {
                Token::Comma => {
                    self.advance()?;
                }
                _ => break,
            }
        }
        self.expect_right_paren()?;
        Ok(DynGeometry::GeometryCollection(items))
    }

    /// Parse one geometry: a leading type keyword, an optional dimension
    /// suffix, then the kind-specific body. Mirrors the keyword dispatch
    /// at the top of `boost/geometry/io/wkt/read.hpp`. `depth` bounds the
    /// `GEOMETRYCOLLECTION` recursion against [`MAX_DEPTH`] so adversarial
    /// nesting fails with a recoverable error instead of overflowing the
    /// stack.
    fn parse_geometry(&mut self, depth: usize) -> Result<DynGeometry<f64, Cartesian>, WktError> {
        if depth >= MAX_DEPTH {
            return Err(WktError::NestingTooDeep);
        }
        let keyword = match self.next()? {
            Token::Ident(word) => word,
            Token::Eof => return Err(WktError::UnexpectedEof),
            other => {
                return Err(WktError::UnexpectedToken {
                    expected: "geometry type keyword",
                    found: format!("{other:?}"),
                });
            }
        };
        self.skip_dimension_suffix()?;
        match keyword.as_str() {
            "POINT" => self.parse_point_body(),
            "LINESTRING" => self.parse_linestring_body(),
            "POLYGON" => self.parse_polygon_body(),
            "MULTIPOINT" => self.parse_multipoint_body(),
            "MULTILINESTRING" => self.parse_multilinestring_body(),
            "MULTIPOLYGON" => self.parse_multipolygon_body(),
            "GEOMETRYCOLLECTION" => self.parse_collection_body(depth),
            _ => Err(WktError::UnknownGeometryType(keyword)),
        }
    }
}

/// Parse a WKT string into a runtime-tagged [`DynGeometry`].
///
/// Implements the OGC kinds `POINT`, `LINESTRING`, `POLYGON`,
/// `MULTIPOINT`, `MULTILINESTRING`, `MULTIPOLYGON`, and
/// `GEOMETRYCOLLECTION` (`POLYHEDRALSURFACE` has no concrete model type
/// yet — see `geometry_model::DynGeometry`). Mirrors the read path in
/// `boost/geometry/io/wkt/read.hpp`.
///
/// # `EMPTY` handling
///
/// The collection kinds accept `EMPTY` and yield an empty container
/// (`LINESTRING EMPTY` → an empty [`geometry_model::Linestring`],
/// `GEOMETRYCOLLECTION EMPTY` → an empty `Vec`, and so on). `POINT
/// EMPTY` is rejected with [`WktError::TypeMismatch`]: a 2D
/// [`geometry_model::Point`] has no representation for "no coordinate".
///
/// # `MULTIPOINT` forms
///
/// Both OGC spellings are accepted: the bare `MULTIPOINT (1 1, 2 2)`
/// and the parenthesised `MULTIPOINT ((1 1), (2 2))`.
///
/// # Errors
///
/// Returns a [`WktError`] on a lexer failure, a token that does not fit
/// the grammar, an unknown leading keyword, or `POINT EMPTY`.
///
/// # Examples
///
/// ```
/// use geometry_io_wkt::from_wkt;
/// use geometry_model::DynKind;
///
/// let g = from_wkt("POINT (10 10)").unwrap();
/// assert_eq!(g.kind(), DynKind::Point);
/// ```
pub fn from_wkt<S: AsRef<str>>(input: S) -> Result<DynGeometry<f64, Cartesian>, WktError> {
    let mut parser = Parser::new(input.as_ref())?;
    let g = parser.parse_geometry(0)?;
    // Reject trailing garbage after a complete geometry.
    match parser.peek() {
        Token::Eof => Ok(g),
        other => Err(WktError::UnexpectedToken {
            expected: "end of input",
            found: format!("{other:?}"),
        }),
    }
}

/// The domain-noun name of a parsed geometry, for [`WktError::TypeMismatch`].
fn kind_name(g: &DynGeometry<f64, Cartesian>) -> &'static str {
    match g {
        DynGeometry::Point(_) => "POINT",
        DynGeometry::LineString(_) => "LINESTRING",
        DynGeometry::Polygon(_) => "POLYGON",
        DynGeometry::MultiPoint(_) => "MULTIPOINT",
        DynGeometry::MultiLineString(_) => "MULTILINESTRING",
        DynGeometry::MultiPolygon(_) => "MULTIPOLYGON",
        DynGeometry::GeometryCollection(_) => "GEOMETRYCOLLECTION",
    }
}

/// Parse a WKT `POINT` into a concrete [`geometry_model::Point`].
///
/// A [`from_wkt`] followed by a variant match — convenience for callers
/// who know the kind up front (IO1.T4). Mirrors reading straight into a
/// static `model::point` in `boost/geometry/io/wkt/read.hpp`.
///
/// # Errors
///
/// [`WktError::TypeMismatch`] if the input is a different kind, plus any
/// error [`from_wkt`] can raise.
///
/// # Examples
///
/// ```
/// use geometry_io_wkt::parse_point;
/// use geometry_trait::Point as _;
///
/// let p = parse_point("POINT (10 10)").unwrap();
/// assert_eq!(p.get::<0>(), 10.0);
/// ```
pub fn parse_point(s: &str) -> Result<Pt, WktError> {
    let g = from_wkt(s)?;
    match g {
        DynGeometry::Point(p) => Ok(p),
        other => Err(WktError::TypeMismatch {
            expected: "POINT",
            found: kind_name(&other),
        }),
    }
}

/// Parse a WKT `LINESTRING` into a concrete [`geometry_model::Linestring`].
///
/// See [`parse_point`] for the pattern.
///
/// # Errors
///
/// [`WktError::TypeMismatch`] if the input is a different kind, plus any
/// error [`from_wkt`] can raise.
///
/// # Examples
///
/// ```
/// use geometry_io_wkt::parse_linestring;
/// use geometry_trait::Linestring as _;
///
/// let ls = parse_linestring("LINESTRING (10 10, 20 20, 30 40)").unwrap();
/// assert_eq!(ls.points().len(), 3);
/// ```
pub fn parse_linestring(s: &str) -> Result<Linestring<Pt>, WktError> {
    let g = from_wkt(s)?;
    match g {
        DynGeometry::LineString(ls) => Ok(ls),
        other => Err(WktError::TypeMismatch {
            expected: "LINESTRING",
            found: kind_name(&other),
        }),
    }
}

/// Parse a WKT `POLYGON` into a concrete [`geometry_model::Polygon`].
///
/// See [`parse_point`] for the pattern.
///
/// # Errors
///
/// [`WktError::TypeMismatch`] if the input is a different kind, plus any
/// error [`from_wkt`] can raise.
///
/// # Examples
///
/// ```
/// use geometry_io_wkt::parse_polygon;
/// use geometry_trait::{Polygon as _, Ring as _};
///
/// let p = parse_polygon("POLYGON ((10 10, 10 20, 20 20, 20 15, 10 10))").unwrap();
/// assert_eq!(p.exterior().points().len(), 5);
/// ```
pub fn parse_polygon(s: &str) -> Result<Polygon<Pt>, WktError> {
    let g = from_wkt(s)?;
    match g {
        DynGeometry::Polygon(p) => Ok(p),
        other => Err(WktError::TypeMismatch {
            expected: "POLYGON",
            found: kind_name(&other),
        }),
    }
}

/// Parse a WKT `MULTIPOINT` into a concrete [`geometry_model::MultiPoint`].
///
/// See [`parse_point`] for the pattern. Both OGC `MULTIPOINT` spellings
/// are accepted (see [`from_wkt`]).
///
/// # Errors
///
/// [`WktError::TypeMismatch`] if the input is a different kind, plus any
/// error [`from_wkt`] can raise.
///
/// # Examples
///
/// ```
/// use geometry_io_wkt::parse_multi_point;
/// use geometry_trait::MultiPoint as _;
///
/// let mp = parse_multi_point("MULTIPOINT ((10 10), (20 20))").unwrap();
/// assert_eq!(mp.points().len(), 2);
/// ```
pub fn parse_multi_point(s: &str) -> Result<MultiPoint<Pt>, WktError> {
    let g = from_wkt(s)?;
    match g {
        DynGeometry::MultiPoint(mp) => Ok(mp),
        other => Err(WktError::TypeMismatch {
            expected: "MULTIPOINT",
            found: kind_name(&other),
        }),
    }
}

/// Parse a WKT `MULTILINESTRING` into a concrete
/// [`geometry_model::MultiLinestring`].
///
/// See [`parse_point`] for the pattern.
///
/// # Errors
///
/// [`WktError::TypeMismatch`] if the input is a different kind, plus any
/// error [`from_wkt`] can raise.
///
/// # Examples
///
/// ```
/// use geometry_io_wkt::parse_multi_linestring;
/// use geometry_trait::MultiLinestring as _;
///
/// let mls = parse_multi_linestring("MULTILINESTRING ((10 10, 20 20), (15 15, 30 15))").unwrap();
/// assert_eq!(mls.linestrings().len(), 2);
/// ```
pub fn parse_multi_linestring(s: &str) -> Result<MultiLinestring<Linestring<Pt>>, WktError> {
    let g = from_wkt(s)?;
    match g {
        DynGeometry::MultiLineString(mls) => Ok(mls),
        other => Err(WktError::TypeMismatch {
            expected: "MULTILINESTRING",
            found: kind_name(&other),
        }),
    }
}

/// Parse a WKT `MULTIPOLYGON` into a concrete
/// [`geometry_model::MultiPolygon`].
///
/// See [`parse_point`] for the pattern.
///
/// # Errors
///
/// [`WktError::TypeMismatch`] if the input is a different kind, plus any
/// error [`from_wkt`] can raise.
///
/// # Examples
///
/// ```
/// use geometry_io_wkt::parse_multi_polygon;
/// use geometry_trait::MultiPolygon as _;
///
/// let mpg = parse_multi_polygon("MULTIPOLYGON (((10 10, 10 20, 20 20, 20 15, 10 10)))").unwrap();
/// assert_eq!(mpg.polygons().len(), 1);
/// ```
pub fn parse_multi_polygon(s: &str) -> Result<MultiPolygon<Polygon<Pt>>, WktError> {
    let g = from_wkt(s)?;
    match g {
        DynGeometry::MultiPolygon(mpg) => Ok(mpg),
        other => Err(WktError::TypeMismatch {
            expected: "MULTIPOLYGON",
            found: kind_name(&other),
        }),
    }
}

#[cfg(test)]
mod tests {
    //! Reproduces the OGC SFA-1 §6.1.10 worked examples: parse each and
    //! assert the kind plus the point count / coordinate values.
    #![allow(
        clippy::float_cmp,
        reason = "coordinate values come from exact integer WKT literals"
    )]

    use super::*;
    use geometry_model::DynKind;
    use geometry_trait::{
        Linestring as _, MultiLinestring as _, MultiPoint as _, MultiPolygon as _, Point as _,
        Polygon as _, Ring as _,
    };

    #[test]
    fn deeply_nested_collections_are_rejected_without_overflow() {
        // Regression: a chain of nested GEOMETRYCOLLECTION headers used to
        // recurse until the native stack overflowed (uncatchable SIGABRT).
        // Now the reader caps recursion and returns NestingTooDeep.
        let mut s = alloc::string::String::new();
        for _ in 0..10_000 {
            s.push_str("GEOMETRYCOLLECTION(");
        }
        s.push_str("POINT(1 1)");
        for _ in 0..10_000 {
            s.push(')');
        }
        assert_eq!(from_wkt(&s).unwrap_err(), WktError::NestingTooDeep);
        // A modest, legitimate nesting depth still parses.
        let ok = "GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 1)))";
        assert!(from_wkt(ok).is_ok());
    }

    #[test]
    fn point_example() {
        let g = from_wkt("POINT (10 10)").unwrap();
        assert_eq!(g.kind(), DynKind::Point);
        if let DynGeometry::Point(p) = g {
            assert_eq!(p.get::<0>(), 10.0);
            assert_eq!(p.get::<1>(), 10.0);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn linestring_example() {
        let g = from_wkt("LINESTRING (10 10, 20 20, 30 40)").unwrap();
        assert_eq!(g.kind(), DynKind::LineString);
        if let DynGeometry::LineString(ls) = g {
            assert_eq!(ls.points().len(), 3);
            let last = ls.points().last().unwrap();
            assert_eq!(last.get::<0>(), 30.0);
            assert_eq!(last.get::<1>(), 40.0);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn polygon_example() {
        let g = from_wkt("POLYGON ((10 10, 10 20, 20 20, 20 15, 10 10))").unwrap();
        assert_eq!(g.kind(), DynKind::Polygon);
        if let DynGeometry::Polygon(p) = g {
            assert_eq!(p.exterior().points().len(), 5);
            assert_eq!(p.interiors().count(), 0);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn polygon_with_hole() {
        let g =
            from_wkt("POLYGON ((0 0, 0 10, 10 10, 10 0, 0 0), (2 2, 2 4, 4 4, 4 2, 2 2))").unwrap();
        if let DynGeometry::Polygon(p) = g {
            assert_eq!(p.interiors().count(), 1);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn multipoint_parenthesised_form() {
        let g = from_wkt("MULTIPOINT ((10 10), (20 20))").unwrap();
        assert_eq!(g.kind(), DynKind::MultiPoint);
        if let DynGeometry::MultiPoint(mp) = g {
            assert_eq!(mp.points().len(), 2);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn multipoint_bare_form() {
        let g = from_wkt("MULTIPOINT (10 10, 20 20)").unwrap();
        if let DynGeometry::MultiPoint(mp) = g {
            assert_eq!(mp.points().len(), 2);
            let second = mp.points().nth(1).unwrap();
            assert_eq!(second.get::<0>(), 20.0);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn multilinestring_example() {
        let g = from_wkt("MULTILINESTRING ((10 10, 20 20), (15 15, 30 15))").unwrap();
        assert_eq!(g.kind(), DynKind::MultiLineString);
        if let DynGeometry::MultiLineString(mls) = g {
            assert_eq!(mls.linestrings().len(), 2);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn multipolygon_example() {
        let g = from_wkt("MULTIPOLYGON (((10 10, 10 20, 20 20, 20 15, 10 10)))").unwrap();
        assert_eq!(g.kind(), DynKind::MultiPolygon);
        if let DynGeometry::MultiPolygon(mpg) = g {
            assert_eq!(mpg.polygons().len(), 1);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn geometrycollection_example() {
        let g = from_wkt("GEOMETRYCOLLECTION (POINT (10 10), LINESTRING (10 10, 20 20))").unwrap();
        assert_eq!(g.kind(), DynKind::GeometryCollection);
        if let DynGeometry::GeometryCollection(items) = g {
            assert_eq!(items.len(), 2);
            assert_eq!(items[0].kind(), DynKind::Point);
            assert_eq!(items[1].kind(), DynKind::LineString);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn linestring_empty() {
        let g = from_wkt("LINESTRING EMPTY").unwrap();
        if let DynGeometry::LineString(ls) = g {
            assert_eq!(ls.points().len(), 0);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn geometrycollection_empty() {
        let g = from_wkt("GEOMETRYCOLLECTION EMPTY").unwrap();
        if let DynGeometry::GeometryCollection(items) = g {
            assert_eq!(items.len(), 0);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn point_empty_is_rejected() {
        let err = from_wkt("POINT EMPTY").unwrap_err();
        assert!(matches!(err, WktError::TypeMismatch { .. }));
    }

    #[test]
    fn typed_parse_type_mismatch() {
        let err = parse_point("LINESTRING (10 10, 20 20)").unwrap_err();
        assert_eq!(
            err,
            WktError::TypeMismatch {
                expected: "POINT",
                found: "LINESTRING",
            }
        );
    }

    #[test]
    fn unknown_keyword_errors() {
        let err = from_wkt("TRIANGLE (0 0, 1 0, 0 1, 0 0)").unwrap_err();
        assert_eq!(err, WktError::UnknownGeometryType("TRIANGLE".into()));
    }

    #[test]
    fn dimension_suffix_is_skipped() {
        // The Z ordinate is dropped; the 2D coordinates survive.
        let g = from_wkt("POINT Z (10 10 5)").unwrap();
        if let DynGeometry::Point(p) = g {
            assert_eq!(p.get::<0>(), 10.0);
            assert_eq!(p.get::<1>(), 10.0);
        } else {
            unreachable!();
        }
    }

    // ---- EMPTY forms for the remaining collection kinds --------------

    /// `POLYGON EMPTY` yields a polygon with an empty exterior ring and
    /// no holes.
    #[test]
    fn polygon_empty() {
        let g = from_wkt("POLYGON EMPTY").unwrap();
        if let DynGeometry::Polygon(p) = g {
            assert_eq!(p.exterior().points().len(), 0);
            assert_eq!(p.interiors().count(), 0);
        } else {
            unreachable!();
        }
    }

    /// `MULTIPOINT EMPTY`, `MULTILINESTRING EMPTY`, and `MULTIPOLYGON
    /// EMPTY` each yield an empty container of the matching kind.
    #[test]
    fn multi_kinds_empty() {
        match from_wkt("MULTIPOINT EMPTY").unwrap() {
            DynGeometry::MultiPoint(mp) => assert_eq!(mp.points().len(), 0),
            _ => unreachable!(),
        }
        match from_wkt("MULTILINESTRING EMPTY").unwrap() {
            DynGeometry::MultiLineString(mls) => assert_eq!(mls.linestrings().len(), 0),
            _ => unreachable!(),
        }
        match from_wkt("MULTIPOLYGON EMPTY").unwrap() {
            DynGeometry::MultiPolygon(mpg) => assert_eq!(mpg.polygons().len(), 0),
            _ => unreachable!(),
        }
    }

    // ---- Grammar error branches -------------------------------------

    /// A body that opens with the wrong token fails
    /// `expect_left_paren` with an `UnexpectedToken` naming `'('`.
    #[test]
    fn missing_left_paren_is_reported() {
        let err = from_wkt("LINESTRING 10 10").unwrap_err();
        assert!(
            matches!(&err, WktError::UnexpectedToken { expected, .. } if *expected == "'('"),
            "got {err:?}"
        );
    }

    /// A coordinate list that never closes fails `expect_right_paren`
    /// (the terminating token is `Eof`, not `)`).
    #[test]
    fn missing_right_paren_is_reported() {
        let err = from_wkt("LINESTRING (10 10, 20 20").unwrap_err();
        assert!(
            matches!(&err, WktError::UnexpectedToken { expected, .. } if *expected == "')'"),
            "got {err:?}"
        );
    }

    /// A coordinate pair truncated after the first ordinate runs into
    /// `Eof` where the second number is required → `UnexpectedEof`.
    #[test]
    fn missing_ordinate_hits_eof() {
        let err = from_wkt("POINT (10").unwrap_err();
        assert_eq!(err, WktError::UnexpectedEof);
    }

    /// A non-number where an ordinate is expected fails `expect_number`
    /// with an `UnexpectedToken` naming `number`.
    #[test]
    fn non_number_ordinate_is_reported() {
        // The second "ordinate" is a `)`, not a number.
        let err = from_wkt("POINT (10 )").unwrap_err();
        assert!(
            matches!(&err, WktError::UnexpectedToken { expected, .. } if *expected == "number"),
            "got {err:?}"
        );
    }

    /// Trailing tokens after a complete geometry are rejected by the
    /// end-of-input guard in `from_wkt`.
    #[test]
    fn trailing_tokens_are_rejected() {
        let err = from_wkt("POINT (1 1) POINT (2 2)").unwrap_err();
        assert!(
            matches!(&err, WktError::UnexpectedToken { expected, .. } if *expected == "end of input"),
            "got {err:?}"
        );
    }

    /// Empty input reaches `parse_geometry` with an immediate `Eof`
    /// keyword slot → `UnexpectedEof`.
    #[test]
    fn empty_input_is_unexpected_eof() {
        assert_eq!(from_wkt("").unwrap_err(), WktError::UnexpectedEof);
    }

    /// A leading token that is neither an identifier nor `Eof` (here a
    /// `(`) fails the keyword dispatch with `UnexpectedToken`.
    #[test]
    fn non_keyword_leading_token_is_reported() {
        let err = from_wkt("(1 1)").unwrap_err();
        assert!(
            matches!(&err, WktError::UnexpectedToken { expected, .. }
                if *expected == "geometry type keyword"),
            "got {err:?}"
        );
    }

    // ---- Typed-parse happy paths and per-kind mismatches -------------

    /// Each typed parser returns its concrete kind on matching input.
    #[test]
    fn typed_parsers_accept_their_own_kind() {
        assert_eq!(parse_point("POINT (1 2)").unwrap().get::<0>(), 1.0);
        assert_eq!(
            parse_linestring("LINESTRING (0 0, 1 1)").unwrap().points().len(),
            2
        );
        assert_eq!(
            parse_polygon("POLYGON ((0 0, 1 0, 1 1, 0 0))")
                .unwrap()
                .exterior()
                .points()
                .len(),
            4
        );
        assert_eq!(
            parse_multi_point("MULTIPOINT (0 0, 1 1)").unwrap().points().len(),
            2
        );
        assert_eq!(
            parse_multi_linestring("MULTILINESTRING ((0 0, 1 1))")
                .unwrap()
                .linestrings()
                .len(),
            1
        );
        assert_eq!(
            parse_multi_polygon("MULTIPOLYGON (((0 0, 1 0, 1 1, 0 0)))")
                .unwrap()
                .polygons()
                .len(),
            1
        );
    }

    /// Each typed parser rejects a different-kind input with a
    /// `TypeMismatch` whose `found` names the actual kind — exercising
    /// every arm of `kind_name`.
    #[test]
    fn typed_parsers_reject_wrong_kind_naming_the_kind() {
        let cases: &[(WktError, &str, &str)] = &[
            (
                parse_linestring("POINT (1 1)").unwrap_err(),
                "LINESTRING",
                "POINT",
            ),
            (
                parse_polygon("LINESTRING (0 0, 1 1)").unwrap_err(),
                "POLYGON",
                "LINESTRING",
            ),
            (
                parse_multi_point("POLYGON ((0 0, 1 0, 1 1, 0 0))").unwrap_err(),
                "MULTIPOINT",
                "POLYGON",
            ),
            (
                parse_multi_linestring("MULTIPOINT (0 0, 1 1)").unwrap_err(),
                "MULTILINESTRING",
                "MULTIPOINT",
            ),
            (
                parse_multi_polygon("MULTILINESTRING ((0 0, 1 1))").unwrap_err(),
                "MULTIPOLYGON",
                "MULTILINESTRING",
            ),
            (
                parse_point("MULTIPOLYGON (((0 0, 1 0, 1 1, 0 0)))").unwrap_err(),
                "POINT",
                "MULTIPOLYGON",
            ),
            (
                parse_point("GEOMETRYCOLLECTION (POINT (1 1))").unwrap_err(),
                "POINT",
                "GEOMETRYCOLLECTION",
            ),
        ];
        for (err, want_expected, want_found) in cases {
            match err {
                WktError::TypeMismatch { expected, found } => {
                    assert_eq!(expected, want_expected);
                    assert_eq!(found, want_found);
                }
                other => panic!("expected TypeMismatch, got {other:?}"),
            }
        }
    }
}
