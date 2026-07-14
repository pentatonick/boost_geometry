//! M-IO1 — WKT round-trip validation.
//!
//! For every OGC Simple Feature Access Part 1 §6.1.10 worked example:
//! `from_wkt(s)` → `to_wkt(dyn)` → `from_wkt(again)` → assert the two
//! [`DynGeometry`] values are deep-equal. `DynGeometry` derives
//! `PartialEq`, so the second comparison is exact. Mirrors the
//! parse/write parity check in `boost/geometry/test/io/wkt/wkt.cpp`.
#![allow(
    clippy::float_cmp,
    reason = "the round-trip compares DynGeometry values built from exact WKT literals"
)]

use core::fmt;

use geometry_cs::Cartesian;
use geometry_io_wkt::{
    WktError, WriteWkt, from_wkt, parse_linestring, parse_multi_linestring, parse_multi_point,
    parse_multi_polygon, parse_point, parse_polygon, to_wkt, write_wkt,
};
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};
use geometry_tag::PointTag;
use geometry_trait::Geometry;

type Pt = Point2D<f64, Cartesian>;
type Dyn = DynGeometry<f64, Cartesian>;

struct FailingSink;

impl fmt::Write for FailingSink {
    fn write_str(&mut self, _s: &str) -> fmt::Result {
        Err(fmt::Error)
    }
}

/// Parse `s`, re-serialise, re-parse, and assert the two parsed values
/// are equal.
fn assert_round_trip(s: &str) {
    let first = from_wkt(s).unwrap_or_else(|e| panic!("first parse of {s:?} failed: {e}"));
    let text = to_wkt(&first);
    let second = from_wkt(&text).unwrap_or_else(|e| panic!("re-parse of {text:?} failed: {e}"));
    assert_eq!(
        first, second,
        "round-trip mismatch for {s:?} (via {text:?})"
    );
}

#[test]
fn point() {
    assert_round_trip("POINT (10 10)");
}

#[test]
fn linestring() {
    assert_round_trip("LINESTRING (10 10, 20 20, 30 40)");
}

#[test]
fn polygon() {
    assert_round_trip("POLYGON ((10 10, 10 20, 20 20, 20 15, 10 10))");
}

#[test]
fn polygon_with_hole() {
    assert_round_trip("POLYGON ((0 0, 0 10, 10 10, 10 0, 0 0), (2 2, 2 4, 4 4, 4 2, 2 2))");
}

#[test]
fn multipoint() {
    assert_round_trip("MULTIPOINT ((10 10), (20 20))");
}

#[test]
fn multipoint_bare_form() {
    // The bare form re-serialises to the parenthesised form; both parse
    // to the same value.
    assert_round_trip("MULTIPOINT (10 10, 20 20)");
}

#[test]
fn multilinestring() {
    assert_round_trip("MULTILINESTRING ((10 10, 20 20), (15 15, 30 15))");
}

#[test]
fn multipolygon() {
    assert_round_trip("MULTIPOLYGON (((10 10, 10 20, 20 20, 20 15, 10 10)))");
}

#[test]
fn geometrycollection() {
    assert_round_trip("GEOMETRYCOLLECTION (POINT (10 10), LINESTRING (10 10, 20 20))");
}

#[test]
fn empty_geometries_round_trip() {
    // Regression: the writer must emit `<TYPE> EMPTY`, not `<TYPE>()`,
    // which the reader (and OGC WKT) rejects. Each of these must survive
    // parse → write → re-parse.
    assert_round_trip("LINESTRING EMPTY");
    assert_round_trip("POLYGON EMPTY");
    assert_round_trip("MULTIPOINT EMPTY");
    assert_round_trip("MULTILINESTRING EMPTY");
    assert_round_trip("MULTIPOLYGON EMPTY");
    assert_round_trip("GEOMETRYCOLLECTION EMPTY");
}

#[test]
fn geometrycollection_with_empty_member_round_trips() {
    // A non-empty collection containing an empty sub-part must also
    // survive: the empty member is serialised as `LINESTRING EMPTY`.
    assert_round_trip("GEOMETRYCOLLECTION (POINT (1 2), LINESTRING EMPTY)");
}

#[test]
fn public_typed_parsers_accept_their_geometry_kinds() {
    assert_eq!(parse_point("POINT (1 2)").unwrap(), Pt::new(1.0, 2.0));
    assert_eq!(
        parse_linestring("LINESTRING (0 0, 1 1)").unwrap().0.len(),
        2
    );
    assert_eq!(
        parse_polygon("POLYGON ((0 0, 1 0, 0 0))")
            .unwrap()
            .outer
            .0
            .len(),
        3
    );
    assert_eq!(
        parse_multi_point("MULTIPOINT (0 0, 1 1)").unwrap().0.len(),
        2
    );
    assert_eq!(
        parse_multi_linestring("MULTILINESTRING ((0 0, 1 1))")
            .unwrap()
            .0
            .len(),
        1
    );
    assert_eq!(
        parse_multi_polygon("MULTIPOLYGON (((0 0, 1 0, 0 0)))")
            .unwrap()
            .0
            .len(),
        1
    );
}

#[test]
fn public_parser_contract_covers_dimensions_unicode_and_errors() {
    assert_round_trip("POINT Z (1 2 3)");
    assert_round_trip("POINT M (1 2 3)");
    assert_round_trip("POINT ZM (1 2 3 4)");
    assert_round_trip("POINT\u{2003}(1 2)");
    assert_round_trip("MULTIPOLYGON (((0 0, 1 0, 0 0)), ((2 2, 3 2, 2 2)))");
    assert_round_trip("POINT (18446744073709551616 0)");

    let cases = [
        ("", WktError::UnexpectedEof),
        ("@", WktError::UnexpectedChar { pos: 0, ch: '@' }),
        (
            "\u{00a9}",
            WktError::UnexpectedChar {
                pos: 0,
                ch: '\u{00a9}',
            },
        ),
        (
            "POINT (1)",
            WktError::UnexpectedToken {
                expected: "number",
                found: "RightParen".into(),
            },
        ),
        ("POINT (1", WktError::UnexpectedEof),
        ("POINT (1.2.3 0)", WktError::InvalidNumber("1.2.3".into())),
        ("CURVE (0 0)", WktError::UnknownGeometryType("CURVE".into())),
    ];
    for (input, expected) in cases {
        assert_eq!(from_wkt(input).unwrap_err(), expected, "input {input:?}");
    }

    assert!(matches!(
        parse_point("LINESTRING (0 0, 1 1)"),
        Err(WktError::TypeMismatch {
            expected: "POINT",
            found: "LINESTRING"
        })
    ));
    let mut deep = "GEOMETRYCOLLECTION(".repeat(129);
    deep.push_str("POINT(0 0)");
    deep.push_str(&")".repeat(129));
    assert_eq!(from_wkt(deep).unwrap_err(), WktError::NestingTooDeep);

    let displays = [
        WktError::UnexpectedChar { pos: 2, ch: '#' },
        WktError::UnexpectedToken {
            expected: "number",
            found: "Comma".into(),
        },
        WktError::UnexpectedEof,
        WktError::InvalidNumber("--1".into()),
        WktError::UnknownGeometryType("CURVE".into()),
        WktError::TypeMismatch {
            expected: "POINT",
            found: "POLYGON",
        },
        WktError::NestingTooDeep,
    ];
    for error in displays {
        assert!(!error.to_string().is_empty());
    }
}

#[test]
fn public_writer_covers_rings_scalars_and_every_dynamic_kind() {
    let ring = Ring::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(1.0, 0.0),
        Pt::new(0.0, 0.0),
    ]);
    assert_eq!(to_wkt(&ring), "POLYGON((0 0,1 0,0 0))");
    assert_eq!(to_wkt(&Ring::<Pt>::new()), "POLYGON EMPTY");
    assert_eq!(
        to_wkt(&Pt::new(-1.0e20, 1.0e-20)),
        "POINT(-100000000000000000000 0.00000000000000000001)"
    );
    assert_eq!(
        to_wkt(&Pt::new(f64::INFINITY, f64::NEG_INFINITY)),
        "POINT(inf -inf)"
    );

    let polygon = Polygon::new(ring.clone());
    let all = Dyn::GeometryCollection(vec![
        Dyn::Point(Pt::new(1.0, 2.0)),
        Dyn::LineString(Linestring(vec![Pt::new(0.0, 0.0), Pt::new(1.0, 1.0)])),
        Dyn::Polygon(polygon.clone()),
        Dyn::MultiPoint(MultiPoint(vec![Pt::new(3.0, 4.0)])),
        Dyn::MultiLineString(MultiLinestring(vec![Linestring(vec![Pt::new(5.0, 6.0)])])),
        Dyn::MultiPolygon(MultiPolygon(vec![polygon])),
        Dyn::GeometryCollection(vec![Dyn::Point(Pt::new(7.0, 8.0))]),
    ]);
    assert_round_trip(&to_wkt(&all));

    assert!(write_wkt(&Pt::new(1.0, 2.0), &mut FailingSink).is_err());
}

struct ExternalWkt;

impl Geometry for ExternalWkt {
    type Kind = PointTag;
    type Point = Pt;
}

impl WriteWkt for ExternalWkt {
    fn write_wkt(&self, out: &mut dyn fmt::Write) -> fmt::Result {
        out.write_str("POINT(9 10)")
    }
}

#[test]
fn external_public_writer_uses_default_extension_methods() {
    assert_eq!(to_wkt(&ExternalWkt), "POINT(9 10)");
}
