//! EWKT round-trip validation and the behaviour matrix.
//!
//! Three things are pinned here. First, the corpus round-trip: for every
//! `PostGIS` manual §4.2.1 EWKT example restricted to the seven OGC kinds,
//! plus the OGC SFA-1 (06-103r4) §7.2.6, Table 6 `LINESTRING` example the
//! manual covers only as `LINESTRINGM`, each bare and with a `SRID=4326;`
//! prefix,
//! `from_ewkt(to_ewkt(&e.geometry, e.srid)) == Ok(e)`. Second, the
//! behaviour matrix, as `assert_eq!` against whole `Ewkt` / `EwktError`
//! values. Third, parity with `geometry-io-wkt`: a prefix-less input must
//! travel through this crate exactly as it travels through that one.
//!
//! The round-trip invariant once carried two carve-outs inherited from
//! `geometry-io-wkt`: `MULTIPOLYGON(EMPTY)` was written back as
//! `MULTIPOLYGON((()))`, and `POINT(1e999 1)` parsed to an infinity and
//! was written back as `POINT(inf 1)`. Neither re-parsed. Both are fixed
//! in that crate — the empty member now writes as `EMPTY`, and an
//! overflowing literal is rejected outright — so no input this crate
//! accepts is excluded from the invariant. The parity lists below cover
//! both.

use geometry_cs::Cartesian;
use geometry_io_ewkt::{
    Ewkt, EwktError, Srid, WktError, from_ewkt, parse_linestring, parse_multi_linestring,
    parse_multi_point, parse_multi_polygon, parse_point, parse_polygon, to_ewkt, to_ewkt_polygon,
};
use geometry_io_wkt::{from_wkt, to_wkt, to_wkt_polygon};
use geometry_model::{DynGeometry, Linestring, MultiPoint, Point2D, Polygon, Ring};

type Pt = Point2D<f64, Cartesian>;
type Dyn = DynGeometry<f64, Cartesian>;

/// The parsed value a success row expects.
fn ok(srid: Option<u32>, geometry: Dyn) -> Ewkt<Dyn> {
    Ewkt {
        srid: srid.map(Srid::new),
        geometry,
    }
}

/// The error value a body-error row expects.
fn wkt(e: WktError) -> EwktError {
    EwktError::Wkt(e)
}

/// A `DynGeometry::Point`.
fn point(x: f64, y: f64) -> Dyn {
    DynGeometry::Point(Point2D::new(x, y))
}

/// An `UnexpectedChar` at `pos`.
fn at(pos: usize, ch: char) -> EwktError {
    wkt(WktError::UnexpectedChar { pos, ch })
}

/// Trailing-garbage `UnexpectedToken` naming `found`.
fn trailing(found: &str) -> EwktError {
    wkt(WktError::UnexpectedToken {
        expected: "end of input",
        found: found.to_string(),
    })
}

/// The eight corpus geometries. Sources: entries 1–7 are the `PostGIS`
/// manual §4.2.1 EWKT examples restricted to the seven OGC kinds; entry 8
/// is OGC SFA-1 (06-103r4) §7.2.6, Table 6.
///
/// All but one are projected to 2D, because this crate discards every
/// ordinate past the second. The exception is entry 2, which keeps the
/// manual's third ordinate so that its glued spelling exercises the
/// normaliser's allocating path rather than the borrowed fast path.
const CORPUS: [&str; 8] = [
    // 1 — manual §4.2.1 `POINT(0 0 0)` / `SRID=32632;POINT(0 0)`
    "POINT(0 0)",
    // 2 — manual §4.2.1 `POINTM(0 0 0)`, the glued spelling `ST_AsEWKT` emits
    "POINTM(0 0 0)",
    // 3 — manual §4.2.1 `SRID=4326;MULTIPOINTM(0 0 0,1 2 1)`
    "MULTIPOINT(0 0,1 2)",
    // 4 — manual §4.2.1 `MULTILINESTRING`
    "MULTILINESTRING((0 0,1 1,1 2),(2 3,3 2,5 4))",
    // 5 — manual §4.2.1 `POLYGON` with one hole
    "POLYGON((0 0,4 0,4 4,0 4,0 0),(1 1,2 1,2 2,1 2,1 1))",
    // 6 — manual §4.2.1 `MULTIPOLYGON`, first member holed
    "MULTIPOLYGON(((0 0,4 0,4 4,0 4,0 0),(1 1,2 1,2 2,1 2,1 1)),((-1 -1,-1 -2,-2 -2,-2 -1,-1 -1)))",
    // 7 — manual §4.2.1 `GEOMETRYCOLLECTIONM(POINTM(2 3 9),LINESTRINGM(2 3 4,3 4 5))`
    "GEOMETRYCOLLECTION(POINT(2 3),LINESTRING(2 3,3 4))",
    // 8 — OGC SFA-1 (06-103r4) §7.2.6, Table 6; the manual covers this kind
    // only as `LINESTRINGM` inside entry 7
    "LINESTRING(10 10,20 20,30 40)",
];

/// Parse `body` (optionally prefixed), write it back, and assert the
/// re-parse equals the first parse.
fn assert_corpus_round_trip(body: &str, srid: Option<Srid>) {
    let input = match srid {
        None => body.to_string(),
        Some(srid) => format!("SRID={srid};{body}"),
    };
    let first = from_ewkt(&input).unwrap_or_else(|e| panic!("parse of {input:?} failed: {e}"));
    assert_eq!(first.srid, srid, "srid of {input:?}");
    let text = to_ewkt(&first.geometry, first.srid);
    assert_eq!(from_ewkt(&text), Ok(first), "round-trip via {text:?}");
}

#[test]
fn corpus_round_trips_bare_and_prefixed() {
    for body in CORPUS {
        assert_corpus_round_trip(body, None);
        assert_corpus_round_trip(body, Some(Srid::new(4326)));
    }
}

#[test]
fn success_rows_prefix_forms() {
    assert_eq!(from_ewkt("POINT(1 2)"), Ok(ok(None, point(1.0, 2.0))));
    assert_eq!(
        from_ewkt("SRID=4326;POINT(1 2)"),
        Ok(ok(Some(4326), point(1.0, 2.0)))
    );
    assert_eq!(
        from_ewkt("srid=4326;point(1 2)"),
        Ok(ok(Some(4326), point(1.0, 2.0)))
    );
    assert_eq!(
        from_ewkt("  SRID=4326;  POINT(1 2)"),
        Ok(ok(Some(4326), point(1.0, 2.0)))
    );
    assert_eq!(
        from_ewkt("\u{a0}SRID=4326;POINT(1 2)"),
        Ok(ok(Some(4326), point(1.0, 2.0)))
    );
    assert_eq!(
        from_ewkt("SRID=4326;\nPOINT(1 2)"),
        Ok(ok(Some(4326), point(1.0, 2.0)))
    );
    assert_eq!(
        from_ewkt("SRID=0;POINT(1 2)"),
        Ok(Ewkt {
            srid: Some(Srid::UNKNOWN),
            geometry: point(1.0, 2.0),
        })
    );
    assert_eq!(
        from_ewkt("SRID=0004326;POINT(1 2)"),
        Ok(ok(Some(4326), point(1.0, 2.0)))
    );
    assert_eq!(
        from_ewkt("SRID=4294967295;POINT(1 2)"),
        Ok(ok(Some(u32::MAX), point(1.0, 2.0)))
    );
    assert_eq!(
        from_ewkt("SRID=4326;POINT(1 2 3)"),
        Ok(ok(Some(4326), point(1.0, 2.0)))
    );
    assert_eq!(
        from_ewkt("SRID=4326;POINT M (1 2 3)"),
        Ok(ok(Some(4326), point(1.0, 2.0)))
    );
}

#[test]
fn success_rows_glued_suffixes() {
    let prefixed = Ok(ok(Some(4326), point(1.0, 2.0)));
    assert_eq!(from_ewkt("POINTM(1 2 3)"), Ok(ok(None, point(1.0, 2.0))));
    assert_eq!(from_ewkt("SRID=4326;POINTM(1 2 3)"), prefixed);
    assert_eq!(from_ewkt("SRID=4326;POINTM(1 2)"), prefixed);
    assert_eq!(from_ewkt("SRID=4326;POINTZ(1 2 3)"), prefixed);
    assert_eq!(from_ewkt("SRID=4326;POINTZM(1 2 3 4)"), prefixed);
    assert_eq!(from_ewkt("SRID=4326;POINTZ M (1 2 3 4)"), prefixed);
    assert_eq!(
        from_ewkt("SRID=4326;MULTIPOINTM(1 2 3,4 5 6)"),
        Ok(ok(
            Some(4326),
            DynGeometry::MultiPoint(MultiPoint(vec![
                Point2D::new(1.0, 2.0),
                Point2D::new(4.0, 5.0),
            ])),
        ))
    );
    assert_eq!(
        from_ewkt("SRID=4326;GEOMETRYCOLLECTIONM(POINTM(1 2 3))"),
        Ok(ok(
            Some(4326),
            DynGeometry::GeometryCollection(vec![point(1.0, 2.0)]),
        ))
    );
    assert_eq!(
        from_ewkt("SRID=4326;LINESTRINGM EMPTY"),
        Ok(ok(
            Some(4326),
            DynGeometry::LineString(Linestring::<Pt>::new()),
        ))
    );
}

#[test]
fn body_error_rows() {
    assert_eq!(from_ewkt("\x0bSRID=1;POINT(1 2)"), Err(at(0, '\u{b}')));
    assert_eq!(from_ewkt("SRIDX=1;POINT(1 2)"), Err(at(5, '=')));
    assert_eq!(from_ewkt("SRID=4326;;POINT(1 2)"), Err(at(10, ';')));
    assert_eq!(from_ewkt("SRID=1;SRID=2;POINT(1 2)"), Err(at(11, '=')));
    assert_eq!(from_ewkt("SRID=4326;POINT(1 2)$"), Err(at(20, '$')));
    assert_eq!(from_ewkt("POINT(1 2)$"), Err(at(10, '$')));
    assert_eq!(from_ewkt("SRID=4326;"), Err(wkt(WktError::UnexpectedEof)));
    assert_eq!(
        from_ewkt("SRID=4326;POINT(1 2) trailing"),
        Err(trailing("Ident(\"TRAILING\")"))
    );
    assert_eq!(
        from_ewkt("POINT(1 2 3)POINTM"),
        Err(trailing("Ident(\"POINT\")"))
    );
    let empty_point = Err(wkt(WktError::TypeMismatch {
        expected: "POINT with coordinates",
        found: "POINT EMPTY",
    }));
    assert_eq!(from_ewkt("SRID=4326;POINT EMPTY"), empty_point);
    assert_eq!(from_ewkt("SRID=4326;POINTM EMPTY"), empty_point);
    assert_eq!(
        from_ewkt("SRID=4326;POINTMM(1 2 3)"),
        Err(wkt(WktError::UnknownGeometryType("POINTMM".to_string())))
    );
    assert_eq!(
        from_ewkt("SRID=4326;CIRCULARSTRING(1 2,3 4,5 6)"),
        Err(wkt(WktError::UnknownGeometryType(
            "CIRCULARSTRING".to_string()
        )))
    );
    let deep = format!("{}POINT(1 2)", "GEOMETRYCOLLECTION(".repeat(200));
    assert_eq!(from_ewkt(&deep), Err(wkt(WktError::NestingTooDeep)));
}

#[test]
fn prefix_error_rows_through_from_ewkt() {
    assert_eq!(
        from_ewkt("SRID=-1;POINT(1 2)"),
        Err(EwktError::InvalidSrid {
            reason: "sign not allowed",
            pos: 5,
        })
    );
    assert_eq!(
        from_ewkt("SRID=4326 ;POINT(1 2)"),
        Err(EwktError::InvalidSrid {
            reason: "expected ';'",
            pos: 9,
        })
    );
}

/// Prefix-less inputs the WKT crate accepts.
const PARITY_ACCEPTED: [&str; 11] = [
    "POINT(1 2)",
    "POINT (10 10)",
    "LINESTRING(1 2,3 4)",
    "LINESTRING EMPTY",
    "POLYGON((0 0,4 0,4 4,0 4,0 0))",
    "MULTIPOINT(1 2,3 4)",
    "MULTILINESTRING((1 2,3 4))",
    "MULTIPOLYGON(((0 0,4 0,4 4,0 4,0 0)))",
    "MULTIPOLYGON(EMPTY,((0 0,4 0,4 4,0 4,0 0)))",
    "MULTILINESTRING(EMPTY,(1 2,3 4))",
    "GEOMETRYCOLLECTION(POINT(1 2))",
];

/// Prefix-less inputs the WKT crate rejects, none carrying a
/// glued-suffix run and none whose leading letter run uppercases to
/// `SRID` — the two cases the parity clause excludes.
const PARITY_REJECTED: [&str; 8] = [
    "POINT(1 2)$",
    "POINT(1 2) trailing",
    "CIRCULARSTRING(1 2,3 4,5 6)",
    ";;",
    "POINT",
    "POINT(1 2",
    "POINT(a b)",
    "POINT(1e999 1)",
];

#[test]
fn parity_with_the_wkt_crate() {
    for s in PARITY_ACCEPTED {
        let e = from_ewkt(s).unwrap_or_else(|e| panic!("parse of {s:?} failed: {e}"));
        assert_eq!(e.srid, None, "srid of {s:?}");
        assert_eq!(
            Ok(e.geometry),
            from_wkt(s).map_err(EwktError::Wkt),
            "geometry of {s:?}"
        );
    }
    for s in PARITY_REJECTED {
        let expected = from_wkt(s).map_err(EwktError::Wkt).map(|_| ());
        assert_eq!(from_ewkt(s).map(|_| ()), expected, "error for {s:?}");
    }
}

#[test]
fn writer_parity_with_the_wkt_crate() {
    let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
    assert_eq!(to_ewkt(&p, None), to_wkt(&p));

    let ring = Ring::<Pt>::from_vec(vec![
        Point2D::new(0.0, 0.0),
        Point2D::new(4.0, 0.0),
        Point2D::new(0.0, 4.0),
        Point2D::new(0.0, 0.0),
    ]);
    let pg = Polygon::<Pt>::new(ring);
    assert_eq!(to_ewkt_polygon(&pg, None), to_wkt_polygon(&pg));
    assert_eq!(
        to_ewkt_polygon(&pg, Some(Srid::new(4326))),
        format!("SRID=4326;{}", to_wkt_polygon(&pg))
    );
}

#[test]
fn a_megabyte_of_digits_overflows_at_the_first_offending_digit() {
    let input = format!("SRID={};POINT(1 2)", "9".repeat(1_000_000));
    assert_eq!(
        from_ewkt(&input),
        Err(EwktError::InvalidSrid {
            reason: "value exceeds u32",
            pos: 14,
        })
    );
}

#[test]
fn a_megabyte_of_zeros_scans_linearly_to_zero() {
    let input = format!("SRID={};POINT(1 2)", "0".repeat(1_000_000));
    let e = from_ewkt(&input).expect("leading zeros are accepted");
    assert_eq!(e.srid, Some(Srid::UNKNOWN));
    assert_eq!(e.geometry, point(1.0, 2.0));
}

#[test]
fn thousands_of_glued_tokens_stay_linear() {
    let members = vec!["POINTM(1 2 3)"; 2000].join(",");
    let input = format!("SRID=4326;GEOMETRYCOLLECTION({members})");
    let e = from_ewkt(&input).expect("a flat collection of measured points parses");
    assert_eq!(e.srid, Some(Srid::new(4326)));
    match e.geometry {
        DynGeometry::GeometryCollection(c) => assert_eq!(c.len(), 2000),
        other => panic!("expected a collection, got {other:?}"),
    }
}

#[test]
fn offsets_after_a_prefix_index_the_original_string() {
    assert_eq!(from_ewkt("SRID=4294967295;POINT(1 2)$"), Err(at(26, '$')));
}

#[test]
fn a_non_ascii_body_byte_keeps_its_offset() {
    let input = "SRID=4326;POINTé";
    assert_eq!(input.find('é'), Some(15));
    assert_eq!(from_ewkt(input), Err(at(15, 'é')));
}

/// Each typed parser on its success path, through the two things this
/// crate adds to a WKT body: the `SRID=` prefix and a glued dimension
/// suffix. Every one of these had a doc example and nothing else — and
/// `cargo llvm-cov` does not count doctests, so five of the six read as
/// entirely uncovered.
#[test]
fn public_typed_parsers_accept_their_geometry_kinds() {
    let e = parse_point("SRID=4326;POINTM(1 2 3)").unwrap();
    assert_eq!(e.srid, Some(Srid::new(4326)));
    assert_eq!(e.geometry, Point2D::new(1.0, 2.0));

    let e = parse_linestring("SRID=4326;LINESTRINGZ(0 0 1,1 1 2)").unwrap();
    assert_eq!(e.srid, Some(Srid::new(4326)));
    assert_eq!(e.geometry.0.len(), 2);

    let e = parse_polygon("SRID=4326;POLYGON((0 0,1 0,1 1,0 0))").unwrap();
    assert_eq!(e.srid, Some(Srid::new(4326)));
    assert_eq!(e.geometry.outer.0.len(), 4);

    let e = parse_multi_point("SRID=4326;MULTIPOINT(0 0,1 1)").unwrap();
    assert_eq!(e.srid, Some(Srid::new(4326)));
    assert_eq!(e.geometry.0.len(), 2);

    let e = parse_multi_linestring("SRID=4326;MULTILINESTRING((0 0,1 1))").unwrap();
    assert_eq!(e.srid, Some(Srid::new(4326)));
    assert_eq!(e.geometry.0.len(), 1);

    let e = parse_multi_polygon("SRID=4326;MULTIPOLYGON(((0 0,1 0,1 1,0 0)))").unwrap();
    assert_eq!(e.srid, Some(Srid::new(4326)));
    assert_eq!(e.geometry.0.len(), 1);
}

/// The same six with no prefix: the geometry still parses and `srid` is
/// `None`, which is the case that distinguishes "absent" from
/// `SRID=0;`.
#[test]
fn typed_parsers_accept_a_prefixless_body() {
    assert_eq!(parse_point("POINT(1 2)").unwrap().srid, None);
    assert_eq!(parse_linestring("LINESTRING(0 0,1 1)").unwrap().srid, None);
    assert_eq!(
        parse_polygon("POLYGON((0 0,1 0,1 1,0 0))").unwrap().srid,
        None
    );
    assert_eq!(parse_multi_point("MULTIPOINT(0 0,1 1)").unwrap().srid, None);
    assert_eq!(
        parse_multi_linestring("MULTILINESTRING((0 0,1 1))")
            .unwrap()
            .srid,
        None
    );
    assert_eq!(
        parse_multi_polygon("MULTIPOLYGON(((0 0,1 0,1 1,0 0)))")
            .unwrap()
            .srid,
        None
    );
}

/// A malformed prefix is reported by every typed parser, not just
/// [`from_ewkt`] — they share one prefix scanner, and this pins that.
#[test]
fn typed_parsers_report_a_malformed_prefix() {
    let expected = EwktError::InvalidSrid {
        reason: "sign not allowed",
        pos: 5,
    };
    assert_eq!(parse_point("SRID=-1;POINT(1 2)").unwrap_err(), expected);
    assert_eq!(
        parse_polygon("SRID=-1;POLYGON((0 0,1 0,1 1,0 0))").unwrap_err(),
        expected
    );
    assert_eq!(
        parse_multi_point("SRID=-1;MULTIPOINT(0 0)").unwrap_err(),
        expected
    );
}

#[test]
fn a_type_mismatch_carries_the_wkt_crates_own_strings() {
    assert_eq!(
        parse_linestring("SRID=1;POINT(1 2)").map(|e| e.geometry),
        geometry_io_wkt::parse_linestring("POINT(1 2)").map_err(EwktError::Wkt)
    );
}
