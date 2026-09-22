//! M-IO3 — EWKB integration: the §14 test plan.
//!
//! The eight sections of `design.md`'s integration plan, split so no
//! function exceeds pedantic clippy's `too_many_lines` cap.
//!
//! Every `golden_*` literal in [`corpus`] came from a real `PostGIS`
//! 3.4.3 (`ST_AsEWKB(ST_GeomFromText(wkt, 4326), 'NDR'|'XDR')`), not
//! from this crate — see `specs/geometry-io-ewkb/execution-notes.md`,
//! task 0.4. The owner's open decision 13 makes a reachable server a
//! precondition for them; none is hand-derived.

use geometry_cs::Cartesian;
use geometry_io_ewkb::{
    ByteOrder, Ewkb, EwkbError, Srid, WkbError, from_ewkb, from_ewkb_hex, to_ewkb, to_ewkb_hex,
    to_ewkb_polygon,
};
use geometry_io_wkb::{from_wkb, to_wkb, to_wkb_polygon};
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};

type Pt = Point2D<f64, Cartesian>;
type Dyn = DynGeometry<f64, Cartesian>;

const BOTH_ORDERS: [ByteOrder; 2] = [ByteOrder::LittleEndian, ByteOrder::BigEndian];

/// The three SRID shapes every corpus entry is exercised with.
const SRIDS: [Option<Srid>; 3] = [None, Some(Srid::UNKNOWN), Some(Srid::new(4326))];

/// `§14.1`'s exterior ring.
fn sample_ring() -> Ring<Pt> {
    Ring::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(0.0, 10.0),
        Pt::new(10.0, 10.0),
        Pt::new(10.0, 0.0),
        Pt::new(0.0, 0.0),
    ])
}

/// `§14.1`'s interior ring.
fn sample_hole() -> Ring<Pt> {
    Ring::from_vec(vec![
        Pt::new(2.0, 2.0),
        Pt::new(2.0, 4.0),
        Pt::new(4.0, 4.0),
        Pt::new(4.0, 2.0),
        Pt::new(2.0, 2.0),
    ])
}

fn sample_polygon() -> Polygon<Pt> {
    Polygon::with_inners(sample_ring(), vec![sample_hole()])
}

/// One row of `§14.1`'s corpus table: the WKT that produced the golden
/// vectors, the model value the same row names, and both orders of
/// `ST_AsEWKB` output for it at SRID 4326.
struct CorpusEntry {
    wkt: &'static str,
    g: Dyn,
    golden_ndr: &'static str,
    golden_xdr: &'static str,
}

/// Seven populated kinds, seven EMPTY forms, and nested empty polygons.
fn corpus() -> Vec<CorpusEntry> {
    let mut entries = vec![
        // 1. POINT(1.5 -2.25)
        CorpusEntry {
            wkt: "POINT(1.5 -2.25)",
            g: Dyn::Point(Pt::new(1.5, -2.25)),
            golden_ndr: "0101000020E6100000000000000000F83F00000000000002C0",
            golden_xdr: "0020000001000010E63FF8000000000000C002000000000000",
        },
        // 2. LINESTRING(10 10,20 20)
        CorpusEntry {
            wkt: "LINESTRING(10 10,20 20)",
            g: Dyn::LineString(Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)])),
            golden_ndr: "0102000020E6100000020000000000000000002440000000000000244000000000000034400000000000003440",
            golden_xdr: "0020000002000010E6000000024024000000000000402400000000000040340000000000004034000000000000",
        },
        // 3. POLYGON((0 0,0 10,10 10,10 0,0 0),(2 2,2 4,4 4,4 2,2 2))
        CorpusEntry {
            wkt: "POLYGON((0 0,0 10,10 10,10 0,0 0),(2 2,2 4,4 4,4 2,2 2))",
            g: Dyn::Polygon(sample_polygon()),
            golden_ndr: "0103000020E610000002000000050000000000000000000000000000000000000000000000000000000000000000002440000000000000244000000000000024400000000000002440000000000000000000000000000000000000000000000000050000000000000000000040000000000000004000000000000000400000000000001040000000000000104000000000000010400000000000001040000000000000004000000000000000400000000000000040",
            golden_xdr: "0020000003000010E600000002000000050000000000000000000000000000000000000000000000004024000000000000402400000000000040240000000000004024000000000000000000000000000000000000000000000000000000000000000000054000000000000000400000000000000040000000000000004010000000000000401000000000000040100000000000004010000000000000400000000000000040000000000000004000000000000000",
        },
        // 4. MULTIPOINT(10 10,20 20)
        CorpusEntry {
            wkt: "MULTIPOINT(10 10,20 20)",
            g: Dyn::MultiPoint(MultiPoint(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)])),
            golden_ndr: "0104000020E610000002000000010100000000000000000024400000000000002440010100000000000000000034400000000000003440",
            golden_xdr: "0020000004000010E600000002000000000140240000000000004024000000000000000000000140340000000000004034000000000000",
        },
        // 5. MULTILINESTRING((10 10,20 20))
        CorpusEntry {
            wkt: "MULTILINESTRING((10 10,20 20))",
            g: Dyn::MultiLineString(MultiLinestring(vec![Linestring(vec![
                Pt::new(10.0, 10.0),
                Pt::new(20.0, 20.0),
            ])])),
            golden_ndr: "0105000020E6100000010000000102000000020000000000000000002440000000000000244000000000000034400000000000003440",
            golden_xdr: "0020000005000010E6000000010000000002000000024024000000000000402400000000000040340000000000004034000000000000",
        },
        // 6. MULTIPOLYGON(((0 0,0 10,10 10,10 0,0 0),(2 2,2 4,4 4,4 2,2 2)))
        CorpusEntry {
            wkt: "MULTIPOLYGON(((0 0,0 10,10 10,10 0,0 0),(2 2,2 4,4 4,4 2,2 2)))",
            g: Dyn::MultiPolygon(MultiPolygon(vec![sample_polygon()])),
            golden_ndr: "0106000020E610000001000000010300000002000000050000000000000000000000000000000000000000000000000000000000000000002440000000000000244000000000000024400000000000002440000000000000000000000000000000000000000000000000050000000000000000000040000000000000004000000000000000400000000000001040000000000000104000000000000010400000000000001040000000000000004000000000000000400000000000000040",
            golden_xdr: "0020000006000010E600000001000000000300000002000000050000000000000000000000000000000000000000000000004024000000000000402400000000000040240000000000004024000000000000000000000000000000000000000000000000000000000000000000054000000000000000400000000000000040000000000000004010000000000000401000000000000040100000000000004010000000000000400000000000000040000000000000004000000000000000",
        },
        // 7. GEOMETRYCOLLECTION(POINT(10 10),LINESTRING(10 10,20 20))
        CorpusEntry {
            wkt: "GEOMETRYCOLLECTION(POINT(10 10),LINESTRING(10 10,20 20))",
            g: Dyn::GeometryCollection(vec![
                Dyn::Point(Pt::new(10.0, 10.0)),
                Dyn::LineString(Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)])),
            ]),
            golden_ndr: "0107000020E6100000020000000101000000000000000000244000000000000024400102000000020000000000000000002440000000000000244000000000000034400000000000003440",
            golden_xdr: "0020000007000010E6000000020000000001402400000000000040240000000000000000000002000000024024000000000000402400000000000040340000000000004034000000000000",
        },
        // 8. POINT EMPTY
        CorpusEntry {
            wkt: "POINT EMPTY",
            g: Dyn::Point(Pt::new(f64::NAN, f64::NAN)),
            golden_ndr: "0101000020E6100000000000000000F87F000000000000F87F",
            golden_xdr: "0020000001000010E67FF80000000000007FF8000000000000",
        },
        // 9. LINESTRING EMPTY
        CorpusEntry {
            wkt: "LINESTRING EMPTY",
            g: Dyn::LineString(Linestring(vec![])),
            golden_ndr: "0102000020E610000000000000",
            golden_xdr: "0020000002000010E600000000",
        },
        // 10. POLYGON EMPTY
        CorpusEntry {
            wkt: "POLYGON EMPTY",
            g: Dyn::Polygon(Polygon::<Pt>::new(Ring::new())),
            golden_ndr: "0103000020E610000000000000",
            golden_xdr: "0020000003000010E600000000",
        },
        // 11. MULTIPOINT EMPTY
        CorpusEntry {
            wkt: "MULTIPOINT EMPTY",
            g: Dyn::MultiPoint(MultiPoint(vec![])),
            golden_ndr: "0104000020E610000000000000",
            golden_xdr: "0020000004000010E600000000",
        },
        // 12. MULTILINESTRING EMPTY
        CorpusEntry {
            wkt: "MULTILINESTRING EMPTY",
            g: Dyn::MultiLineString(MultiLinestring(vec![])),
            golden_ndr: "0105000020E610000000000000",
            golden_xdr: "0020000005000010E600000000",
        },
        // 13. MULTIPOLYGON EMPTY
        CorpusEntry {
            wkt: "MULTIPOLYGON EMPTY",
            g: Dyn::MultiPolygon(MultiPolygon(vec![])),
            golden_ndr: "0106000020E610000000000000",
            golden_xdr: "0020000006000010E600000000",
        },
        // 14. GEOMETRYCOLLECTION EMPTY
        CorpusEntry {
            wkt: "GEOMETRYCOLLECTION EMPTY",
            g: Dyn::GeometryCollection(vec![]),
            golden_ndr: "0107000020E610000000000000",
            golden_xdr: "0020000007000010E600000000",
        },
    ];
    entries.extend(nested_polygon_corpus());
    entries
}

fn nested_polygon_corpus() -> Vec<CorpusEntry> {
    vec![
        // PostGIS 3.4.3, ST_AsEWKB at SRID 4326, captured 2026-09-22.
        CorpusEntry {
            wkt: "MULTIPOLYGON(EMPTY)",
            g: Dyn::MultiPolygon(MultiPolygon(vec![Polygon::default()])),
            golden_ndr: "0106000020E610000001000000010300000000000000",
            golden_xdr: "0020000006000010E600000001000000000300000000",
        },
        CorpusEntry {
            wkt: "MULTIPOLYGON(EMPTY,((0 0,0 10,10 10,10 0,0 0)))",
            g: Dyn::MultiPolygon(MultiPolygon(vec![
                Polygon::default(),
                Polygon::new(sample_ring()),
            ])),
            golden_ndr: "0106000020E610000002000000010300000000000000010300000001000000050000000000000000000000000000000000000000000000000000000000000000002440000000000000244000000000000024400000000000002440000000000000000000000000000000000000000000000000",
            golden_xdr: "0020000006000010E600000002000000000300000000000000000300000001000000050000000000000000000000000000000000000000000000004024000000000000402400000000000040240000000000004024000000000000000000000000000000000000000000000000000000000000",
        },
        CorpusEntry {
            wkt: "GEOMETRYCOLLECTION(POLYGON EMPTY)",
            g: Dyn::GeometryCollection(vec![Dyn::Polygon(Polygon::default())]),
            golden_ndr: "0107000020E610000001000000010300000000000000",
            golden_xdr: "0020000007000010E600000001000000000300000000",
        },
        CorpusEntry {
            wkt: "GEOMETRYCOLLECTION(POINT(1.5 -2.25),POLYGON EMPTY,GEOMETRYCOLLECTION(POLYGON EMPTY))",
            g: Dyn::GeometryCollection(vec![
                Dyn::Point(Pt::new(1.5, -2.25)),
                Dyn::Polygon(Polygon::default()),
                Dyn::GeometryCollection(vec![Dyn::Polygon(Polygon::default())]),
            ]),
            golden_ndr: "0107000020E6100000030000000101000000000000000000F83F00000000000002C0010300000000000000010700000001000000010300000000000000",
            golden_xdr: "0020000007000010E60000000300000000013FF8000000000000C002000000000000000000000300000000000000000700000001000000000300000000",
        },
    ]
}

/// Decode a hex literal from the golden table.
fn unhex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("golden literal is hex"))
        .collect()
}

/// `POINT EMPTY` — NaN ordinates, so a value comparison is vacuously
/// false and every assertion for it is on bytes (`§9` exception 2).
const POINT_EMPTY: usize = 7;

// ---------------------------------------------------------------- 1

/// The corpus × three SRIDs × two orders. NaN point ordinates
/// are compared through their bytes; all other values compare directly.
#[test]
fn corpus_round_trips_at_every_srid_and_order() {
    for (i, e) in corpus().iter().enumerate() {
        for order in BOTH_ORDERS {
            for srid in SRIDS {
                let bytes = to_ewkb(&e.g, srid, order);
                let back = from_ewkb(&bytes)
                    .unwrap_or_else(|err| panic!("{} / {srid:?} / {order:?}: {err}", e.wkt));
                assert_eq!(back.srid, srid, "{} srid", e.wkt);
                assert_eq!(back.byte_order, order, "{} order", e.wkt);
                if i != POINT_EMPTY {
                    assert_eq!(back.geometry, e.g, "{} value", e.wkt);
                }
                // Re-emitting every canonical record reproduces its bytes.
                assert_eq!(
                    to_ewkb(&back.geometry, back.srid, back.byte_order),
                    bytes,
                    "{} re-emit",
                    e.wkt
                );
            }
        }
    }
}

// ---------------------------------------------------------------- 2

/// Every golden `ST_AsEWKB` vector must re-emit byte for byte,
/// including empty polygons. Only NaN point equality is skipped.
#[test]
fn golden_postgis_vectors_parse_and_re_emit() {
    for (i, e) in corpus().iter().enumerate() {
        for (order, hex) in [
            (ByteOrder::LittleEndian, e.golden_ndr),
            (ByteOrder::BigEndian, e.golden_xdr),
        ] {
            let golden = unhex(hex);
            let read =
                from_ewkb(&golden).unwrap_or_else(|err| panic!("{} / {order:?}: {err}", e.wkt));
            assert_eq!(read.srid, Some(Srid::new(4326)), "{} srid", e.wkt);
            assert_eq!(read.byte_order, order, "{} order", e.wkt);
            if i != POINT_EMPTY {
                assert_eq!(read.geometry, e.g, "{} value", e.wkt);
            }
            assert_eq!(
                to_ewkb(&read.geometry, read.srid, read.byte_order),
                golden,
                "{} bytes",
                e.wkt
            );
        }
    }
}

/// The reader accepts the historical one-empty-ring spelling, but the
/// writer emits the zero-ring spelling accepted by binary consumers.
#[test]
fn historical_empty_polygon_normalizes_to_zero_rings() {
    for (historical, canonical) in [
        (
            "0103000020E61000000100000000000000",
            "0103000020E610000000000000",
        ),
        (
            "0020000003000010E60000000100000000",
            "0020000003000010E600000000",
        ),
    ] {
        let read = from_ewkb_hex(historical).unwrap();
        assert_eq!(read.geometry, Dyn::Polygon(Polygon::<Pt>::default()));
        assert_eq!(
            to_ewkb_hex(&read.geometry, read.srid, read.byte_order),
            canonical
        );
    }
}

// ---------------------------------------------------------------- 3

/// Section 3 — pass-through parity. For every buffer whose outermost
/// type word sets no EWKB flag, `from_ewkb` equals `from_wkb` mapped
/// into `Ewkb { srid: None, .. }` / `EwkbError::Wkb`.
#[test]
fn pass_through_parity_over_the_corpus() {
    for (i, e) in corpus().iter().enumerate() {
        for order in BOTH_ORDERS {
            let plain = to_wkb(&e.g, order);
            let via_ewkb = from_ewkb(&plain).expect("plain WKB is valid EWKB");
            assert_eq!(via_ewkb.srid, None, "{}", e.wkt);
            assert_eq!(via_ewkb.byte_order, order, "{}", e.wkt);
            if i == POINT_EMPTY {
                // §9 exception 2: `NaN != NaN` makes the value
                // comparison vacuously false, so parity is asserted on
                // bytes for this row — re-emitting what each reader
                // produced gives the same buffer.
                assert_eq!(to_wkb(&via_ewkb.geometry, order), plain, "{}", e.wkt);
                assert_eq!(
                    to_wkb(&from_wkb(&plain).unwrap(), order),
                    plain,
                    "{}",
                    e.wkt
                );
            } else {
                assert_eq!(Ok(via_ewkb.geometry), from_wkb(&plain), "{}", e.wkt);
            }
        }
    }
}

/// The same parity on every error-shaped buffer of `§9` that is class 1
/// — enumerated rather than given as a range, because the table is not
/// in numeric order. Row 23 is excluded: its outermost record sets the
/// SRID flag, so it is class 2.
#[test]
fn pass_through_parity_on_malformed_buffers() {
    let malformed: Vec<Vec<u8>> = vec![
        vec![],                                                     // row 17
        vec![0x01, 0x01, 0x00],                                     // row 18
        vec![0x02],                                                 // row 19
        vec![0x01, 0x08, 0x00, 0x00, 0x00],                         // row 20
        vec![0x01, 0xE9, 0x03, 0x00, 0x00],                         // row 11 (ISO 1001)
        vec![0x01, 0x02, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF], // row 25
        vec![0x01, 0x06, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF], // row 25b
        vec![
            0x01, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ], // row 24
    ];
    for b in &malformed {
        assert_eq!(
            from_ewkb(b).map(|e| e.geometry).map_err(|e| match e {
                EwkbError::Wkb(w) => w,
                other => panic!("expected a delegated error, got {other}"),
            }),
            from_wkb(b),
            "{b:02X?}"
        );
    }
}

// ---------------------------------------------------------------- 4

/// Section 4 — writer parity. With no SRID the output is byte-identical
/// to plain WKB, for the whole corpus in both orders.
#[test]
fn writer_parity_without_a_srid() {
    for e in &corpus() {
        for order in BOTH_ORDERS {
            assert_eq!(to_ewkb(&e.g, None, order), to_wkb(&e.g, order), "{}", e.wkt);
        }
    }
}

/// The bring-your-own-polygon writer mirrors it.
#[test]
fn polygon_writer_parity_without_a_srid() {
    for pg in [
        sample_polygon(),
        Polygon::<Pt>::new(Ring::new()),
        Polygon::with_inners(Ring::new(), vec![sample_hole()]),
    ] {
        for order in BOTH_ORDERS {
            assert_eq!(
                to_ewkb_polygon(&pg, None, order),
                to_wkb_polygon(&pg, order)
            );
            // and with an SRID it is the same record plus the header
            let with = to_ewkb_polygon(&pg, Some(Srid::new(4326)), order);
            assert_eq!(with.len(), to_wkb_polygon(&pg, order).len() + 4);
            let read = from_ewkb(&with).unwrap();
            assert_eq!(read.srid, Some(Srid::new(4326)));
            assert_eq!(read.geometry, Dyn::Polygon(pg.clone()));
        }
    }
}

// ---------------------------------------------------------------- 5

/// Section 5 — `§9`'s reader rows that this crate decides itself
/// (classes 2 and 3). Where a row's buffer ends in `…` the elided tail
/// is empty: every one is decided from the header alone.
#[test]
fn matrix_rows_this_crate_decides() {
    let cases: [(&[u8], EwkbError); 8] = [
        // rows 8, 9, 10 — Z, M, Z+SRID
        (
            &[0x01, 0x01, 0x00, 0x00, 0x80],
            EwkbError::DimensionFlag {
                type_word: 0x8000_0001,
            },
        ),
        (
            &[0x01, 0x01, 0x00, 0x00, 0x40],
            EwkbError::DimensionFlag {
                type_word: 0x4000_0001,
            },
        ),
        (
            &[0x01, 0x01, 0x00, 0x00, 0xA0],
            EwkbError::DimensionFlag {
                type_word: 0xA000_0001,
            },
        ),
        // rows 12, 13 — bounding box, decided before Z
        (
            &[0x01, 0x01, 0x00, 0x00, 0x10],
            EwkbError::BoundingBoxFlag {
                type_word: 0x1000_0001,
            },
        ),
        (
            &[0x01, 0x01, 0x00, 0x00, 0xB0],
            EwkbError::BoundingBoxFlag {
                type_word: 0xB000_0001,
            },
        ),
        // rows 14, 15 — the SRID field is this crate's own
        (
            &[0x01, 0x01, 0x00, 0x00, 0x20],
            EwkbError::TruncatedSrid {
                type_word: 0x2000_0001,
            },
        ),
        (
            &[0x01, 0x01, 0x00, 0x00, 0x20, 0xE6, 0x10],
            EwkbError::TruncatedSrid {
                type_word: 0x2000_0001,
            },
        ),
        // row 16 — the SRID was complete, so from_wkb reports the body
        (
            &[0x01, 0x01, 0x00, 0x00, 0x20, 0xE6, 0x10, 0x00, 0x00],
            EwkbError::Wkb(WkbError::UnexpectedEof),
        ),
    ];
    for (bytes, want) in cases {
        assert_eq!(from_ewkb(bytes).unwrap_err(), want, "{bytes:02X?}");
    }
}

/// `§9`'s rows whose verdict belongs to `from_wkb`, under the names the
/// owner's decision 5 gave them. Before that split all six returned one
/// `UnsupportedDimension`, which was true of two and false of four.
#[test]
fn matrix_rows_the_wkb_reader_decides() {
    let cases: [(&[u8], WkbError); 6] = [
        // row 11 — ISO 1001 sets no high bit, so it is the WKB crate's
        (
            &[0x01, 0xE9, 0x03, 0x00, 0x00],
            WkbError::HigherDimension { type_word: 1001 },
        ),
        // row 23 — SRID flag on a *nested* member
        (
            &[
                0x01, 0x04, 0x00, 0x00, 0x20, 0xE6, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
                0x01, 0x00, 0x00, 0x20, 0xE6, 0x10, 0x00, 0x00,
            ],
            WkbError::UnexpectedSridFlag {
                type_word: 0x2000_0001,
            },
        ),
        // row 23b — bounding-box flag on a nested member
        (
            &[
                0x01, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x10,
            ],
            WkbError::UnrecognisedTypeWord {
                type_word: 0x1000_0001,
            },
        ),
        // row 23c — truncated SRID on a nested member
        (
            &[
                0x01, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x20,
            ],
            WkbError::UnexpectedSridFlag {
                type_word: 0x2000_0001,
            },
        ),
        // row 21 — an unknown *base* code survives the strip intact
        (
            &[0x01, 0x08, 0x00, 0x00, 0x20, 0xE6, 0x10, 0x00, 0x00],
            WkbError::UnknownGeometryType(8),
        ),
        // row 30 — an undefined high bit, no SRID to strip
        (
            &[0x01, 0x01, 0x00, 0x00, 0x08],
            WkbError::UnrecognisedTypeWord {
                type_word: 0x0800_0001,
            },
        ),
    ];
    for (bytes, want) in cases {
        assert_eq!(
            from_ewkb(bytes).unwrap_err(),
            EwkbError::Wkb(want),
            "{bytes:02X?}"
        );
    }
}

/// `§9`'s rows 1, 2, 3, 5, 6 and 22. Rows 4, 7 and 27-29 are
/// covered by the corpus and golden-vector sections above.
#[test]
fn matrix_rows_that_succeed() {
    let point = Dyn::Point(Pt::new(1.0, 2.0));
    let body_le = [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x40,
    ];

    // row 1 — pass-through, no SRID
    let mut r1 = vec![0x01, 0x01, 0x00, 0x00, 0x00];
    r1.extend_from_slice(&body_le);
    assert_eq!(
        from_ewkb(&r1),
        Ok(Ewkb {
            srid: None,
            geometry: point.clone(),
            byte_order: ByteOrder::LittleEndian
        })
    );

    // rows 3, 5, 6 — SRID 4326, 0, and u32::MAX
    for (srid_bytes, want) in [
        ([0xE6, 0x10, 0x00, 0x00], Srid::new(4326)),
        ([0x00, 0x00, 0x00, 0x00], Srid::UNKNOWN),
        ([0xFF, 0xFF, 0xFF, 0xFF], Srid::new(u32::MAX)),
    ] {
        let mut b = vec![0x01, 0x01, 0x00, 0x00, 0x20];
        b.extend_from_slice(&srid_bytes);
        b.extend_from_slice(&body_le);
        let e = from_ewkb(&b).unwrap();
        assert_eq!(e.srid, Some(want));
        assert_eq!(e.geometry, point);
    }

    // row 2 — big-endian, no SRID. Compared field-wise, because `Ewkb`'s
    // derived PartialEq includes the order, so rows 1 and 2 are
    // genuinely unequal values.
    let mut r2 = vec![0x00, 0x00, 0x00, 0x00, 0x01];
    r2.extend_from_slice(&1.0f64.to_be_bytes());
    r2.extend_from_slice(&2.0f64.to_be_bytes());
    let e2 = from_ewkb(&r2).unwrap();
    assert_eq!(e2.srid, None);
    assert_eq!(e2.geometry, point);
    assert_eq!(e2.byte_order, ByteOrder::BigEndian);

    // row 22 — a trailing byte survives the strip
    let mut r22 = vec![0x01, 0x01, 0x00, 0x00, 0x20, 0xE6, 0x10, 0x00, 0x00];
    r22.extend_from_slice(&body_le);
    r22.push(0xFF);
    assert_eq!(
        from_ewkb(&r22),
        Err(EwkbError::Wkb(WkbError::TrailingBytes))
    );
}

// ---------------------------------------------------------------- 6

/// Section 6 — byte-order fidelity. `from_ewkb(b)?.byte_order` agrees
/// with `b[0]` for every self-produced corpus buffer. The golden
/// literals are out of scope here (`R15.6`).
#[test]
fn byte_order_matches_the_flag_the_writer_emitted() {
    for e in &corpus() {
        for order in BOTH_ORDERS {
            for srid in SRIDS {
                let b = to_ewkb(&e.g, srid, order);
                let read = from_ewkb(&b).unwrap();
                assert_eq!(read.byte_order, order, "{}", e.wkt);
                assert_eq!(b[0], u8::from(order == ByteOrder::LittleEndian));
                assert_eq!(to_ewkb(&read.geometry, read.srid, read.byte_order), b);
            }
        }
    }
}

// ---------------------------------------------------------------- 7

/// Section 7 — the adversarial set.
#[test]
fn adversarial_inputs_error_rather_than_panic() {
    // Deep nesting past MAX_DEPTH.
    let mut deep = Vec::new();
    for _ in 0..10_000 {
        deep.extend_from_slice(&[0x01, 0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00]);
    }
    assert_eq!(
        from_ewkb(&deep),
        Err(EwkbError::Wkb(WkbError::NestingTooDeep))
    );

    // A 1 MiB buffer that *does* enter the SRID branch.
    let mut big = vec![0x01, 0x01, 0x00, 0x00, 0x20, 0xE6, 0x10, 0x00, 0x00];
    big.extend(core::iter::repeat_n(0xFF, 1 << 20));
    assert!(matches!(from_ewkb(&big), Err(EwkbError::Wkb(_))));

    // Every one- and two-byte prefix of every corpus buffer: none may panic.
    for e in &corpus() {
        for order in BOTH_ORDERS {
            let b = to_ewkb(&e.g, Some(Srid::new(4326)), order);
            for end in 0..b.len().min(3) {
                let _ = from_ewkb(&b[..end]);
            }
            // and every proper prefix errors
            for end in 0..b.len() {
                assert!(from_ewkb(&b[..end]).is_err(), "{} prefix {end}", e.wkt);
            }
        }
    }
}

// ---------------------------------------------------------------- 8

/// Section 8 — `§10`'s downstream integration contract, as properties of
/// the public API. Names no database.
#[test]
fn integration_contract_holds() {
    const fn assert_send_sync_static<T: Send + Sync + 'static>() {}
    const _: () = assert_send_sync_static::<EwkbError>();
    const _: () = assert_send_sync_static::<Ewkb<Dyn>>();
    const _: () = assert_send_sync_static::<Srid>();

    // B1 — Debug + Display, and Error under std.
    let e = EwkbError::DimensionFlag {
        type_word: 0x8000_0001,
    };
    assert!(!format!("{e}").is_empty());
    assert!(!format!("{e:?}").is_empty());
    #[cfg(feature = "std")]
    let _: &dyn std::error::Error = &e;

    // B3 — reading takes a borrowed slice of exactly the field's bytes.
    let owned = to_ewkb(
        &Dyn::Point(Pt::new(1.0, 2.0)),
        Some(Srid::new(4326)),
        ByteOrder::LittleEndian,
    );
    let borrowed: &[u8] = &owned;
    assert!(from_ewkb(borrowed).is_ok());

    // B5 — the crate does not validate geometric well-formedness; a
    // one-point LINESTRING encodes here and is the server's to reject.
    let one_point = Dyn::LineString(Linestring(vec![Pt::new(1.0, 2.0)]));
    let b = to_ewkb(&one_point, Some(Srid::new(4326)), ByteOrder::LittleEndian);
    assert_eq!(from_ewkb(&b).unwrap().geometry, one_point);
}

// ---------------------------------------------------------------- hex

/// Hex EWKB — the text form of a `PostGIS` `geometry` column (owner
/// decision 3 put this in v1; row A11 settles uppercase-out).
#[test]
fn hex_round_trips_the_whole_corpus() {
    for (i, e) in corpus().iter().enumerate() {
        for order in BOTH_ORDERS {
            for srid in SRIDS {
                let s = to_ewkb_hex(&e.g, srid, order);
                assert!(
                    s.bytes()
                        .all(|b| b.is_ascii_digit() || (b'A'..=b'F').contains(&b)),
                    "row A11: uppercase only, got {s}"
                );
                let back = from_ewkb_hex(&s).unwrap();
                assert_eq!(back.srid, srid, "{}", e.wkt);
                if i != POINT_EMPTY {
                    assert_eq!(back.geometry, e.g, "{}", e.wkt);
                }
                // lowercase is accepted on input
                assert_eq!(from_ewkb_hex(&s.to_lowercase()).unwrap().srid, srid);
            }
        }
    }
}

/// The hex form agrees with the binary form byte for byte, and with
/// `PostGIS`'s own golden literals.
#[test]
fn hex_agrees_with_binary_and_with_postgis() {
    for e in &corpus() {
        assert_eq!(
            from_ewkb_hex(e.golden_ndr).unwrap().srid,
            Some(Srid::new(4326)),
            "{}",
            e.wkt
        );
        // uppercase golden literal == what we would print for those bytes
        let bytes = unhex(e.golden_ndr);
        let read = from_ewkb(&bytes).unwrap();
        let ours = to_ewkb_hex(&read.geometry, read.srid, read.byte_order);
        assert_eq!(ours, e.golden_ndr, "{}", e.wkt);
    }
}

/// Malformed hex names the offending offset.
#[test]
fn hex_errors_name_their_offset() {
    assert_eq!(
        from_ewkb_hex("ABC"),
        Err(EwkbError::InvalidHex { index: 2 })
    );
    assert_eq!(
        from_ewkb_hex("00GG"),
        Err(EwkbError::InvalidHex { index: 2 })
    );
    assert_eq!(
        from_ewkb_hex(""),
        Err(EwkbError::Wkb(WkbError::UnexpectedEof))
    );
}

/// §9 row 31, and §7.1's fourth property: **step 4 clears only the SRID
/// bit.** An undefined high bit alongside the SRID flag survives the
/// strip and fails closed, rather than being silently masked away.
///
/// This is the one row that pins that property. Without it, changing
/// the strip's mask from `tag & !EWKB_SRID` to `tag & 0x0000_FFFF`
/// leaves the whole suite green while `0x2800_0001` starts parsing as
/// an ordinary point — established by mutation, not by argument.
#[test]
fn row_31_only_the_srid_bit_is_cleared() {
    let mut b = vec![0x01, 0x01, 0x00, 0x00, 0x28, 0xE6, 0x10, 0x00, 0x00];
    b.extend_from_slice(&1.0f64.to_le_bytes());
    b.extend_from_slice(&2.0f64.to_le_bytes());
    assert_eq!(
        from_ewkb(&b),
        Err(EwkbError::Wkb(WkbError::UnrecognisedTypeWord {
            type_word: 0x0800_0001
        })),
        "bit 27 must survive the strip and fail closed"
    );
    // The same word without the SRID flag behaves identically, which is
    // what "only the SRID bit" means.
    assert_eq!(
        from_ewkb(&[0x01, 0x01, 0x00, 0x00, 0x08]),
        Err(EwkbError::Wkb(WkbError::UnrecognisedTypeWord {
            type_word: 0x0800_0001
        }))
    );
}

/// §10 B7 is a read-path guarantee and the crate has two read entry
/// points. Hex input must error rather than panic, on anything.
#[test]
fn hex_reader_never_panics() {
    let long_valid = "FF".repeat(100_000);
    let long_odd = "0".repeat(99_999);
    for s in [
        "",
        "0",
        "0Z1",
        "GG",
        "0\u{e9}",
        "\u{1F600}",
        "010100002",
        long_valid.as_str(),
        long_odd.as_str(),
    ] {
        let _ = from_ewkb_hex(s);
    }
    let good = to_ewkb_hex(
        &Dyn::Point(Pt::new(1.0, 2.0)),
        Some(Srid::new(4326)),
        ByteOrder::LittleEndian,
    );
    for end in 0..good.len() {
        let _ = from_ewkb_hex(&good[..end]);
    }
    assert!(from_ewkb_hex(&good).is_ok());
}
