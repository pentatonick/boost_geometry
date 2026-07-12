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

use geometry_io_wkt::{from_wkt, to_wkt};

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
