//! RFC 7946 round-trip parity (milestone M-IO4).
//!
//! For every normative RFC 7946 example: `from_geojson` → `to_geojson` →
//! `from_geojson` and assert the two [`DynGeometry`] values are deeply
//! equal. `DynGeometry` derives `PartialEq` and the CS markers implement
//! `PartialEq`/`Eq`, so equality compares coordinates structurally.

use geometry_cs::Cartesian;
use geometry_io_geojson::{from_geojson, to_geojson};
use geometry_model::DynGeometry;

/// Parse, re-emit, re-parse, and assert the round trip is a fixed point.
fn assert_round_trip(input: &str) {
    let first: DynGeometry<f64, Cartesian> = from_geojson(input).unwrap();
    let emitted = to_geojson(&first);
    let second: DynGeometry<f64, Cartesian> = from_geojson(&emitted).unwrap();
    assert_eq!(first, second, "round trip changed the geometry: {emitted}");
}

#[test]
fn point_round_trips() {
    assert_round_trip(r#"{"type":"Point","coordinates":[100.0,0.0]}"#);
}

#[test]
fn linestring_round_trips() {
    assert_round_trip(r#"{"type":"LineString","coordinates":[[100.0,0.0],[101.0,1.0]]}"#);
}

#[test]
fn polygon_with_hole_round_trips() {
    assert_round_trip(
        r#"{"type":"Polygon","coordinates":[
            [[100.0,0.0],[101.0,0.0],[101.0,1.0],[100.0,1.0],[100.0,0.0]],
            [[100.8,0.8],[100.8,0.2],[100.2,0.2],[100.2,0.8],[100.8,0.8]]
        ]}"#,
    );
}

#[test]
fn multipoint_round_trips() {
    assert_round_trip(r#"{"type":"MultiPoint","coordinates":[[100.0,0.0],[101.0,1.0]]}"#);
}

#[test]
fn multilinestring_round_trips() {
    assert_round_trip(
        r#"{"type":"MultiLineString","coordinates":[
            [[100.0,0.0],[101.0,1.0]],
            [[102.0,2.0],[103.0,3.0]]
        ]}"#,
    );
}

#[test]
fn multipolygon_round_trips() {
    assert_round_trip(
        r#"{"type":"MultiPolygon","coordinates":[
            [[[102.0,2.0],[103.0,2.0],[103.0,3.0],[102.0,3.0],[102.0,2.0]]],
            [[[100.0,0.0],[101.0,0.0],[101.0,1.0],[100.0,1.0],[100.0,0.0]]]
        ]}"#,
    );
}

#[test]
fn geometry_collection_round_trips() {
    assert_round_trip(
        r#"{"type":"GeometryCollection","geometries":[
            {"type":"Point","coordinates":[100.0,0.0]},
            {"type":"LineString","coordinates":[[101.0,0.0],[102.0,1.0]]}
        ]}"#,
    );
}
