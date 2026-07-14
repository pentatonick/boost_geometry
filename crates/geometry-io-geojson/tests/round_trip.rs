//! RFC 7946 round-trip parity (milestone M-IO4).
//!
//! For every normative RFC 7946 example: `from_geojson` → `to_geojson` →
//! `from_geojson` and assert the two [`DynGeometry`] values are deeply
//! equal. `DynGeometry` derives `PartialEq` and the CS markers implement
//! `PartialEq`/`Eq`, so equality compares coordinates structurally.

use geometry_cs::Cartesian;
use geometry_io_geojson::{GeoJsonError, WriteGeoJson, from_geojson, to_geojson};
use geometry_model::{DynGeometry, Point2D, Polygon, Ring};
use geometry_tag::PointTag;
use geometry_trait::Geometry;

type Pt = Point2D<f64, Cartesian>;

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

/// RFC 7946 §3.1.8 allows every geometry kind, including another collection,
/// as a collection member. This drives every public dynamic writer arm.
#[test]
fn collection_with_every_geometry_kind_round_trips() {
    assert_round_trip(
        r#"{"type":"GeometryCollection","geometries":[
            {"type":"Point","coordinates":[1,2]},
            {"type":"LineString","coordinates":[[0,0],[1,1]]},
            {"type":"Polygon","coordinates":[[[0,0],[0,2],[2,2],[0,0]]]},
            {"type":"MultiPoint","coordinates":[[1,2],[3,4]]},
            {"type":"MultiLineString","coordinates":[[[0,0],[1,1]],[[2,2],[3,3]]]},
            {"type":"MultiPolygon","coordinates":[[[[0,0],[0,1],[1,1],[0,0]]]]},
            {"type":"GeometryCollection","geometries":[
                {"type":"Point","coordinates":[5,6]}
            ]}
        ]}"#,
    );
}

/// RFC 7946 §3 and RFC 8259 grammar failures are reported through the public
/// error type rather than panicking or accepting malformed coordinates.
#[test]
#[allow(
    clippy::too_many_lines,
    reason = "one public error-contract table is easier to audit against RFC grammar cases"
)]
fn malformed_documents_cover_public_error_contract() {
    assert_eq!(from_geojson(""), Err(GeoJsonError::UnexpectedEof));
    assert!(matches!(from_geojson("?"), Err(GeoJsonError::Json(_))));
    assert!(matches!(from_geojson("tru"), Err(GeoJsonError::Json(_))));
    assert!(matches!(
        from_geojson("{} trailing"),
        Err(GeoJsonError::Json(_))
    ));
    assert!(matches!(from_geojson("{]"), Err(GeoJsonError::Json(_))));
    assert!(matches!(
        from_geojson(r#"{"type" "Point"}"#),
        Err(GeoJsonError::Json(_))
    ));
    assert!(matches!(
        from_geojson(r#"{"type":"Point" "coordinates":[0,0]}"#),
        Err(GeoJsonError::Json(_))
    ));
    assert!(matches!(
        from_geojson(r#"{"type":"Point","coordinates":[0 0]}"#),
        Err(GeoJsonError::Json(_))
    ));
    assert!(matches!(
        from_geojson(r#"{"type":"Po\uint","coordinates":[0,0]}"#),
        Err(GeoJsonError::Json(_))
    ));
    assert_eq!(
        from_geojson(r#"{"type":"Point""#),
        Err(GeoJsonError::UnexpectedEof)
    );
    assert_eq!(
        from_geojson(r#"{"type":"Point}"#),
        Err(GeoJsonError::UnexpectedEof)
    );
    assert_eq!(
        from_geojson(r#"{"type":"Point\"#),
        Err(GeoJsonError::UnexpectedEof)
    );
    assert_eq!(
        from_geojson(r#"{"type":"Point\t","coordinates":[0,0]}"#),
        Err(GeoJsonError::UnknownGeometryType("Point\t".into()))
    );

    assert_eq!(from_geojson("true"), Err(GeoJsonError::ExpectedType));
    assert_eq!(from_geojson("false"), Err(GeoJsonError::ExpectedType));
    assert_eq!(from_geojson("null"), Err(GeoJsonError::ExpectedType));
    assert_eq!(from_geojson("{}"), Err(GeoJsonError::ExpectedType));
    assert_eq!(
        from_geojson(r#"{"type":1,"coordinates":[0,0]}"#),
        Err(GeoJsonError::ExpectedType)
    );
    assert_eq!(
        from_geojson(r#"{"type":"FeatureCollection","features":[]}"#),
        Err(GeoJsonError::UnsupportedType("FeatureCollection".into()))
    );
    assert_eq!(
        from_geojson(r#"{"type":"Curve","coordinates":[]}"#),
        Err(GeoJsonError::UnknownGeometryType("Curve".into()))
    );

    for malformed in [
        r#"{"type":"Point"}"#,
        r#"{"type":"Point","coordinates":"bad"}"#,
        r#"{"type":"Point","coordinates":[0]}"#,
        r#"{"type":"Point","coordinates":[null,0]}"#,
        r#"{"type":"Point","coordinates":[0,null]}"#,
        r#"{"type":"LineString","coordinates":[0]}"#,
        r#"{"type":"Polygon","coordinates":[0]}"#,
        r#"{"type":"MultiLineString","coordinates":[0]}"#,
        r#"{"type":"MultiPolygon","coordinates":[0]}"#,
        r#"{"type":"GeometryCollection"}"#,
        r#"{"type":"GeometryCollection","geometries":{}}"#,
        r#"{"type":"Polygon","coordinates":[[],0]}"#,
        r#"{"type":"MultiLineString","coordinates":[[0]]}"#,
        r#"{"type":"MultiPolygon","coordinates":[[[0]]]}"#,
    ] {
        assert_eq!(
            from_geojson(malformed),
            Err(GeoJsonError::MalformedCoordinates),
            "accepted malformed document: {malformed}"
        );
    }

    let unicode = r#"{"é":"✓😀","type":"Point","coordinates":[0,0]}"#;
    assert!(from_geojson(unicode).is_ok());

    let deeply_nested = format!("{}0{}", "[".repeat(130), "]".repeat(130));
    assert!(matches!(
        from_geojson(&deeply_nested),
        Err(GeoJsonError::Json(message)) if message == "nesting too deep"
    ));

    assert_eq!(
        GeoJsonError::UnexpectedEof.to_string(),
        "unexpected end of input"
    );
    assert_eq!(
        GeoJsonError::ExpectedType.to_string(),
        "missing GeoJSON \"type\" member"
    );
    assert_eq!(
        GeoJsonError::MalformedCoordinates.to_string(),
        "malformed or missing coordinates"
    );
    assert!(GeoJsonError::Json("bad".into()).to_string().contains("bad"));
    assert!(
        GeoJsonError::UnknownGeometryType("Curve".into())
            .to_string()
            .contains("Curve")
    );
    assert!(
        GeoJsonError::UnsupportedType("Feature".into())
            .to_string()
            .contains("Feature")
    );
}

#[test]
fn empty_polygon_and_bare_ring_use_the_public_geometry_api() {
    let empty = from_geojson(r#"{"type":"Polygon","coordinates":[]}"#).unwrap();
    assert_eq!(empty, DynGeometry::Polygon(Polygon::<Pt>::new(Ring::new())));

    let ring = Ring::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(0.0, 1.0),
        Pt::new(1.0, 0.0),
        Pt::new(0.0, 0.0),
    ]);
    assert_eq!(
        to_geojson(&ring),
        r#"{"type":"Polygon","coordinates":[[[0,0],[0,1],[1,0],[0,0]]]}"#
    );
}

struct ExternalPointWriter;

impl Geometry for ExternalPointWriter {
    type Kind = PointTag;
    type Point = Pt;
}

impl WriteGeoJson for ExternalPointWriter {
    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"Point","coordinates":[3,4]}"#)
    }
}

#[test]
fn external_writer_uses_the_public_default_capacity_hint() {
    assert_eq!(
        to_geojson(&ExternalPointWriter),
        r#"{"type":"Point","coordinates":[3,4]}"#
    );
}
