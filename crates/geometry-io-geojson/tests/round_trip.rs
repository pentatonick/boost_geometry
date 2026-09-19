//! RFC 7946 round-trip parity (milestone M-IO4).
//!
//! For every normative RFC 7946 example: `from_geojson` → `to_geojson` →
//! `from_geojson` and assert the two [`DynGeometry`] values are deeply
//! equal. `DynGeometry` derives `PartialEq` and the CS markers implement
//! `PartialEq`/`Eq`, so equality compares coordinates structurally.

use geometry_cs::Cartesian;
use geometry_io_geojson::{GeoJsonError, WriteGeoJson, from_geojson, to_geojson};
use geometry_model::{DynGeometry, MultiPolygon, Point2D, Polygon, Ring};
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

// ---- Sweeps over every kind -------------------------------------------

/// Valid documents covering every kind, empty members, holes, a 3-D
/// position, and nesting — the corpus for the truncation sweep.
const EVERY_KIND_CORPUS: [&str; 12] = [
    r#"{"type":"Point","coordinates":[100.0,0.0]}"#,
    r#"{"type":"Point","coordinates":[100.0,0.0,500.0]}"#,
    r#"{"type":"LineString","coordinates":[[100.0,0.0],[101.0,1.0]]}"#,
    r#"{"type":"LineString","coordinates":[]}"#,
    r#"{"type":"Polygon","coordinates":[[[100.0,0.0],[101.0,0.0],[101.0,1.0],[100.0,1.0],[100.0,0.0]],[[100.8,0.8],[100.8,0.2],[100.2,0.2],[100.2,0.8],[100.8,0.8]]]}"#,
    r#"{"type":"Polygon","coordinates":[]}"#,
    r#"{"type":"MultiPoint","coordinates":[[100.0,0.0],[101.0,1.0]]}"#,
    r#"{"type":"MultiPoint","coordinates":[]}"#,
    r#"{"type":"MultiLineString","coordinates":[[],[[100.0,0.0],[101.0,1.0]]]}"#,
    r#"{"type":"MultiPolygon","coordinates":[[],[[[102.0,2.0],[103.0,2.0],[103.0,3.0],[102.0,3.0],[102.0,2.0]]]]}"#,
    r#"{"type":"GeometryCollection","geometries":[{"type":"Point","coordinates":[1,2]},{"type":"GeometryCollection","geometries":[]},{"type":"GeometryCollection","geometries":[{"type":"LineString","coordinates":[]}]}]}"#,
    r#"{"type":"GeometryCollection","geometries":[]}"#,
];

/// Every proper prefix of a valid document (original and canonical
/// spelling) must be an error and never a panic; non-whitespace trailing
/// garbage must be rejected.
#[test]
fn every_proper_prefix_is_an_error_and_trailing_garbage_is_rejected() {
    for input in EVERY_KIND_CORPUS {
        let parsed = from_geojson(input).unwrap_or_else(|e| panic!("{input}: {e}"));
        let canonical = to_geojson(&parsed);
        assert_eq!(
            from_geojson(&canonical),
            Ok(parsed.clone()),
            "canonical re-parse of {input}"
        );
        for text in [input, canonical.as_str()] {
            for end in (0..text.len()).filter(|&i| text.is_char_boundary(i)) {
                let prefix = &text[..end];
                assert!(
                    from_geojson(prefix).is_err(),
                    "prefix {prefix:?} of {text:?} was accepted"
                );
            }
            for garbage in ["}", "]", ",", "0", " x", "{}", "\u{feff}"] {
                let s = format!("{text}{garbage}");
                assert!(from_geojson(&s).is_err(), "{s:?} was accepted");
            }
        }
    }
}

/// A numeric literal whose exponent overflows `f64` is an invalid number,
/// not an infinity: RFC 8259 §6 has no Infinity/NaN, the writer has no
/// JSON spelling for one, and the sibling WKT crate rejects the same
/// literal.
#[test]
fn overflowing_exponent_is_an_invalid_number_not_an_infinity() {
    for doc in [
        r#"{"type":"Point","coordinates":[1e400,0]}"#,
        r#"{"type":"Point","coordinates":[0,-1e400]}"#,
        r#"{"type":"LineString","coordinates":[[1e400,0],[1,1]]}"#,
        r#"{"type":"Polygon","coordinates":[[[0,0],[1e400,0],[0,1],[0,0]]]}"#,
    ] {
        assert!(
            matches!(
                from_geojson(doc),
                Err(GeoJsonError::Json(message)) if message.contains("invalid number")
            ),
            "{doc} -> {:?}",
            from_geojson(doc)
        );
    }
}

/// The scalar writer documents a debug assertion as the tripwire for a
/// non-finite coordinate a caller built directly. Debug-only:
/// `cargo test --release` compiles the assertion out.
#[cfg(debug_assertions)]
#[test]
#[should_panic(expected = "GeoJSON cannot represent a non-finite coordinate")]
fn non_finite_point_coordinate_trips_the_writer_guard() {
    let _ = to_geojson(&Pt::new(f64::NAN, 0.0));
}

/// The same tripwire fires for a non-finite ordinate buried in a hole of a
/// `MultiPolygon` member, since every ordinate flows through the scalar
/// writer.
#[cfg(debug_assertions)]
#[test]
#[should_panic(expected = "GeoJSON cannot represent a non-finite coordinate")]
fn non_finite_hole_coordinate_trips_the_writer_guard() {
    let outer = Ring::<Pt>::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(0.0, 10.0),
        Pt::new(10.0, 10.0),
        Pt::new(0.0, 0.0),
    ]);
    let hole = Ring::<Pt>::from_vec(vec![
        Pt::new(1.0, 1.0),
        Pt::new(1.0, f64::INFINITY),
        Pt::new(2.0, 2.0),
        Pt::new(1.0, 1.0),
    ]);
    let _ = to_geojson(&MultiPolygon(vec![Polygon::with_inners(outer, vec![hole])]));
}

/// Extreme but finite scalars survive write → parse bit-exactly (the
/// shortest round-trip spelling is used, so no precision is lost).
#[test]
fn extreme_finite_scalars_round_trip_bit_exactly() {
    use geometry_trait::Point as _;
    for v in [
        0.1,
        1.0 / 3.0,
        1e300,
        -1e300,
        1e-300,
        5e-324,
        f64::MAX,
        f64::MIN_POSITIVE,
        9_007_199_254_740_993.0,
        1e16,
        123_456_789.123_456_79,
        2.5e-5,
        1e21,
        -0.0,
    ] {
        let p = Pt::new(v, -v);
        let text = to_geojson(&p);
        let back = match from_geojson(&text) {
            Ok(DynGeometry::Point(q)) => q,
            other => panic!("{text}: {other:?}"),
        };
        assert_eq!(back, p, "{text}");
        if v != 0.0 {
            assert_eq!(back.get::<0>().to_bits(), v.to_bits(), "{text}");
            assert_eq!(back.get::<1>().to_bits(), (-v).to_bits(), "{text}");
        }
    }
}

/// Shape errors the RFC grammar makes possible: wrong nesting depth for
/// the declared kind, an inline position where a sequence is required,
/// key order and foreign members that must not matter, and JSON-level
/// slips that must be clear errors rather than mangled names.
#[test]
fn coordinate_shape_and_member_order_edge_cases() {
    for malformed in [
        r#"{"type":"Point","coordinates":[]}"#,
        r#"{"type":"Point","coordinates":[[1,2]]}"#,
        r#"{"type":"Point","coordinates":[[1,2],[3,4]]}"#,
        r#"{"type":"Point","coordinates":null}"#,
        r#"{"type":"LineString","coordinates":[1,2]}"#,
        r#"{"type":"LineString","coordinates":[1,2,3]}"#,
        r#"{"type":"MultiPoint","coordinates":[1,2]}"#,
        r#"{"type":"Polygon","coordinates":[[1,2],[3,4]]}"#,
        r#"{"type":"Polygon","coordinates":[[[1,2],[3,4]],[5,6]]}"#,
        r#"{"type":"MultiPolygon","coordinates":[[[1,2]]]}"#,
        r#"{"type":"GeometryCollection","geometries":[1]}"#,
        r#"{"type":"GeometryCollection","geometries":null}"#,
    ] {
        assert_eq!(
            from_geojson(malformed).ok(),
            None,
            "accepted malformed document: {malformed}"
        );
    }
    let p = DynGeometry::Point(Pt::new(1.0, 2.0));
    assert_eq!(
        from_geojson(r#"{"coordinates":[1,2],"type":"Point"}"#),
        Ok(p.clone())
    );
    assert_eq!(
        from_geojson(
            r#"{"bbox":[0,0,1,1],"type":"Point","foo":{"a":[null,true]},"coordinates":[1,2]}"#
        ),
        Ok(p)
    );
    // A backslash-u escape (RFC 8259 §7) in a type name or key is
    // documented as unsupported, so it must be a clear JSON error rather
    // than a silently mangled name. The backslash is built at runtime so
    // no tooling can pre-decode the escape out of this source.
    let backslash = char::from(92_u8);
    let escaped_type = format!(r#"{{"type":"Point{backslash}u0041","coordinates":[1,2]}}"#);
    let escaped_key = format!(r#"{{"ty{backslash}u0070e":"Point","coordinates":[1,2]}}"#);
    for malformed in [
        r#"{"type":"Point","coordinates":[1,2,]}"#,
        r#"{"type":"Point","coordinates":[.1,2]}"#,
        r#"{"type":"Point","coordinates":[-,2]}"#,
        r#"{"type":"Point","coordinates":[+1,2]}"#,
        r#"{"type":"Point","coordinates":[1,2]"#,
        "\u{feff}{\"type\":\"Point\",\"coordinates\":[1,2]}",
        escaped_type.as_str(),
        escaped_key.as_str(),
    ] {
        assert!(
            matches!(
                from_geojson(malformed),
                Err(GeoJsonError::Json(_) | GeoJsonError::UnexpectedEof)
            ),
            "{malformed}: {:?}",
            from_geojson(malformed)
        );
    }
}
