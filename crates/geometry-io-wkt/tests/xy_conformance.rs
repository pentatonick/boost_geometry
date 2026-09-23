//! `PostGIS` XY conformance regressions.

use geometry_cs::Cartesian;
use geometry_io_wkt::{from_wkt, write_wkt};
use geometry_model::Point2D;

type Pt = Point2D<f64, Cartesian>;

#[test]
fn extra_ordinates_are_not_discarded() {
    assert!(
        from_wkt("POINT(1 2 3)").is_err(),
        "[xy-conformance] expected rejection, not a projected XY point"
    );
}

#[test]
fn nan_is_a_supported_coordinate() {
    assert!(
        from_wkt("POINT(NaN 1)").is_ok(),
        "[xy-conformance] expected a populated point containing NaN"
    );
}

#[test]
fn infinity_returns_an_error_instead_of_panicking_or_writing_text() {
    let observed =
        std::panic::catch_unwind(|| write_wkt(&Pt::new(f64::INFINITY, 1.0), &mut String::new()));
    assert!(
        matches!(observed, Ok(Err(_))),
        "[xy-conformance] expected an encoding error, observed {observed:?}"
    );
}

#[test]
fn negative_zero_survives_text_writing() {
    let mut text = String::new();
    write_wkt(&Pt::new(-0.0, 0.0), &mut text).unwrap();
    assert_eq!(text, "POINT(-0 0)", "[xy-conformance] signed zero");
}

#[test]
fn generic_storage_does_not_imply_xyz_codec_support() {
    use geometry_io_wkt::{WktWriteError, to_wkt};
    use geometry_model::{GeometryValue, Point3D};

    type Value = GeometryValue<Point3D<f64>>;
    for value in [
        Value::Point(None),
        Value::Point(Some(Point3D::new(1.0, 2.0, 3.0))),
        Value::MultiPoint(vec![None]),
        Value::GeometryCollection(vec![]),
    ] {
        assert_eq!(
            to_wkt(&value),
            Err(WktWriteError::UnsupportedDimension { dimensions: 3 })
        );
    }
}

#[test]
fn empty_members_survive_text_but_cannot_be_converted_to_legacy_geometry() {
    use geometry_io_wkt::{from_wkt_2d, to_wkt};
    use geometry_model::{DynGeometry, EmptyPointError};

    let text = "GEOMETRYCOLLECTION(POINT EMPTY,MULTIPOINT(EMPTY,(1 2),EMPTY))";
    let geometry = from_wkt_2d(text).unwrap();
    assert_eq!(to_wkt(&geometry).unwrap(), text);
    assert_eq!(DynGeometry::try_from(geometry), Err(EmptyPointError));
}

#[test]
fn populated_nan_pair_is_not_an_empty_point() {
    use geometry_io_wkt::{from_wkt_2d, to_wkt};
    use geometry_model::{DynGeometry, GeometryValue};

    let geometry = from_wkt_2d("POINT(NaN NaN)").unwrap();
    assert!(matches!(&geometry, GeometryValue::Point(Some(_))));
    assert_eq!(to_wkt(&geometry).unwrap(), "POINT(NaN NaN)");
    assert!(matches!(
        DynGeometry::try_from(geometry),
        Ok(DynGeometry::Point(_))
    ));
}

#[test]
fn all_empty_kinds_and_member_positions_round_trip() {
    use geometry_io_wkt::{from_wkt_2d, to_wkt};

    for text in [
        "POINT EMPTY",
        "LINESTRING EMPTY",
        "POLYGON EMPTY",
        "MULTIPOINT EMPTY",
        "MULTILINESTRING EMPTY",
        "MULTIPOLYGON EMPTY",
        "GEOMETRYCOLLECTION EMPTY",
        "MULTIPOINT(EMPTY,(1 2),EMPTY)",
        "MULTILINESTRING(EMPTY,(0 0,1 1),EMPTY)",
        "MULTIPOLYGON(EMPTY,((0 0,1 0,1 1,0 0)),EMPTY)",
        "GEOMETRYCOLLECTION(POINT EMPTY,GEOMETRYCOLLECTION(MULTIPOINT(EMPTY),POLYGON EMPTY))",
    ] {
        let geometry = from_wkt_2d(text).unwrap();
        assert_eq!(to_wkt(&geometry).unwrap(), text);
        assert_eq!(from_wkt_2d(to_wkt(&geometry).unwrap()).unwrap(), geometry);
    }
    assert_eq!(
        to_wkt(&from_wkt_2d("pointempty").unwrap()).unwrap(),
        "POINT EMPTY"
    );
}

#[test]
fn every_kind_rejects_dimension_qualifiers_even_when_empty() {
    use geometry_io_wkt::{WktError, from_wkt_2d};

    for kind in [
        "POINT",
        "LINESTRING",
        "POLYGON",
        "MULTIPOINT",
        "MULTILINESTRING",
        "MULTIPOLYGON",
        "GEOMETRYCOLLECTION",
    ] {
        for qualifier in ["Z", "M", "ZM"] {
            for gap in ["", " "] {
                let text = format!("{kind}{gap}{qualifier} EMPTY");
                assert!(
                    matches!(
                        from_wkt_2d(&text),
                        Err(WktError::UnsupportedDimension { .. })
                    ),
                    "{text}"
                );
                assert!(matches!(
                    from_wkt_2d(format!("GEOMETRYCOLLECTION({text})")),
                    Err(WktError::UnsupportedDimension { .. })
                ));
            }
        }
    }
}

#[test]
fn structure_checks_apply_to_reads_and_writes() {
    use geometry_io_wkt::{
        GeometryStructureError, WktError, WktWriteError, from_wkt_2d, to_wkt, to_wkt_polygon,
    };
    use geometry_model::{Linestring, Polygon, Ring};

    for text in [
        "LINESTRING(0 0)",
        "MULTILINESTRING((0 0))",
        "POLYGON((0 0,1 0,0 0))",
        "POLYGON((0 0,1 0,1 1,0 1))",
        "MULTIPOLYGON(((0 0,1 0,1 1,0 1)))",
    ] {
        assert!(
            matches!(from_wkt_2d(text), Err(WktError::InvalidGeometry { .. })),
            "{text}"
        );
    }
    assert_eq!(
        to_wkt(&Linestring(vec![Pt::new(0.0, 0.0)])),
        Err(WktWriteError::InvalidGeometry(
            GeometryStructureError::TooFewPoints {
                minimum: 2,
                actual: 1
            }
        ))
    );
    let ring = Ring::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(1.0, 0.0),
        Pt::new(1.0, 1.0),
        Pt::new(0.0, 0.0),
    ]);
    for (polygon, reason) in [
        (
            Polygon::with_inners(Ring::new(), vec![ring.clone()]),
            GeometryStructureError::MissingExterior,
        ),
        (
            Polygon::with_inners(ring, vec![Ring::new()]),
            GeometryStructureError::EmptyInterior,
        ),
    ] {
        assert_eq!(
            to_wkt(&polygon),
            Err(WktWriteError::InvalidGeometry(reason))
        );
        assert_eq!(
            to_wkt_polygon(&polygon),
            Err(WktWriteError::InvalidGeometry(reason))
        );
    }
}

#[test]
fn ring_closure_uses_emitted_nan_and_signed_zero_tokens() {
    use geometry_io_wkt::{from_wkt_2d, to_wkt};
    use geometry_model::{Polygon, Ring};

    let nan_ring = Polygon::new(Ring::from_vec(vec![
        Pt::new(f64::from_bits(0x7ff8_0000_0000_0001), 0.0),
        Pt::new(1.0, 0.0),
        Pt::new(1.0, 1.0),
        Pt::new(f64::from_bits(0xfff8_0000_0000_0002), 0.0),
    ]));
    let text = to_wkt(&nan_ring).unwrap();
    assert_eq!(text, "POLYGON((NaN 0,1 0,1 1,NaN 0))");
    assert!(from_wkt_2d(text).is_ok());
    assert!(from_wkt_2d("POLYGON((0 0,1 0,1 1,-0 0))").is_err());
    let signed_ring = Polygon::new(Ring::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(1.0, 0.0),
        Pt::new(1.0, 1.0),
        Pt::new(-0.0, 0.0),
    ]));
    assert!(to_wkt(&signed_ring).is_err());
}

#[test]
fn empty_generic_containers_do_not_bypass_dimension_checks() {
    use geometry_io_wkt::{WktWriteError, to_wkt};
    use geometry_model::{
        Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point3D, Polygon, Ring,
    };
    let expected = Err(WktWriteError::UnsupportedDimension { dimensions: 3 });
    assert_eq!(to_wkt(&Linestring::<Point3D<f64>>::new()), expected);
    assert_eq!(to_wkt(&Ring::<Point3D<f64>>::new()), expected);
    assert_eq!(to_wkt(&Polygon::<Point3D<f64>>::new(Ring::new())), expected);
    assert_eq!(to_wkt(&MultiPoint::<Point3D<f64>>(vec![])), expected);
    assert_eq!(
        to_wkt(&MultiLinestring::<Linestring<Point3D<f64>>>(vec![])),
        expected
    );
    assert_eq!(
        to_wkt(&MultiPolygon::<Polygon<Point3D<f64>>>(vec![])),
        expected
    );
}

#[test]
fn custom_writer_errors_propagate_from_owned_and_streaming_apis() {
    use geometry_io_wkt::{WktWriteError, WriteWkt, to_wkt};
    struct Refuses;
    impl WriteWkt for Refuses {
        fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
            out.write_str("POINT(")?;
            Err(WktWriteError::InfiniteCoordinate)
        }
    }
    assert_eq!(to_wkt(&Refuses), Err(WktWriteError::InfiniteCoordinate));
    let mut partial = String::new();
    assert_eq!(
        write_wkt(&Refuses, &mut partial),
        Err(WktWriteError::InfiniteCoordinate)
    );
    assert_eq!(partial, "POINT(");
}

#[test]
fn nesting_limit_is_identical_for_reading_and_writing() {
    use geometry_io_wkt::{WktError, WktWriteError, from_wkt_2d, to_wkt};
    use geometry_model::GeometryValue;
    let mut geometry = GeometryValue::<Pt>::Point(None);
    let mut text = "POINT EMPTY".to_owned();
    for _ in 0..127 {
        geometry = GeometryValue::GeometryCollection(vec![geometry]);
        text = format!("GEOMETRYCOLLECTION({text})");
    }
    assert_eq!(to_wkt(&geometry).unwrap(), text);
    assert_eq!(from_wkt_2d(&text).unwrap(), geometry);
    geometry = GeometryValue::GeometryCollection(vec![geometry]);
    text = format!("GEOMETRYCOLLECTION({text})");
    assert_eq!(to_wkt(&geometry), Err(WktWriteError::NestingTooDeep));
    assert_eq!(from_wkt_2d(text), Err(WktError::NestingTooDeep));
}

#[test]
fn custom_polygon_checks_structure_and_infinity() {
    use geometry_io_wkt::{
        GeometryStructureError, WktWriteError, to_wkt_polygon, write_wkt_polygon,
    };
    use geometry_model::Ring;
    use geometry_trait::{Geometry, Polygon};
    struct Parcel {
        shell: Ring<Pt>,
        holes: Vec<Ring<Pt>>,
    }
    impl Geometry for Parcel {
        type Kind = geometry_tag::PolygonTag;
        type Point = Pt;
    }
    impl Polygon for Parcel {
        type Ring = Ring<Pt>;
        fn exterior(&self) -> &Self::Ring {
            &self.shell
        }
        fn interiors(&self) -> impl ExactSizeIterator<Item = &Self::Ring> {
            self.holes.iter()
        }
    }
    let ring = Ring::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(1.0, 0.0),
        Pt::new(1.0, 1.0),
        Pt::new(0.0, 0.0),
    ]);
    let mut parcel = Parcel {
        shell: Ring::new(),
        holes: vec![ring.clone()],
    };
    assert_eq!(
        to_wkt_polygon(&parcel),
        Err(WktWriteError::InvalidGeometry(
            GeometryStructureError::MissingExterior
        ))
    );
    parcel.shell = ring;
    parcel.holes[0].0[1] = Pt::new(f64::INFINITY, 0.0);
    assert_eq!(
        to_wkt_polygon(&parcel),
        Err(WktWriteError::InfiniteCoordinate)
    );
    assert_eq!(
        write_wkt_polygon(&parcel, &mut String::new()),
        Err(WktWriteError::InfiniteCoordinate)
    );
}

#[cfg(feature = "std")]
#[test]
fn public_errors_implement_std_error() {
    use geometry_io_wkt::{GeometryStructureError, WktError, WktWriteError};
    fn error<E: std::error::Error>() {}
    error::<WktError>();
    error::<WktWriteError>();
    error::<GeometryStructureError>();
    error::<geometry_model::EmptyPointError>();
}

#[test]
fn structural_errors_explain_the_invalid_shape() {
    use geometry_io_wkt::GeometryStructureError as Error;
    for (error, message) in [
        (
            Error::TooFewPoints {
                minimum: 4,
                actual: 2,
            },
            "expected at least 4 points, found 2",
        ),
        (Error::UnclosedRing, "polygon ring is not closed"),
        (
            Error::MissingExterior,
            "polygon has holes without an exterior",
        ),
        (Error::EmptyInterior, "polygon has an empty interior ring"),
    ] {
        assert_eq!(error.to_string(), message);
    }
}

#[test]
fn read_errors_explain_rejected_input_and_original_offsets() {
    for (input, message) in [
        (
            "POINT(1e999 0)",
            "number \"1e999\" at byte 6 exceeds finite f64 range",
        ),
        (
            "POINT Z EMPTY",
            "unsupported Z dimension at byte 6: this reader is XY only",
        ),
        ("POINT(1)", "expected 2 ordinates at byte 7, found 1"),
        (
            "LINESTRING(0 0)",
            "invalid geometry at byte 10: expected at least 2 points, found 1",
        ),
        (
            "POINT EMPTY",
            "the destination geometry cannot represent an empty point",
        ),
    ] {
        assert_eq!(from_wkt(input).unwrap_err().to_string(), message, "{input}");
    }
}

#[test]
fn write_errors_explain_geometry_and_sink_failures() {
    use geometry_io_wkt::{GeometryStructureError, WktWriteError as Error};
    for (error, message) in [
        (
            Error::UnsupportedDimension { dimensions: 3 },
            "expected XY point coordinates, found 3 ordinates",
        ),
        (
            Error::NestingTooDeep,
            "WKT nesting exceeds the supported depth",
        ),
        (
            Error::InfiniteCoordinate,
            "infinity has no PostGIS text spelling",
        ),
        (
            Error::from(GeometryStructureError::UnclosedRing),
            "invalid WKT geometry: polygon ring is not closed",
        ),
        (
            Error::from(core::fmt::Error),
            "WKT output failed: an error occurred when formatting an argument",
        ),
    ] {
        assert_eq!(error.to_string(), message);
    }
}
