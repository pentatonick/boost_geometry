//! The RFC 7946 `GeoJSON` reader.
//!
//! [`from_geojson`] parses a JSON document into a runtime-tagged
//! [`DynGeometry`], dispatching on the object's `"type"` member and
//! walking `"coordinates"` (or `"geometries"` for a
//! `GeometryCollection`). `GeoJSON` is heterogeneous by construction — a
//! `GeometryCollection` mixes kinds — so the reader emits a
//! [`DynGeometry`] exactly like the sibling WKT crate's `from_wkt`.
//!
//! # Coordinate order and dimensions
//!
//! RFC 7946 §3.1.1 fixes coordinate order as `[longitude, latitude]`,
//! which maps directly to `(x, y)`. A third (altitude) ordinate, if
//! present, is read and discarded — this is a 2D port.
//!
//! # Scope
//!
//! Only the seven geometry object kinds (RFC 7946 §3.1) are supported.
//! `Feature` and `FeatureCollection` wrappers (§3.2/§3.3) are rejected
//! with [`GeoJsonError::UnsupportedType`]; unwrap the `geometry` member
//! yourself and pass it in. `"bbox"` and any foreign members are ignored.
//!
//! Reference: RFC 7946 §3 (geometry objects) and §1.5 (worked examples).

use alloc::vec::Vec;

use geometry_cs::Cartesian;
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};

use crate::json::{GeoJsonError, JsonValue, parse_geojson};

/// A concrete 2D Cartesian point — the coordinate type every parsed
/// geometry is built from.
type Pt = Point2D<f64, Cartesian>;

/// Parse a `GeoJSON` string into a runtime-tagged [`DynGeometry`].
///
/// Implements the seven RFC 7946 §3.1 geometry kinds: `Point`,
/// `LineString`, `Polygon`, `MultiPoint`, `MultiLineString`,
/// `MultiPolygon`, and `GeometryCollection`. Coordinates are read in
/// `GeoJSON` `[longitude, latitude]` order straight into `(x, y)`; a third
/// ordinate is discarded (this is a 2D port).
///
/// # Feature wrappers
///
/// A `Feature` or `FeatureCollection` object is **not** unwrapped: it is
/// rejected with [`GeoJsonError::UnsupportedType`]. Only bare geometry
/// objects are accepted. `"bbox"` and any unrecognised members are
/// ignored.
///
/// # Errors
///
/// Returns a [`GeoJsonError`] on a JSON syntax error, a missing or
/// non-string `"type"` ([`GeoJsonError::ExpectedType`]), an unknown kind
/// ([`GeoJsonError::UnknownGeometryType`]), a `Feature`/`FeatureCollection`
/// wrapper ([`GeoJsonError::UnsupportedType`]), or a `"coordinates"` /
/// `"geometries"` member of the wrong shape
/// ([`GeoJsonError::MalformedCoordinates`]).
///
/// # Examples
///
/// ```
/// use geometry_io_geojson::from_geojson;
/// use geometry_model::DynKind;
///
/// let g = from_geojson(r#"{"type":"Point","coordinates":[100.0,0.0]}"#).unwrap();
/// assert_eq!(g.kind(), DynKind::Point);
/// ```
pub fn from_geojson<S: AsRef<str>>(input: S) -> Result<DynGeometry<f64, Cartesian>, GeoJsonError> {
    let value = parse_geojson(input.as_ref())?;
    parse_geometry(&value)
}

/// Read the `"type"` member of a `GeoJSON` object and dispatch to the
/// matching kind reader. Mirrors the leading-keyword dispatch in the WKT
/// parser, but keyed on a JSON string instead of a token.
fn parse_geometry(value: &JsonValue) -> Result<DynGeometry<f64, Cartesian>, GeoJsonError> {
    let type_name = value
        .get("type")
        .and_then(JsonValue::as_str)
        .ok_or(GeoJsonError::ExpectedType)?;
    match type_name {
        "Point" => Ok(DynGeometry::Point(read_point_value(coordinate(value)?)?)),
        "LineString" => Ok(DynGeometry::LineString(Linestring(read_positions(
            coords(value)?,
        )?))),
        "Polygon" => Ok(DynGeometry::Polygon(read_polygon(coords(value)?)?)),
        "MultiPoint" => {
            let points = read_positions(coords(value)?)?;
            Ok(DynGeometry::MultiPoint(MultiPoint(points)))
        }
        "MultiLineString" => read_multi_linestring(coords(value)?),
        "MultiPolygon" => read_multi_polygon(coords(value)?),
        "GeometryCollection" => read_geometry_collection(value),
        "Feature" | "FeatureCollection" => Err(GeoJsonError::UnsupportedType(
            alloc::string::String::from(type_name),
        )),
        other => Err(GeoJsonError::UnknownGeometryType(
            alloc::string::String::from(other),
        )),
    }
}

/// Borrow the `"coordinates"` array of a geometry object, or fail with
/// [`GeoJsonError::MalformedCoordinates`].
fn coords(value: &JsonValue) -> Result<&[JsonValue], GeoJsonError> {
    coordinate(value)?
        .as_array()
        .ok_or(GeoJsonError::MalformedCoordinates)
}

fn coordinate(value: &JsonValue) -> Result<&JsonValue, GeoJsonError> {
    value
        .get("coordinates")
        .ok_or(GeoJsonError::MalformedCoordinates)
}

/// Read one position `[x, y]` (ignoring any 3rd ordinate) into a 2D point.
fn read_point(position: &[JsonValue]) -> Result<Pt, GeoJsonError> {
    // RFC 7946 §3.1.1: at least longitude + latitude. Extra ordinates
    // (altitude and beyond) are ignored by this 2D port.
    if position.len() < 2 {
        return Err(GeoJsonError::MalformedCoordinates);
    }
    let x = position[0]
        .as_f64()
        .ok_or(GeoJsonError::MalformedCoordinates)?;
    let y = position[1]
        .as_f64()
        .ok_or(GeoJsonError::MalformedCoordinates)?;
    Ok(Point2D::new(x, y))
}

fn read_point_value(position: &JsonValue) -> Result<Pt, GeoJsonError> {
    if let Some([x, y]) = position.as_position() {
        return Ok(Point2D::new(x, y));
    }
    read_point(
        position
            .as_array()
            .ok_or(GeoJsonError::MalformedCoordinates)?,
    )
}

/// Read an array of positions `[[x, y], …]` into a `Vec` of points. Used
/// by `LineString`, `MultiPoint`, and each ring of a `Polygon`.
fn read_positions(positions: &[JsonValue]) -> Result<Vec<Pt>, GeoJsonError> {
    positions.iter().map(read_point_value).collect()
}

/// Read a `Polygon`'s ring array `[[[x, y], …], …]`: the first ring is
/// the exterior, the rest are holes.
fn read_polygon(rings: &[JsonValue]) -> Result<Polygon<Pt>, GeoJsonError> {
    read_polygon_value(rings)
}

/// The shared `Polygon` value builder, reused by `MultiPolygon`.
fn read_polygon_value(rings: &[JsonValue]) -> Result<Polygon<Pt>, GeoJsonError> {
    let mut iter = rings.iter();
    let outer = match iter.next() {
        Some(r) => {
            let pts = read_positions(r.as_array().ok_or(GeoJsonError::MalformedCoordinates)?)?;
            Ring::from_vec(pts)
        }
        None => Ring::new(),
    };
    let inners = iter
        .map(|r| r.as_array().ok_or(GeoJsonError::MalformedCoordinates))
        .map(|r| r.and_then(read_positions).map(Ring::from_vec))
        .collect::<Result<Vec<Ring<Pt>>, GeoJsonError>>()?;
    Ok(Polygon::with_inners(outer, inners))
}

/// Read a `MultiLineString`'s coordinate array `[[[x, y], …], …]`.
fn read_multi_linestring(lines: &[JsonValue]) -> Result<DynGeometry<f64, Cartesian>, GeoJsonError> {
    let linestrings = lines
        .iter()
        .map(|l| l.as_array().ok_or(GeoJsonError::MalformedCoordinates))
        .map(|l| l.and_then(read_positions).map(Linestring))
        .collect::<Result<Vec<Linestring<Pt>>, GeoJsonError>>()?;
    Ok(DynGeometry::MultiLineString(MultiLinestring(linestrings)))
}

/// Read a `MultiPolygon`'s coordinate array `[[[[x, y], …], …], …]`.
fn read_multi_polygon(polys: &[JsonValue]) -> Result<DynGeometry<f64, Cartesian>, GeoJsonError> {
    let polygons = polys
        .iter()
        .map(|p| p.as_array().ok_or(GeoJsonError::MalformedCoordinates))
        .map(|p| p.and_then(read_polygon_value))
        .collect::<Result<Vec<Polygon<Pt>>, GeoJsonError>>()?;
    Ok(DynGeometry::MultiPolygon(MultiPolygon(polygons)))
}

/// Read a `GeometryCollection`'s `"geometries"` array, recursing into
/// [`parse_geometry`] for each member.
fn read_geometry_collection(
    value: &JsonValue,
) -> Result<DynGeometry<f64, Cartesian>, GeoJsonError> {
    let geometries = value
        .get("geometries")
        .and_then(JsonValue::as_array)
        .ok_or(GeoJsonError::MalformedCoordinates)?;
    let items = geometries
        .iter()
        .map(parse_geometry)
        .collect::<Result<Vec<_>, GeoJsonError>>()?;
    Ok(DynGeometry::GeometryCollection(items))
}

#[cfg(test)]
mod tests {
    //! Reproduces the RFC 7946 §1.5 / §3.1.x worked examples: parse each
    //! and assert the kind plus representative coordinate values.
    #![allow(
        clippy::float_cmp,
        reason = "coordinate values come from exact decimal GeoJSON literals"
    )]

    use super::from_geojson;
    use geometry_model::{DynGeometry, DynKind};
    use geometry_trait::{Linestring as _, MultiPolygon as _, Point as _, Polygon as _, Ring as _};

    #[test]
    fn point_example() {
        let g = from_geojson(r#"{"type":"Point","coordinates":[100.0,0.0]}"#).unwrap();
        assert_eq!(g.kind(), DynKind::Point);
        if let DynGeometry::Point(p) = g {
            assert_eq!(p.get::<0>(), 100.0);
            assert_eq!(p.get::<1>(), 0.0);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn point_altitude_ordinate_is_ignored() {
        let g = from_geojson(r#"{"type":"Point","coordinates":[100.0,0.0,500.0]}"#).unwrap();
        if let DynGeometry::Point(p) = g {
            assert_eq!(p.get::<0>(), 100.0);
            assert_eq!(p.get::<1>(), 0.0);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn linestring_example() {
        let g = from_geojson(r#"{"type":"LineString","coordinates":[[100.0,0.0],[101.0,1.0]]}"#)
            .unwrap();
        assert_eq!(g.kind(), DynKind::LineString);
        if let DynGeometry::LineString(ls) = g {
            assert_eq!(ls.points().len(), 2);
            let last = ls.points().last().unwrap();
            assert_eq!(last.get::<0>(), 101.0);
            assert_eq!(last.get::<1>(), 1.0);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn polygon_with_hole_example() {
        // RFC 7946 §3.1.6 — polygon with a hole.
        let g = from_geojson(
            r#"{"type":"Polygon","coordinates":[
                [[100.0,0.0],[101.0,0.0],[101.0,1.0],[100.0,1.0],[100.0,0.0]],
                [[100.8,0.8],[100.8,0.2],[100.2,0.2],[100.2,0.8],[100.8,0.8]]
            ]}"#,
        )
        .unwrap();
        assert_eq!(g.kind(), DynKind::Polygon);
        if let DynGeometry::Polygon(p) = g {
            assert_eq!(p.exterior().points().len(), 5);
            assert_eq!(p.interiors().count(), 1);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn multipoint_example() {
        let g = from_geojson(r#"{"type":"MultiPoint","coordinates":[[100.0,0.0],[101.0,1.0]]}"#)
            .unwrap();
        assert_eq!(g.kind(), DynKind::MultiPoint);
    }

    #[test]
    fn multipolygon_example() {
        // RFC 7946 §3.1.7.
        let g = from_geojson(
            r#"{"type":"MultiPolygon","coordinates":[
                [[[102.0,2.0],[103.0,2.0],[103.0,3.0],[102.0,3.0],[102.0,2.0]]],
                [[[100.0,0.0],[101.0,0.0],[101.0,1.0],[100.0,1.0],[100.0,0.0]]]
            ]}"#,
        )
        .unwrap();
        assert_eq!(g.kind(), DynKind::MultiPolygon);
        if let DynGeometry::MultiPolygon(mpg) = g {
            assert_eq!(mpg.polygons().count(), 2);
        } else {
            unreachable!();
        }
    }

    #[test]
    fn geometry_collection_example() {
        // RFC 7946 §1.5.
        let g = from_geojson(
            r#"{"type":"GeometryCollection","geometries":[
                {"type":"Point","coordinates":[100.0,0.0]},
                {"type":"LineString","coordinates":[[101.0,0.0],[102.0,1.0]]}
            ]}"#,
        )
        .unwrap();
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
    fn feature_wrapper_is_unsupported() {
        use crate::GeoJsonError;
        let err = from_geojson(
            r#"{"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{}}"#,
        )
        .unwrap_err();
        assert_eq!(err, GeoJsonError::UnsupportedType("Feature".into()));
    }

    #[test]
    fn unknown_type_errors() {
        use crate::GeoJsonError;
        let err = from_geojson(r#"{"type":"Curve","coordinates":[]}"#).unwrap_err();
        assert_eq!(err, GeoJsonError::UnknownGeometryType("Curve".into()));
    }

    #[test]
    fn missing_type_errors() {
        use crate::GeoJsonError;
        let err = from_geojson(r#"{"coordinates":[1,2]}"#).unwrap_err();
        assert_eq!(err, GeoJsonError::ExpectedType);
    }

    #[test]
    fn missing_coordinates_errors() {
        use crate::GeoJsonError;
        let err = from_geojson(r#"{"type":"Point"}"#).unwrap_err();
        assert_eq!(err, GeoJsonError::MalformedCoordinates);
    }
}
