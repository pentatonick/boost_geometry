//! The RFC 7946 `GeoJSON` serializer.
//!
//! [`to_geojson`] emits any concrete model geometry (and [`DynGeometry`])
//! as a compact `GeoJSON` string. Every kind routes through the
//! [`WriteGeoJson`] trait so all types share one implementation per kind,
//! mirroring the sibling WKT crate's `WriteWkt` shape.
//!
//! # Compact output
//!
//! No pretty-printing: `{"type":"Point","coordinates":[100,0]}`. A
//! coordinate is written in RFC 7946 `[longitude, latitude]` order —
//! that is, `[x, y]` — with integer-valued `f64` printed without a
//! trailing `.0` (`100`, not `100.0`) and everything else in Rust's
//! shortest round-tripping form, so a value survives a
//! `to_geojson` → `from_geojson` round-trip exactly.
//!
//! Reference: RFC 7946 §3 (geometry objects) and §3.1.1 (position order).

use alloc::string::String;

use geometry_cs::CoordinateSystem;
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon, Ring,
};
use geometry_trait::{
    Geometry, Linestring as LinestringTrait, MultiLinestring as MultiLinestringTrait,
    MultiPoint as MultiPointTrait, MultiPolygon as MultiPolygonTrait, Point as PointTrait,
    Polygon as PolygonTrait, Ring as RingTrait,
};

/// Approximate bytes per position, including brackets and a separator.
///
/// This deliberately remains a hint rather than an upper bound: ordinary
/// coordinates avoid geometric `String` growth while unusually long decimal
/// spellings can still grow the buffer normally.
const POSITION_CAPACITY: usize = 20;

/// Magnitude bits for `2^53`, the first `f64` beyond the range where every
/// integer has an exact representation.
const MAX_EXACT_INTEGER_BITS: u64 = 0x4340_0000_0000_0000;

/// Serialise a geometry to a compact `GeoJSON` [`String`].
///
/// The output is a bare geometry object (RFC 7946 §3.1) with no
/// whitespace: `{"type":"Point","coordinates":[100,0]}`. Coordinates are
/// emitted in `[longitude, latitude]` (i.e. `[x, y]`) order; integer-
/// valued coordinates print without a trailing `.0`.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_geojson::to_geojson;
/// use geometry_model::Point2D;
///
/// let p = Point2D::<f64, Cartesian>::new(100.0, 0.0);
/// assert_eq!(to_geojson(&p), r#"{"type":"Point","coordinates":[100,0]}"#);
/// ```
#[must_use]
pub fn to_geojson<G: Geometry + WriteGeoJson>(g: &G) -> String {
    let mut out = String::with_capacity(g.geojson_capacity_hint().unwrap_or(0));
    // Writing into a `String` never fails, so the `Result` is discarded.
    let _ = g.write_geojson_string(&mut out);
    out
}

/// Serialise any polygon implementing [`PolygonTrait`] to compact `GeoJSON`.
///
/// This is the bring-your-own-type counterpart to [`to_geojson`]. It reads the
/// polygon through the public geometry traits, including its interior rings,
/// without first converting it to a `geometry_model` type.
#[must_use]
pub fn to_geojson_polygon<Pg>(polygon: &Pg) -> String
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    let mut out = String::with_capacity(polygon_capacity(polygon).unwrap_or(0));
    out.push_str(r#"{"type":"Polygon","coordinates":"#);
    // Writing into a `String` never fails, so the `Result` is discarded.
    let _ = write_polygon_rings(&mut out, polygon);
    out.push('}');
    out
}

/// The per-kind `GeoJSON` emitter, implemented for every concrete model
/// type and for [`DynGeometry`].
///
/// Hidden from the public docs: callers use [`to_geojson`], which bounds
/// on this trait. It exists so the entry point shares one implementation
/// per geometry kind, mirroring the sibling WKT crate's `WriteWkt`.
#[doc(hidden)]
pub trait WriteGeoJson {
    /// Approximate output capacity for the built-in geometry models.
    ///
    /// External implementations can keep the default and retain the previous
    /// grow-on-demand behavior.
    fn geojson_capacity_hint(&self) -> Option<usize> {
        None
    }

    /// Emit `self` as a `GeoJSON` geometry object into `out`.
    ///
    /// # Errors
    ///
    /// Propagates any [`core::fmt::Error`] from the sink.
    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result;

    /// Emit directly into the owned buffer used by [`to_geojson`].
    ///
    /// The default preserves external implementations. Built-in models with
    /// hot coordinate-sequence paths override it so their scalar loop can be
    /// monomorphized for [`String`] without changing the object-safe sink
    /// method.
    fn write_geojson_string(&self, out: &mut String) -> core::fmt::Result {
        self.write_geojson(out)
    }
}

fn position_seq_capacity(point_count: usize) -> Option<usize> {
    point_count.checked_mul(POSITION_CAPACITY)
}

fn polygon_capacity<Pg>(polygon: &Pg) -> Option<usize>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    let mut capacity =
        48usize.checked_add(position_seq_capacity(polygon.exterior().points().len())?)?;
    for ring in polygon.interiors() {
        capacity = capacity.checked_add(position_seq_capacity(ring.points().len())?)?;
    }
    Some(capacity)
}

/// Format one `f64` the `GeoJSON` way: integer-valued numbers lose their
/// trailing `.0`, everything else uses Rust's shortest round-tripping
/// representation. Keeps `[100,0]` free of `.0` noise while still
/// round-tripping fractional coordinates exactly.
fn is_exact_integer(v: f64) -> bool {
    let magnitude = v.to_bits() & 0x7fff_ffff_ffff_ffff;
    if magnitude == 0 {
        return true;
    }
    if magnitude >= MAX_EXACT_INTEGER_BITS {
        return false;
    }

    let biased_exponent = (magnitude >> 52) as u32;
    if biased_exponent < 1023 {
        return false;
    }
    let fractional_bit_count = 1075 - biased_exponent;
    let fractional_mask = (1_u64 << fractional_bit_count) - 1;
    magnitude & fractional_mask == 0
}

fn write_scalar<W: core::fmt::Write + ?Sized>(out: &mut W, v: f64) -> core::fmt::Result {
    if is_exact_integer(v) {
        #[allow(
            clippy::cast_possible_truncation,
            reason = "guarded by the magnitude check above; v is integer-valued"
        )]
        let integer = v as i64;
        return out.write_str(itoa::Buffer::new().format(integer));
    }
    write!(out, "{v}")
}

/// Emit one point as a `GeoJSON` position `[x,y]` — `[longitude,latitude]`
/// order (RFC 7946 §3.1.1). This 2D port writes exactly two ordinates.
fn write_position<P: PointTrait<Scalar = f64>, W: core::fmt::Write + ?Sized>(
    out: &mut W,
    p: &P,
) -> core::fmt::Result {
    out.write_char('[')?;
    write_scalar(out, p.get::<0>())?;
    out.write_char(',')?;
    write_scalar(out, p.get::<1>())?;
    out.write_char(']')
}

/// Emit a bracketed, comma-separated position list `[[x,y],…]`. Shared by
/// linestrings, rings, and multipoints.
fn write_position_seq<'a, P, I, W>(out: &mut W, points: I) -> core::fmt::Result
where
    P: PointTrait<Scalar = f64> + 'a,
    I: Iterator<Item = &'a P>,
    W: core::fmt::Write + ?Sized,
{
    out.write_char('[')?;
    for (i, p) in points.enumerate() {
        if i > 0 {
            out.write_char(',')?;
        }
        write_position(out, p)?;
    }
    out.write_char(']')
}

/// Emit a polygon's rings `[[outer],[hole],…]` (no `"type"` wrapper).
/// Shared by `Polygon` and each member of `MultiPolygon`.
fn write_polygon_rings<Pg, W>(out: &mut W, pg: &Pg) -> core::fmt::Result
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
    W: core::fmt::Write + ?Sized,
{
    out.write_char('[')?;
    write_position_seq(out, pg.exterior().points())?;
    for ring in pg.interiors() {
        out.write_char(',')?;
        write_position_seq(out, ring.points())?;
    }
    out.write_char(']')
}

impl<Cs: CoordinateSystem> WriteGeoJson for Point<f64, 2, Cs> {
    fn geojson_capacity_hint(&self) -> Option<usize> {
        Some(64)
    }

    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"Point","coordinates":"#)?;
        write_position(out, self)?;
        out.write_char('}')
    }
}

impl<P: PointTrait<Scalar = f64>> WriteGeoJson for Linestring<P> {
    fn geojson_capacity_hint(&self) -> Option<usize> {
        48usize.checked_add(position_seq_capacity(self.points().len())?)
    }

    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"LineString","coordinates":"#)?;
        write_position_seq(out, self.points())?;
        out.write_char('}')
    }

    fn write_geojson_string(&self, out: &mut String) -> core::fmt::Result {
        out.push_str(r#"{"type":"LineString","coordinates":"#);
        write_position_seq(out, self.points())?;
        out.push('}');
        Ok(())
    }
}

// `Ring` / `Polygon` carry two const-generic booleans (clockwise,
// closed). Pinning the impls to Boost's defaults (`true, true`) — the
// shape every `DynGeometry` variant is built from — keeps const-generic
// inference unambiguous at the `to_geojson(&ring)` call site.
impl<P: PointTrait<Scalar = f64>> WriteGeoJson for Ring<P, true, true> {
    fn geojson_capacity_hint(&self) -> Option<usize> {
        48usize.checked_add(position_seq_capacity(self.points().len())?)
    }

    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        // A bare ring serialises as a single-ring polygon — GeoJSON has
        // no standalone ring type.
        out.write_str(r#"{"type":"Polygon","coordinates":["#)?;
        write_position_seq(out, self.points())?;
        out.write_str("]}")
    }
}

impl<P: PointTrait<Scalar = f64>> WriteGeoJson for Polygon<P, true, true> {
    fn geojson_capacity_hint(&self) -> Option<usize> {
        polygon_capacity(self)
    }

    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"Polygon","coordinates":"#)?;
        write_polygon_rings(out, self)?;
        out.write_char('}')
    }

    fn write_geojson_string(&self, out: &mut String) -> core::fmt::Result {
        out.push_str(r#"{"type":"Polygon","coordinates":"#);
        write_polygon_rings(out, self)?;
        out.push('}');
        Ok(())
    }
}

impl<P: PointTrait<Scalar = f64>> WriteGeoJson for MultiPoint<P> {
    fn geojson_capacity_hint(&self) -> Option<usize> {
        48usize.checked_add(position_seq_capacity(self.points().len())?)
    }

    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"MultiPoint","coordinates":"#)?;
        write_position_seq(out, self.points())?;
        out.write_char('}')
    }
}

impl<L> WriteGeoJson for MultiLinestring<L>
where
    L: LinestringTrait,
    L::Point: PointTrait<Scalar = f64>,
{
    fn geojson_capacity_hint(&self) -> Option<usize> {
        let mut capacity = 48usize;
        for linestring in self.linestrings() {
            capacity = capacity.checked_add(position_seq_capacity(linestring.points().len())?)?;
        }
        Some(capacity)
    }

    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"MultiLineString","coordinates":["#)?;
        for (i, ls) in self.linestrings().enumerate() {
            if i > 0 {
                out.write_char(',')?;
            }
            write_position_seq(out, ls.points())?;
        }
        out.write_str("]}")
    }
}

impl<Pg> WriteGeoJson for MultiPolygon<Pg>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    fn geojson_capacity_hint(&self) -> Option<usize> {
        let mut capacity = 48usize;
        for polygon in self.polygons() {
            capacity = capacity.checked_add(polygon_capacity(polygon)?)?;
        }
        Some(capacity)
    }

    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"MultiPolygon","coordinates":["#)?;
        for (i, pg) in self.polygons().enumerate() {
            if i > 0 {
                out.write_char(',')?;
            }
            write_polygon_rings(out, pg)?;
        }
        out.write_str("]}")
    }
}

impl<Cs: CoordinateSystem> WriteGeoJson for DynGeometry<f64, Cs> {
    fn geojson_capacity_hint(&self) -> Option<usize> {
        match self {
            DynGeometry::Point(point) => point.geojson_capacity_hint(),
            DynGeometry::LineString(linestring) => linestring.geojson_capacity_hint(),
            DynGeometry::Polygon(polygon) => polygon.geojson_capacity_hint(),
            DynGeometry::MultiPoint(multipoint) => multipoint.geojson_capacity_hint(),
            DynGeometry::MultiLineString(multilinestring) => {
                multilinestring.geojson_capacity_hint()
            }
            DynGeometry::MultiPolygon(multipolygon) => multipolygon.geojson_capacity_hint(),
            DynGeometry::GeometryCollection(items) => {
                let mut capacity = 48usize;
                let mut stack = alloc::vec::Vec::with_capacity(items.len());
                stack.extend(items);
                while let Some(item) = stack.pop() {
                    match item {
                        DynGeometry::Point(point) => {
                            capacity = capacity.checked_add(point.geojson_capacity_hint()?)?;
                        }
                        DynGeometry::LineString(linestring) => {
                            capacity = capacity.checked_add(linestring.geojson_capacity_hint()?)?;
                        }
                        DynGeometry::Polygon(polygon) => {
                            capacity = capacity.checked_add(polygon.geojson_capacity_hint()?)?;
                        }
                        DynGeometry::MultiPoint(multipoint) => {
                            capacity = capacity.checked_add(multipoint.geojson_capacity_hint()?)?;
                        }
                        DynGeometry::MultiLineString(multilinestring) => {
                            capacity =
                                capacity.checked_add(multilinestring.geojson_capacity_hint()?)?;
                        }
                        DynGeometry::MultiPolygon(multipolygon) => {
                            capacity =
                                capacity.checked_add(multipolygon.geojson_capacity_hint()?)?;
                        }
                        DynGeometry::GeometryCollection(nested) => {
                            capacity = capacity.checked_add(48)?;
                            stack.extend(nested);
                        }
                    }
                }
                Some(capacity)
            }
        }
    }

    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        // Only the `GeometryCollection` arm nests; the leaf/multi arms
        // delegate to their own non-recursive writers. Walk the nesting
        // with an explicit stack rather than recursion so a deeply nested
        // `DynGeometry` cannot overflow the native stack (an uncatchable
        // process abort). Each stack item is a fragment of pending work.
        enum Frag<'a, Cs: CoordinateSystem> {
            /// Emit this geometry.
            Geom(&'a DynGeometry<f64, Cs>),
            /// Emit a literal (structural JSON punctuation).
            Lit(&'static str),
        }

        let mut stack = alloc::vec![Frag::Geom(self)];
        while let Some(frag) = stack.pop() {
            match frag {
                Frag::Lit(s) => out.write_str(s)?,
                Frag::Geom(g) => match g {
                    DynGeometry::Point(p) => p.write_geojson(out)?,
                    DynGeometry::LineString(ls) => ls.write_geojson(out)?,
                    DynGeometry::Polygon(pg) => pg.write_geojson(out)?,
                    DynGeometry::MultiPoint(mp) => mp.write_geojson(out)?,
                    DynGeometry::MultiLineString(mls) => mls.write_geojson(out)?,
                    DynGeometry::MultiPolygon(mpg) => mpg.write_geojson(out)?,
                    DynGeometry::GeometryCollection(items) => {
                        // Push in reverse so they pop in source order:
                        // header, g0, ",", g1, …, "]}". The stack is LIFO.
                        stack.push(Frag::Lit("]}"));
                        for (i, item) in items.iter().enumerate().rev() {
                            stack.push(Frag::Geom(item));
                            if i > 0 {
                                stack.push(Frag::Lit(","));
                            }
                        }
                        stack.push(Frag::Lit(r#"{"type":"GeometryCollection","geometries":["#));
                    }
                },
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    //! Compact-output witnesses for each RFC 7946 §3.1 kind.
    #![allow(
        clippy::float_cmp,
        reason = "coordinates are exact decimal literals in these fixtures"
    )]

    use super::{is_exact_integer, to_geojson, to_geojson_polygon, write_scalar};
    use alloc::{format, string::String, vec};
    use geometry_cs::Cartesian;
    use geometry_model::{DynGeometry, Linestring, MultiPoint, Point2D, Polygon, Ring};

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn point_compact() {
        let p = Pt::new(100.0, 0.0);
        assert_eq!(to_geojson(&p), r#"{"type":"Point","coordinates":[100,0]}"#);
    }

    #[test]
    fn nested_collection_writer_is_iterative_and_correct() {
        // The DynGeometry writer walks nesting with an explicit stack, not
        // recursion — verify nested output is unchanged and a deeply
        // nested value does not overflow the stack.
        let g = DynGeometry::<f64, Cartesian>::GeometryCollection(vec![
            DynGeometry::Point(Pt::new(1.0, 1.0)),
            DynGeometry::GeometryCollection(vec![DynGeometry::Point(Pt::new(2.0, 2.0))]),
        ]);
        assert_eq!(
            to_geojson(&g),
            r#"{"type":"GeometryCollection","geometries":[{"type":"Point","coordinates":[1,1]},{"type":"GeometryCollection","geometries":[{"type":"Point","coordinates":[2,2]}]}]}"#
        );
        let mut deep = DynGeometry::<f64, Cartesian>::Point(Pt::new(0.0, 0.0));
        for _ in 0..200_000 {
            deep = DynGeometry::GeometryCollection(vec![deep]);
        }
        assert!(to_geojson(&deep).starts_with(r#"{"type":"GeometryCollection""#));
        core::mem::forget(deep); // avoid the still-recursive value Drop
    }

    #[test]
    fn fractional_coord_round_trips() {
        let p = Pt::new(1.5, -2.25);
        assert_eq!(
            to_geojson(&p),
            r#"{"type":"Point","coordinates":[1.5,-2.25]}"#
        );
    }

    #[test]
    fn scalar_spellings_match_previous_formatter() {
        let values = [
            0.0,
            -0.0,
            1.0,
            -1.0,
            1.5,
            -2.25,
            9_007_199_254_740_991.0,
            9_007_199_254_740_992.0,
            1.0e-7,
            1.0e16,
            f64::MIN_POSITIVE,
            f64::MAX,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NAN,
        ];

        for value in values {
            let expected = if value.is_finite()
                && value.fract() == 0.0
                && value.abs() < 9.007_199_254_740_992e15
            {
                #[allow(
                    clippy::cast_possible_truncation,
                    reason = "this reproduces the previous formatter's guarded conversion"
                )]
                {
                    format!("{}", value as i64)
                }
            } else {
                format!("{value}")
            };
            let mut actual = String::new();
            write_scalar(&mut actual, value).unwrap();
            assert_eq!(actual, expected, "value {value:?}");
        }
    }

    #[test]
    fn integer_classifier_matches_previous_predicate() {
        let mut bits = 0_u64;
        for _ in 0..100_000 {
            bits = bits
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let value = f64::from_bits(bits);
            let expected =
                value.is_finite() && value.fract() == 0.0 && value.abs() < 9.007_199_254_740_992e15;
            assert_eq!(is_exact_integer(value), expected, "bits {bits:#018x}");
        }
    }

    #[test]
    fn writer_trait_remains_object_safe() {
        use super::WriteGeoJson;

        let point = Pt::new(1.0, 2.0);
        let writer: &dyn WriteGeoJson = &point;
        let mut output = String::new();
        writer.write_geojson(&mut output).unwrap();
        assert_eq!(output, r#"{"type":"Point","coordinates":[1,2]}"#);
    }

    #[test]
    fn linestring_compact() {
        let ls = Linestring(vec![Pt::new(100.0, 0.0), Pt::new(101.0, 1.0)]);
        assert_eq!(
            to_geojson(&ls),
            r#"{"type":"LineString","coordinates":[[100,0],[101,1]]}"#
        );
    }

    #[test]
    fn polygon_with_hole_compact() {
        let outer = Ring::from_vec(vec![
            Pt::new(100.0, 0.0),
            Pt::new(101.0, 0.0),
            Pt::new(101.0, 1.0),
            Pt::new(100.0, 1.0),
            Pt::new(100.0, 0.0),
        ]);
        let hole = Ring::from_vec(vec![
            Pt::new(100.8, 0.8),
            Pt::new(100.8, 0.2),
            Pt::new(100.2, 0.2),
            Pt::new(100.2, 0.8),
            Pt::new(100.8, 0.8),
        ]);
        let poly = Polygon::with_inners(outer, vec![hole]);
        assert_eq!(
            to_geojson(&poly),
            r#"{"type":"Polygon","coordinates":[[[100,0],[101,0],[101,1],[100,1],[100,0]],[[100.8,0.8],[100.8,0.2],[100.2,0.2],[100.2,0.8],[100.8,0.8]]]}"#
        );
    }

    #[test]
    fn trait_polygon_entry_point_matches_model_writer() {
        let polygon = Polygon::with_inners(
            Ring::from_vec(vec![
                Pt::new(0.0, 0.0),
                Pt::new(0.0, 2.0),
                Pt::new(2.0, 2.0),
                Pt::new(2.0, 0.0),
                Pt::new(0.0, 0.0),
            ]),
            vec![],
        );

        assert_eq!(to_geojson_polygon(&polygon), to_geojson(&polygon));
    }

    #[test]
    fn multipoint_compact() {
        let mp = MultiPoint(vec![Pt::new(100.0, 0.0), Pt::new(101.0, 1.0)]);
        assert_eq!(
            to_geojson(&mp),
            r#"{"type":"MultiPoint","coordinates":[[100,0],[101,1]]}"#
        );
    }

    #[test]
    fn geometry_collection_compact() {
        let g = DynGeometry::<f64, Cartesian>::GeometryCollection(vec![
            DynGeometry::Point(Pt::new(100.0, 0.0)),
            DynGeometry::LineString(Linestring(vec![Pt::new(101.0, 0.0), Pt::new(102.0, 1.0)])),
        ]);
        assert_eq!(
            to_geojson(&g),
            r#"{"type":"GeometryCollection","geometries":[{"type":"Point","coordinates":[100,0]},{"type":"LineString","coordinates":[[101,0],[102,1]]}]}"#
        );
    }
}
