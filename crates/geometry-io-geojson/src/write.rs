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
    let mut out = String::new();
    // Writing into a `String` never fails, so the `Result` is discarded.
    let _ = g.write_geojson(&mut out);
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
    /// Emit `self` as a `GeoJSON` geometry object into `out`.
    ///
    /// # Errors
    ///
    /// Propagates any [`core::fmt::Error`] from the sink.
    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result;
}

/// Format one `f64` the `GeoJSON` way: integer-valued numbers lose their
/// trailing `.0`, everything else uses Rust's shortest round-tripping
/// representation. Keeps `[100,0]` free of `.0` noise while still
/// round-tripping fractional coordinates exactly.
fn write_scalar(out: &mut dyn core::fmt::Write, v: f64) -> core::fmt::Result {
    if v.is_finite() && v.fract() == 0.0 && v.abs() < 9.007_199_254_740_992e15 {
        #[allow(
            clippy::cast_possible_truncation,
            reason = "guarded by the magnitude check above; v is integer-valued"
        )]
        return write!(out, "{}", v as i64);
    }
    write!(out, "{v}")
}

/// Emit one point as a `GeoJSON` position `[x,y]` — `[longitude,latitude]`
/// order (RFC 7946 §3.1.1). This 2D port writes exactly two ordinates.
fn write_position<P: PointTrait<Scalar = f64>>(
    out: &mut dyn core::fmt::Write,
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
fn write_position_seq<'a, P, I>(out: &mut dyn core::fmt::Write, points: I) -> core::fmt::Result
where
    P: PointTrait<Scalar = f64> + 'a,
    I: Iterator<Item = &'a P>,
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
fn write_polygon_rings<Pg>(out: &mut dyn core::fmt::Write, pg: &Pg) -> core::fmt::Result
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
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
    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"Point","coordinates":"#)?;
        write_position(out, self)?;
        out.write_char('}')
    }
}

impl<P: PointTrait<Scalar = f64>> WriteGeoJson for Linestring<P> {
    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"LineString","coordinates":"#)?;
        write_position_seq(out, self.points())?;
        out.write_char('}')
    }
}

// `Ring` / `Polygon` carry two const-generic booleans (clockwise,
// closed). Pinning the impls to Boost's defaults (`true, true`) — the
// shape every `DynGeometry` variant is built from — keeps const-generic
// inference unambiguous at the `to_geojson(&ring)` call site.
impl<P: PointTrait<Scalar = f64>> WriteGeoJson for Ring<P, true, true> {
    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        // A bare ring serialises as a single-ring polygon — GeoJSON has
        // no standalone ring type.
        out.write_str(r#"{"type":"Polygon","coordinates":["#)?;
        write_position_seq(out, self.points())?;
        out.write_str("]}")
    }
}

impl<P: PointTrait<Scalar = f64>> WriteGeoJson for Polygon<P, true, true> {
    fn write_geojson(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str(r#"{"type":"Polygon","coordinates":"#)?;
        write_polygon_rings(out, self)?;
        out.write_char('}')
    }
}

impl<P: PointTrait<Scalar = f64>> WriteGeoJson for MultiPoint<P> {
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

    use super::to_geojson;
    use alloc::vec;
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
