//! The WKT serializer.
//!
//! Mirrors `boost/geometry/io/wkt/write.hpp` — the C++ side dispatches
//! on the geometry tag to a per-kind stream inserter (`wkt_point`,
//! `wkt_range`, `wkt_poly`, and the multi variants) that emits the type
//! keyword followed by the parenthesised coordinate list. This port
//! routes every concrete model type (and [`DynGeometry`]) through the
//! [`WriteWkt`] trait so both the owned-`String` [`to_wkt`] and the
//! streaming [`write_wkt`] share one implementation.
//!
//! # Canonical output
//!
//! Uppercase keyword, **no space** before the opening paren, single
//! spaces between the two ordinates of a point and after each comma-less
//! separator is a bare `,` (no leading space). Integer-valued
//! coordinates print without a trailing `.0` (`10`, not `10.0`);
//! non-integer coordinates use Rust's shortest round-tripping `f64`
//! formatting. Example: `POINT(10 10)`,
//! `POLYGON((10 10,10 20,20 20,20 15,10 10))`. Matches the spacing Boost
//! writes in `boost/geometry/io/wkt/write.hpp` (`stream_wkt` inserts no
//! space after the type keyword and separates coordinates with a single
//! space, points with `,`).
//!
//! Reference: OGC Simple Feature Access Part 1 §7 and
//! `boost/geometry/io/wkt/write.hpp`.

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

/// Typical bytes reserved per 2D coordinate pair by [`to_wkt`].
///
/// This deliberately remains a hint rather than an upper bound: ordinary
/// coordinates avoid geometric `String` growth while unusually long decimal
/// spellings can still grow the buffer normally.
const POINT_CAPACITY: usize = 16;

/// Serialise a geometry to a canonical WKT [`String`].
///
/// A thin wrapper over [`write_wkt`] that owns the output buffer. The
/// canonical spacing and number format are: uppercase keyword, no space
/// before `(`, coordinates separated by a single space, points by a
/// bare `,`, and integer-valued coordinates printed without a trailing
/// `.0`. Mirrors `boost::geometry::wkt(g)` used as a manipulator into a
/// `std::ostringstream` in `boost/geometry/io/wkt/write.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_wkt::to_wkt;
/// use geometry_model::Point2D;
///
/// let p = Point2D::<f64, Cartesian>::new(10.0, 10.0);
/// assert_eq!(to_wkt(&p), "POINT(10 10)");
/// ```
#[must_use]
pub fn to_wkt<G: Geometry + WriteWkt>(g: &G) -> String {
    let mut out = String::with_capacity(g.wkt_capacity_hint().unwrap_or(0));
    // Writing into a `String` never fails, so the `Result` is discarded.
    let _ = g.write_wkt_string(&mut out);
    out
}

/// Serialise any polygon implementing [`PolygonTrait`] to canonical WKT.
///
/// This is the bring-your-own-type counterpart to [`to_wkt`]. It reads the
/// polygon through the public geometry traits, including its interior rings,
/// without first converting it to a `geometry_model` type.
#[must_use]
pub fn to_wkt_polygon<Pg>(polygon: &Pg) -> String
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    let mut out = String::with_capacity(polygon_capacity(polygon).unwrap_or(0));
    // Writing into a `String` never fails, so the `Result` is discarded.
    let _ = write_polygon(polygon, &mut out);
    out
}

/// Serialise a geometry into any [`core::fmt::Write`] sink.
///
/// The streaming counterpart to [`to_wkt`]; use it to write straight
/// into a caller-owned buffer or formatter. Mirrors the operator
/// `<<` overload in `boost/geometry/io/wkt/write.hpp`.
///
/// # Errors
///
/// Propagates any [`core::fmt::Error`] the sink returns.
///
/// # Examples
///
/// ```
/// use core::fmt::Write;
/// use geometry_cs::Cartesian;
/// use geometry_io_wkt::write_wkt;
/// use geometry_model::Point2D;
///
/// let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
/// let mut s = String::new();
/// write_wkt(&p, &mut s).unwrap();
/// assert_eq!(s, "POINT(1 2)");
/// ```
pub fn write_wkt<G: WriteWkt, W: core::fmt::Write>(g: &G, out: &mut W) -> core::fmt::Result {
    g.write_wkt(out)
}

/// The per-kind WKT emitter, implemented for every concrete model type
/// and for [`DynGeometry`].
///
/// Hidden from the public docs: callers use [`to_wkt`] / [`write_wkt`],
/// which bound on this trait. It exists so the two entry points share
/// one implementation per geometry kind, mirroring the tag-dispatched
/// stream inserters in `boost/geometry/io/wkt/write.hpp`.
#[doc(hidden)]
pub trait WriteWkt {
    /// Approximate output capacity for the built-in geometry models.
    ///
    /// External implementations can keep the default and retain the previous
    /// grow-on-demand behavior.
    fn wkt_capacity_hint(&self) -> Option<usize> {
        None
    }

    /// Emit `self` as WKT into `out`.
    ///
    /// # Errors
    ///
    /// Propagates any [`core::fmt::Error`] from the sink.
    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result;

    /// Emit directly into the owned buffer used by [`to_wkt`].
    ///
    /// The default preserves external implementations. Built-in models with
    /// hot coordinate-sequence paths override it so their scalar loop can be
    /// monomorphized for [`String`] without changing the object-safe
    /// streaming method.
    fn write_wkt_string(&self, out: &mut String) -> core::fmt::Result {
        self.write_wkt(out)
    }
}

fn point_seq_capacity(point_count: usize) -> Option<usize> {
    point_count.checked_mul(POINT_CAPACITY)
}

fn polygon_capacity<Pg>(polygon: &Pg) -> Option<usize>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    let mut capacity =
        32usize.checked_add(point_seq_capacity(polygon.exterior().points().len())?)?;
    for ring in polygon.interiors() {
        capacity = capacity.checked_add(point_seq_capacity(ring.points().len())?)?;
    }
    Some(capacity)
}

/// Format one `f64` the WKT way: integer-valued numbers lose their
/// trailing `.0`, everything else uses Rust's shortest round-tripping
/// representation. Keeps `POINT(10 10)` free of `.0` noise while still
/// round-tripping fractional coordinates exactly.
fn write_scalar<W: core::fmt::Write + ?Sized>(out: &mut W, v: f64) -> core::fmt::Result {
    if v == 0.0 {
        return out.write_char('0');
    }
    if v.is_finite() && v > -9.007_199_254_740_992e15 && v < 9.007_199_254_740_992e15 {
        // The bounded conversion is exact precisely when `v` is integral.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "guarded by the finite 2^53 magnitude range"
        )]
        let integer = v as i64;
        #[allow(
            clippy::cast_precision_loss,
            reason = "all integers within the guarded 2^53 range are exactly representable"
        )]
        #[allow(
            clippy::float_cmp,
            reason = "exact equality intentionally identifies exactly representable integers"
        )]
        if v == integer as f64 {
            return out.write_str(itoa::Buffer::new().format(integer));
        }
    }
    let mut buffer = ryu::Buffer::new();
    let formatted = buffer.format(v);
    let Some(exponent_pos) = formatted.find('e') else {
        return out.write_str(formatted.strip_suffix(".0").unwrap_or(formatted));
    };

    write_expanded_scalar(
        out,
        &formatted[..exponent_pos],
        &formatted[exponent_pos + 1..],
    )
}

/// Expand Ryu's scientific notation to the non-exponent spelling used
/// by `f64`'s `Display` implementation and by the existing WKT output.
fn write_expanded_scalar<W: core::fmt::Write + ?Sized>(
    out: &mut W,
    mantissa: &str,
    exponent: &str,
) -> core::fmt::Result {
    let (negative, mantissa) = match mantissa.strip_prefix('-') {
        Some(unsigned) => (true, unsigned),
        None => (false, mantissa),
    };
    let (exponent_negative, exponent) = match exponent.strip_prefix('-') {
        Some(unsigned) => (true, unsigned),
        None => (false, exponent.strip_prefix('+').unwrap_or(exponent)),
    };
    let mut exponent_value = 0i32;
    for byte in exponent.bytes() {
        exponent_value = exponent_value * 10 + i32::from(byte - b'0');
    }
    if exponent_negative {
        exponent_value = -exponent_value;
    }

    let integer_digits = mantissa.find('.').unwrap_or(mantissa.len());
    let mut digit_buffer = [0u8; 24];
    let mut digit_count = 0;
    for byte in mantissa.bytes() {
        if byte != b'.' {
            digit_buffer[digit_count] = byte;
            digit_count += 1;
        }
    }
    let digits = core::str::from_utf8(&digit_buffer[..digit_count])
        .expect("Ryu always emits ASCII decimal digits");
    let decimal_pos =
        i32::try_from(integer_digits).expect("Ryu mantissa is short") + exponent_value;

    if negative {
        out.write_char('-')?;
    }
    if decimal_pos <= 0 {
        out.write_str("0.")?;
        write_zeroes(
            out,
            usize::try_from(-decimal_pos).expect("negative decimal position"),
        )?;
        return out.write_str(digits);
    }

    let decimal_pos = usize::try_from(decimal_pos).expect("positive decimal position");
    if decimal_pos >= digit_count {
        out.write_str(digits)?;
        return write_zeroes(out, decimal_pos - digit_count);
    }

    out.write_str(&digits[..decimal_pos])?;
    out.write_char('.')?;
    out.write_str(&digits[decimal_pos..])
}

fn write_zeroes<W: core::fmt::Write + ?Sized>(out: &mut W, mut count: usize) -> core::fmt::Result {
    const ZEROES: &str = "00000000000000000000000000000000";
    while count >= ZEROES.len() {
        out.write_str(ZEROES)?;
        count -= ZEROES.len();
    }
    out.write_str(&ZEROES[..count])
}

/// Emit one point's ordinates as `x y` (no keyword, no parens). Shared
/// by every coordinate-bearing kind. Only the first two dimensions are
/// written — this is a 2D port.
fn write_coords<P: PointTrait<Scalar = f64>, W: core::fmt::Write + ?Sized>(
    out: &mut W,
    p: &P,
) -> core::fmt::Result {
    write_scalar(out, p.get::<0>())?;
    out.write_char(' ')?;
    write_scalar(out, p.get::<1>())
}

/// Emit a comma-separated coordinate list `x y,x y,…` (no surrounding
/// parens). Shared by linestrings and rings.
fn write_point_seq<'a, P, I, W>(out: &mut W, points: I) -> core::fmt::Result
where
    P: PointTrait<Scalar = f64> + 'a,
    I: Iterator<Item = &'a P>,
    W: core::fmt::Write + ?Sized,
{
    for (i, p) in points.enumerate() {
        if i > 0 {
            out.write_char(',')?;
        }
        write_coords(out, p)?;
    }
    Ok(())
}

/// Emit `((outer),(hole),…)` for a polygon's rings (no keyword). Shared
/// by `POLYGON` and each member of `MULTIPOLYGON`.
fn write_polygon_rings<Pg, W>(out: &mut W, pg: &Pg) -> core::fmt::Result
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
    W: core::fmt::Write + ?Sized,
{
    out.write_char('(')?;
    out.write_char('(')?;
    write_point_seq(out, pg.exterior().points())?;
    out.write_char(')')?;
    for ring in pg.interiors() {
        out.write_char(',')?;
        out.write_char('(')?;
        write_point_seq(out, ring.points())?;
        out.write_char(')')?;
    }
    out.write_char(')')
}

impl<Cs: CoordinateSystem> WriteWkt for Point<f64, 2, Cs> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        Some(64)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        out.write_str("POINT(")?;
        write_coords(out, self)?;
        out.write_char(')')
    }
}

impl<P: PointTrait<Scalar = f64>> WriteWkt for Linestring<P> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        32usize.checked_add(point_seq_capacity(self.points().len())?)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        write_linestring(self, out)
    }

    fn write_wkt_string(&self, out: &mut String) -> core::fmt::Result {
        write_linestring(self, out)
    }
}

fn write_linestring<P, W>(linestring: &Linestring<P>, out: &mut W) -> core::fmt::Result
where
    P: PointTrait<Scalar = f64>,
    W: core::fmt::Write + ?Sized,
{
    // OGC WKT spells an empty geometry `<TYPE> EMPTY`, not `<TYPE>()`
    // — the latter is not grammar the reader (or Boost) accepts.
    if linestring.points().next().is_none() {
        return out.write_str("LINESTRING EMPTY");
    }
    out.write_str("LINESTRING(")?;
    write_point_seq(out, linestring.points())?;
    out.write_char(')')
}

// `Ring` / `Polygon` carry two const-generic booleans (clockwise,
// closed). Pinning the `WriteWkt` impls to Boost's defaults
// (`true, true`) — the shape every `DynGeometry` variant is built from —
// keeps const-generic inference unambiguous at the `to_wkt(&ring)` call
// site; a ring's serialisation does not depend on those flags anyway.
impl<P: PointTrait<Scalar = f64>> WriteWkt for Ring<P, true, true> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        32usize.checked_add(point_seq_capacity(self.points().len())?)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        // A bare ring serialises as a single-ring polygon — the OGC WKT
        // grammar has no standalone RING keyword.
        if self.points().next().is_none() {
            return out.write_str("POLYGON EMPTY");
        }
        out.write_str("POLYGON((")?;
        write_point_seq(out, self.points())?;
        out.write_str("))")
    }
}

impl<P: PointTrait<Scalar = f64>> WriteWkt for Polygon<P, true, true> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        polygon_capacity(self)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        write_polygon(self, out)
    }

    fn write_wkt_string(&self, out: &mut String) -> core::fmt::Result {
        write_polygon(self, out)
    }
}

fn write_polygon<Pg, W>(polygon: &Pg, out: &mut W) -> core::fmt::Result
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
    W: core::fmt::Write + ?Sized,
{
    // A polygon with no exterior vertices is empty.
    if polygon.exterior().points().next().is_none() {
        return out.write_str("POLYGON EMPTY");
    }
    out.write_str("POLYGON")?;
    write_polygon_rings(out, polygon)
}

impl<P: PointTrait<Scalar = f64>> WriteWkt for MultiPoint<P> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        32usize.checked_add(point_seq_capacity(self.points().len())?)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        if self.points().next().is_none() {
            return out.write_str("MULTIPOINT EMPTY");
        }
        out.write_str("MULTIPOINT(")?;
        for (i, p) in self.points().enumerate() {
            if i > 0 {
                out.write_char(',')?;
            }
            out.write_char('(')?;
            write_coords(out, p)?;
            out.write_char(')')?;
        }
        out.write_char(')')
    }
}

impl<L> WriteWkt for MultiLinestring<L>
where
    L: LinestringTrait,
    L::Point: PointTrait<Scalar = f64>,
{
    fn wkt_capacity_hint(&self) -> Option<usize> {
        let mut capacity = 32usize;
        for linestring in self.linestrings() {
            capacity = capacity.checked_add(point_seq_capacity(linestring.points().len())?)?;
        }
        Some(capacity)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        if self.linestrings().next().is_none() {
            return out.write_str("MULTILINESTRING EMPTY");
        }
        out.write_str("MULTILINESTRING(")?;
        for (i, ls) in self.linestrings().enumerate() {
            if i > 0 {
                out.write_char(',')?;
            }
            out.write_char('(')?;
            write_point_seq(out, ls.points())?;
            out.write_char(')')?;
        }
        out.write_char(')')
    }
}

impl<Pg> WriteWkt for MultiPolygon<Pg>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    fn wkt_capacity_hint(&self) -> Option<usize> {
        let mut capacity = 32usize;
        for polygon in self.polygons() {
            capacity = capacity.checked_add(polygon_capacity(polygon)?)?;
        }
        Some(capacity)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        if self.polygons().next().is_none() {
            return out.write_str("MULTIPOLYGON EMPTY");
        }
        out.write_str("MULTIPOLYGON(")?;
        for (i, pg) in self.polygons().enumerate() {
            if i > 0 {
                out.write_char(',')?;
            }
            write_polygon_rings(out, pg)?;
        }
        out.write_char(')')
    }
}

impl<Cs: CoordinateSystem> WriteWkt for DynGeometry<f64, Cs> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        match self {
            DynGeometry::Point(point) => point.wkt_capacity_hint(),
            DynGeometry::LineString(linestring) => linestring.wkt_capacity_hint(),
            DynGeometry::Polygon(polygon) => polygon.wkt_capacity_hint(),
            DynGeometry::MultiPoint(multipoint) => multipoint.wkt_capacity_hint(),
            DynGeometry::MultiLineString(multilinestring) => multilinestring.wkt_capacity_hint(),
            DynGeometry::MultiPolygon(multipolygon) => multipolygon.wkt_capacity_hint(),
            DynGeometry::GeometryCollection(items) => {
                let mut capacity = 32usize;
                let mut stack = alloc::vec::Vec::with_capacity(items.len());
                stack.extend(items);
                while let Some(geometry) = stack.pop() {
                    match geometry {
                        DynGeometry::Point(point) => {
                            capacity = capacity.checked_add(point.wkt_capacity_hint()?)?;
                        }
                        DynGeometry::LineString(linestring) => {
                            capacity = capacity.checked_add(linestring.wkt_capacity_hint()?)?;
                        }
                        DynGeometry::Polygon(polygon) => {
                            capacity = capacity.checked_add(polygon.wkt_capacity_hint()?)?;
                        }
                        DynGeometry::MultiPoint(multipoint) => {
                            capacity = capacity.checked_add(multipoint.wkt_capacity_hint()?)?;
                        }
                        DynGeometry::MultiLineString(multilinestring) => {
                            capacity =
                                capacity.checked_add(multilinestring.wkt_capacity_hint()?)?;
                        }
                        DynGeometry::MultiPolygon(multipolygon) => {
                            capacity = capacity.checked_add(multipolygon.wkt_capacity_hint()?)?;
                        }
                        DynGeometry::GeometryCollection(nested) => {
                            capacity = capacity.checked_add(32)?;
                            stack.extend(nested);
                        }
                    }
                }
                Some(capacity)
            }
        }
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> core::fmt::Result {
        // Only the `GeometryCollection` arm nests; the leaf/multi arms
        // delegate to their own non-recursive writers. Walk the nesting
        // with an explicit stack rather than recursion so a deeply nested
        // `DynGeometry` cannot overflow the native stack (an uncatchable
        // process abort). Each stack item is a fragment of pending work.
        enum Frag<'a, Cs: CoordinateSystem> {
            /// Emit this geometry (a leaf writes directly; a collection
            /// pushes its own open/members/close fragments).
            Geom(&'a DynGeometry<f64, Cs>),
            /// Emit a literal (open paren, comma, or close paren).
            Lit(&'static str),
        }

        let mut stack = alloc::vec![Frag::Geom(self)];
        while let Some(frag) = stack.pop() {
            match frag {
                Frag::Lit(s) => out.write_str(s)?,
                Frag::Geom(g) => match g {
                    DynGeometry::Point(p) => p.write_wkt(out)?,
                    DynGeometry::LineString(ls) => ls.write_wkt(out)?,
                    DynGeometry::Polygon(pg) => pg.write_wkt(out)?,
                    DynGeometry::MultiPoint(mp) => mp.write_wkt(out)?,
                    DynGeometry::MultiLineString(mls) => mls.write_wkt(out)?,
                    DynGeometry::MultiPolygon(mpg) => mpg.write_wkt(out)?,
                    DynGeometry::GeometryCollection(items) => {
                        if items.is_empty() {
                            out.write_str("GEOMETRYCOLLECTION EMPTY")?;
                            continue;
                        }
                        // Push in reverse so they pop in source order:
                        // "(", g0, ",", g1, ",", …, ")". The stack is LIFO.
                        stack.push(Frag::Lit(")"));
                        for (i, item) in items.iter().enumerate().rev() {
                            stack.push(Frag::Geom(item));
                            if i > 0 {
                                stack.push(Frag::Lit(","));
                            }
                        }
                        stack.push(Frag::Lit("GEOMETRYCOLLECTION("));
                    }
                },
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    //! Canonical-output witnesses. Mirrors the string-equality checks in
    //! `boost/geometry/test/io/wkt/wkt.cpp`.
    #![allow(
        clippy::float_cmp,
        reason = "coordinates are exact integer literals in these fixtures"
    )]

    use super::*;
    use alloc::vec;
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type Pt = Point2D<f64, Cartesian>;

    #[test]
    fn point_canonical() {
        let p = Pt::new(10.0, 10.0);
        assert_eq!(to_wkt(&p), "POINT(10 10)");
    }

    #[test]
    fn nested_collection_writer_is_iterative_and_correct() {
        use geometry_model::DynGeometry;
        // The DynGeometry writer walks nesting with an explicit stack, not
        // recursion. Verify it emits the same nested output as before...
        let g = DynGeometry::<f64, Cartesian>::GeometryCollection(vec![
            DynGeometry::Point(Pt::new(1.0, 1.0)),
            DynGeometry::GeometryCollection(vec![DynGeometry::Point(Pt::new(2.0, 2.0))]),
        ]);
        assert_eq!(
            to_wkt(&g),
            "GEOMETRYCOLLECTION(POINT(1 1),GEOMETRYCOLLECTION(POINT(2 2)))"
        );
        // ...and does not overflow the stack on a deeply nested value.
        let mut deep = DynGeometry::<f64, Cartesian>::Point(Pt::new(0.0, 0.0));
        for _ in 0..200_000 {
            deep = DynGeometry::GeometryCollection(vec![deep]);
        }
        assert!(to_wkt(&deep).starts_with("GEOMETRYCOLLECTION("));
        core::mem::forget(deep); // avoid the still-recursive value Drop
    }

    #[test]
    fn fractional_coord_round_trips() {
        let p = Pt::new(1.5, -2.25);
        assert_eq!(to_wkt(&p), "POINT(1.5 -2.25)");
    }

    #[test]
    fn scalar_format_stays_compatible_with_rust_display() {
        for value in [
            -0.0,
            1.0,
            -2.25,
            0.000_001,
            1.0e-7,
            1.234_567_890_123_456_7e100,
            f64::MIN_POSITIVE,
            f64::MAX,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NAN,
        ] {
            let point = Pt::new(value, value);
            let expected = if value == 0.0 {
                "POINT(0 0)".into()
            } else {
                alloc::format!("POINT({value} {value})")
            };
            let observed = to_wkt(&point);
            assert_eq!(observed, expected, "format changed for {value:?}");
        }
    }

    #[test]
    fn scientific_notation_expansion_covers_each_decimal_position() {
        let mut out = String::new();
        write_expanded_scalar(&mut out, "1", "-7").unwrap();
        assert_eq!(out, "0.0000001");

        out.clear();
        write_expanded_scalar(&mut out, "-1.25", "+5").unwrap();
        assert_eq!(out, "-125000");

        out.clear();
        write_expanded_scalar(&mut out, "1.234", "2").unwrap();
        assert_eq!(out, "123.4");
    }

    #[test]
    fn linestring_canonical() {
        let ls = Linestring(vec![
            Pt::new(10.0, 10.0),
            Pt::new(20.0, 20.0),
            Pt::new(30.0, 40.0),
        ]);
        assert_eq!(to_wkt(&ls), "LINESTRING(10 10,20 20,30 40)");
    }

    #[test]
    fn polygon_with_hole_canonical() {
        let outer = Ring::from_vec(vec![
            Pt::new(0.0, 0.0),
            Pt::new(0.0, 10.0),
            Pt::new(10.0, 10.0),
            Pt::new(10.0, 0.0),
            Pt::new(0.0, 0.0),
        ]);
        let hole = Ring::from_vec(vec![
            Pt::new(2.0, 2.0),
            Pt::new(2.0, 4.0),
            Pt::new(4.0, 4.0),
            Pt::new(4.0, 2.0),
            Pt::new(2.0, 2.0),
        ]);
        let poly = Polygon::with_inners(outer, vec![hole]);
        assert_eq!(
            to_wkt(&poly),
            "POLYGON((0 0,0 10,10 10,10 0,0 0),(2 2,2 4,4 4,4 2,2 2))"
        );
        assert_eq!(to_wkt_polygon(&poly), to_wkt(&poly));
    }

    #[test]
    fn multipoint_canonical() {
        let mp = MultiPoint(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)]);
        assert_eq!(to_wkt(&mp), "MULTIPOINT((10 10),(20 20))");
    }

    #[test]
    fn geometry_collection_canonical() {
        let g = DynGeometry::<f64, Cartesian>::GeometryCollection(vec![
            DynGeometry::Point(Pt::new(10.0, 10.0)),
            DynGeometry::LineString(Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)])),
        ]);
        assert_eq!(
            to_wkt(&g),
            "GEOMETRYCOLLECTION(POINT(10 10),LINESTRING(10 10,20 20))"
        );
    }

    /// Each empty container serialises to the OGC `<TYPE> EMPTY` form,
    /// never `<TYPE>()`.
    #[test]
    fn empty_containers_use_the_empty_keyword() {
        use geometry_model::{MultiLinestring, MultiPolygon, Polygon, Ring};
        assert_eq!(to_wkt(&Linestring::<Pt>(vec![])), "LINESTRING EMPTY");
        assert_eq!(to_wkt(&Ring::<Pt>::from_vec(vec![])), "POLYGON EMPTY");
        assert_eq!(
            to_wkt(&Polygon::<Pt>::new(Ring::from_vec(vec![]))),
            "POLYGON EMPTY"
        );
        assert_eq!(to_wkt(&MultiPoint::<Pt>(vec![])), "MULTIPOINT EMPTY");
        assert_eq!(
            to_wkt(&MultiLinestring::<Linestring<Pt>>(vec![])),
            "MULTILINESTRING EMPTY"
        );
        assert_eq!(
            to_wkt(&MultiPolygon::<Polygon<Pt>>(vec![])),
            "MULTIPOLYGON EMPTY"
        );
        assert_eq!(
            to_wkt(&DynGeometry::<f64, Cartesian>::GeometryCollection(vec![])),
            "GEOMETRYCOLLECTION EMPTY"
        );
    }

    /// A bare, non-empty `Ring` serialises as a single-ring polygon
    /// (there is no standalone `RING` keyword in OGC WKT).
    #[test]
    fn bare_ring_serialises_as_single_ring_polygon() {
        use geometry_model::Ring;
        let ring: Ring<Pt> = Ring::from_vec(vec![
            Pt::new(0.0, 0.0),
            Pt::new(1.0, 0.0),
            Pt::new(1.0, 1.0),
            Pt::new(0.0, 0.0),
        ]);
        assert_eq!(to_wkt(&ring), "POLYGON((0 0,1 0,1 1,0 0))");
    }

    /// An integer-valued coordinate too large for the `i64` fast path
    /// falls back to the default float format (which keeps `.0`-free
    /// scientific/decimal form as Rust prints it), still round-tripping.
    #[test]
    fn huge_integer_coordinate_uses_the_float_fallback() {
        // 1e16 is integer-valued but exceeds the 2^53 fast-path guard.
        let p = Pt::new(1e16, 0.0);
        let s = to_wkt(&p);
        // The x ordinate is emitted via the float path; parse it back to
        // confirm the value survives.
        let inner = s.trim_start_matches("POINT(").trim_end_matches(')');
        let x: f64 = inner.split(' ').next().unwrap().parse().unwrap();
        assert_eq!(x, 1e16);
    }
}
