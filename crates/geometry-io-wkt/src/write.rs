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
    DynGeometry, GeometryValue, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point,
    Polygon, Ring,
};
use geometry_trait::{
    Linestring as LinestringTrait, MultiLinestring as MultiLinestringTrait,
    MultiPoint as MultiPointTrait, MultiPolygon as MultiPolygonTrait, Point as PointTrait,
    Polygon as PolygonTrait, Ring as RingTrait,
};

use crate::geometry_structure::{self, GeometryStructureError};
use crate::wkt_write_error::WktWriteError;

/// Typical bytes reserved per 2D coordinate pair by [`to_wkt`].
///
/// This deliberately remains a hint rather than an upper bound: ordinary
/// coordinates avoid geometric `String` growth while unusually long decimal
/// spellings can still grow the buffer normally.
const POINT_CAPACITY: usize = 16;

/// Serialise a geometry to a canonical WKT [`String`].
///
/// Built-in output re-parses through [`from_wkt_2d`](crate::from_wkt_2d).
/// Empty points and populated NaN points remain distinct.
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
/// assert_eq!(to_wkt(&p).unwrap(), "POINT(10 10)");
/// ```
///
/// # Errors
///
/// Returns an error for values outside the supported XY profile or a failing
/// custom writer. See the crate documentation for validation and migration rules.
pub fn to_wkt<G: WriteWkt + ?Sized>(g: &G) -> Result<String, WktWriteError> {
    let mut out = String::with_capacity(g.wkt_capacity_hint().unwrap_or(0));
    g.write_wkt_string(&mut out)?;
    Ok(out)
}

/// Serialise any polygon implementing [`PolygonTrait`] to canonical WKT.
///
/// This is the bring-your-own-type counterpart to [`to_wkt`]. It reads the
/// polygon through the public geometry traits, including its interior rings,
/// without first converting it to a `geometry_model` type.
///
/// # Errors
///
/// Returns an error for values outside the supported XY profile or a failing
/// custom writer. See the crate documentation for validation and migration rules.
pub fn to_wkt_polygon<Pg>(polygon: &Pg) -> Result<String, WktWriteError>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    let mut out = String::with_capacity(polygon_capacity(polygon).unwrap_or(0));
    write_wkt_polygon(polygon, &mut out)?;
    Ok(out)
}

/// Serialise a geometry into any [`core::fmt::Write`] sink.
///
/// The streaming counterpart to [`to_wkt`]; use it to write straight
/// into a caller-owned buffer or formatter. Mirrors the operator
/// `<<` overload in `boost/geometry/io/wkt/write.hpp`.
///
/// # Errors
///
/// Returns geometry validation errors and wraps sink failures. On error,
/// the sink may contain a partial geometry; discard that partial output.
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
pub fn write_wkt<G: WriteWkt, W: core::fmt::Write>(
    g: &G,
    out: &mut W,
) -> Result<(), WktWriteError> {
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
    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError>;

    /// Emit directly into the owned buffer used by [`to_wkt`].
    ///
    /// The default preserves external implementations. Built-in models with
    /// hot coordinate-sequence paths override it so their scalar loop can be
    /// monomorphized for [`String`] without changing the object-safe
    /// streaming method.
    fn write_wkt_string(&self, out: &mut String) -> Result<(), WktWriteError> {
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
///
/// NaN is a `PostGIS` text token; infinity is not. Zero retains its sign.
fn write_scalar<W: core::fmt::Write + ?Sized>(out: &mut W, v: f64) -> Result<(), WktWriteError> {
    if v.is_infinite() {
        return Err(WktWriteError::InfiniteCoordinate);
    }
    if v.is_nan() {
        return Ok(out.write_str("NaN")?);
    }
    if v == 0.0 {
        return Ok(out.write_str(if v.is_sign_negative() { "-0" } else { "0" })?);
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
            return out
                .write_str(itoa::Buffer::new().format(integer))
                .map_err(WktWriteError::from);
        }
    }
    let mut buffer = ryu::Buffer::new();
    let formatted = buffer.format(v);
    let Some(exponent_pos) = formatted.find('e') else {
        return out
            .write_str(formatted.strip_suffix(".0").unwrap_or(formatted))
            .map_err(WktWriteError::from);
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
) -> Result<(), WktWriteError> {
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
        let zeroes = usize::try_from(-decimal_pos).expect("negative decimal position");
        write_zeroes(out, zeroes)?;
        return out.write_str(digits).map_err(WktWriteError::from);
    }

    let decimal_pos = usize::try_from(decimal_pos).expect("positive decimal position");
    if decimal_pos >= digit_count {
        out.write_str(digits)?;
        return write_zeroes(out, decimal_pos - digit_count);
    }

    out.write_str(&digits[..decimal_pos])?;
    out.write_char('.')?;
    out.write_str(&digits[decimal_pos..])
        .map_err(WktWriteError::from)
}

fn write_zeroes<W: core::fmt::Write + ?Sized>(
    out: &mut W,
    mut count: usize,
) -> Result<(), WktWriteError> {
    const ZEROES: &str = "00000000000000000000000000000000";
    while count >= ZEROES.len() {
        out.write_str(ZEROES)?;
        count -= ZEROES.len();
    }
    out.write_str(&ZEROES[..count]).map_err(WktWriteError::from)
}

/// Emit one point's ordinates as `x y` (no keyword, no parens). Shared
/// by every coordinate-bearing kind. Only the first two dimensions are
/// written — this is a 2D port.
fn write_coords<P: PointTrait<Scalar = f64>, W: core::fmt::Write + ?Sized>(
    out: &mut W,
    p: &P,
) -> Result<(), WktWriteError> {
    xy::<P>()?;
    write_scalar(out, p.get::<0>())?;
    out.write_char(' ')?;
    write_scalar(out, p.get::<1>())
}

/// Emit a comma-separated coordinate list `x y,x y,…` (no surrounding
/// parens). Shared by linestrings and rings.
fn write_point_seq<'a, P, I, W>(out: &mut W, points: I) -> Result<(), WktWriteError>
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
fn write_polygon_rings<Pg, W>(out: &mut W, pg: &Pg) -> Result<(), WktWriteError>
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
    out.write_char(')').map_err(WktWriteError::from)
}

fn write_point<P: PointTrait<Scalar = f64>>(
    point: &P,
    out: &mut dyn core::fmt::Write,
) -> Result<(), WktWriteError> {
    xy::<P>()?;
    out.write_str("POINT(")?;
    write_coords(out, point)?;
    out.write_char(')').map_err(WktWriteError::from)
}

impl<Cs: CoordinateSystem> WriteWkt for Point<f64, 2, Cs> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        Some(64)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        write_point(self, out)
    }
}

impl<P: PointTrait<Scalar = f64>> WriteWkt for Linestring<P> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        32usize.checked_add(point_seq_capacity(self.points().len())?)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        write_linestring(self, out)
    }

    fn write_wkt_string(&self, out: &mut String) -> Result<(), WktWriteError> {
        write_linestring(self, out)
    }
}

fn write_linestring<P, W>(linestring: &Linestring<P>, out: &mut W) -> Result<(), WktWriteError>
where
    P: PointTrait<Scalar = f64>,
    W: core::fmt::Write + ?Sized,
{
    xy::<P>()?;
    geometry_structure::linestring_count(linestring.points().count())?;
    // OGC WKT spells an empty geometry `<TYPE> EMPTY`, not `<TYPE>()`
    // — the latter is not grammar the reader (or Boost) accepts.
    if linestring.points().next().is_none() {
        return out
            .write_str("LINESTRING EMPTY")
            .map_err(WktWriteError::from);
    }
    out.write_str("LINESTRING(")?;
    write_point_seq(out, linestring.points())?;
    out.write_char(')').map_err(WktWriteError::from)
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

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        xy::<P>()?;
        // A bare ring serialises as a single-ring polygon — the OGC WKT
        // grammar has no standalone RING keyword.
        if self.points().next().is_none() {
            return out.write_str("POLYGON EMPTY").map_err(WktWriteError::from);
        }
        geometry_structure::ring(self.points())?;
        out.write_str("POLYGON((")?;
        write_point_seq(out, self.points())?;
        out.write_str("))").map_err(WktWriteError::from)
    }
}

impl<P: PointTrait<Scalar = f64>> WriteWkt for Polygon<P, true, true> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        polygon_capacity(self)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        write_wkt_polygon(self, out)
    }

    fn write_wkt_string(&self, out: &mut String) -> Result<(), WktWriteError> {
        write_wkt_polygon(self, out)
    }
}

/// Write any XY polygon through its geometry traits.
///
/// # Errors
///
/// Returns dimension, scalar, structure or sink errors. Sink contents may
/// be partial after an error and must not be used as a geometry record.
///
/// ```
/// use geometry_io_wkt::write_wkt_polygon;
/// use geometry_model::{Point2D, Polygon};
/// let polygon = Polygon::<Point2D<f64>>::default();
/// let mut text = String::new();
/// write_wkt_polygon(&polygon, &mut text).unwrap();
/// assert_eq!(text, "POLYGON EMPTY");
/// ```
pub fn write_wkt_polygon<Pg, W>(polygon: &Pg, out: &mut W) -> Result<(), WktWriteError>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
    W: core::fmt::Write + ?Sized,
{
    xy::<Pg::Point>()?;
    polygon_structure(polygon)?;
    if polygon.exterior().points().next().is_none() {
        return out.write_str("POLYGON EMPTY").map_err(WktWriteError::from);
    }
    out.write_str("POLYGON")?;
    write_polygon_rings(out, polygon)
}

impl<P: PointTrait<Scalar = f64>> WriteWkt for MultiPoint<P> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        32usize.checked_add(point_seq_capacity(self.points().len())?)
    }

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        write_multipoint(self.points().map(Some), out)
    }
}

fn write_multipoint<'a, P, W>(
    points: impl ExactSizeIterator<Item = Option<&'a P>>,
    out: &mut W,
) -> Result<(), WktWriteError>
where
    P: PointTrait<Scalar = f64> + 'a,
    W: core::fmt::Write + ?Sized,
{
    xy::<P>()?;
    if points.len() == 0 {
        return Ok(out.write_str("MULTIPOINT EMPTY")?);
    }
    out.write_str("MULTIPOINT(")?;
    for (index, point) in points.enumerate() {
        if index > 0 {
            out.write_char(',')?;
        }
        if let Some(point) = point {
            out.write_char('(')?;
            write_coords(out, point)?;
            out.write_char(')')?;
        } else {
            out.write_str("EMPTY")?;
        }
    }
    Ok(out.write_char(')')?)
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

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        xy::<L::Point>()?;
        if self.linestrings().next().is_none() {
            return out
                .write_str("MULTILINESTRING EMPTY")
                .map_err(WktWriteError::from);
        }
        out.write_str("MULTILINESTRING(")?;
        for (i, ls) in self.linestrings().enumerate() {
            if i > 0 {
                out.write_char(',')?;
            }
            geometry_structure::linestring_count(ls.points().count())?;
            // A member with no vertices is spelled `EMPTY`, not `()`:
            // `<multilinestring text>` is a list of `<linestring text>`,
            // and that production admits `<empty set>` in its own right
            // (OGC SFA-1 06-103r4 §7.2.2). Emitting `()` produces a
            // string this crate's own reader rejects.
            if ls.points().next().is_none() {
                out.write_str("EMPTY")?;
                continue;
            }
            out.write_char('(')?;
            write_point_seq(out, ls.points())?;
            out.write_char(')')?;
        }
        out.write_char(')').map_err(WktWriteError::from)
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

    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        xy::<Pg::Point>()?;
        if self.polygons().next().is_none() {
            return out
                .write_str("MULTIPOLYGON EMPTY")
                .map_err(WktWriteError::from);
        }
        out.write_str("MULTIPOLYGON(")?;
        for (i, pg) in self.polygons().enumerate() {
            if i > 0 {
                out.write_char(',')?;
            }
            polygon_structure(pg)?;
            // Same rule as `MULTILINESTRING`: `<multipolygon text>` is a
            // list of `<polygon text>`, which admits `<empty set>`. A
            // member with no exterior vertices is `EMPTY`, not `(())`.
            if pg.exterior().points().next().is_none() {
                out.write_str("EMPTY")?;
                continue;
            }
            write_polygon_rings(out, pg)?;
        }
        out.write_char(')').map_err(WktWriteError::from)
    }
}

/// A private view used by both XY value representations.
trait GeometryText: Sized {
    fn collection(&self) -> Option<&[Self]>;
    fn leaf_capacity(&self) -> Option<usize>;
    fn write_leaf(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError>;
}

fn geometry_capacity<G: GeometryText>(geometry: &G) -> Option<usize> {
    let mut pending = alloc::vec![(geometry, 0)];
    let mut size = 0_usize;
    while let Some((value, depth)) = pending.pop() {
        if depth >= geometry_structure::MAX_DEPTH {
            return None;
        }
        if let Some(children) = value.collection() {
            size = size.checked_add(32)?;
            pending.extend(children.iter().map(|child| (child, depth + 1)));
        } else {
            size = size.checked_add(value.leaf_capacity()?)?;
        }
    }
    Some(size)
}

fn write_geometry<G: GeometryText>(
    geometry: &G,
    out: &mut dyn core::fmt::Write,
) -> Result<(), WktWriteError> {
    enum Fragment<'a, G> {
        Geometry { value: &'a G, depth: usize },
        Literal(&'static str),
    }
    let mut pending = alloc::vec![Fragment::Geometry {
        value: geometry,
        depth: 0
    }];
    while let Some(fragment) = pending.pop() {
        let (value, depth) = match fragment {
            Fragment::Literal(text) => {
                out.write_str(text)?;
                continue;
            }
            Fragment::Geometry { value, depth } => (value, depth),
        };
        if depth >= geometry_structure::MAX_DEPTH {
            return Err(WktWriteError::NestingTooDeep);
        }
        let Some(children) = value.collection() else {
            value.write_leaf(out)?;
            continue;
        };
        if children.is_empty() {
            out.write_str("GEOMETRYCOLLECTION EMPTY")?;
            continue;
        }
        pending.push(Fragment::Literal(")"));
        for (index, child) in children.iter().enumerate().rev() {
            pending.push(Fragment::Geometry {
                value: child,
                depth: depth + 1,
            });
            if index > 0 {
                pending.push(Fragment::Literal(","));
            }
        }
        out.write_str("GEOMETRYCOLLECTION(")?;
    }
    Ok(())
}

impl<Cs: CoordinateSystem> GeometryText for DynGeometry<f64, Cs> {
    fn collection(&self) -> Option<&[Self]> {
        if let Self::GeometryCollection(children) = self {
            Some(children)
        } else {
            None
        }
    }

    fn leaf_capacity(&self) -> Option<usize> {
        match self {
            Self::Point(g) => g.wkt_capacity_hint(),
            Self::LineString(g) => g.wkt_capacity_hint(),
            Self::Polygon(g) => g.wkt_capacity_hint(),
            Self::MultiPoint(g) => g.wkt_capacity_hint(),
            Self::MultiLineString(g) => g.wkt_capacity_hint(),
            Self::MultiPolygon(g) => g.wkt_capacity_hint(),
            Self::GeometryCollection(_) => unreachable!("collections are handled by the traversal"),
        }
    }

    fn write_leaf(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        match self {
            Self::Point(g) => g.write_wkt(out),
            Self::LineString(g) => g.write_wkt(out),
            Self::Polygon(g) => g.write_wkt(out),
            Self::MultiPoint(g) => g.write_wkt(out),
            Self::MultiLineString(g) => g.write_wkt(out),
            Self::MultiPolygon(g) => g.write_wkt(out),
            Self::GeometryCollection(_) => unreachable!("collections are handled by the traversal"),
        }
    }
}

impl<P: PointTrait<Scalar = f64>> GeometryText for GeometryValue<P> {
    fn collection(&self) -> Option<&[Self]> {
        if let Self::GeometryCollection(children) = self {
            Some(children)
        } else {
            None
        }
    }

    fn leaf_capacity(&self) -> Option<usize> {
        match self {
            Self::Point(_) => Some(64),
            Self::LineString(g) => g.wkt_capacity_hint(),
            Self::Polygon(g) => g.wkt_capacity_hint(),
            Self::MultiPoint(points) => 32_usize.checked_add(point_seq_capacity(points.len())?),
            Self::MultiLineString(g) => g.wkt_capacity_hint(),
            Self::MultiPolygon(g) => g.wkt_capacity_hint(),
            Self::GeometryCollection(_) => unreachable!("collections are handled by the traversal"),
        }
    }

    fn write_leaf(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        match self {
            Self::Point(Some(point)) => write_point(point, out),
            Self::Point(None) => Ok(out.write_str("POINT EMPTY")?),
            Self::LineString(g) => g.write_wkt(out),
            Self::Polygon(g) => g.write_wkt(out),
            Self::MultiPoint(points) => write_multipoint(points.iter().map(Option::as_ref), out),
            Self::MultiLineString(g) => g.write_wkt(out),
            Self::MultiPolygon(g) => g.write_wkt(out),
            Self::GeometryCollection(_) => unreachable!("collections are handled by the traversal"),
        }
    }
}

impl<Cs: CoordinateSystem> WriteWkt for DynGeometry<f64, Cs> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        geometry_capacity(self)
    }
    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        write_geometry(self, out)
    }
}

impl<P: PointTrait<Scalar = f64>> WriteWkt for GeometryValue<P> {
    fn wkt_capacity_hint(&self) -> Option<usize> {
        geometry_capacity(self)
    }
    fn write_wkt(&self, out: &mut dyn core::fmt::Write) -> Result<(), WktWriteError> {
        xy::<P>()?;
        write_geometry(self, out)
    }
}

fn xy<P: PointTrait>() -> Result<(), WktWriteError> {
    if P::DIM == 2 {
        Ok(())
    } else {
        Err(WktWriteError::UnsupportedDimension { dimensions: P::DIM })
    }
}

fn polygon_structure<Pg: PolygonTrait>(polygon: &Pg) -> Result<(), GeometryStructureError>
where
    Pg::Point: PointTrait<Scalar = f64>,
{
    if polygon.exterior().points().next().is_none() {
        return if polygon.interiors().next().is_none() {
            Ok(())
        } else {
            Err(GeometryStructureError::MissingExterior)
        };
    }
    geometry_structure::ring(polygon.exterior().points())?;
    for ring in polygon.interiors() {
        if ring.points().next().is_none() {
            return Err(GeometryStructureError::EmptyInterior);
        }
        geometry_structure::ring(ring.points())?;
    }
    Ok(())
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
        assert_eq!(to_wkt(&p).unwrap(), "POINT(10 10)");
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
            to_wkt(&g).unwrap(),
            "GEOMETRYCOLLECTION(POINT(1 1),GEOMETRYCOLLECTION(POINT(2 2)))"
        );
        // ...and does not overflow the stack on a deeply nested value.
        let mut deep = DynGeometry::<f64, Cartesian>::Point(Pt::new(0.0, 0.0));
        for _ in 0..200_000 {
            deep = DynGeometry::GeometryCollection(vec![deep]);
        }
        assert_eq!(to_wkt(&deep), Err(WktWriteError::NestingTooDeep));
        core::mem::forget(deep); // avoid the still-recursive value Drop
    }

    #[test]
    fn fractional_coord_round_trips() {
        let p = Pt::new(1.5, -2.25);
        assert_eq!(to_wkt(&p).unwrap(), "POINT(1.5 -2.25)");
    }

    /// Finite formatting follows Rust Display, including negative zero.
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
        ] {
            let point = Pt::new(value, value);
            let expected = alloc::format!("POINT({value} {value})");
            let observed = to_wkt(&point).unwrap();
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
        assert_eq!(to_wkt(&ls).unwrap(), "LINESTRING(10 10,20 20,30 40)");
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
            to_wkt(&poly).unwrap(),
            "POLYGON((0 0,0 10,10 10,10 0,0 0),(2 2,2 4,4 4,4 2,2 2))"
        );
        assert_eq!(to_wkt_polygon(&poly).unwrap(), to_wkt(&poly).unwrap());
    }

    #[test]
    fn multipoint_canonical() {
        let mp = MultiPoint(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)]);
        assert_eq!(to_wkt(&mp).unwrap(), "MULTIPOINT((10 10),(20 20))");
    }

    #[test]
    fn geometry_collection_canonical() {
        let g = DynGeometry::<f64, Cartesian>::GeometryCollection(vec![
            DynGeometry::Point(Pt::new(10.0, 10.0)),
            DynGeometry::LineString(Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)])),
        ]);
        assert_eq!(
            to_wkt(&g).unwrap(),
            "GEOMETRYCOLLECTION(POINT(10 10),LINESTRING(10 10,20 20))"
        );
    }

    /// Each empty container serialises to the OGC `<TYPE> EMPTY` form,
    /// never `<TYPE>()`.
    #[test]
    fn empty_containers_use_the_empty_keyword() {
        use geometry_model::{MultiLinestring, MultiPolygon, Polygon, Ring};
        assert_eq!(
            to_wkt(&Linestring::<Pt>(vec![])).unwrap(),
            "LINESTRING EMPTY"
        );
        assert_eq!(
            to_wkt(&Ring::<Pt>::from_vec(vec![])).unwrap(),
            "POLYGON EMPTY"
        );
        assert_eq!(
            to_wkt(&Polygon::<Pt>::new(Ring::from_vec(vec![]))).unwrap(),
            "POLYGON EMPTY"
        );
        assert_eq!(
            to_wkt(&MultiPoint::<Pt>(vec![])).unwrap(),
            "MULTIPOINT EMPTY"
        );
        assert_eq!(
            to_wkt(&MultiLinestring::<Linestring<Pt>>(vec![])).unwrap(),
            "MULTILINESTRING EMPTY"
        );
        assert_eq!(
            to_wkt(&MultiPolygon::<Polygon<Pt>>(vec![])).unwrap(),
            "MULTIPOLYGON EMPTY"
        );
        assert_eq!(
            to_wkt(&DynGeometry::<f64, Cartesian>::GeometryCollection(vec![])).unwrap(),
            "GEOMETRYCOLLECTION EMPTY"
        );
    }

    /// An *empty member* of a multi-geometry is spelled `EMPTY` too.
    /// `<multipolygon text>` is a list of `<polygon text>` and
    /// `<multilinestring text>` a list of `<linestring text>`, both of
    /// which admit `<empty set>` (OGC SFA-1 06-103r4 §7.2.2). The
    /// degenerate `(())` / `()` spellings this once emitted are not
    /// grammar and do not re-parse.
    #[test]
    fn empty_multi_members_use_the_empty_keyword() {
        use geometry_model::{MultiLinestring, MultiPolygon, Polygon, Ring};
        let empty_polygon = Polygon::<Pt>::new(Ring::from_vec(vec![]));
        assert_eq!(
            to_wkt(&MultiPolygon(vec![empty_polygon.clone()])).unwrap(),
            "MULTIPOLYGON(EMPTY)"
        );
        let filled = Polygon::<Pt>::new(Ring::from_vec(vec![
            Pt::new(0.0, 0.0),
            Pt::new(1.0, 0.0),
            Pt::new(1.0, 1.0),
            Pt::new(0.0, 0.0),
        ]));
        assert_eq!(
            to_wkt(&MultiPolygon(vec![empty_polygon, filled])).unwrap(),
            "MULTIPOLYGON(EMPTY,((0 0,1 0,1 1,0 0)))"
        );
        assert_eq!(
            to_wkt(&MultiLinestring(vec![Linestring::<Pt>(vec![])])).unwrap(),
            "MULTILINESTRING(EMPTY)"
        );
        assert_eq!(
            to_wkt(&MultiLinestring(vec![
                Linestring::<Pt>(vec![]),
                Linestring(vec![Pt::new(0.0, 0.0), Pt::new(1.0, 1.0)]),
            ]))
            .unwrap(),
            "MULTILINESTRING(EMPTY,(0 0,1 1))"
        );
    }

    /// Every spelling the writer produces for an empty member is one the
    /// reader accepts — the property that `(())` violated.
    #[test]
    fn empty_multi_members_round_trip() {
        use geometry_model::{MultiLinestring, MultiPolygon, Polygon, Ring};
        let mp = MultiPolygon(vec![Polygon::<Pt>::new(Ring::from_vec(vec![]))]);
        let written = to_wkt(&mp).unwrap();
        assert_eq!(
            to_wkt(&crate::from_wkt(&written).unwrap()).unwrap(),
            written
        );
        let ml = MultiLinestring(vec![
            Linestring::<Pt>(vec![]),
            Linestring(vec![Pt::new(0.0, 0.0), Pt::new(1.0, 1.0)]),
        ]);
        let written = to_wkt(&ml).unwrap();
        assert_eq!(
            to_wkt(&crate::from_wkt(&written).unwrap()).unwrap(),
            written
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
        assert_eq!(to_wkt(&ring).unwrap(), "POLYGON((0 0,1 0,1 1,0 0))");
    }

    /// An integer-valued coordinate too large for the `i64` fast path
    /// falls back to the default float format (which keeps `.0`-free
    /// scientific/decimal form as Rust prints it), still round-tripping.
    #[test]
    fn huge_integer_coordinate_uses_the_float_fallback() {
        // 1e16 is integer-valued but exceeds the 2^53 fast-path guard.
        let p = Pt::new(1e16, 0.0);
        let s = to_wkt(&p).unwrap();
        // The x ordinate is emitted via the float path; parse it back to
        // confirm the value survives.
        let inner = s.trim_start_matches("POINT(").trim_end_matches(')');
        let x: f64 = inner.split(' ').next().unwrap().parse().unwrap();
        assert_eq!(x, 1e16);
    }
}
