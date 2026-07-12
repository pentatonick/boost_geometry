//! Bidirectional conversion between `geo_types::Geometry<T>` and
//! [`DynGeometry<T, Cartesian>`](geometry_model::DynGeometry).
//!
//! Mirrors the role of Boost's variant adapters
//! (`boost/geometry/geometries/adapted/boost_variant.hpp`), which let a
//! `dynamic_geometry` flow between the ecosystem's variant type and the
//! kernel's dispatch. Here the two free functions
//! [`to_dyn_geometry`] / [`from_dyn_geometry`] move a whole geometry
//! tree between `geo-types`' `Geometry` enum and the kernel's
//! [`DynGeometry`] without the caller re-implementing per-kind
//! dispatch.
//!
//! # Why free functions, not `From`
//!
//! A `From<geo_types::Geometry<T>> for DynGeometry<T, Cartesian>` impl
//! is rejected by the orphan rule (`E0117`): `From` is `std`,
//! `geo_types::Geometry` is foreign, and `DynGeometry` is foreign
//! (`geometry-model`) — no type in the impl head is local to this
//! crate, so coherence forbids it. The capability is therefore exposed
//! as two crate-local free functions instead.
//!
//! # Kind normalisation
//!
//! `geo-types`' `Geometry` enum has three variants the kernel's
//! [`DynGeometry`] does not model directly — `Line`, `Rect`, and
//! `Triangle`. They are mapped to their natural kernel equivalents:
//!
//! * `Line`     → `DynGeometry::LineString` (a two-point line string),
//! * `Rect`     → `DynGeometry::Polygon`    (the rectangle as a ring),
//! * `Triangle` → `DynGeometry::Polygon`    (the triangle as a ring).
//!
//! Coordinates are preserved exactly across every mapping; only the
//! *kind* of those three normalises. The reverse direction therefore
//! reproduces a `LineString` / `Polygon` rather than the original
//! `Line` / `Rect` / `Triangle`.

use alloc::vec::Vec;

use geo_types::{Coord, CoordNum, Geometry as GtGeometry, LineString};
use geometry_coords::CoordinateScalar;
use geometry_cs::Cartesian;
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};

/// The kernel point type every [`DynGeometry`] variant is built on.
type KernelPoint<T> = Point2D<T, Cartesian>;

#[inline]
fn coord_to_point<T: CoordinateScalar + CoordNum>(c: Coord<T>) -> KernelPoint<T> {
    Point2D::new(c.x, c.y)
}

#[inline]
fn point_to_coord<T: CoordinateScalar + CoordNum>(p: &KernelPoint<T>) -> Coord<T> {
    use geometry_trait::Point as _;
    Coord {
        x: p.get::<0>(),
        y: p.get::<1>(),
    }
}

#[inline]
fn line_string_to_kernel<T: CoordinateScalar + CoordNum>(
    ls: LineString<T>,
) -> Linestring<KernelPoint<T>> {
    Linestring::from_vec(ls.0.into_iter().map(coord_to_point).collect())
}

#[inline]
fn line_string_to_ring<T: CoordinateScalar + CoordNum>(ls: LineString<T>) -> Ring<KernelPoint<T>> {
    Ring::from_vec(ls.0.into_iter().map(coord_to_point).collect())
}

#[inline]
fn kernel_to_line_string<T: CoordinateScalar + CoordNum>(
    ls: &Linestring<KernelPoint<T>>,
) -> LineString<T> {
    LineString(ls.0.iter().map(point_to_coord).collect())
}

#[inline]
fn ring_to_line_string<T: CoordinateScalar + CoordNum>(r: &Ring<KernelPoint<T>>) -> LineString<T> {
    LineString(r.0.iter().map(point_to_coord).collect())
}

#[inline]
fn polygon_to_kernel<T: CoordinateScalar + CoordNum>(
    p: geo_types::Polygon<T>,
) -> Polygon<KernelPoint<T>> {
    let (exterior, interiors) = p.into_inner();
    let inners: Vec<_> = interiors.into_iter().map(line_string_to_ring).collect();
    Polygon::with_inners(line_string_to_ring(exterior), inners)
}

#[inline]
fn kernel_to_polygon<T: CoordinateScalar + CoordNum>(
    p: &Polygon<KernelPoint<T>>,
) -> geo_types::Polygon<T> {
    let exterior = ring_to_line_string(&p.outer);
    let interiors: Vec<_> = p.inners.iter().map(ring_to_line_string).collect();
    geo_types::Polygon::new(exterior, interiors)
}

/// Convert a `geo_types::Geometry<T>` tree into a kernel
/// [`DynGeometry<T, Cartesian>`].
///
/// See the [module docs](self) for the `Line` / `Rect` / `Triangle`
/// kind-normalisation rules, and for why this is a free function
/// rather than a `From` impl.
///
/// # Examples
///
/// ```
/// use geo_types::{Geometry, Point};
/// use geometry_adapt_geo_types::dyn_conversion::to_dyn_geometry;
/// use geometry_model::DynKind;
///
/// let g: Geometry<f64> = Point::new(1.0, 2.0).into();
/// let dyn_g = to_dyn_geometry(g);
/// assert_eq!(dyn_g.kind(), DynKind::Point);
/// ```
#[must_use]
pub fn to_dyn_geometry<T: CoordinateScalar + CoordNum>(
    g: GtGeometry<T>,
) -> DynGeometry<T, Cartesian> {
    match g {
        GtGeometry::Point(p) => DynGeometry::Point(coord_to_point(p.0)),
        GtGeometry::Line(l) => DynGeometry::LineString(Linestring::from_vec(vec![
            coord_to_point(l.start),
            coord_to_point(l.end),
        ])),
        GtGeometry::LineString(ls) => DynGeometry::LineString(line_string_to_kernel(ls)),
        GtGeometry::Polygon(p) => DynGeometry::Polygon(polygon_to_kernel(p)),
        GtGeometry::MultiPoint(mp) => DynGeometry::MultiPoint(MultiPoint::from_vec(
            mp.0.into_iter().map(|p| coord_to_point(p.0)).collect(),
        )),
        GtGeometry::MultiLineString(mls) => DynGeometry::MultiLineString(
            MultiLinestring::from_vec(mls.0.into_iter().map(line_string_to_kernel).collect()),
        ),
        GtGeometry::MultiPolygon(mpg) => DynGeometry::MultiPolygon(MultiPolygon::from_vec(
            mpg.0.into_iter().map(polygon_to_kernel).collect(),
        )),
        GtGeometry::GeometryCollection(gc) => {
            DynGeometry::GeometryCollection(gc.0.into_iter().map(to_dyn_geometry).collect())
        }
        GtGeometry::Rect(r) => DynGeometry::Polygon(polygon_to_kernel(r.to_polygon())),
        GtGeometry::Triangle(t) => DynGeometry::Polygon(polygon_to_kernel(t.to_polygon())),
    }
}

/// Convert a kernel [`DynGeometry<T, Cartesian>`] back into a
/// `geo_types::Geometry<T>` tree.
///
/// The inverse of [`to_dyn_geometry`] up to the `Line` / `Rect` /
/// `Triangle` kind-normalisation described in the [module docs](self):
/// a `DynGeometry::LineString` always maps back to a
/// `geo_types::LineString` and a `DynGeometry::Polygon` to a
/// `geo_types::Polygon`, never to the narrower originals.
///
/// # Examples
///
/// ```
/// use geo_types::{Geometry, Point};
/// use geometry_adapt_geo_types::dyn_conversion::from_dyn_geometry;
/// use geometry_cs::Cartesian;
/// use geometry_model::{DynGeometry, Point2D};
///
/// let dyn_g = DynGeometry::<f64, Cartesian>::Point(Point2D::new(1.0, 2.0));
/// let g = from_dyn_geometry(dyn_g);
/// assert_eq!(g, Geometry::Point(Point::new(1.0, 2.0)));
/// ```
#[must_use]
pub fn from_dyn_geometry<T: CoordinateScalar + CoordNum>(
    g: DynGeometry<T, Cartesian>,
) -> GtGeometry<T> {
    match g {
        DynGeometry::Point(p) => GtGeometry::Point(geo_types::Point(point_to_coord(&p))),
        DynGeometry::LineString(ls) => GtGeometry::LineString(kernel_to_line_string(&ls)),
        DynGeometry::Polygon(p) => GtGeometry::Polygon(kernel_to_polygon(&p)),
        DynGeometry::MultiPoint(mp) => GtGeometry::MultiPoint(geo_types::MultiPoint(
            mp.0.iter()
                .map(|p| geo_types::Point(point_to_coord(p)))
                .collect(),
        )),
        DynGeometry::MultiLineString(mls) => GtGeometry::MultiLineString(
            geo_types::MultiLineString::new(mls.0.iter().map(kernel_to_line_string).collect()),
        ),
        DynGeometry::MultiPolygon(mpg) => GtGeometry::MultiPolygon(geo_types::MultiPolygon::new(
            mpg.0.iter().map(kernel_to_polygon).collect(),
        )),
        DynGeometry::GeometryCollection(gc) => GtGeometry::GeometryCollection(
            geo_types::GeometryCollection(gc.into_iter().map(from_dyn_geometry).collect()),
        ),
    }
}
