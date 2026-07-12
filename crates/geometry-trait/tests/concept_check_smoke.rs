//! Positive concept-check tests — every helper in `check.rs` must accept
//! a well-formed model of its concept.
//!
//! Counterpart to `boost/geometry/test/concepts/point_well_formed*.cpp`
//! and the per-concept `well_formed` tests Boost ships. The fail-side
//! fixtures live under `tests/ui/` and are driven by `trybuild`.

use geometry_cs::Cartesian;
use geometry_tag::{
    BoxTag, GeometryCollectionTag, LinestringTag, MultiLinestringTag, MultiPointTag,
    MultiPolygonTag, PointTag, PolygonTag, PolyhedralSurfaceTag, RingTag, SegmentTag,
};
use geometry_trait::{
    Box, Geometry, GeometryCollection, IndexedAccess, Linestring, MultiLinestring, MultiPoint,
    MultiPolygon, Point, PointMut, Polygon, PolyhedralSurface, Ring, Segment, check_box,
    check_geometry_collection, check_indexed_access, check_linestring, check_multi_linestring,
    check_multi_point, check_multi_polygon, check_point, check_polygon, check_polyhedral_surface,
    check_ring, check_segment,
};

// --- A well-formed 2D Cartesian point used by every concept below. ---
#[derive(Default, Clone)]
struct Xy {
    x: f64,
    y: f64,
}

impl Geometry for Xy {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for Xy {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;

    fn get<const D: usize>(&self) -> f64 {
        if D == 0 { self.x } else { self.y }
    }
}

impl PointMut for Xy {
    fn set<const D: usize>(&mut self, v: f64) {
        if D == 0 {
            self.x = v;
        } else {
            self.y = v;
        }
    }
}

#[test]
fn well_formed_point_passes_concept_check() {
    check_point::<Xy>();
}

// --- Box / Segment share `IndexedAccess`. ---
struct AaBox {
    c: [[f64; 2]; 2],
}

impl Geometry for AaBox {
    type Kind = BoxTag;
    type Point = Xy;
}

impl IndexedAccess for AaBox {
    fn get_indexed<const I: usize, const D: usize>(&self) -> f64 {
        self.c[I][D]
    }
    fn set_indexed<const I: usize, const D: usize>(&mut self, v: f64) {
        self.c[I][D] = v;
    }
}

impl Box for AaBox {}

#[test]
fn well_formed_box_passes_concept_check() {
    check_box::<AaBox>();
    check_indexed_access::<AaBox>();
}

struct Seg {
    e: [[f64; 2]; 2],
}

impl Geometry for Seg {
    type Kind = SegmentTag;
    type Point = Xy;
}

impl IndexedAccess for Seg {
    fn get_indexed<const I: usize, const D: usize>(&self) -> f64 {
        self.e[I][D]
    }
    fn set_indexed<const I: usize, const D: usize>(&mut self, v: f64) {
        self.e[I][D] = v;
    }
}

impl Segment for Seg {}

#[test]
fn well_formed_segment_passes_concept_check() {
    check_segment::<Seg>();
}

// --- Vec-backed Linestring / Ring. ---
struct VLs(Vec<Xy>);

impl Geometry for VLs {
    type Kind = LinestringTag;
    type Point = Xy;
}

impl Linestring for VLs {
    fn points(&self) -> impl ExactSizeIterator<Item = &Xy> + Clone {
        self.0.iter()
    }
}

#[test]
fn well_formed_linestring_passes_concept_check() {
    check_linestring::<VLs>();
}

struct VRing(Vec<Xy>);

impl Geometry for VRing {
    type Kind = RingTag;
    type Point = Xy;
}

impl Ring for VRing {
    fn points(&self) -> impl ExactSizeIterator<Item = &Xy> + Clone {
        self.0.iter()
    }
}

#[test]
fn well_formed_ring_passes_concept_check() {
    check_ring::<VRing>();
}

// --- Polygon, MultiPoint, MultiLinestring, MultiPolygon. ---
struct VPoly {
    outer: VRing,
    inners: Vec<VRing>,
}

impl Geometry for VPoly {
    type Kind = PolygonTag;
    type Point = Xy;
}

impl Polygon for VPoly {
    type Ring = VRing;
    fn exterior(&self) -> &VRing {
        &self.outer
    }
    fn interiors(&self) -> impl ExactSizeIterator<Item = &VRing> {
        self.inners.iter()
    }
}

#[test]
fn well_formed_polygon_passes_concept_check() {
    check_polygon::<VPoly>();
}

struct VMp(Vec<Xy>);

impl Geometry for VMp {
    type Kind = MultiPointTag;
    type Point = Xy;
}

impl MultiPoint for VMp {
    type ItemPoint = Xy;
    fn points(&self) -> impl ExactSizeIterator<Item = &Xy> {
        self.0.iter()
    }
}

#[test]
fn well_formed_multi_point_passes_concept_check() {
    check_multi_point::<VMp>();
}

struct VMls(Vec<VLs>);

impl Geometry for VMls {
    type Kind = MultiLinestringTag;
    type Point = Xy;
}

impl MultiLinestring for VMls {
    type ItemLinestring = VLs;
    fn linestrings(&self) -> impl ExactSizeIterator<Item = &VLs> {
        self.0.iter()
    }
}

#[test]
fn well_formed_multi_linestring_passes_concept_check() {
    check_multi_linestring::<VMls>();
}

struct VMpg(Vec<VPoly>);

impl Geometry for VMpg {
    type Kind = MultiPolygonTag;
    type Point = Xy;
}

impl MultiPolygon for VMpg {
    type ItemPolygon = VPoly;
    fn polygons(&self) -> impl ExactSizeIterator<Item = &VPoly> {
        self.0.iter()
    }
}

#[test]
fn well_formed_multi_polygon_passes_concept_check() {
    check_multi_polygon::<VMpg>();
}

// --- GeometryCollection (homogeneous). ---
struct ManyPoints(Vec<Xy>);

impl Geometry for ManyPoints {
    type Kind = GeometryCollectionTag;
    type Point = Xy;
}

impl GeometryCollection for ManyPoints {
    type Item = Xy;
    fn items(&self) -> impl ExactSizeIterator<Item = &Xy> {
        self.0.iter()
    }
}

#[test]
fn well_formed_geometry_collection_passes_concept_check() {
    check_geometry_collection::<ManyPoints>();
}

// --- PolyhedralSurface (3D faces). ---
#[derive(Clone, Default)]
struct Xyz(f64, f64, f64);

impl Geometry for Xyz {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for Xyz {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 3;

    fn get<const D: usize>(&self) -> f64 {
        match D {
            0 => self.0,
            1 => self.1,
            _ => self.2,
        }
    }
}

impl PointMut for Xyz {
    fn set<const D: usize>(&mut self, v: f64) {
        match D {
            0 => self.0 = v,
            1 => self.1 = v,
            _ => self.2 = v,
        }
    }
}

struct VRing3(Vec<Xyz>);

impl Geometry for VRing3 {
    type Kind = RingTag;
    type Point = Xyz;
}

impl Ring for VRing3 {
    fn points(&self) -> impl ExactSizeIterator<Item = &Xyz> + Clone {
        self.0.iter()
    }
}

struct VPoly3 {
    outer: VRing3,
    inners: Vec<VRing3>,
}

impl Geometry for VPoly3 {
    type Kind = PolygonTag;
    type Point = Xyz;
}

impl Polygon for VPoly3 {
    type Ring = VRing3;
    fn exterior(&self) -> &VRing3 {
        &self.outer
    }
    fn interiors(&self) -> impl ExactSizeIterator<Item = &VRing3> {
        self.inners.iter()
    }
}

struct VPolyhedral(Vec<VPoly3>);

impl Geometry for VPolyhedral {
    type Kind = PolyhedralSurfaceTag;
    type Point = Xyz;
}

impl PolyhedralSurface for VPolyhedral {
    type Face = VPoly3;
    fn faces(&self) -> impl ExactSizeIterator<Item = &VPoly3> {
        self.0.iter()
    }
}

#[test]
fn well_formed_polyhedral_surface_passes_concept_check() {
    check_polyhedral_surface::<VPolyhedral>();
}
