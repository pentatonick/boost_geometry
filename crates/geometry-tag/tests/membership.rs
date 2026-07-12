//! Compile-time witnesses that each tag is a member of the right
//! categories in the hierarchy. The test bodies are deliberately just
//! turbofish calls — if a required `impl` is missing the file fails to
//! compile, which is exactly the dispatch property the kernel relies on.
//!
//! The witness helpers are intentionally underscore-prefixed (per the
//! T04 spec) so they are unmistakably *only* compile-time probes and do
//! not get picked up as part of the public test surface; clippy's
//! `used_underscore_items` lint would otherwise object to calling them.

#![allow(clippy::used_underscore_items)]

use geometry_tag::{
    Areal, BoxTag, GeometryCollectionTag, Linear, LinestringTag, Multi, MultiLinestringTag,
    MultiPointTag, MultiPolygonTag, PointTag, Pointlike, PolygonTag, Polygonal,
    PolyhedralSurfaceTag, Polylinear, RingTag, SameAs, SegmentTag, Single, Volumetric,
};

fn _is_single<T: Single>() {}
fn _is_multi<T: Multi>() {}
fn _is_pointlike<T: Pointlike>() {}
fn _is_linear<T: Linear>() {}
fn _is_polylinear<T: Polylinear>() {}
fn _is_areal<T: Areal>() {}
fn _is_polygonal<T: Polygonal>() {}
fn _is_volumetric<T: Volumetric>() {}

#[test]
fn tag_memberships_compile() {
    // Single vs Multi axis.
    _is_single::<PointTag>();
    _is_single::<SegmentTag>();
    _is_single::<LinestringTag>();
    _is_single::<RingTag>();
    _is_single::<PolygonTag>();
    _is_single::<BoxTag>();
    _is_single::<PolyhedralSurfaceTag>();
    _is_multi::<MultiPointTag>();
    _is_multi::<MultiLinestringTag>();
    _is_multi::<MultiPolygonTag>();
    _is_multi::<GeometryCollectionTag>();

    // Pointlike.
    _is_pointlike::<PointTag>();
    _is_pointlike::<MultiPointTag>();

    // Linear / Polylinear.
    _is_linear::<SegmentTag>();
    _is_linear::<LinestringTag>();
    _is_linear::<MultiLinestringTag>();
    _is_polylinear::<LinestringTag>();
    _is_polylinear::<MultiLinestringTag>();

    // Areal / Polygonal — Polygonal: Areal so polygonal witnesses also satisfy Areal.
    _is_areal::<RingTag>();
    _is_areal::<PolygonTag>();
    _is_areal::<BoxTag>();
    _is_areal::<MultiPolygonTag>();
    _is_polygonal::<RingTag>();
    _is_polygonal::<PolygonTag>();
    _is_polygonal::<MultiPolygonTag>();

    // Volumetric.
    _is_volumetric::<PolyhedralSurfaceTag>();
}

fn _same_as<A, B: SameAs<A>>() {}

#[test]
fn same_as_only_holds_for_equal_types() {
    _same_as::<PointTag, PointTag>();
    _same_as::<SegmentTag, SegmentTag>();
    _same_as::<GeometryCollectionTag, GeometryCollectionTag>();
    // _same_as::<PointTag, SegmentTag>();   // would fail to compile (good)
}
