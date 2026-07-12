//! Concept-check helpers — call these in unit tests next to your type
//! to surface adaptation errors close to the source.
//!
//! Mirrors `boost::geometry::concepts::check<T>()` from
//! `boost/geometry/geometries/concepts/check.hpp` and the per-concept
//! `*_concept.hpp` files. Each helper has an empty body — the trait
//! bound *is* the check. Wiring it into a unit test next to the type
//! definition makes the compiler report the adaptation error against
//! that unit test line, not against the algorithm call site that
//! happened to be the first thing to instantiate the bound.
//!
//! See `boost/geometry/test/concepts/point_well_formed*.cpp` for the
//! positive Boost counterparts and
//! `boost/geometry/test/concepts/point_without_*.cpp` for the
//! compile-fail counterparts that the `tests/ui/*` fixtures mirror.
//!
//! # Examples
//!
//! ```
//! use geometry_cs::Cartesian;
//! use geometry_tag::PointTag;
//! use geometry_trait::{check_point, Geometry, Point};
//!
//! #[derive(Default)]
//! struct Xy { x: f64, y: f64 }
//! impl Geometry for Xy { type Kind = PointTag; type Point = Self; }
//! impl Point for Xy {
//!     type Scalar = f64; type Cs = Cartesian; const DIM: usize = 2;
//!     fn get<const D: usize>(&self) -> f64 { if D == 0 { self.x } else { self.y } }
//! }
//!
//! // Compile-time check that `Xy` correctly models the `Point` concept.
//! check_point::<Xy>();
//! ```

use crate::{
    Box, GeometryCollection, IndexedAccess, Linestring, MultiLinestring, MultiPoint, MultiPolygon,
    Point, Polygon, PolyhedralSurface, Ring, Segment,
};

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors `boost::geometry::concepts::check<Point>()` from
/// `boost/geometry/geometries/concepts/check.hpp` instantiated with
/// the point-concept specialisation in
/// `boost/geometry/geometries/concepts/point_concept.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::check_point;
/// fn _call<P: geometry_trait::Point>() { check_point::<P>(); }
/// ```
pub fn check_point<T: Point>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors `boost::geometry::concepts::check<Segment>()` driven by
/// `boost/geometry/geometries/concepts/segment_concept.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::check_segment;
/// fn _call<S: geometry_trait::Segment>() { check_segment::<S>(); }
/// ```
pub fn check_segment<T: Segment>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors `boost::geometry::concepts::check<Box>()` driven by
/// `boost/geometry/geometries/concepts/box_concept.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::check_box;
/// fn _call<B: geometry_trait::Box>() { check_box::<B>(); }
/// ```
pub fn check_box<T: Box>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors `boost::geometry::concepts::check<Linestring>()` driven by
/// `boost/geometry/geometries/concepts/linestring_concept.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::check_linestring;
/// fn _call<L: geometry_trait::Linestring>() { check_linestring::<L>(); }
/// ```
pub fn check_linestring<T: Linestring>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors `boost::geometry::concepts::check<Ring>()` driven by
/// `boost/geometry/geometries/concepts/ring_concept.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::check_ring;
/// fn _call<R: geometry_trait::Ring>() { check_ring::<R>(); }
/// ```
pub fn check_ring<T: Ring>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors `boost::geometry::concepts::check<Polygon>()` driven by
/// `boost/geometry/geometries/concepts/polygon_concept.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::check_polygon;
/// fn _call<P: geometry_trait::Polygon>() { check_polygon::<P>(); }
/// ```
pub fn check_polygon<T: Polygon>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors `boost::geometry::concepts::check<MultiPoint>()` driven by
/// `boost/geometry/geometries/concepts/multi_point_concept.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::check_multi_point;
/// fn _call<M: geometry_trait::MultiPoint>() { check_multi_point::<M>(); }
/// ```
pub fn check_multi_point<T: MultiPoint>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors `boost::geometry::concepts::check<MultiLinestring>()`
/// driven by
/// `boost/geometry/geometries/concepts/multi_linestring_concept.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::check_multi_linestring;
/// fn _call<M: geometry_trait::MultiLinestring>() { check_multi_linestring::<M>(); }
/// ```
pub fn check_multi_linestring<T: MultiLinestring>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors `boost::geometry::concepts::check<MultiPolygon>()` driven
/// by `boost/geometry/geometries/concepts/multi_polygon_concept.hpp`.
///
/// # Examples
///
/// ```
/// use geometry_trait::check_multi_polygon;
/// fn _call<M: geometry_trait::MultiPolygon>() { check_multi_polygon::<M>(); }
/// ```
pub fn check_multi_polygon<T: MultiPolygon>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors the geometry-collection arm of
/// `boost::geometry::concepts::check<T>()` (see
/// `boost/geometry/geometries/concepts/check.hpp`).
///
/// # Examples
///
/// ```
/// use geometry_trait::check_geometry_collection;
/// fn _call<C: geometry_trait::GeometryCollection>() { check_geometry_collection::<C>(); }
/// ```
pub fn check_geometry_collection<T: GeometryCollection>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Mirrors the polyhedral-surface arm of
/// `boost::geometry::concepts::check<T>()` (see
/// `boost/geometry/geometries/concepts/check.hpp`).
///
/// # Examples
///
/// ```
/// use geometry_trait::check_polyhedral_surface;
/// fn _call<P: geometry_trait::PolyhedralSurface>() { check_polyhedral_surface::<P>(); }
/// ```
pub fn check_polyhedral_surface<T: PolyhedralSurface>() {}

/// Call this in a unit test next to your type definition to surface
/// adaptation errors close to the source rather than at the algorithm
/// call site.
///
/// Generic checker for impls of [`IndexedAccess`] — useful when a
/// custom geometry needs the `(I, D)` two-axis access but is neither a
/// [`Box`] nor a [`Segment`].
///
/// # Examples
///
/// ```
/// use geometry_trait::check_indexed_access;
/// fn _call<G: geometry_trait::IndexedAccess>() { check_indexed_access::<G>(); }
/// ```
pub fn check_indexed_access<T: IndexedAccess>() {}
