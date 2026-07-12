//! The [`Geometry`] super-trait: every concept refines it.
//!
//! Mirrors the role of the two C++ trait metafunctions
//! `boost::geometry::traits::tag<G>::type`
//! (`boost/geometry/core/tag.hpp`) and
//! `boost::geometry::traits::point_type<G>::type`
//! (`boost/geometry/core/point_type.hpp`): every modelled geometry
//! exposes (a) its kind, as one of the `*Tag` types from
//! [`geometry_tag`], and (b) the point type its coordinates live in.
//!
//! Splitting these two pieces out into a single Rust super-trait
//! lets every other concept (Point, Segment, Box, Linestring, Ring,
//! Polygon, …) refine `Geometry<Kind = …>` and inherit the
//! `type Point` projection without re-stating it.

use crate::point::Point;

/// Every modelled geometry has a *kind* (its tag) and a *point type*
/// it is built from.
///
/// `Kind` matches `boost::geometry::traits::tag<G>::type`
/// (`boost/geometry/core/tag.hpp`); `Point` matches
/// `boost::geometry::traits::point_type<G>::type`
/// (`boost/geometry/core/point_type.hpp`). For a value whose own
/// kind is [`geometry_tag::PointTag`], the `Point` projection is
/// `Self`.
///
/// # Examples
///
/// Dispatch on the geometry kind via the `Kind` associated type:
///
/// ```
/// use geometry_tag::PointTag;
/// use geometry_trait::Geometry;
///
/// fn is_point<G: Geometry<Kind = PointTag>>(_g: &G) {}
/// ```
pub trait Geometry {
    /// One of the `*Tag` types from
    /// [`geometry_tag`](../geometry_tag/index.html).
    ///
    /// Mirrors `boost::geometry::traits::tag<G>::type`
    /// (`boost/geometry/core/tag.hpp`).
    type Kind;

    /// The point type this geometry is built from. For a value
    /// modelling [`Point`], this is `Self`.
    ///
    /// Mirrors `boost::geometry::traits::point_type<G>::type`
    /// (`boost/geometry/core/point_type.hpp`).
    type Point: Point;
}
