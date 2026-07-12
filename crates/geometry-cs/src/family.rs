//! Coordinate-system family markers.
//!
//! Mirrors `boost::geometry::traits::cs_tag<Cs>` from
//! `boost/geometry/core/cs.hpp:194-225`: the metafunction that collapses
//! every concrete `cs::spherical<Degree>`, `cs::spherical<Radian>`,
//! `cs::spherical_equatorial<…>` down to a single family tag
//! (`spherical_polar_tag` / `spherical_equatorial_tag`) so strategies
//! dispatch once per family instead of once per `<Unit>` instantiation.
//!
//! Per proposal §3.3 we collapse Boost's two spherical tags into one
//! `SphericalFamily` (the equatorial convention, matching the
//! quickstart and OGC). Strategies ported from
//! `boost/geometry/strategies/spherical/*` that expect colatitude must
//! flip the sign on `π/2 − lat` when translated.

/// Cartesian family marker.
///
/// Mirrors `boost::geometry::cartesian_tag`
/// (`boost/geometry/core/tags.hpp`, the tag chain at
/// `cs.hpp:213-217`).
///
/// # Examples
///
/// ```
/// use geometry_cs::{Cartesian, CartesianFamily, CoordinateSystem};
/// let _: <Cartesian as CoordinateSystem>::Family = CartesianFamily;
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct CartesianFamily;

/// Spherical family marker.
///
/// Mirrors `boost::geometry::spherical_polar_tag` /
/// `spherical_equatorial_tag` (`boost/geometry/core/tags.hpp`, the tag
/// chain at `cs.hpp:200-211`). We collapse both Boost tags into a
/// single family per proposal §3.3.
///
/// # Examples
///
/// ```
/// use geometry_cs::{CoordinateSystem, Degree, Spherical, SphericalFamily};
/// let _: <Spherical<Degree> as CoordinateSystem>::Family = SphericalFamily;
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct SphericalFamily;

/// Geographic family marker.
///
/// Mirrors `boost::geometry::geographic_tag` (`boost/geometry/core/tags.hpp`,
/// the tag chain at `cs.hpp:194-198`).
///
/// # Examples
///
/// ```
/// use geometry_cs::{CoordinateSystem, Degree, Geographic, GeographicFamily};
/// let _: <Geographic<Degree> as CoordinateSystem>::Family = GeographicFamily;
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct GeographicFamily;

/// Polar family marker.
///
/// Mirrors the family tag implied by `cs::polar<DegreeOrRadian>` in
/// `boost/geometry/core/cs.hpp:155-165`. Boost never gives the polar
/// system a dedicated `cs_tag` specialisation (the type is mostly
/// vestigial in the C++ code base); we name it explicitly so that
/// future polar strategies have a family to bind on.
///
/// # Examples
///
/// ```
/// use geometry_cs::{CoordinateSystem, Polar, PolarFamily, Radian};
/// let _: <Polar<Radian> as CoordinateSystem>::Family = PolarFamily;
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct PolarFamily;
