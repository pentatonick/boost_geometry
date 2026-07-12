//! The [`CoordinateSystem`] trait and its four concrete types.
//!
//! Mirrors the `cs::{cartesian,spherical,geographic,polar}` empty /
//! `<Unit>`-parameterised tag types in
//! `boost/geometry/core/cs.hpp:85-165`, together with the
//! `traits::cs_tag<Cs>` family classifier at `cs.hpp:194-225`.
//!
//! The `Family` associated type is the Rust analogue of Boost's
//! `cs_tag<Cs>::type`. Algorithm bounds key on the family
//! (`<P::Cs as CoordinateSystem>::Family`) so a strategy written for
//! `SphericalFamily` automatically applies to both
//! `Spherical<Degree>` and `Spherical<Radian>`.

use core::marker::PhantomData;

use crate::family::{CartesianFamily, GeographicFamily, PolarFamily, SphericalFamily};
use crate::unit::AngleUnit;

/// A coordinate system.
///
/// Mirrors `traits::coordinate_system<Point>::type` from
/// `boost/geometry/core/coordinate_system.hpp:43-49`: the typedef every
/// point exposes that identifies *which* coordinate system its values
/// live in. The `Family` associated type is the analogue of Boost's
/// `traits::cs_tag<Cs>::type` (`cs.hpp:186-225`), the family-level
/// classifier that strategies actually dispatch on.
///
/// # Examples
///
/// ```
/// use geometry_cs::{Cartesian, CartesianFamily, CoordinateSystem};
/// // Every CS exposes its `Family`; strategies bind on the family.
/// fn _cartesian_family<C: CoordinateSystem<Family = CartesianFamily>>() {}
/// _cartesian_family::<Cartesian>();
/// ```
pub trait CoordinateSystem {
    /// The CS family this concrete system belongs to.
    ///
    /// Mirrors `boost::geometry::traits::cs_tag<Cs>::type`
    /// (`boost/geometry/core/cs.hpp:194-225`).
    type Family;
}

/// Cartesian / rectangular coordinates.
///
/// Mirrors `boost::geometry::cs::cartesian`
/// (`boost/geometry/core/cs.hpp:85-93`).
///
/// # Examples
///
/// ```
/// use geometry_cs::{Cartesian, CartesianFamily, CoordinateSystem};
/// let _: <Cartesian as CoordinateSystem>::Family = CartesianFamily;
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Cartesian;

impl CoordinateSystem for Cartesian {
    type Family = CartesianFamily;
}

/// Spherical coordinates in unit `U`.
///
/// Mirrors `boost::geometry::cs::spherical<DegreeOrRadian>`
/// (`boost/geometry/core/cs.hpp:115-135`).
///
/// # Convention: latitude measured from the equator
///
/// Boost has *two* spherical tags
/// (`boost/geometry/core/tags.hpp:38-41`):
///
/// * `spherical_polar_tag` — colatitude. The second coordinate is
///   the angle from the **pole**, ranging `[0, π]`.
/// * `spherical_equatorial_tag` — latitude. The second coordinate
///   is the angle from the **equator**, ranging `[-π/2, π/2]`.
///
/// v1 collapses both into one [`Spherical<U>`] family and picks the
/// **equatorial** convention because that is what the OGC standard
/// and the Boost.Geometry quickstart use
/// (`doc/src/examples/quick_start.cpp` Amsterdam → Paris example).
///
/// In coordinate order:
///
/// ```text
/// get::<0>(p)  →  longitude (azimuth, around the polar axis)
/// get::<1>(p)  →  latitude  (measured from the equator)
/// ```
///
/// In a `Spherical<Degree>` point, longitude ranges `-180.0..=180.0`
/// (east of Greenwich is positive) and latitude ranges
/// `-90.0..=90.0` (north of the equator is positive). Amsterdam
/// (≈ 4.90° E, 52.37° N) is `(4.90, 52.37)` — the "north of the
/// equator" component is the *second* coordinate.
///
/// # If you have colatitude data
///
/// Convert to latitude first: `latitude = 90° − colatitude`
/// (degrees) or `π/2 − colatitude` (radians). The dedicated
/// `SphericalPolar` / `SphericalEquatorial` split is deferred — see
/// `specs/FUTURE_ITERATIONS.md` §1.2.
///
/// # Examples
///
/// ```
/// use geometry_cs::{CoordinateSystem, Degree, Spherical, SphericalFamily};
/// fn _spherical<C: CoordinateSystem<Family = SphericalFamily>>() {}
/// _spherical::<Spherical<Degree>>();
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Spherical<U: AngleUnit>(PhantomData<U>);

impl<U: AngleUnit> CoordinateSystem for Spherical<U> {
    type Family = SphericalFamily;
}

/// Geographic (lat / lon) coordinates on a spheroid in unit `U`.
///
/// Mirrors `boost::geometry::cs::geographic<DegreeOrRadian>`
/// (`boost/geometry/core/cs.hpp:98-110`).
///
/// # Convention: latitude measured from the equator
///
/// Same equatorial convention as [`Spherical<U>`] — the difference
/// is the *earth model*: `Geographic` distances run on a spheroid
/// (Andoyer / Vincenty / Thomas), `Spherical` on a perfect sphere
/// (Haversine). In coordinate order:
///
/// ```text
/// get::<0>(p)  →  longitude (east of Greenwich positive)
/// get::<1>(p)  →  latitude  (north of the equator positive)
/// ```
///
/// In a `Geographic<Degree>` point, longitude ranges `-180.0..=180.0`
/// and latitude ranges `-90.0..=90.0`.
///
/// # Examples
///
/// ```
/// use geometry_cs::{CoordinateSystem, Degree, Geographic, GeographicFamily};
/// fn _geo<C: CoordinateSystem<Family = GeographicFamily>>() {}
/// _geo::<Geographic<Degree>>();
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Geographic<U: AngleUnit>(PhantomData<U>);

impl<U: AngleUnit> CoordinateSystem for Geographic<U> {
    type Family = GeographicFamily;
}

/// Polar coordinates in unit `U`.
///
/// Mirrors `boost::geometry::cs::polar<DegreeOrRadian>`
/// (`boost/geometry/core/cs.hpp:155-165`).
///
/// # Examples
///
/// ```
/// use geometry_cs::{CoordinateSystem, Polar, PolarFamily, Radian};
/// fn _polar<C: CoordinateSystem<Family = PolarFamily>>() {}
/// _polar::<Polar<Radian>>();
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Polar<U: AngleUnit>(PhantomData<U>);

impl<U: AngleUnit> CoordinateSystem for Polar<U> {
    type Family = PolarFamily;
}
