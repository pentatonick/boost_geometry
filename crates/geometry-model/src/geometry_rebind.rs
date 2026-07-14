//! Type-level stock-model rebinding for scalar and coordinate-system types.
//!
//! Mirrors `boost::geometry::helper_geometry` from
//! `geometries/helper_geometry.hpp:38-121`.

use geometry_coords::CoordinateScalar;
use geometry_cs::CoordinateSystem;

use crate::{Box, Linestring, Point, Ring};

/// Select the stock model matching `Self` while replacing coordinate
/// scalar and coordinate-system types.
///
/// Mirrors the specializations of `helper_geometry` for point, box,
/// linestring, and ring in `geometries/helper_geometry.hpp:57-103`.
/// Boost accepts any adapted geometry through tag dispatch; this Rust port is
/// deliberately defined for the stock model types, which are the only types
/// it can reconstruct without a user-provided output constructor.
pub trait RebindGeometry<T: CoordinateScalar, Cs: CoordinateSystem> {
    /// The corresponding rebound stock model.
    type Output;
}

/// Rebound model for `G` with scalar `T` and coordinate system `Cs`.
///
/// Mirrors `helper_geometry<Geometry, NewCoordinateType, NewUnits>::type` at
/// `geometries/helper_geometry.hpp:108-120`. Rust takes the complete output
/// coordinate-system type instead of a C++ angular-units tag.
pub type Rebound<G, T, Cs> = <G as RebindGeometry<T, Cs>>::Output;

impl<A, const D: usize, OldCs, T, Cs> RebindGeometry<T, Cs> for Point<A, D, OldCs>
where
    A: CoordinateScalar,
    OldCs: CoordinateSystem,
    T: CoordinateScalar,
    Cs: CoordinateSystem,
{
    type Output = Point<T, D, Cs>;
}

impl<A, const D: usize, OldCs, T, Cs> RebindGeometry<T, Cs> for Box<Point<A, D, OldCs>>
where
    A: CoordinateScalar,
    OldCs: CoordinateSystem,
    T: CoordinateScalar,
    Cs: CoordinateSystem,
{
    type Output = Box<Point<T, D, Cs>>;
}

impl<A, const D: usize, OldCs, T, Cs> RebindGeometry<T, Cs> for Linestring<Point<A, D, OldCs>>
where
    A: CoordinateScalar,
    OldCs: CoordinateSystem,
    T: CoordinateScalar,
    Cs: CoordinateSystem,
{
    type Output = Linestring<Point<T, D, Cs>>;
}

impl<A, const D: usize, OldCs, T, Cs, const CW: bool, const CL: bool> RebindGeometry<T, Cs>
    for Ring<Point<A, D, OldCs>, CW, CL>
where
    A: CoordinateScalar,
    OldCs: CoordinateSystem,
    T: CoordinateScalar,
    Cs: CoordinateSystem,
{
    type Output = Ring<Point<T, D, Cs>, CW, CL>;
}
