//! Angular normalisation at strategy entry.
//!
//! Internal helper used by every spherical and geographic strategy to
//! produce a `(lon_rad, lat_rad)` pair from a 2-D point regardless of
//! whether the point's coordinate system carries [`Degree`] or
//! [`Radian`] units.
//!
//! Mirrors the implicit normalisation Boost.Geometry performs at the
//! entry of each spherical / geographic strategy — see
//! `boost/geometry/core/cs.hpp:251-281` (`traits::cs_angular_units<Cs>`)
//! and the per-strategy entry pattern in
//! `boost/geometry/strategies/spherical/*.hpp` (any one strategy — the
//! pattern is uniform: multiply by `math::d2r<T>()` when the units
//! tag is `degree`, otherwise pass through).
//!
//! [`Degree`]: geometry_cs::Degree
//! [`Radian`]: geometry_cs::Radian

use geometry_coords::CoordinateScalar;
use geometry_cs::{AngleUnit, FromF64, Geographic, Polar, Spherical};
use geometry_trait::Point;

/// Extracts the `Units` associated type from a coordinate system that
/// carries one.
///
/// Mirrors `boost::geometry::traits::cs_angular_units<Cs>`
/// (`boost/geometry/core/cs.hpp:251-281`). Implemented only for the
/// angle-bearing systems ([`Spherical`], [`Geographic`], [`Polar`]) —
/// `Cartesian` has no angular units and therefore no impl, so a
/// strategy that asks for `HasAngularUnits` on a Cartesian point
/// fails to compile rather than silently treating values as radians.
pub(crate) trait HasAngularUnits {
    /// The angular unit ([`geometry_cs::Degree`] or
    /// [`geometry_cs::Radian`]) the coordinate system carries.
    type Units: AngleUnit;
}

impl<U: AngleUnit> HasAngularUnits for Spherical<U> {
    type Units = U;
}

impl<U: AngleUnit> HasAngularUnits for Geographic<U> {
    type Units = U;
}

impl<U: AngleUnit> HasAngularUnits for Polar<U> {
    type Units = U;
}

/// Convert a 2-D point's first two coordinates to `(lon_rad, lat_rad)`.
///
/// Reads `P::Cs::Units` to decide whether the stored values are in
/// degrees or radians, and converts each component via
/// [`AngleUnit::to_radians`]. The result is always in radians so the
/// rest of a spherical / geographic strategy can do the maths in one
/// unit system.
///
/// Mirrors the implicit `math::d2r<T>()` multiplication that opens
/// every Boost.Geometry spherical / geographic strategy — see
/// `boost/geometry/core/cs.hpp:251-281` and the per-strategy entries
/// in `boost/geometry/strategies/spherical/*.hpp`.
pub(crate) fn lonlat_radians<P>(p: &P) -> (P::Scalar, P::Scalar)
where
    P: Point,
    P::Scalar: CoordinateScalar + FromF64,
    P::Cs: HasAngularUnits,
{
    let lon = p.get::<0>();
    let lat = p.get::<1>();
    (
        <P::Cs as HasAngularUnits>::Units::to_radians(lon),
        <P::Cs as HasAngularUnits>::Units::to_radians(lat),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use geometry_adapt::{Adapt, WithCs};
    use geometry_cs::{Degree, Radian, Spherical};

    #[test]
    fn degree_input_matches_radian_input() {
        let in_deg: WithCs<Adapt<[f64; 2]>, Spherical<Degree>> =
            WithCs::new(Adapt([10.0_f64, 20.0]));
        let in_rad: WithCs<Adapt<[f64; 2]>, Spherical<Radian>> =
            WithCs::new(Adapt([10.0_f64.to_radians(), 20.0_f64.to_radians()]));

        let (lon_d, lat_d) = lonlat_radians(&in_deg);
        let (lon_r, lat_r) = lonlat_radians(&in_rad);

        assert!((lon_d - lon_r).abs() < 1e-12);
        assert!((lat_d - lat_r).abs() < 1e-12);
    }
}
