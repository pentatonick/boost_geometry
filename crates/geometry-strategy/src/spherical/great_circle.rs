//! Where a point's great-circle perpendicular meets a segment, the walk the
//! spherical cross-track distance and closest-point strategies share.
//!
//! Mirrors the common body of Boost's two spherical `cross_track`
//! strategies (`strategies/spherical/distance_cross_track.hpp:441-513`,
//! `strategies/spherical/closest_points_pt_seg.hpp:86-187`): comparable
//! haversine terms to both endpoints, the course differences of
//! `detail::compute_cross_track_pair`, and the projections whose signs say
//! whether the perpendicular foot falls inside the segment.

#[cfg(not(feature = "std"))]
use geometry_coords::math::Float;
use geometry_trait::Point;

use crate::normalise::{HasAngularUnits, lonlat_radians};

use super::azimuth::spherical_azimuth;
use super::distance_haversine::comparable_haversine_h;

/// Where a point lies nearest a segment, with the comparable distances —
/// the haversine `h = sin²(d/2)` of the angular distance `d` — Boost's
/// comparable strategies return.
pub(super) enum Foot {
    /// The segment's start, at `h`: the segment is a single point, or the
    /// foot falls outside the segment nearer the start.
    Start(f64),
    /// The segment's end, at `h`: the foot falls outside the segment no
    /// nearer the start.
    End(f64),
    /// The foot itself, inside the segment: `h` from the point, which lies
    /// `d1` (comparable) from the segment's start.
    Inside { h: f64, d1: f64 },
}

/// Where `point` lies nearest the segment from `start` to `end`.
pub(super) fn foot<P1, P2>(point: &P1, start: &P2, end: &P2) -> Foot
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    P1::Cs: HasAngularUnits,
    P2::Cs: HasAngularUnits,
{
    let d1 = comparable_haversine_h(start, point);
    let d3 = comparable_haversine_h(start, end);
    if geometry_coords::CoordinateScalar::tolerant_eq(d3, 0.0) {
        return Foot::Start(d1);
    }
    let d2 = comparable_haversine_h(end, point);
    let (d_crs1, d_crs2) = course_differences(point, start, end);
    // Only the signs matter: the foot is inside when the point is ahead of
    // both endpoints along the segment.
    let projection1 = d_crs1.cos() * d1 / d3;
    let projection2 = d_crs2.cos() * d2 / d3;
    if projection1 > 0.0 && projection2 > 0.0 {
        Foot::Inside {
            h: cross_track_h(d_crs1, d1),
            d1,
        }
    } else if d1 < d2 {
        Foot::Start(d1)
    } else {
        Foot::End(d2)
    }
}

/// The course from each endpoint to the point less the segment's course
/// at that endpoint, in radians.
///
/// Mirrors `detail::compute_cross_track_pair`
/// (`strategies/spherical/distance_cross_track.hpp:56-98`).
#[allow(
    clippy::similar_names,
    reason = "the names are those of `compute_cross_track_pair`"
)]
fn course_differences<P1, P2>(point: &P1, start: &P2, end: &P2) -> (f64, f64)
where
    P1: Point<Scalar = f64>,
    P2: Point<Scalar = f64>,
    P1::Cs: HasAngularUnits,
    P2::Cs: HasAngularUnits,
{
    let (lon1, lat1) = lonlat_radians(start);
    let (lon2, lat2) = lonlat_radians(end);
    let (lon, lat) = lonlat_radians(point);
    let crs_ad = spherical_azimuth::<false>(lon1, lat1, lon, lat).0;
    let (crs_ab, reverse) = spherical_azimuth::<true>(lon1, lat1, lon2, lat2);
    let crs_ba = reverse - core::f64::consts::PI;
    let crs_bd = spherical_azimuth::<false>(lon2, lat2, lon, lat).0;
    (crs_ad - crs_ab, crs_bd - crs_ba)
}

/// The comparable cross-track distance of a point `d1` (comparable) from
/// the segment's start, whose course turns `d_crs1` off the segment's.
///
/// Mirrors `detail::compute_cross_track_distance`
/// (`strategies/spherical/distance_cross_track.hpp:100-124`), Boost's
/// rearrangement of `(1 − sqrt(1 − 4·(d1 − d1²)·sin²(d_crs1))) / 2` that
/// keeps the small distances precise.
fn cross_track_h(d_crs1: f64, d1: f64) -> f64 {
    let sin_d_crs1 = d_crs1.sin();
    let d1_x_sin = d1 * sin_d_crs1;
    let d = d1_x_sin * (sin_d_crs1 - d1_x_sin);
    d / (0.5 + (0.25 - d).sqrt())
}
