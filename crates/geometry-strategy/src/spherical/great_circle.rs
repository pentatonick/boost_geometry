//! Great-circle point-to-segment projection shared by spherical strategies.

#[cfg(not(feature = "std"))]
use geometry_coords::math::Float;
use geometry_cs::AngleUnit;
use geometry_trait::{Point, PointMut};

use crate::normalise::{HasAngularUnits, lonlat_radians};

pub(super) struct Projection<P> {
    pub(super) point: P,
    pub(super) angular_distance: f64,
}

pub(super) fn project<P>(point: &P, start: &P, end: &P) -> Projection<P>
where
    P: Point<Scalar = f64> + PointMut + Default + Copy,
    P::Cs: HasAngularUnits,
{
    let point_vector = vector(point);
    let start_vector = vector(start);
    let end_vector = vector(end);
    let normal = cross(start_vector, end_vector);
    let normal_length = magnitude(normal);
    if normal_length <= f64::EPSILON {
        return nearest_endpoint(point_vector, start, start_vector, end, end_vector);
    }
    let normal = scale(normal, 1.0 / normal_length);
    let projected = subtract(point_vector, scale(normal, dot(point_vector, normal)));
    let projected_length = magnitude(projected);
    if projected_length <= f64::EPSILON {
        return nearest_endpoint(point_vector, start, start_vector, end, end_vector);
    }
    let mut projected = scale(projected, 1.0 / projected_length);
    if dot(point_vector, projected) < 0.0 {
        projected = scale(projected, -1.0);
    }

    let segment_angle = angle(start_vector, end_vector);
    let on_minor_arc =
        angle(start_vector, projected) + angle(projected, end_vector) <= segment_angle + 1e-10;
    if !on_minor_arc {
        return nearest_endpoint(point_vector, start, start_vector, end, end_vector);
    }

    Projection {
        point: point_from_vector::<P>(projected),
        angular_distance: angle(point_vector, projected),
    }
}

fn nearest_endpoint<P>(
    point: [f64; 3],
    start: &P,
    start_vector: [f64; 3],
    end: &P,
    end_vector: [f64; 3],
) -> Projection<P>
where
    P: Point<Scalar = f64> + Copy,
{
    let start_distance = angle(point, start_vector);
    let end_distance = angle(point, end_vector);
    if start_distance <= end_distance {
        Projection {
            point: *start,
            angular_distance: start_distance,
        }
    } else {
        Projection {
            point: *end,
            angular_distance: end_distance,
        }
    }
}

fn vector<P>(point: &P) -> [f64; 3]
where
    P: Point<Scalar = f64>,
    P::Cs: HasAngularUnits,
{
    let (longitude, latitude) = lonlat_radians(point);
    let cos_latitude = latitude.cos();
    [
        cos_latitude * longitude.cos(),
        cos_latitude * longitude.sin(),
        latitude.sin(),
    ]
}

fn point_from_vector<P>(vector: [f64; 3]) -> P
where
    P: Point<Scalar = f64> + PointMut + Default,
    P::Cs: HasAngularUnits,
{
    type Units<P> = <<P as Point>::Cs as HasAngularUnits>::Units;
    let longitude = vector[1].atan2(vector[0]);
    let latitude = vector[2].atan2(vector[0].hypot(vector[1]));
    let mut point = P::default();
    point.set::<0>(Units::<P>::from_radians(longitude));
    point.set::<1>(Units::<P>::from_radians(latitude));
    point
}

fn angle(first: [f64; 3], second: [f64; 3]) -> f64 {
    magnitude(cross(first, second)).atan2(dot(first, second).clamp(-1.0, 1.0))
}

fn dot(first: [f64; 3], second: [f64; 3]) -> f64 {
    first[0] * second[0] + first[1] * second[1] + first[2] * second[2]
}

fn cross(first: [f64; 3], second: [f64; 3]) -> [f64; 3] {
    [
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    ]
}

fn subtract(first: [f64; 3], second: [f64; 3]) -> [f64; 3] {
    [
        first[0] - second[0],
        first[1] - second[1],
        first[2] - second[2],
    ]
}

fn scale(vector: [f64; 3], factor: f64) -> [f64; 3] {
    [vector[0] * factor, vector[1] * factor, vector[2] * factor]
}

fn magnitude(vector: [f64; 3]) -> f64 {
    dot(vector, vector).sqrt()
}
