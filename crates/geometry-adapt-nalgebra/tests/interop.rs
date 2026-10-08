//! M-AD2 validation milestone: drive `nalgebra` points and vectors
//! through the kernel's `distance` algorithm.
#![allow(
    clippy::float_cmp,
    reason = "Reference values (3-4-5, unit axes) are exact in IEEE-754 f64."
)]

use geometry_adapt_nalgebra::{NaPoint2, NaPoint3, NaVector2, NaVector3};
use geometry_algorithm::distance;
use nalgebra::{Point2, Point3, Vector2, Vector3};

#[test]
fn point2_three_four_five() {
    let a = NaPoint2::new(Point2::new(0.0_f64, 0.0));
    let b = NaPoint2::new(Point2::new(3.0_f64, 4.0));
    assert_eq!(distance(&a, &b), 5.0);
}

#[test]
fn point3_unit_axes() {
    let origin = NaPoint3::new(Point3::new(0.0_f64, 0.0, 0.0));
    let x_axis = NaPoint3::new(Point3::new(1.0_f64, 0.0, 0.0));
    let y_axis = NaPoint3::new(Point3::new(0.0_f64, 1.0, 0.0));
    let z_axis = NaPoint3::new(Point3::new(0.0_f64, 0.0, 1.0));

    assert_eq!(distance(&origin, &x_axis), 1.0);
    assert_eq!(distance(&origin, &y_axis), 1.0);
    assert_eq!(distance(&origin, &z_axis), 1.0);
}

#[test]
fn vector2_three_four_five() {
    let a = NaVector2::new(Vector2::new(0.0_f64, 0.0));
    let b = NaVector2::new(Vector2::new(3.0_f64, 4.0));
    assert_eq!(distance(&a, &b), 5.0);
}

#[test]
fn vector3_unit_axes() {
    let origin = NaVector3::new(Vector3::new(0.0_f64, 0.0, 0.0));
    let x_axis = NaVector3::new(Vector3::new(1.0_f64, 0.0, 0.0));
    let y_axis = NaVector3::new(Vector3::new(0.0_f64, 1.0, 0.0));
    let z_axis = NaVector3::new(Vector3::new(0.0_f64, 0.0, 1.0));

    assert_eq!(distance(&origin, &x_axis), 1.0);
    assert_eq!(distance(&origin, &y_axis), 1.0);
    assert_eq!(distance(&origin, &z_axis), 1.0);
}

/// The wrappers are plain values like the points they wrap — copied,
/// compared, defaulted to the origin — so they enter the algorithms that
/// build points, such as `closest_points`.
#[test]
fn wrappers_are_values_algorithms_can_build() {
    use geometry_algorithm::closest_points;
    use geometry_model::Segment;

    let a = NaPoint2::new(Point2::new(1.0_f64, 2.0));
    let copy = a;
    assert_eq!(copy, a);
    assert_eq!(NaPoint2::<f64>::default(), NaPoint2::new(Point2::origin()));
    assert_eq!(
        NaVector3::<f64>::default(),
        NaVector3::new(Vector3::zeros())
    );
    let segment = Segment::new(
        NaPoint2::new(Point2::new(4.0_f64, 0.0)),
        NaPoint2::new(Point2::new(4.0_f64, 6.0)),
    );
    let (from, to) = closest_points(&a, &segment);
    assert_eq!((from, to.0), (a, Point2::new(4.0, 2.0)));
}
