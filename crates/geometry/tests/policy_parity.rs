//! Public-facade parity tests for Boost.Geometry policy headers.

use core::cmp::Ordering;

use boost_geometry::coords::Rational;
use boost_geometry::model::{Point as ModelPoint, Point2D, Polygon, Ring};
use boost_geometry::prelude::{
    Cartesian, Degree, Geographic, Radian, Spherical, ValidityFailure, ValidityOptions, is_valid,
    is_valid_with, validity_reason, validity_reason_with,
};
use boost_geometry::strategy::compare::{EqualTo, Greater, Less, LessExact};
use boost_geometry::trait_::PointMut as _;

type CartesianPoint = Point2D<f64, Cartesian>;
const LESS: Less = Less;
const LESS_EXACT: LessExact = LessExact;
const GREATER: Greater = Greater;
const EQUAL_TO: EqualTo = EqualTo;

/// `test/policies/compare.cpp:48-132` — the default policy compares all
/// coordinates lexicographically; dimension policies inspect one ordinate.
#[test]
fn cartesian_compare_matches_the_reference_matrix() {
    let p1 = CartesianPoint::new(3.0, 1.0);
    let p2 = CartesianPoint::new(3.0, 1.0);
    let p3 = CartesianPoint::new(1.0, 3.0);
    let p4 = CartesianPoint::new(5.0, 2.0);
    let p5 = CartesianPoint::new(3.0, 2.0);

    assert!(EQUAL_TO.apply(&p1, &p2));
    assert!(!EQUAL_TO.apply(&p1, &p3));
    assert!(LESS.apply(&p1, &p4));
    assert!(LESS.apply(&p1, &p5));
    assert!(LESS.apply(&p3, &p4));
    assert!(GREATER.apply(&p1, &p3));

    assert!(EqualTo::<0>.apply(&p1, &p5));
    assert!(!Less::<0>.apply(&p1, &p5));
    assert!(Greater::<0>.apply(&p1, &p3));

    assert!(!EqualTo::<1>.apply(&p1, &p5));
    assert!(Less::<1>.apply(&p1, &p3));
    assert!(Less::<1>.apply(&p1, &p5));
    assert!(Greater::<1>.apply(&p3, &p4));
}

/// `test/policies/compare.cpp:135-201` — the policies are suitable for
/// ascending, descending, and single-dimension sorting.
#[test]
fn cartesian_compare_sorts_like_the_reference_policy() {
    let mut points = [
        CartesianPoint::new(3.0, 1.0),
        CartesianPoint::new(2.0, 3.0),
        CartesianPoint::new(2.0, 2.0),
        CartesianPoint::new(1.0, 3.0),
    ];

    points.sort_by(|left, right| {
        if LESS.apply(left, right) {
            Ordering::Less
        } else if GREATER.apply(left, right) {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    });
    assert_eq!(
        points,
        [
            CartesianPoint::new(1.0, 3.0),
            CartesianPoint::new(2.0, 2.0),
            CartesianPoint::new(2.0, 3.0),
            CartesianPoint::new(3.0, 1.0),
        ]
    );

    points.sort_by(|left, right| {
        if GREATER.apply(left, right) {
            Ordering::Less
        } else if LESS.apply(left, right) {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    });
    assert_eq!(points[0], CartesianPoint::new(3.0, 1.0));

    points.sort_by(|left, right| {
        if Less::<1>.apply(left, right) {
            Ordering::Less
        } else if Greater::<1>.apply(left, right) {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    });
    assert_eq!(points[0], CartesianPoint::new(3.0, 1.0));
}

/// `policies/compare.hpp:35-73` — ordinary comparison treats values within
/// Boost's epsilon as equal while `less_exact` does not.
#[test]
fn exact_and_epsilon_less_policies_are_distinct() {
    let left = CartesianPoint::new(1.0, 0.0);
    let right = CartesianPoint::new(1.0 + f64::EPSILON, 0.0);

    assert!(EQUAL_TO.apply(&left, &right));
    assert!(!LESS.apply(&left, &right));
    assert!(LESS_EXACT.apply(&left, &right));
}

/// `test/policies/compare.cpp:241-250` — integer and floating coordinate
/// models use the same public policy, including mixed-scalar comparisons.
#[test]
fn cartesian_compare_accepts_integer_and_mixed_scalars() {
    let integer = Point2D::<i32, Cartesian>::new(3, 1);
    let wider_integer = Point2D::<i64, Cartesian>::new(3, 2);
    let floating = CartesianPoint::new(4.0, 0.0);

    assert!(LESS.apply(&integer, &wider_integer));
    assert!(LESS.apply(&integer, &floating));
    assert!(GREATER.apply(&floating, &wider_integer));
}

/// `test/policies/compare.cpp:204-238` and
/// `strategies/spherical/compare.hpp:96-164` — the antimeridian sorts after
/// ordinary longitudes, its two spellings compare equal on longitude, and
/// degree/radian inputs compare in a shared unit.
#[test]
fn spherical_and_geographic_compare_handle_angular_coordinates() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let mut points = [
        SphericalPoint::new(180.0, 70.56),
        SphericalPoint::new(179.73, 71.56),
        SphericalPoint::new(177.47, 71.23),
        SphericalPoint::new(-178.78, 72.78),
        SphericalPoint::new(-180.0, 73.12),
    ];
    points.sort_by(|left, right| {
        if LESS.apply(left, right) {
            Ordering::Less
        } else if GREATER.apply(left, right) {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    });
    assert_eq!((points[0].x(), points[0].y()), (-178.78, 72.78));
    assert_eq!((points[3].x(), points[3].y()), (180.0, 70.56));
    assert_eq!((points[4].x(), points[4].y()), (-180.0, 73.12));
    assert!(EqualTo::<0>.apply(
        &SphericalPoint::new(180.0, 0.0),
        &SphericalPoint::new(-180.0, 10.0),
    ));

    let degrees = Point2D::<f64, Geographic<Degree>>::new(180.0, 45.0);
    let radians = Point2D::<f64, Geographic<Radian>>::new(
        core::f64::consts::PI,
        core::f64::consts::FRAC_PI_4,
    );
    assert!(EQUAL_TO.apply(&degrees, &radians));
}

/// The public angular policies cover explicit latitude and higher-dimension
/// selection, pole equivalence, both antimeridian orderings, and every scalar
/// conversion supported by angular coordinate systems.
#[test]
fn angular_compare_covers_dimensions_and_scalar_conversions() {
    type SphericalPoint = Point2D<f64, Spherical<Degree>>;
    let ordinary = SphericalPoint::new(20.0, 10.0);
    let antimeridian = SphericalPoint::new(180.0, 10.0);
    assert!(LESS.apply(&ordinary, &antimeridian));
    assert!(GREATER.apply(&antimeridian, &ordinary));
    assert!(LESS_EXACT.apply(&ordinary, &antimeridian));

    assert!(EqualTo::<0>.apply(
        &SphericalPoint::new(-30.0, 90.0),
        &SphericalPoint::new(70.0, 90.0),
    ));
    assert!(EqualTo::<0>.apply(
        &SphericalPoint::new(10.0, 0.0),
        &SphericalPoint::new(10.0, 20.0),
    ));
    assert!(Less::<1>.apply(
        &SphericalPoint::new(0.0, -10.0),
        &SphericalPoint::new(0.0, 20.0),
    ));
    assert!(Greater::<1>.apply(
        &SphericalPoint::new(0.0, 20.0),
        &SphericalPoint::new(0.0, -10.0),
    ));
    assert!(EqualTo::<1>.apply(
        &SphericalPoint::new(0.0, 20.0),
        &SphericalPoint::new(10.0, 20.0),
    ));

    type SphericalPoint4 = ModelPoint<f64, 4, Spherical<Degree>>;
    let point4 = |longitude, latitude, z, m| {
        let mut point = SphericalPoint4::default();
        point.set::<0>(longitude);
        point.set::<1>(latitude);
        point.set::<2>(z);
        point.set::<3>(m);
        point
    };
    let lower = point4(10.0, 20.0, 30.0, 40.0);
    let higher_z = point4(10.0, 20.0, 31.0, 40.0);
    let higher_m = point4(10.0, 20.0, 30.0, 41.0);
    assert!(LESS.apply(&lower, &higher_z));
    assert!(EQUAL_TO.apply(&lower, &lower));
    assert!(Less::<2>.apply(&lower, &higher_z));
    assert!(Less::<3>.apply(&lower, &higher_m));
    assert!(EqualTo::<3>.apply(&lower, &higher_z));

    let integer = Point2D::<i32, Geographic<Degree>>::new(0, 0);
    let floating = Point2D::<f64, Geographic<Degree>>::new(1.0, 0.0);
    assert!(LESS.apply(&integer, &floating));
    assert!(GREATER.apply(&floating, &integer));

    let single = Point2D::<f32, Spherical<Degree>>::new(1.0, 2.0);
    let double = Point2D::<f64, Spherical<Degree>>::new(2.0, 2.0);
    assert!(LESS.apply(&single, &double));

    let rational = Point2D::<Rational<i64>, Spherical<Degree>>::new(
        Rational::from_integer(1),
        Rational::from_integer(2),
    );
    let rational_higher = Point2D::<Rational<i64>, Spherical<Degree>>::new(
        Rational::from_integer(2),
        Rational::from_integer(2),
    );
    assert!(LESS.apply(&rational, &rational_higher));
}

/// `test/policies/compare.cpp:241-250` exercises scalar-independent policy
/// dispatch. The Rust public policy additionally accepts every pair in its
/// integer, floating, and exact-rational comparison lattice.
#[test]
fn cartesian_compare_covers_the_public_scalar_lattice() {
    macro_rules! assert_less {
        ($left:expr, $right:expr) => {{
            let left = Point2D::<_, Cartesian>::new($left, $left);
            let right = Point2D::<_, Cartesian>::new($right, $right);
            assert!(LESS.apply(&left, &right));
        }};
    }

    assert_less!(1_i32, 2_i32);
    assert_less!(1_i64, 2_i64);
    assert_less!(1_i64, 2_i32);
    assert_less!(1_f32, 2_f64);
    assert_less!(1_f64, 2_f32);
    assert_less!(1_i32, 2_f32);
    assert_less!(1_f32, 2_i32);
    assert_less!(1_i32, 2_f64);
    assert_less!(1_f64, 2_i32);
    assert_less!(1_i64, 2_f32);
    assert_less!(1_f32, 2_i64);
    assert_less!(1_i64, 2_f64);
    assert_less!(1_f64, 2_i64);

    let q32_one = Rational::<i32>::from_integer(1);
    let q32_two = Rational::<i32>::from_integer(2);
    let q64_one = Rational::<i64>::from_integer(1);
    let q64_two = Rational::<i64>::from_integer(2);
    assert_less!(q32_one, q64_two);
    assert_less!(q64_one, q32_two);
    assert_less!(q32_one, 2_i32);
    assert_less!(1_i32, q32_two);
    assert_less!(q64_one, 2_i64);
    assert_less!(1_i64, q64_two);
    assert_less!(q32_one, 2_f32);
    assert_less!(1_f32, q32_two);
    assert_less!(q64_one, 2_f64);
    assert_less!(1_f64, q64_two);
}

fn duplicate_polygon() -> Polygon<CartesianPoint> {
    Polygon::new(Ring::from_vec(vec![
        CartesianPoint::new(0.0, 0.0),
        CartesianPoint::new(0.0, 2.0),
        CartesianPoint::new(0.0, 2.0),
        CartesianPoint::new(2.0, 2.0),
        CartesianPoint::new(2.0, 0.0),
        CartesianPoint::new(0.0, 0.0),
    ]))
}

/// `test/algorithms/is_valid_failure.cpp` and
/// `policies/is_valid/failing_reason_policy.hpp:32-63` — every public result
/// has the stable base reason used by Boost's reason policy.
#[test]
fn validity_failures_expose_reference_reason_messages() {
    let reference_reasons = [
        (ValidityFailure::FewPoints, "Geometry has too few points"),
        (
            ValidityFailure::WrongTopologicalDimension,
            "Geometry has wrong topological dimension",
        ),
        (ValidityFailure::Spikes, "Geometry has spikes"),
        (
            ValidityFailure::DuplicatePoints,
            "Geometry has duplicate (consecutive) points",
        ),
        (
            ValidityFailure::NotClosed,
            "Geometry is defined as closed but is open",
        ),
        (
            ValidityFailure::SelfIntersection,
            "Geometry has invalid self-intersections",
        ),
        (
            ValidityFailure::WrongOrientation,
            "Geometry has wrong orientation",
        ),
        (
            ValidityFailure::InteriorRingOutside,
            "Geometry has interior rings defined outside the outer boundary",
        ),
        (
            ValidityFailure::NestedInteriorRings,
            "Geometry has nested interior rings",
        ),
        (
            ValidityFailure::DisconnectedInterior,
            "Geometry has disconnected interior",
        ),
        (
            ValidityFailure::IntersectingInteriors,
            "Multi-polygon has intersecting interiors",
        ),
        (
            ValidityFailure::WrongCornerOrder,
            "Box has corners in wrong order",
        ),
        (
            ValidityFailure::InvalidCoordinate,
            "Geometry has point(s) with invalid coordinate(s)",
        ),
    ];
    for (failure, reason) in reference_reasons {
        assert_eq!(failure.message(), reason);
        assert_eq!(failure.to_string(), reason);
    }

    let valid: Polygon<CartesianPoint> = Polygon::new(Ring::from_vec(vec![
        CartesianPoint::new(0.0, 0.0),
        CartesianPoint::new(0.0, 2.0),
        CartesianPoint::new(2.0, 2.0),
        CartesianPoint::new(2.0, 0.0),
        CartesianPoint::new(0.0, 0.0),
    ]));
    assert_eq!(validity_reason(&valid), "Geometry is valid");
    assert_eq!(
        validity_reason(&duplicate_polygon()),
        "Geometry has duplicate (consecutive) points"
    );
}

/// `policies/is_valid/default_policy.hpp:26-61` — Boost's default allows
/// consecutive duplicates. The existing strict Rust default stays intact,
/// and the Boost behavior is selected explicitly through the public facade.
#[test]
fn validity_options_preserve_strict_behavior_and_offer_boost_defaults() {
    let duplicate = duplicate_polygon();
    assert_eq!(is_valid(&duplicate), Err(ValidityFailure::DuplicatePoints));
    assert!(is_valid_with(&duplicate, ValidityOptions::BOOST_DEFAULT).is_ok());
    assert_eq!(
        validity_reason_with(&duplicate, ValidityOptions::BOOST_DEFAULT),
        "Geometry is valid"
    );
}
