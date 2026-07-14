//! Tests for `Adapt<[T; N]>` and `Adapt<(T, T[, T])>`.
//!
//! Mirrors `test/core/{tag,access,coordinate_dimension}.cpp` applied
//! to adapted types, plus a Pythagoras round-trip through
//! `geometry_algorithm::distance` that matches
//! `test/strategies/pythagoras.cpp` (the 3-4-5 case).

#![allow(
    clippy::float_cmp,
    reason = "Every assertion here compares integer-valued IEEE-754 doubles \
              (3-4-5 distances, or values written then read back without \
              any arithmetic in between), so strict equality is exact."
)]

use geometry_adapt::Adapt;
use geometry_algorithm::distance;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut, check_point};

// --- 3-4-5 round-trip via `distance` -------------------------------
// test/strategies/pythagoras.cpp

#[test]
fn array_adapt_3_4_5() {
    let a = Adapt([0.0_f64, 0.0]);
    let b = Adapt([3.0_f64, 4.0]);
    assert_eq!(distance(&a, &b), 5.0);
}

#[test]
fn tuple_adapt_3_4_5() {
    let a = Adapt((0.0_f64, 0.0));
    let b = Adapt((3.0_f64, 4.0));
    assert_eq!(distance(&a, &b), 5.0);
}

#[test]
fn array_adapt_3_4_5_3d_zero_z() {
    let a = Adapt([0.0_f64, 0.0, 0.0]);
    let b = Adapt([3.0_f64, 4.0, 0.0]);
    assert_eq!(distance(&a, &b), 5.0);
}

#[test]
fn tuple_adapt_3_4_5_3d_zero_z() {
    let a = Adapt((0.0_f64, 0.0, 0.0));
    let b = Adapt((3.0_f64, 4.0, 0.0));
    assert_eq!(distance(&a, &b), 5.0);
}

// --- Tag witnesses --------------------------------------------------
// test/core/tag.cpp

fn assert_is_point<T: Geometry<Kind = PointTag>>() {}

#[test]
fn adapt_array_2d_is_point() {
    assert_is_point::<Adapt<[f64; 2]>>();
}
#[test]
fn adapt_array_3d_is_point() {
    assert_is_point::<Adapt<[f64; 3]>>();
}
#[test]
fn adapt_tuple_2d_is_point() {
    assert_is_point::<Adapt<(f64, f64)>>();
}
#[test]
fn adapt_tuple_3d_is_point() {
    assert_is_point::<Adapt<(f64, f64, f64)>>();
}

// --- Dimension witnesses --------------------------------------------
// test/core/coordinate_dimension.cpp

#[test]
fn dim_matches_array_length() {
    assert_eq!(<Adapt<[f64; 2]> as Point>::DIM, 2);
    assert_eq!(<Adapt<[f64; 3]> as Point>::DIM, 3);
    assert_eq!(<Adapt<[f64; 7]> as Point>::DIM, 7);
}

#[test]
fn dim_matches_tuple_arity() {
    assert_eq!(<Adapt<(f64, f64)> as Point>::DIM, 2);
    assert_eq!(<Adapt<(f64, f64, f64)> as Point>::DIM, 3);
}

// --- Coordinate-type witnesses (per element) ------------------------
// test/core/coordinate_type.cpp — the assoc-type spelling.

#[test]
fn scalar_witnesses() {
    fn assert_scalar<P: Point<Scalar = T>, T>() {}
    assert_scalar::<Adapt<[f64; 2]>, f64>();
    assert_scalar::<Adapt<[f32; 3]>, f32>();
    assert_scalar::<Adapt<(f64, f64)>, f64>();
    assert_scalar::<Adapt<(f32, f32, f32)>, f32>();
}

// --- Access round-trips ---------------------------------------------
// test/core/access.cpp

#[test]
fn adapt_array_round_trip() {
    let mut a = Adapt([0.0_f64, 0.0]);
    a.set::<0>(1.0);
    a.set::<1>(2.0);
    assert_eq!(a.get::<0>(), 1.0);
    assert_eq!(a.get::<1>(), 2.0);
}

#[test]
fn adapt_tuple_2d_round_trip() {
    let mut a = Adapt((0.0_f64, 0.0));
    a.set::<0>(1.0);
    a.set::<1>(2.0);
    assert_eq!(a.get::<0>(), 1.0);
    assert_eq!(a.get::<1>(), 2.0);
}

#[test]
fn adapt_tuple_3d_round_trip() {
    let mut a = Adapt((0.0_f64, 0.0, 0.0));
    a.set::<0>(1.0);
    a.set::<1>(2.0);
    a.set::<2>(3.0);
    assert_eq!(a.get::<0>(), 1.0);
    assert_eq!(a.get::<1>(), 2.0);
    assert_eq!(a.get::<2>(), 3.0);
}

#[test]
fn tuple_adapters_reject_out_of_range_public_access() {
    let mut two = Adapt((0.0_f64, 0.0));
    assert!(std::panic::catch_unwind(|| two.get::<2>()).is_err());
    assert!(
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            two.set::<2>(1.0);
        }))
        .is_err()
    );

    let mut three = Adapt((0.0_f64, 0.0, 0.0));
    assert!(std::panic::catch_unwind(|| three.get::<3>()).is_err());
    assert!(
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            three.set::<3>(1.0);
        }))
        .is_err()
    );
}

// --- Concept-check helpers from T15 ---------------------------------

#[test]
fn concept_checks_pass() {
    check_point::<Adapt<[f64; 2]>>();
    check_point::<Adapt<[f64; 3]>>();
    check_point::<Adapt<(f64, f64)>>();
    check_point::<Adapt<(f64, f64, f64)>>();
}

/// `Adapt::new` wraps a value and `into_inner` recovers it unchanged —
/// the named constructor/destructor equivalent of the `Adapt(...)` /
/// `.0` tuple syntax used everywhere else.
#[test]
fn adapt_new_and_into_inner_round_trip() {
    let a = Adapt::new([3.0_f64, 4.0]);
    // The wrapper exposes the geometry concept over the constructed value.
    assert_eq!(a.get::<0>(), 3.0);
    assert_eq!(a.get::<1>(), 4.0);
    // And `into_inner` returns the exact array it was built from.
    assert_eq!(a.into_inner(), [3.0, 4.0]);
}
