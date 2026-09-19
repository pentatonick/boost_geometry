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

// --- Borrowed arrays: read-only by construction ---------------------
// `adapt_borrowed_array` implements `Point` but deliberately not
// `PointMut`, mirroring the single read-only `traits::access`
// specialisation C++ uses for both `c_array.hpp` and `std_array.hpp`.
// The facade crate exercises this adapter; geometry-adapt's own suite
// did not, so a change here could only have been caught one crate away.

#[test]
fn borrowed_array_reads_through_to_its_storage() {
    let storage = [3.0_f64, 4.0];
    let p = Adapt(&storage);
    assert_eq!(p.get::<0>(), 3.0);
    assert_eq!(p.get::<1>(), 4.0);
    assert_eq!(<Adapt<&[f64; 2]> as Point>::DIM, 2);
}

#[test]
fn borrowed_array_is_a_point_at_other_arities() {
    assert_is_point::<Adapt<&[f64; 2]>>();
    assert_is_point::<Adapt<&[f64; 3]>>();
    let storage = [1.0_f64, 2.0, 3.0];
    let p = Adapt(&storage);
    assert_eq!(p.get::<2>(), 3.0);
    assert_eq!(<Adapt<&[f64; 3]> as Point>::DIM, 3);
}

/// The borrow is shared, so one array can back two adapters at once —
/// the case the read-only split exists to make safe.
#[test]
fn one_array_can_back_two_borrowed_adapters() {
    let storage = [5.0_f64, 6.0];
    let a = Adapt(&storage);
    let b = Adapt(&storage);
    assert_eq!(a.get::<0>(), b.get::<0>());
    assert_eq!(storage[1], b.get::<1>());
}

// --- Integer coordinates --------------------------------------------
// `CoordinateScalar` is implemented for `i32`/`i64` so callers can hand
// the kernel integer-coordinate geometries; the `Promote` lattice
// widens to a float only where the arithmetic demands it. Every
// adapter test above used `f64`/`f32`, so the integer path through the
// adapters was never instantiated.

#[test]
fn integer_scalars_round_trip_through_the_array_adapter() {
    let mut a = Adapt([0_i32, 0]);
    a.set::<0>(-7);
    a.set::<1>(9);
    assert_eq!(a.get::<0>(), -7);
    assert_eq!(a.get::<1>(), 9);
}

#[test]
fn integer_scalars_round_trip_through_the_tuple_adapters() {
    let mut two = Adapt((0_i64, 0));
    two.set::<0>(i64::MIN);
    two.set::<1>(i64::MAX);
    assert_eq!(two.get::<0>(), i64::MIN);
    assert_eq!(two.get::<1>(), i64::MAX);

    let mut three = Adapt((0_i32, 0, 0));
    three.set::<0>(1);
    three.set::<1>(-2);
    three.set::<2>(3);
    assert_eq!(three.get::<0>(), 1);
    assert_eq!(three.get::<1>(), -2);
    assert_eq!(three.get::<2>(), 3);
}

#[test]
fn integer_adapters_carry_their_scalar_and_dim() {
    fn assert_scalar<P: Point<Scalar = T>, T>() {}
    assert_scalar::<Adapt<[i32; 2]>, i32>();
    assert_scalar::<Adapt<(i64, i64)>, i64>();
    assert_eq!(<Adapt<[i32; 2]> as Point>::DIM, 2);
    assert_eq!(<Adapt<(i32, i32, i32)> as Point>::DIM, 3);
    assert_is_point::<Adapt<[i64; 3]>>();
}
