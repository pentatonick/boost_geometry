//! Witnesses for every cell of the `Promote` lattice.
//!
//! Each test pins one ordered pair `(T, U)` and asserts what
//! `<T as Promote<U>>::Out` is. The assertion is encoded as a binding
//! of that associated type to a concrete value: if the lattice ever
//! shifts, the test stops compiling. This is the Rust analogue of the
//! `static_assert<std::is_same<..>>` checks that exist in
//! `boost/geometry/test/util/select_most_precise.cpp`.

use geometry_coords::{Comparable, Promote};

// ---- float × float -------------------------------------------------

#[test]
fn f32_x_f32_promotes_to_f32() {
    let _: <f32 as Promote<f32>>::Out = 1.0_f32;
}

#[test]
fn f32_x_f64_promotes_to_f64() {
    let _: <f32 as Promote<f64>>::Out = 1.0_f64;
}

#[test]
fn f64_x_f32_promotes_to_f64() {
    let _: <f64 as Promote<f32>>::Out = 1.0_f64;
}

#[test]
fn f64_x_f64_promotes_to_f64() {
    let _: <f64 as Promote<f64>>::Out = 1.0_f64;
}

// ---- int × int -----------------------------------------------------

#[test]
fn i32_x_i32_promotes_to_i32() {
    let _: <i32 as Promote<i32>>::Out = 1_i32;
}

#[test]
fn i32_x_i64_promotes_to_i64() {
    let _: <i32 as Promote<i64>>::Out = 1_i64;
}

#[test]
fn i64_x_i32_promotes_to_i64() {
    let _: <i64 as Promote<i32>>::Out = 1_i64;
}

#[test]
fn i64_x_i64_promotes_to_i64() {
    let _: <i64 as Promote<i64>>::Out = 1_i64;
}

// ---- int × float (always widen to f64) -----------------------------

#[test]
fn i32_x_f32_promotes_to_f64() {
    let _: <i32 as Promote<f32>>::Out = 1.0_f64;
}

#[test]
fn f32_x_i32_promotes_to_f64() {
    let _: <f32 as Promote<i32>>::Out = 1.0_f64;
}

#[test]
fn i32_x_f64_promotes_to_f64() {
    let _: <i32 as Promote<f64>>::Out = 1.0_f64;
}

#[test]
fn f64_x_i32_promotes_to_f64() {
    let _: <f64 as Promote<i32>>::Out = 1.0_f64;
}

#[test]
fn i64_x_f32_promotes_to_f64() {
    let _: <i64 as Promote<f32>>::Out = 1.0_f64;
}

#[test]
fn f32_x_i64_promotes_to_f64() {
    let _: <f32 as Promote<i64>>::Out = 1.0_f64;
}

#[test]
fn i64_x_f64_promotes_to_f64() {
    let _: <i64 as Promote<f64>>::Out = 1.0_f64;
}

#[test]
fn f64_x_i64_promotes_to_f64() {
    let _: <f64 as Promote<i64>>::Out = 1.0_f64;
}

// ---- Comparable<T> behaviour --------------------------------------

#[test]
fn comparable_orders_but_does_not_convert_f64() {
    assert!(Comparable::<f64>(9.0) < Comparable::<f64>(16.0));
    assert_eq!(Comparable::<f64>(9.0), Comparable::<f64>(9.0));
    assert!((Comparable::<f64>(25.0).into_distance() - 5.0).abs() < 1e-12);
}

#[test]
fn comparable_orders_but_does_not_convert_f32() {
    assert!(Comparable::<f32>(9.0) < Comparable::<f32>(16.0));
    let d = Comparable::<f32>(25.0).into_distance();
    assert!((d - 5.0).abs() < 1e-6);
}
