//! Public-facade parity tests for integral calculation-type promotion.
//!
//! Reference cases come from Boost.Geometry's
//! `test/util/promote_integral.cpp`.

use core::any::TypeId;

use boost_geometry::coords::PromoteIntegral;

fn same_type<A: 'static, B: 'static>() -> bool {
    TypeId::of::<A>() == TypeId::of::<B>()
}

/// `test/util/promote_integral.cpp:396-450` — signed inputs promote to the
/// first signed fixed-width type large enough for roughly twice their bits.
#[test]
fn signed_integrals_promote_to_a_wider_signed_type() {
    assert!(same_type::<<i8 as PromoteIntegral>::Out, i16>());
    assert!(same_type::<<i16 as PromoteIntegral>::Out, i32>());
    assert!(same_type::<<i32 as PromoteIntegral>::Out, i64>());
    assert!(same_type::<<i64 as PromoteIntegral>::Out, i128>());
    assert!(same_type::<<i128 as PromoteIntegral>::Out, i128>());
}

/// `test/util/promote_integral.cpp:396-450` — Boost defaults unsigned inputs
/// to a signed result with one extra sign bit when such a fixed-width type is
/// available.
#[test]
fn unsigned_integrals_promote_to_a_wider_signed_type_by_default() {
    assert!(same_type::<<u8 as PromoteIntegral>::Out, i32>());
    assert!(same_type::<<u16 as PromoteIntegral>::Out, i64>());
    assert!(same_type::<<u32 as PromoteIntegral>::Out, i128>());
    assert!(same_type::<<u64 as PromoteIntegral>::Out, u64>());
    assert!(same_type::<<u128 as PromoteIntegral>::Out, u128>());
}

/// `test/util/promote_integral.cpp:326-390` — callers can keep unsignedness;
/// each input then promotes to the first unsigned type with twice its width.
#[test]
fn unsigned_integrals_can_promote_to_a_wider_unsigned_type() {
    assert!(same_type::<<u8 as PromoteIntegral<true>>::Out, u16>());
    assert!(same_type::<<u16 as PromoteIntegral<true>>::Out, u32>());
    assert!(same_type::<<u32 as PromoteIntegral<true>>::Out, u64>());
    assert!(same_type::<<u64 as PromoteIntegral<true>>::Out, u128>());
    assert!(same_type::<<u128 as PromoteIntegral<true>>::Out, u128>());
}

/// `test/util/promote_integral.cpp:521-533` — non-integral calculation types
/// pass through unchanged regardless of the unsigned-output policy.
#[test]
fn floating_types_are_not_integral_promoted() {
    assert!(same_type::<<f32 as PromoteIntegral>::Out, f32>());
    assert!(same_type::<<f64 as PromoteIntegral>::Out, f64>());
    assert!(same_type::<<f32 as PromoteIntegral<true>>::Out, f32>());
    assert!(same_type::<<f64 as PromoteIntegral<true>>::Out, f64>());
}
