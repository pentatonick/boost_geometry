//! M-KC1 — Read-only Point end-to-end.
//!
//! Three flavours of read-only point flow through every v1 read-only
//! algorithm; the reference values match v1's M1/M3 numerical
//! assertions verbatim:
//!
//! * Hand-impl `RoPoint([f64; 2])` (the Boost `ro_point` shape).
//! * `Adapt<&[f64; 2]>` (KC1.T4) — borrows into a stack-owned array.
//! * Bare `&[f64; 2]` re-borrowed through `Adapt` — the same thing in
//!   one call site, no intermediate binding.
//!
//! Reference values:
//! * `distance((0,0), (3,4)) == 5.0`                    (T26 M1)
//! * `length(LINESTRING(0,0)->(3,4)->(4,3)) == 5 + √2`  (T38 M3)
//! * `area(quickstart polygon) ≈ 3.015`                 (T38 M3) —
//!   a smoke check on owned `model::Polygon`, not a read-only
//!   exercise (the `polygon!` macro builds an owned polygon whose
//!   point already has `PointMut`).
#![allow(
    clippy::float_cmp,
    reason = "Reference distances are exact integer-valued literals (3-4-5)."
)]

use boost_geometry::adapt::Adapt;
use boost_geometry::algorithm::{area, distance, length};
use boost_geometry::cs::Cartesian;
use boost_geometry::model::{Point2D, Polygon};
use boost_geometry::tag::PointTag;
use boost_geometry::trait_::{Geometry, Point};
use geometry_model::polygon;

// -------- Flavour 1: hand-impl read-only point --------

#[derive(Default)]
struct RoPoint([f64; 2]);

impl Geometry for RoPoint {
    type Kind = PointTag;
    type Point = Self;
}
impl Point for RoPoint {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;
    fn get<const D: usize>(&self) -> f64 {
        self.0[D]
    }
}
// Note: NO `impl PointMut for RoPoint`.

#[test]
fn distance_through_hand_impl_ro_point() {
    let a = RoPoint([0.0, 0.0]);
    let b = RoPoint([3.0, 4.0]);
    assert_eq!(distance(&a, &b), 5.0);
}

// -------- Flavour 2: `Adapt<&[f64; 2]>` (KC1.T4) --------

#[test]
fn distance_through_borrowed_array() {
    let a_storage = [0.0_f64, 0.0];
    let b_storage = [3.0_f64, 4.0];
    assert_eq!(distance(&Adapt(&a_storage), &Adapt(&b_storage)), 5.0);
}

// -------- Flavour 3: bare borrow re-adapted inline --------

#[test]
fn distance_through_inline_adapt_borrow() {
    let storage = [[0.0_f64, 0.0], [3.0, 4.0]];
    let d = distance(&Adapt(&storage[0]), &Adapt(&storage[1]));
    assert_eq!(d, 5.0);
}

// -------- Length over a Vec of read-only borrows --------
//
// Requires a Linestring that holds borrows. The container's
// `Geometry` + `Linestring` impls are hand-written (see the note on
// `Borrowed` below) so the existing length algorithm dispatches
// against it.

mod linestring_of_borrows {
    use super::*;
    use boost_geometry::tag::LinestringTag;
    use boost_geometry::trait_::Linestring;

    /// Container of borrows. Sharing the lifetime makes this
    /// `Send + Sync` only when the underlying slice is.
    ///
    /// Hand-written `Geometry` + `Linestring` impls rather than the
    /// `register_linestring!` macro: the macro's `$Ty:ty` slot cannot
    /// introduce the `'a` generic that a borrowed container needs
    /// (it would emit `impl Trait for Borrowed<'a>` with `'a`
    /// undeclared). Extending the macro to a lifetime-generic form is
    /// out of scope for M-KC1 — flagged for a follow-up task.
    pub struct Borrowed<'a>(pub Vec<Adapt<&'a [f64; 2]>>);

    impl<'a> Geometry for Borrowed<'a> {
        type Kind = LinestringTag;
        type Point = Adapt<&'a [f64; 2]>;
    }
    impl<'a> Linestring for Borrowed<'a> {
        fn points(&self) -> impl ExactSizeIterator<Item = &Adapt<&'a [f64; 2]>> + Clone {
            self.0.iter()
        }
    }

    #[test]
    fn length_of_borrowed_three_point_polyline() {
        // Mirrors T38: LINESTRING(0,0 -> 3,4 -> 4,3) length = 5 + sqrt(2).
        let storage = [[0.0_f64, 0.0], [3.0, 4.0], [4.0, 3.0]];
        let ls = Borrowed(storage.iter().map(Adapt).collect());
        let got = length(&ls);
        let expected = 5.0_f64 + 2.0_f64.sqrt();
        assert!((got - expected).abs() < 1e-12);
    }
}

// -------- Smoke check: owned model::Polygon area still works --------

#[test]
fn owned_polygon_area_unchanged_after_kc1() {
    // T38 M3 case — quickstart polygon. We're not exercising RoPoint
    // here; we're checking that the area code path (ShoelacePolygonArea)
    // does not regress now its inputs gained PointMut via KC1.T1.
    // The Boost quickstart polygon (`doc/src/examples/quick_start.cpp`
    // lines 119-121): five points, `Area: 3.015`.
    let p: Polygon<Point2D<f64, Cartesian>> =
        polygon![[(2.0, 1.3), (4.1, 3.0), (5.3, 2.6), (2.9, 0.7), (2.0, 1.3)]];
    assert!((area(&p) - 3.015).abs() < 1e-3);
}
