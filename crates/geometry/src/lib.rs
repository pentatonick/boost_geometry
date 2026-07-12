//! # geometry
//!
//! A Rust port of Boost.Geometry, following the same philosophy:
//! dimension-agnostic, coordinate-system-agnostic, bring-your-own-type,
//! strategy-pluggable.
//!
//! Add this crate alone to your `Cargo.toml` to use the library. See
//! [`prelude`] for the recommended star-imports.
//!
//! Layered design — each module is a re-export of one underlying crate:
//!  * [`tag`]       — geometry kind tags + tag-hierarchy marker traits
//!  * [`coords`]    — coordinate scalars, type promotion, `Comparable<T>`
//!  * [`cs`]        — coordinate systems and reference spheroids
//!  * [`trait_`]    — the concepts: `Geometry`, `Point`, `Linestring`, …
//!  * [`model`]     — default concrete types (`Point2D`, `Polygon`, …)
//!  * [`strategy`]  — pluggable strategies per algorithm × CS family
//!  * [`algorithm`] — free functions users call (`distance`, `area`, …)
//!  * [`adapt`]     — `Adapt<T>`, `WithCs<T, Cs>`, register macros
//!
//! The umbrella mirrors `boost/geometry/geometry.hpp`: one entry-point
//! that fans out into every public sub-namespace.
//!
//! ## End-user walkthrough
//!
//! The proposal's §5 end-user surface, fully runnable. Each fenced
//! block below compiles as a doctest, so the examples cannot drift
//! from the actual API surface.
//!
//! ### Cartesian distance — picks Pythagoras by default
//!
//! ```
//! use boost_geometry::prelude::*;
//!
//! let a = Point2D::<f64, Cartesian>::new(0.0, 0.0);
//! let b = Point2D::<f64, Cartesian>::new(3.0, 4.0);
//! assert_eq!(distance(&a, &b), 5.0);
//!
//! // Compare cheaply — no `sqrt`:
//! assert_eq!(comparable_distance(&a, &b), 25.0);
//! ```
//!
//! ### Spherical distance — Haversine on the unit sphere
//!
//! Multiplying the unit-sphere angle by a radius (km, miles, …) gives
//! the surface distance in that unit. This mirrors the
//! `quick_start.cpp` Amsterdam → Paris example from Boost.Geometry.
//!
//! ```
//! use boost_geometry::prelude::*;
//! use boost_geometry::adapt::{Adapt, WithCs};
//! use boost_geometry::strategy::spherical::Haversine;
//!
//! let ams = WithCs::<_, Spherical<Degree>>::new(Adapt([4.90_f64, 52.37]));
//! let par = WithCs::<_, Spherical<Degree>>::new(Adapt([2.35_f64, 48.86]));
//! let angle = distance_with(&ams, &par, Haversine::UNIT);
//! let miles = angle * 3959.0;
//! assert!((miles - 267.02).abs() < 0.05);
//! ```
//!
//! ### Geographic distance — Andoyer (default) and Vincenty (explicit) on WGS84
//!
//! ```
//! use boost_geometry::prelude::*;
//! use boost_geometry::adapt::{Adapt, WithCs};
//! use boost_geometry::strategy::geographic::Vincenty;
//!
//! let ams = WithCs::<_, Geographic<Degree>>::new(Adapt([4.90_f64, 52.37]));
//! let par = WithCs::<_, Geographic<Degree>>::new(Adapt([2.35_f64, 48.86]));
//!
//! // No strategy argument — picks Andoyer for the geographic family.
//! let m_a = distance(&ams, &par);
//! // Same pair through Vincenty — agrees with Andoyer to within metres.
//! let m_v = distance_with(&ams, &par, Vincenty::WGS84);
//! assert!((m_a - m_v).abs() < 50.0);
//! assert!(((m_a / 1_000.0) - 430.0).abs() < 1.0);
//! ```
//!
//! ### `#[derive(Point)]` — your own struct as a Point
//!
//! For the common case ("a struct with `x` and `y`"), the derive macro
//! generates the [`Point`] impl. The macro plays the role of
//! `BOOST_GEOMETRY_REGISTER_POINT_2D` on the C++ side.
//!
//! ```
//! use boost_geometry::Point;            // the derive macro
//! use boost_geometry::prelude::*;       // brings in `Point` (the trait) and `distance`
//!
//! #[derive(Default, Point)]
//! #[geometry(cs = "Cartesian", scalar = "f64")]
//! struct MyXy { x: f64, y: f64 }
//!
//! let d = distance(&MyXy { x: 0.0, y: 0.0 }, &MyXy { x: 3.0, y: 4.0 });
//! assert_eq!(d, 5.0);
//! ```
//!
//! ### `Adapt<T>` — foreign data layouts (arrays, tuples)
//!
//! For types you do not own (`[T; N]`, `(T, T)`, third-party point
//! types), wrap them in [`adapt::Adapt`]. The wrapper defaults the
//! coordinate system to Cartesian — for any other CS layer
//! [`adapt::WithCs`] on top (see the spherical / geographic examples
//! above).
//!
//! ```
//! use boost_geometry::prelude::*;
//! use boost_geometry::adapt::Adapt;
//!
//! let a = Adapt([0.0_f64, 0.0]);
//! let b = Adapt([3.0_f64, 4.0]);
//! assert_eq!(distance(&a, &b), 5.0);
//! ```
//!
//! ### Polygon, area, point-in-polygon
//!
//! The `polygon!`, `linestring!`, and `point!` literal macros are
//! `#[macro_export]`ed by `geometry-model`, so they are reachable as
//! `geometry_model::polygon!` once the `geometry` crate is in your
//! dependency tree.
//!
//! ```
//! use boost_geometry::prelude::*;
//! use boost_geometry::model::Polygon;
//! use geometry_model::polygon;
//!
//! // Ring traversed clockwise (the default `PointOrder` for `Ring`).
//! let p: Polygon<Point2D<f64, Cartesian>> = polygon![
//!     [(0.0, 0.0), (0.0, 3.0), (4.0, 3.0), (4.0, 0.0), (0.0, 0.0)]
//! ];
//! assert_eq!(area(&p), 12.0);
//! assert!(within(&Point2D::<f64, Cartesian>::new(2.0, 1.0), &p));
//! assert!(!within(&Point2D::<f64, Cartesian>::new(5.0, 5.0), &p));
//! ```
//!
//! ## See also
//!
//! * `specs/rust-port-proposal.md` — full design rationale.
//! * `specs/rust-port-implementation-plan.md` — milestone breakdown.
//! * [Boost.Geometry documentation][boost-geometry] — the original
//!   design we are porting.
//!
//! [boost-geometry]: https://www.boost.org/doc/libs/release/libs/geometry/

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

/// Geometry kind tags and the tag-hierarchy marker traits.
///
/// Re-exports every item from [`geometry-tag`](geometry_tag): the
/// eleven `*Tag` types and the marker traits (`Single`, `Multi`,
/// `Pointlike`, `Linear`, `Polylinear`, `Areal`, `Polygonal`,
/// `Volumetric`, `SameAs`).
pub mod tag {
    pub use geometry_tag::*;
}

/// Coordinate scalars, type promotion, and the comparable-distance wrapper.
///
/// Re-exports every item from [`geometry-coords`](geometry_coords):
/// the [`CoordinateScalar`](geometry_coords::CoordinateScalar) trait,
/// the [`Promote`](geometry_coords::Promote) type-widening
/// metafunction, the [`Comparable`](geometry_coords::Comparable)
/// newtype, and the `math` numeric shim.
pub mod coords {
    pub use geometry_coords::*;
}

/// Coordinate systems, angle units, and reference spheroids.
///
/// Re-exports every item from [`geometry-cs`](geometry_cs):
/// `Cartesian`, `Spherical<U>`, `Geographic<U>`, `Polar<U>`, the
/// `Degree` / `Radian` unit tags, the `*Family` classifiers, and the
/// `Spheroid` reference ellipsoid.
pub mod cs {
    pub use geometry_cs::*;
}

/// Geometry concept traits — `Geometry`, `Point`, `Linestring`, …
///
/// Re-exports every item from [`geometry-trait`](geometry_trait).
/// Named `trait_` (with a trailing underscore) because `trait` is a
/// reserved Rust keyword.
pub mod trait_ {
    pub use geometry_trait::*;
}

/// Default concrete geometry types.
///
/// Re-exports every item from [`geometry-model`](geometry_model):
/// `Point<T, D, Cs>`, `Point2D`, `Point3D`, `Segment`, `Box`,
/// `Linestring`, `Ring`, `Polygon`, `MultiPoint`, `MultiLinestring`,
/// `MultiPolygon`. The declarative macros `point!`, `linestring!`,
/// and `polygon!` are `#[macro_export]`ed by `geometry-model` and so
/// appear at the crate root of `geometry` itself, not under
/// `boost_geometry::model`.
pub mod model {
    pub use geometry_model::*;
}

/// Pluggable algorithm strategies per coordinate-system family.
///
/// Re-exports every item from [`geometry-strategy`](geometry_strategy),
/// including the per-family submodules `cartesian`, `spherical`, and
/// `geographic` (so users can write
/// `boost_geometry::strategy::geographic::Vincenty`).
pub mod strategy {
    pub use geometry_strategy::*;
}

/// Free-function algorithm entry points — `distance`, `area`, …
///
/// Re-exports every item from [`geometry-algorithm`](geometry_algorithm).
pub mod algorithm {
    pub use geometry_algorithm::*;
}

/// Adapters for foreign types — `Adapt<T>`, `WithCs<T, Cs>`, and
/// the `register_*!` macros.
///
/// Re-exports every item from [`geometry-adapt`](geometry_adapt).
/// The `register_linestring!`, `register_ring!`, and
/// `register_polygon!` macros are `#[macro_export]`ed and so live at
/// the crate root of `geometry` itself, not under
/// `boost_geometry::adapt`.
pub mod adapt {
    pub use geometry_adapt::*;
}

/// Re-exports every item from [`geometry-overlay`](geometry_overlay) —
/// the boolean overlay engine (`intersection`, `union`, `difference`,
/// `sym_difference`) and its predicate / turn-graph / traversal layers.
pub mod overlay {
    pub use geometry_overlay::*;
}

/// Re-exports every item from [`geometry-rtree`](geometry_rtree) — the
/// R-tree spatial index (`Rtree`, `Predicate`, split strategies, bulk
/// load).
pub mod rtree {
    pub use geometry_rtree::*;
}

/// `#[derive(Point)]`. Re-exported so users only need a single
/// `geometry` dependency in their `Cargo.toml`.
pub use geometry_derive::Point;

pub mod prelude;

/// Private re-exports for procedural macros to point at — never type
/// these paths directly.
///
/// `#[derive(Point)]`'s generated impl block names the trait paths
/// `::geometry_trait::Geometry`, `::geometry_trait::Point`,
/// `::geometry_tag::PointTag`, and CS types from `::geometry_cs::*`.
/// We re-export them here so future revisions of the derive can
/// switch to `::boost_geometry::__private::…` paths without breaking
/// downstream crates that pin only on `boost_geometry`.
#[doc(hidden)]
pub mod __private {
    pub use geometry_cs::{Cartesian, Geographic, Polar, Spherical};
    pub use geometry_tag::PointTag;
    pub use geometry_trait::{Geometry, Point};
}
