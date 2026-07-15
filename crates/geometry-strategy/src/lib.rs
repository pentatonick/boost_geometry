//! # Pluggable algorithm strategies, keyed by coordinate-system family.
//!
//! Mirrors `boost/geometry/strategies/` — every algorithm has a
//! strategy trait here; concrete strategies live in submodules keyed
//! by coordinate-system family (`cartesian`, `spherical`,
//! `geographic`).
//!
//! ## Writing a new strategy
//!
//! Take the worked example: a Cartesian point-to-point distance
//! strategy (`Pythagoras`). A strategy for any algorithm follows the
//! same three steps.
//!
//! ### Step 1 — Pick the coordinate-system family to bind on
//!
//! Strategies bind on the [`CoordinateSystem::Family`](geometry_cs::CoordinateSystem::Family)
//! — never on the concrete CS — so that one impl covers both
//! `Spherical<Degree>` and `Spherical<Radian>` (or
//! `Geographic<Degree>` / `Geographic<Radian>`). The bound is
//! expressed via the [`geometry_tag::SameAs`] trait, the Rust
//! analogue of C++'s `std::is_same`:
//!
//! ```ignore
//! impl<P1, P2> DistanceStrategy<P1, P2> for Pythagoras
//! where
//!     P1: Point,
//!     P2: Point<Scalar = P1::Scalar>,
//!     <P1::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
//!     <P2::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
//! { /* ... */ }
//! ```
//!
//! The two `SameAs<CartesianFamily>` bounds form the family fence:
//! they refuse to monomorphise for a `Spherical` or `Geographic` point.
//! The `#[diagnostic::on_unimplemented]` plate on
//! [`geometry_tag::SameAs`] then redirects the resulting
//! compile error to the correct mitigation (wrap in
//! `geometry_adapt::WithCs<_, Geographic<_>>`, or pick a
//! CS-appropriate strategy such as `Haversine` / `Andoyer` /
//! `Vincenty`).
//!
//! ### Step 2 — Decide whether to provide a `Comparable` form
//!
//! A "comparable" form is a sibling strategy that returns the *same
//! ordering* as the real strategy but skips work the ordering does
//! not need. For Pythagoras this means returning the *squared*
//! distance and skipping the final `sqrt`. The squared form sorts
//! identically and is roughly twice as fast on a hot inner loop:
//!
//! ```ignore
//! impl<P1, P2> DistanceStrategy<P1, P2> for Pythagoras /* ... */ {
//!     type Out = P1::Scalar;
//!     type Comparable = ComparablePythagoras; // <- skip-sqrt sibling
//!     fn distance(&self, a: &P1, b: &P2) -> Self::Out {
//!         self.comparable().distance(a, b).sqrt()
//!     }
//!     fn comparable(&self) -> Self::Comparable { ComparablePythagoras }
//! }
//! ```
//!
//! If the math has no equivalent shortcut (Andoyer, Vincenty,
//! Haversine after the half-angle formula), set
//! `type Comparable = Self;` — the optimiser collapses the
//! indirection. The doc on [`distance::DistanceStrategy::Comparable`]
//! warns implementers not to over-engineer this.
//!
//! ### Step 3 — Wire the default selection
//!
//! Each coordinate-system family picks one default strategy per
//! algorithm via [`distance::DefaultDistance`]:
//!
//! ```ignore
//! impl DefaultDistance<CartesianFamily>  for CartesianFamily  { type Strategy = Pythagoras; }
//! impl DefaultDistance<SphericalFamily>  for SphericalFamily  { type Strategy = Haversine;  }
//! impl DefaultDistance<GeographicFamily> for GeographicFamily { type Strategy = Andoyer;    }
//! ```
//!
//! That is what makes the no-strategy `distance(a, b)` overload
//! resolve to the right algorithm: the type-level walk
//! `A → A::Point → Cs → Family → DefaultDistance<…B's family…>::Strategy`
//! resolves to the correct concrete strategy at the call site.
//!
//! ## Reverse dispatch — argument symmetry
//!
//! For algorithms whose arguments are symmetric, write one impl per
//! tag pair `(A, B)` and the `Reversed<S>` blanket impl in
//! [`distance::Reversed`] picks up `(B, A)` automatically. The
//! analogue of Boost's `core/reverse_dispatch.hpp` partial
//! specialisation, done once at the strategy-trait layer instead of
//! per-algorithm.
//!
//! ## Module layout
//!
//! Each algorithm has its own strategy trait module — `distance`,
//! `area`, `length`, `envelope`, `within`, `intersects`, `disjoint`,
//! `equals`. Concrete strategies live under `cartesian`,
//! `spherical`, or `geographic` per coordinate-system family.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

pub mod area;
pub mod azimuth;
pub mod buffer;
pub mod cartesian;
pub mod centroid;
pub mod closest_points;
pub mod compare;
pub mod convex_hull;
pub mod densify;
pub mod destination;
pub mod disjoint;
pub mod distance;
pub mod envelope;
pub mod equals;
pub mod geographic;
pub mod intersects;
pub mod length;
pub mod line_interpolate;
pub(crate) mod normalise;
mod reversal;
pub mod segmentize;
pub mod simplify;
pub mod spherical;
pub mod transform;
pub mod within;

pub use area::{
    AreaStrategy, DefaultArea, DefaultAreaStrategy, ShoelaceArea, ShoelaceBoxArea,
    ShoelaceMultiPolygonArea, ShoelacePolygonArea,
};
pub use azimuth::{AzimuthStrategy, CartesianAzimuth, DefaultAzimuth, DefaultAzimuthStrategy};
pub use buffer::{
    BufferDistanceStrategy, BufferEndStrategy, BufferJoinStrategy, BufferPointStrategy,
    BufferSettings, BufferSideStrategy, CartesianBuffer, DefaultBuffer, DefaultBufferStrategy,
    GeographicBuffer, SphericalBuffer,
};
pub use cartesian::{ComparablePythagoras, PointToSegment, Pythagoras};
pub use centroid::{
    CartesianBoxCentroid, CartesianLinestringCentroid, CartesianMultiPointCentroid,
    CartesianPolygonCentroid, CartesianRingCentroid, CartesianSegmentCentroid, CentroidStrategy,
    CentroidStrategyForKind,
};
pub use closest_points::{CartesianClosestPoints, ClosestPointsStrategy};
pub use compare::{ALL_DIMENSIONS, EqualTo, Greater, Less, LessExact};
pub use convex_hull::{ConvexHullStrategy, MonotoneChain};
pub use densify::{CartesianDensify, DensifyStrategy};
pub use destination::{DefaultDestination, DefaultDestinationStrategy, DestinationStrategy};
pub use disjoint::{CartesianDisjoint, DisjointStrategy};
pub use distance::{DefaultDistance, DefaultDistanceStrategy, DistanceStrategy};
pub use envelope::{
    EnvelopeBox, EnvelopeLinestring, EnvelopeMultiLinestring, EnvelopeMultiPoint,
    EnvelopeMultiPolygon, EnvelopePoint, EnvelopePolygon, EnvelopeRing, EnvelopeSegment,
    EnvelopeStrategy, EnvelopeStrategyForKind,
};
pub use equals::{
    EqPointPoint, EqPolygonPolygon, EqSegmentSegment, EqualsPairStrategy, EqualsStrategy,
};
pub use geographic::{
    Andoyer, DirectResult, GeographicArea, GeographicAzimuth, GeographicLength,
    GeographicPerimeter, GeographicPolygonArea, InverseResult, Karney, KarneyDirect, KarneyInverse,
    Thomas, ThomasDirect, Vincenty, VincentyDirect,
};
pub use intersects::{CartesianIntersects, IntersectsPairStrategy, IntersectsStrategy};
pub use length::{
    CartesianLength, CartesianPerimeter, DefaultLength, DefaultLengthStrategy, DefaultPerimeter,
    DefaultPerimeterStrategy, LengthStrategy,
};
pub use line_interpolate::{CartesianLineInterpolate, LineInterpolateStrategy};
pub use reversal::Reversed;
pub use segmentize::{CartesianSegmentize, SegmentizeStrategy};
pub use simplify::{
    DouglasPeucker, SimplifyStrategy, VisvalingamWhyatt, VisvalingamWhyattPreserve,
};
pub use spherical::{
    ComparableHaversine, CrossTrack, Haversine, HaversineClosestPoints, SphericalArea,
    SphericalAzimuth, SphericalLength, SphericalPerimeter, SphericalPolygonArea,
};
pub use transform::{Affine2, Affine3, Rotate, Scale, Skew, TransformStrategy, Translate};
pub use within::{WithinBox, WithinPoly, WithinRing, WithinStrategy, WithinStrategyForKind};
