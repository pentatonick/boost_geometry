//! Coordinate scalars, type promotion, and the comparable-distance newtype.
//!
//! Mirrors the trio of Boost.Geometry headers that together encode
//! "what numeric types are allowed as coordinates, how do we widen them
//! when we have to do arithmetic across two geometries, and how do we
//! defer the `sqrt` in a distance comparison":
//!
//! - `boost/geometry/util/select_most_precise.hpp` — the type-priority
//!   metafunction picking the wider of two scalars.
//! - `boost/geometry/util/calculation_type.hpp` — the policy that
//!   chooses the working type for binary/ternary algorithms.
//! - `boost/geometry/util/math.hpp` — fundamental scalar primitives
//!   (`abs`, `sqrt`, …) abstracted over the numeric type.
//! - `boost/geometry/util/precise_math.hpp` — adaptive expansion arithmetic
//!   for robust orientation and in-circle signs.
//! - `boost/geometry/util/series_expansion.hpp` — eighth-order Karney
//!   coefficient tables and Clenshaw evaluation.
//! - `boost/geometry/strategies/cartesian/distance_pythagoras.hpp`
//!   (lines 71-117, `namespace comparable`) — the squared-distance
//!   wrapper that callers can compare without paying for a `sqrt`.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

mod comparable;
pub mod math;
pub mod precise_math;
mod promote;
mod rational;
mod scalar;
pub mod series_expansion;

pub use comparable::Comparable;
pub use promote::{Promote, PromoteIntegral};
pub use rational::{ParseRationalError, Rational, RationalInteger};
pub use scalar::CoordinateScalar;
