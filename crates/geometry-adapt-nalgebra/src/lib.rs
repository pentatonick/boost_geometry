//! Adapt `nalgebra` points and vectors to the `geometry-trait`
//! concept surface.
//!
//! Mirrors the role of the Boost `BOOST_GEOMETRY_REGISTER_*` macros in
//! `boost/geometry/geometries/adapted/` — those specialise the C++
//! `traits::{tag, dimension, coordinate_type, coordinate_system,
//! access}` metafunctions for a foreign storage layout. Rust has no
//! specialisation and the orphan rule forbids implementing a foreign
//! trait ([`geometry_trait::Point`]) for a foreign type
//! (`nalgebra::Point2` et al.), so — as with
//! [`geometry_adapt::Adapt`](https://docs.rs/geometry-adapt) — each
//! foreign value is wrapped in a local `#[repr(transparent)]` newtype
//! and the geometry concepts hang off the wrapper.
//!
//! Two domain nouns are adapted:
//!
//! * [`NaPoint2`] / [`NaPoint3`] — `nalgebra::Point2` / `Point3`.
//! * [`NaVector2`] / [`NaVector3`] — `nalgebra::Vector2` / `Vector3`.
//!
//! Every wrapper pins `Cs = Cartesian`. `nalgebra` points and vectors
//! are read-write, so each also implements
//! [`geometry_trait::PointMut`].

#![forbid(unsafe_code)]

mod point;
mod vector;

pub use point::{NaPoint2, NaPoint3};
pub use vector::{NaVector2, NaVector3};
