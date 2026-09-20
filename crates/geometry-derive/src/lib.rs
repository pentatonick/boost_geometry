//! `#[derive(Point)]` for structs whose named fields are dimensions in
//! declaration order.
//!
//! Rust analogue of the `BOOST_GEOMETRY_REGISTER_POINT_2D` family of
//! macros declared at `boost/geometry/geometries/register/point.hpp:81-87`.
//! Where the C++ macro injects template specialisations in the
//! `boost::geometry::traits` namespace, this proc-macro emits an
//! `impl Geometry` + `impl Point` block on the annotated struct.
//!
//! The derive accepts an optional `#[geometry(...)]` attribute:
//!
//! ```text
//! #[derive(Point)]
//! #[geometry(cs = "Cartesian", scalar = "f64")]
//! struct MyPoint { x: f64, y: f64 }
//! ```
//!
//! Both keys are optional. `cs` defaults to `Cartesian` and `scalar`
//! to `f64`. Field order in the struct becomes dimension order in the
//! emitted `Point::get::<D>` / `Point::set::<D>` match arms.
//!
//! # Crate dependencies
//!
//! The generated code names the kernel crates by absolute path. When the
//! `boost_geometry` facade is a dependency of the crate being compiled
//! (under whatever name Cargo gave it), the paths route through the
//! facade's hidden `__private` re-exports, so depending on the facade
//! alone is enough. Otherwise they name `geometry_trait`, `geometry_tag`,
//! and `geometry_cs` directly, so a caller using this crate without the
//! facade must depend on those three. The coordinate-system path in
//! `#[geometry(cs = "…")]` resolves against `geometry_cs` first, so
//! `Spherical<Degree>` needs no import at the use site.

extern crate proc_macro;

mod derive_point;

use proc_macro::TokenStream;

/// `#[derive(Point)]` — see the crate-level docs for the attribute shape.
#[proc_macro_derive(Point, attributes(geometry))]
pub fn derive_point(input: TokenStream) -> TokenStream {
    derive_point::expand(input.into()).into()
}
