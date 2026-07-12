//! Compile-fail fixture: an unknown coordinate-system name must surface
//! as a type-resolution error pointing at `::geometry_cs::NoSuchCs`.
//!
//! The derive itself does not vet the CS name — it splices the parsed
//! `syn::Path` after `::geometry_cs::`. The error therefore comes from
//! `rustc` resolving the generated path, which is exactly the diagnostic
//! a user wants: it names the missing item.

use geometry_derive::Point;

#[derive(Point)]
#[geometry(cs = "NoSuchCs")]
struct Bad {
    x: f64,
    y: f64,
}

fn main() {}
