//! Free-function algorithm entry points users actually call.
//!
//! Each function mirrors the matching free function in
//! `boost/geometry/algorithms/`. Strategy-driven algorithms expose a
//! strategy-less default plus a `_with` explicit-strategy companion. Algorithms
//! that require the overlay engine live in `geometry-overlay` to preserve the
//! workspace's one-way dependency graph.
//!
//! # References
//!
//! * `boost/geometry/algorithms/`
//! * `boost/geometry/algorithms/detail/`

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

pub mod append;
pub mod area;
pub mod assign;
pub mod azimuth;
pub mod centroid;
pub mod chaikin_smoothing;
pub mod clear;
pub mod closest_points;
pub mod concave_hull;
pub mod convert;
pub mod convex_hull;
pub mod coordinate_position;
pub mod correct;
pub mod densify;
pub mod destination;
pub mod discrete_frechet;
pub mod discrete_hausdorff;
pub mod disjoint;
pub mod distance;
pub mod envelope;
pub mod equals;
pub mod expand;
pub mod for_each;
pub mod intersects;
pub mod is_convex;
pub mod is_empty;
pub mod is_simple;
pub mod length;
pub mod line_interpolate;
pub mod line_locate_point;
pub mod linestring_segmentize;
pub mod make;
pub mod map_coords;
pub mod minimum_rotated_rect;
pub mod monotone_subdivision;
pub mod num_geometries;
pub mod num_interior_rings;
pub mod num_points;
pub mod num_segments;
pub mod remove_spikes;
pub mod reverse;
pub mod rhumb;
pub mod simplify;
pub mod transform;
pub mod triangulate_earcut;
pub mod unique;
pub mod within;

mod dyn_area;
mod dyn_distance;
mod dyn_envelope;
mod dyn_error;
mod dyn_length;
mod dyn_within;

// feature-group: Mutation & assembly
// feature-desc: Build up or normalise a geometry in place
pub use append::{append, append_to_ring};
// feature-group: Measures
// feature-desc: Scalar quantities of a geometry
pub use area::{area, area_with, box_area, multi_polygon_area, ring_area};
// feature-group: Mutation & assembly
pub use assign::assign_values;
// feature-group: Measures
pub use azimuth::{azimuth, azimuth_with};
// feature-group: Measures
pub use centroid::{centroid, centroid_with};
// feature-group: Construction & transformation
// feature-desc: Derive a new geometry from an existing one
pub use chaikin_smoothing::{ChaikinSmoothing, chaikin_smoothing};
// feature-group: Mutation & assembly
pub use clear::clear;
// feature-group: Measures
pub use closest_points::{closest_points, closest_points_with};
// feature-group: Construction & transformation
pub use concave_hull::{
    ConcaveHullParams, concave_hull, concave_hull_with, k_nearest_concave_hull,
};
// feature-group: Mutation & assembly
pub use convert::convert;
// feature-group: Construction & transformation
pub use convex_hull::convex_hull;
// feature-group: Spatial predicates
// feature-desc: Boolean relationships between geometries
pub use coordinate_position::{CoordinatePosition, coordinate_position};
// feature-group: Mutation & assembly
pub use correct::{correct, correct_closure};
// feature-group: Construction & transformation
pub use densify::densify;
// feature-group: Construction & transformation
pub use destination::{destination, destination_with};
// feature-group: Measures
pub use discrete_frechet::{discrete_frechet_distance, discrete_frechet_distance_with};
// feature-group: Measures
pub use discrete_hausdorff::{discrete_hausdorff_distance, discrete_hausdorff_distance_with};
// feature-group: Spatial predicates
pub use disjoint::{disjoint, disjoint_box_box};
// feature-group: Measures
pub use distance::{comparable_distance, comparable_distance_with, distance, distance_with};
// feature-group: Construction & transformation
pub use envelope::envelope;
// feature-group: Spatial predicates
pub use equals::equals;
// feature-group: Construction & transformation
pub use expand::{expand, expand_with};
// feature-group: Inspection
// feature-desc: Query a geometry's shape or membership
pub use for_each::{for_each_point, for_each_segment};
// feature-group: Spatial predicates
pub use intersects::{intersects, intersects_reversed};
// feature-group: Inspection
pub use is_convex::is_convex;
// feature-group: Inspection
pub use is_empty::is_empty;
// feature-group: Inspection
pub use is_simple::is_simple;
// feature-group: Measures
pub use length::{
    length, length_with, perimeter, perimeter_with, ring_perimeter, ring_perimeter_with,
};
// feature-group: Construction & transformation
pub use line_interpolate::line_interpolate;
// feature-group: Construction & transformation
pub use line_locate_point::line_locate_point;
// feature-group: Construction & transformation
pub use linestring_segmentize::{linestring_segmentize, linestring_segmentize_with};
// feature-group: Mutation & assembly
pub use make::{make_box, make_point, make_segment};
// feature-group: Construction & transformation
pub use map_coords::{MapCoords, MapCoordsInPlace, map_coords, map_coords_in_place};
// feature-group: Construction & transformation
pub use minimum_rotated_rect::minimum_rotated_rect;
// feature-group: Construction & transformation
pub use monotone_subdivision::monotone_subdivision;
// feature-group: Inspection
pub use num_geometries::num_geometries;
// feature-group: Inspection
pub use num_interior_rings::num_interior_rings;
// feature-group: Inspection
pub use num_points::num_points;
// feature-group: Inspection
pub use num_segments::num_segments;
// feature-group: Mutation & assembly
pub use remove_spikes::remove_spikes;
// feature-group: Mutation & assembly
pub use reverse::reverse;
// feature-group: Construction & transformation
pub use rhumb::{
    rhumb_azimuth, rhumb_azimuth_with, rhumb_destination, rhumb_destination_with, rhumb_distance,
    rhumb_distance_with, rhumb_length, rhumb_length_with,
};
// feature-group: Construction & transformation
pub use simplify::{simplify, simplify_with};
// feature-group: Construction & transformation
pub use transform::transform;
// feature-group: Construction & transformation
pub use triangulate_earcut::triangulate_earcut;
// feature-group: Mutation & assembly
pub use unique::unique;
// feature-group: Spatial predicates
pub use within::{covered_by, within};

// feature-group: Measures
pub use dyn_area::area_dyn;
// feature-group: Measures
pub use dyn_distance::distance_dyn;
// feature-group: Construction & transformation
pub use dyn_envelope::envelope_dyn;
pub use dyn_error::DynKindMismatch;
// feature-group: Measures
pub use dyn_length::length_dyn;
// feature-group: Spatial predicates
pub use dyn_within::within_dyn;
