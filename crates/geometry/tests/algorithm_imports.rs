//! Compile-time contract for the complete mapped algorithm facade.
//!
//! Every name in `specs/feature_parity/algorithms.md` is imported from the
//! end-user prelude here. Behavioural parity remains covered by the
//! C++-breadcrumbed tests owned by each algorithm slice; this test proves the
//! final facade-import step required by the porting checklist.

#[allow(
    unused_imports,
    reason = "the imports themselves are the compile-time public-facade contract"
)]
use boost_geometry::prelude::{
    append, append_to_ring, area, area_dyn, area_with, assign_values, azimuth, azimuth_with,
    box_area, buffer, buffer_with, centroid, centroid_with, clear, closest_points,
    comparable_distance, comparable_distance_with, convert, convex_hull, correct, correct_closure,
    covered_by, crosses, densify, difference, discrete_frechet_distance,
    discrete_frechet_distance_with, discrete_hausdorff_distance, discrete_hausdorff_distance_with,
    disjoint, disjoint_box_box, distance, distance_dyn, distance_with, envelope, envelope_dyn,
    equals, expand, expand_with, for_each_point, for_each_segment, intersection, intersects,
    intersects_reversed, is_convex, is_empty, is_simple, is_valid, length, length_dyn, length_with,
    line_interpolate, make_box, make_point, make_segment, merge_elements, multi_polygon_area,
    num_geometries, num_interior_rings, num_points, num_segments, overlaps, perimeter,
    perimeter_with, point_on_surface, relate, relation, remove_spikes, reverse, ring_area,
    ring_perimeter, ring_perimeter_with, simplify, sym_difference, touches, transform, r#union,
    unique, within, within_dyn,
};

#[test]
fn all_mapped_algorithm_entries_import_from_the_public_prelude() {}

/// A foreign linestring type registered through the facade-root macro.
struct RegisteredPath {
    pts: Vec<boost_geometry::model::Point2D<f64, boost_geometry::prelude::Cartesian>>,
}
boost_geometry::register_linestring!(
    RegisteredPath,
    boost_geometry::model::Point2D<f64, boost_geometry::prelude::Cartesian>,
    |s| s.pts.iter()
);

/// The `#[macro_export]`ed model and adapter macros are reachable at the
/// facade's crate root, as its module docs promise — a downstream crate
/// with `boost_geometry` as its only dependency can invoke them there.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "3-4-5 lengths and integer literals are exact in f64"
)]
fn model_and_register_macros_are_reachable_at_the_facade_root() {
    use boost_geometry::model::Point2D;
    use boost_geometry::prelude::{Cartesian, length};
    use boost_geometry::trait_::Point as _;

    let p: Point2D<f64, Cartesian> = boost_geometry::point!((1.0, 2.0));
    assert_eq!(p.get::<0>(), 1.0);
    let ls: boost_geometry::model::Linestring<Point2D<f64, Cartesian>> =
        boost_geometry::linestring![(0.0, 0.0), (3.0, 4.0)];
    assert_eq!(length(&ls), 5.0);
    let pg: boost_geometry::model::Polygon<Point2D<f64, Cartesian>> =
        boost_geometry::polygon![[(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (0.0, 0.0)]];
    assert_eq!(pg.outer.0.len(), 4);

    let path = RegisteredPath {
        pts: vec![Point2D::new(0.0, 0.0), Point2D::new(3.0, 4.0)],
    };
    assert_eq!(length(&path), 5.0);
}
