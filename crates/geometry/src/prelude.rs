//! Conventional star-imports for end users:
//!
//! ```ignore
//! use boost_geometry::prelude::*;
//! ```
//!
//! Brings in:
//!  * the algorithm free functions (`distance`, `area`, `length`,
//!    `within`, `intersects`, …);
//!  * the concept traits (`Point`, `Linestring`, `Polygon`, …) so
//!    that method calls on them resolve;
//!  * the coordinate-system types (`Cartesian`, `Spherical<U>`,
//!    `Geographic<U>`) and angle units (`Degree`, `Radian`);
//!  * the default model points (`Point2D`, `Point3D`).
//!
//! Items intentionally *not* in the prelude:
//!
//!  * the model linestring / ring / polygon types — they share their
//!    names with the same-named concept traits in [`crate::trait_`],
//!    so a single glob would collide. Pull the model variants
//!    explicitly when you need them:
//!
//!    ```ignore
//!    use boost_geometry::model::{Linestring, Polygon, Ring};
//!    ```
//!
//!  * the adapter types and the concrete strategies. They are
//!    seldom-used in any given file and pulling them via a glob
//!    obscures their origin. Import them explicitly:
//!
//!    ```ignore
//!    use boost_geometry::adapt::{Adapt, WithCs};
//!    use boost_geometry::strategy::geographic::Vincenty;
//!    use boost_geometry::strategy::spherical::Haversine;
//!    ```

pub use crate::algorithm::{
    DynKindMismatch, append, append_to_ring, area, area_dyn, assign_values, azimuth, box_area,
    centroid, centroid_with, clear, closest_points, comparable_distance, convert, convex_hull,
    correct, covered_by, densify, discrete_frechet_distance, discrete_frechet_distance_with,
    discrete_hausdorff_distance, discrete_hausdorff_distance_with, disjoint, disjoint_box_box,
    distance, distance_dyn, distance_with, envelope, envelope_dyn, equals, for_each_point,
    for_each_segment, intersects, is_convex, is_empty, is_simple, length, length_dyn,
    line_interpolate, make_box, make_point, make_segment, multi_polygon_area, num_geometries,
    num_interior_rings, num_points, num_segments, perimeter, remove_spikes, reverse, ring_area,
    ring_perimeter, simplify, transform, unique, within, within_dyn,
};
pub use crate::cs::{Cartesian, CoordinateSystem, Degree, Geographic, Radian, Spherical};
pub use crate::model::{Point2D, Point3D};
pub use crate::overlay::{
    JoinStrategy, OverlayError, PointStrategy, ValidityFailure, buffer_convex_polygon,
    buffer_point, crosses, difference, intersection, is_valid_polygon, is_valid_ring,
    merge_multipolygon, merge_polygons, overlaps, point_on_surface, sym_difference, touches,
    union_poly,
};
pub use crate::rtree::{Bounds, Indexable, Linear, Predicate, Quadratic, Rtree};
pub use crate::trait_::{
    Box, Geometry, GeometryCollection, Linestring, MultiLinestring, MultiPoint, MultiPolygon,
    Point, PointMut, PointOrder, Polygon, PolyhedralSurface, Ring, Segment,
};
