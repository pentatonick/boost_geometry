//! Public-facade parity tests for the remaining stock geometry models.

#![allow(
    clippy::float_cmp,
    reason = "the model-access witnesses compare exactly stored coordinates"
)]

use core::any::TypeId;

use boost_geometry::cs::{Cartesian, Degree, Geographic};
use boost_geometry::model::{
    Box as ModelBox, InfiniteLine, Linestring, Point2D, Point3D, PointingSegment, Polygon,
    PolyhedralSurface, Rebound, Ring,
};
use boost_geometry::prelude::{Point, PolyhedralSurface as _};
use boost_geometry::trait_::IndexedAccess;

/// `test/geometries/point_xyz.cpp:44-72` — the named xyz accessors read
/// and write the same three ordinates as indexed point access.
#[test]
fn point_xyz_has_named_coordinate_access() {
    let mut point2 = Point2D::<f64, Cartesian>::new(1.0, 2.0);
    point2.set_x(7.0);
    point2.set_y(8.0);
    assert_eq!((point2.x(), point2.y()), (7.0, 8.0));

    let mut point = Point3D::<f64, Cartesian>::new(1.0, 2.0, 3.0);
    assert_eq!((point.x(), point.y(), point.z()), (1.0, 2.0, 3.0));
    point.set_x(4.0);
    point.set_y(5.0);
    point.set_z(6.0);
    assert_eq!(
        (point.get::<0>(), point.get::<1>(), point.get::<2>()),
        (4.0, 5.0, 6.0)
    );
}

/// `test/geometries/pointing_segment.cpp:40-65` — indexed access forwards
/// through the two referenced endpoints, including mutation.
#[test]
fn pointing_segment_borrows_its_endpoints() {
    let mut start = Point2D::<f64, Cartesian>::new(1.0, 2.0);
    let mut end = Point2D::<f64, Cartesian>::new(3.0, 4.0);
    {
        let mut segment = PointingSegment::new(&mut start, &mut end);
        assert_eq!((segment.start().x(), segment.start().y()), (1.0, 2.0));
        assert_eq!((segment.end().x(), segment.end().y()), (3.0, 4.0));
        assert_eq!(segment.get_indexed::<0, 0>(), 1.0);
        assert_eq!(segment.get_indexed::<1, 1>(), 4.0);
        segment.set_indexed::<0, 1>(8.0);
        segment.set_indexed::<1, 0>(9.0);
    }
    assert_eq!(start.get::<1>(), 8.0);
    assert_eq!(end.get::<0>(), 9.0);
}

/// `test/geometries/infinite_line.cpp:43-75` and
/// `test/arithmetic/infinite_line_functions.cpp:81-95` — construction from
/// endpoints, side classification, and line intersection.
#[test]
fn infinite_line_supports_reference_arithmetic() {
    let horizontal = InfiniteLine::<f64>::from_coordinates(0.0, 0.0, 10.0, 0.0);
    assert!(horizontal.side_value(5.0, 5.0) > 0.0);
    assert!(horizontal.side_value(5.0, -5.0) < 0.0);

    let first = InfiniteLine::<f64>::from_coordinates(5.0, 2.0, -7.0, 10.0);
    let second = InfiniteLine::<f64>::from_coordinates(4.0, 7.0, 0.0, 1.0);
    let intersection = first.intersection(&second).unwrap();
    assert!((intersection.x() - 2.0).abs() < 1e-12);
    assert!((intersection.y() - 4.0).abs() < 1e-12);
    assert!(!first.is_degenerate());
    assert!(InfiniteLine::<f64>::default().is_degenerate());

    let start = Point2D::<f64, Cartesian>::new(0.0, 5.0);
    let end = Point2D::<f64, Cartesian>::new(10.0, 5.0);
    let from_points = InfiniteLine::from_points(&start, &end);
    assert_eq!(from_points.side_value(3.0, 5.0), 0.0);
    assert!(from_points.intersection(&horizontal).is_none());
}

/// `geometries/helper_geometry.hpp:44-121` — rebound models preserve the
/// geometry kind while replacing scalar and coordinate-system types.
#[test]
fn rebound_geometry_preserves_model_shape() {
    type P = Point2D<f32, Cartesian>;
    type B = ModelBox<P>;
    type L = Linestring<P>;
    type R = Ring<P, false, false>;
    type Geo = Geographic<Degree>;

    assert_eq!(
        TypeId::of::<Rebound<P, f64, Geo>>(),
        TypeId::of::<Point2D<f64, Geo>>()
    );
    assert_eq!(
        TypeId::of::<Rebound<B, f64, Geo>>(),
        TypeId::of::<ModelBox<Point2D<f64, Geo>>>()
    );
    assert_eq!(
        TypeId::of::<Rebound<L, f64, Geo>>(),
        TypeId::of::<Linestring<Point2D<f64, Geo>>>()
    );
    assert_eq!(
        TypeId::of::<Rebound<R, f64, Geo>>(),
        TypeId::of::<Ring<Point2D<f64, Geo>, false, false>>()
    );
}

/// `geometries/polyhedral_surface.hpp:48-83` — the stock model owns an
/// ordered polygon collection and exposes it through the public concept.
#[test]
fn polyhedral_surface_owns_faces() {
    type P = Point3D<f64, Cartesian>;
    type Face = Polygon<P>;
    let face = Face::new(Ring::from_vec(vec![
        P::new(0.0, 0.0, 0.0),
        P::new(1.0, 0.0, 0.0),
        P::new(0.0, 1.0, 0.0),
        P::new(0.0, 0.0, 0.0),
    ]));
    let empty = PolyhedralSurface::<Face>::default();
    assert_eq!(empty.faces().len(), 0);

    let mut surface = PolyhedralSurface::from_vec(vec![face.clone()]);
    surface.push(face);
    assert_eq!(surface.faces().len(), 2);
}
