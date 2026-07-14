//! Public-facade tests for registering user-owned multi geometries.

use boost_geometry::adapt::{
    register_multi_linestring, register_multi_point, register_multi_polygon,
};
use boost_geometry::cs::Cartesian;
use boost_geometry::model::{Linestring, Point2D, Polygon, Ring};
use boost_geometry::trait_::{MultiLinestring, MultiPoint, MultiPolygon};

type P = Point2D<f64, Cartesian>;

struct Places(Vec<P>);
register_multi_point!(Places, P, |value| value.0.iter());

struct Paths(Vec<Linestring<P>>);
register_multi_linestring!(Paths, P, item = Linestring<P>, |value| value.0.iter());

struct Regions(Vec<Polygon<P>>);
register_multi_polygon!(Regions, P, item = Polygon<P>, |value| value.0.iter());

/// `test/geometries/register/multi.cpp:35-88` — registered containers
/// expose their elements through the public multi-geometry concepts.
#[test]
fn registered_multis_iterate_user_storage() {
    let places = Places(vec![P::new(1.0, 2.0), P::new(3.0, 4.0)]);
    assert_eq!(places.points().len(), 2);

    let paths = Paths(vec![Linestring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(1.0, 1.0),
    ])]);
    assert_eq!(paths.linestrings().len(), 1);

    let regions = Regions(vec![Polygon::new(Ring::from_vec(vec![
        P::new(0.0, 0.0),
        P::new(1.0, 0.0),
        P::new(0.0, 1.0),
        P::new(0.0, 0.0),
    ]))]);
    assert_eq!(regions.polygons().len(), 1);
}
