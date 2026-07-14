//! `Default` for every growable model container builds the same empty
//! value as `new()` — the Rust analogue of the C++ default constructors
//! on `model::linestring` / `model::ring` / the `multi_*` types, which
//! all default-construct their inherited `std::vector` empty.

use geometry_cs::Cartesian;
use geometry_model::{
    Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};

type Pt = Point2D<f64, Cartesian>;

#[test]
fn linestring_default_is_empty() {
    let ls = Linestring::<Pt>::default();
    assert_eq!(ls.0.len(), 0);
    assert_eq!(ls, Linestring::new());
}

#[test]
fn ring_default_is_empty() {
    let r = Ring::<Pt>::default();
    assert_eq!(r.0.len(), 0);
    assert_eq!(r, Ring::new());
}

#[test]
fn multi_point_default_is_empty() {
    let mp = MultiPoint::<Pt>::default();
    assert_eq!(mp.0.len(), 0);
    assert_eq!(mp, MultiPoint::new());
}

#[test]
fn multi_linestring_default_is_empty() {
    let mls = MultiLinestring::<Linestring<Pt>>::default();
    assert_eq!(mls.0.len(), 0);
    assert_eq!(mls, MultiLinestring::new());
}

#[test]
fn multi_polygon_default_is_empty() {
    let mpg = MultiPolygon::<Polygon<Pt>>::default();
    assert_eq!(mpg.0.len(), 0);
    assert_eq!(mpg, MultiPolygon::new());
}
