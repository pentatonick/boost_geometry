//! Trait-only polygon output, including malformed structures that must be preserved.

use geometry_cs::Cartesian;
use geometry_io_ewkb::{ByteOrder, Srid, from_ewkb, to_ewkb, to_ewkb_polygon};
use geometry_io_wkb::polygon_wkb_len;
use geometry_model::{DynGeometry, Point2D, Polygon, Ring};
use geometry_tag::PolygonTag;
use geometry_trait::{Geometry, Polygon as PolygonTrait};

type Pt = Point2D<f64, Cartesian>;

struct Parcel {
    exterior: Ring<Pt>,
    holes: Vec<Ring<Pt>>,
}

impl Geometry for Parcel {
    type Kind = PolygonTag;
    type Point = Pt;
}

impl PolygonTrait for Parcel {
    type Ring = Ring<Pt>;

    fn exterior(&self) -> &Self::Ring {
        &self.exterior
    }

    fn interiors(&self) -> impl ExactSizeIterator<Item = &Self::Ring> {
        self.holes.iter()
    }
}

#[test]
fn custom_empty_polygon_matches_postgis() {
    let parcel = Parcel {
        exterior: Ring::new(),
        holes: vec![],
    };
    for (order, expected) in [
        (
            ByteOrder::LittleEndian,
            [1, 3, 0, 0, 32, 230, 16, 0, 0, 0, 0, 0, 0],
        ),
        (
            ByteOrder::BigEndian,
            [0, 32, 0, 0, 3, 0, 0, 16, 230, 0, 0, 0, 0],
        ),
    ] {
        assert_eq!(
            to_ewkb_polygon(&parcel, Some(Srid::new(4326)), order),
            expected
        );
        assert_eq!(polygon_wkb_len(&parcel), Some(9));
    }
}

#[test]
fn empty_exterior_or_interior_does_not_erase_other_rings() {
    let boundary = Ring::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(0.0, 2.0),
        Pt::new(2.0, 2.0),
        Pt::new(0.0, 0.0),
    ]);
    for parcel in [
        Parcel {
            exterior: Ring::new(),
            holes: vec![boundary.clone()],
        },
        Parcel {
            exterior: boundary.clone(),
            holes: vec![Ring::new()],
        },
        Parcel {
            exterior: Ring::new(),
            holes: vec![Ring::new()],
        },
        Parcel {
            exterior: boundary.clone(),
            holes: vec![Ring::new(), boundary],
        },
    ] {
        let expected = DynGeometry::Polygon(Polygon::with_inners(
            parcel.exterior.clone(),
            parcel.holes.clone(),
        ));
        for order in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
            for srid in [None, Some(Srid::UNKNOWN), Some(Srid::new(4326))] {
                let bytes = to_ewkb_polygon(&parcel, srid, order);
                let read = from_ewkb(&bytes).unwrap();
                assert_eq!(read.geometry, expected);
                assert_eq!(read.srid, srid);
                assert_eq!(bytes, to_ewkb(&expected, srid, order));
                assert_eq!(
                    polygon_wkb_len(&parcel),
                    Some(bytes.len() - if srid.is_some() { 4 } else { 0 })
                );
            }
        }
    }
}
