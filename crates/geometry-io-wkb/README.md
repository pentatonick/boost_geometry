# geometry-io-wkb

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

OGC Well-Known Binary (WKB) reader and writer.

Follows OGC Simple Feature Access 06-103r4 §8 (Boost.Geometry ships
no WKB). The parser emits a [`geometry_model::DynGeometry`]; the
writer serialises concrete model geometries with [`to_wkb`] and any
user-defined polygon implementing the geometry traits with
[`to_wkb_polygon`].

### Serialize a user-defined polygon

Application types can implement the lightweight [`geometry_trait`] traits
directly; they do not need to be converted to a `geometry_model` polygon.

```rust
use geometry_cs::Cartesian;
use geometry_io_wkb::{from_wkb, to_wkb_polygon, ByteOrder};
use geometry_tag::{PointTag, PolygonTag, RingTag};
use geometry_trait::{Geometry, Point, Polygon, Ring};

struct Coordinate(f64, f64);

impl Geometry for Coordinate {
    type Kind = PointTag;
    type Point = Self;
}

impl Point for Coordinate {
    type Scalar = f64;
    type Cs = Cartesian;
    const DIM: usize = 2;

    fn get<const D: usize>(&self) -> f64 {
        match D {
            0 => self.0,
            1 => self.1,
            _ => unreachable!("a Coordinate has two dimensions"),
        }
    }
}

struct Boundary(Vec<Coordinate>);

impl Geometry for Boundary {
    type Kind = RingTag;
    type Point = Coordinate;
}

impl Ring for Boundary {
    fn points(&self) -> impl ExactSizeIterator<Item = &Coordinate> + Clone {
        self.0.iter()
    }
}

struct Parcel {
    exterior: Boundary,
    holes: Vec<Boundary>,
}

impl Geometry for Parcel {
    type Kind = PolygonTag;
    type Point = Coordinate;
}

impl Polygon for Parcel {
    type Ring = Boundary;

    fn exterior(&self) -> &Boundary {
        &self.exterior
    }

    fn interiors(&self) -> impl ExactSizeIterator<Item = &Boundary> {
        self.holes.iter()
    }
}

let parcel = Parcel {
    exterior: Boundary(vec![
        Coordinate(0.0, 0.0),
        Coordinate(0.0, 2.0),
        Coordinate(2.0, 2.0),
        Coordinate(2.0, 0.0),
        Coordinate(0.0, 0.0),
    ]),
    holes: vec![],
};

let bytes = to_wkb_polygon(&parcel, ByteOrder::LittleEndian);
assert_eq!(bytes[0], 0x01);
assert!(from_wkb(&bytes).is_ok());
```

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
