# geometry-io-wkt

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

OGC Well-Known Text (WKT) reader and writer.

Mirrors `boost/geometry/io/wkt/{read,write,wkt}.hpp`. The parser
emits a [`geometry_model::DynGeometry`] because WKT is heterogeneous
by construction (a `GEOMETRYCOLLECTION` mixes kinds); the writer
accepts concrete model geometries with [`to_wkt`] and user-defined
polygons implementing the geometry traits with [`to_wkt_polygon`].

Reference: OGC Simple Feature Access Part 1 (SFA-1) §7 for the WKT
grammar.

## Checked XY profile

The text codec accepts exactly two ordinates and rejects Z/M/ZM qualifiers.
Its lexical and structural rules follow `PostGIS` 3.4.3: unsigned `NaN` is
accepted, infinity is rejected, lines need at least two points, and polygon
rings need at least four points with matching endpoints. Topological validity
is not checked. Decimal overflow is rejected even though `PostGIS` accepts it.
Finite coordinates, including signed zero, round-trip without precision loss.

[`from_wkt_2d`] preserves empty points and empty multipoint members through
[`geometry_model::GeometryValue`]. Its generic point parameter keeps storage
dimension agnostic; this codec remains XY-only. [`from_wkt`] returns the
existing dynamic model and rejects empty points it cannot represent.

## Migration

Owned and streaming writers now return [`WktWriteError`]. Handle the result
rather than assuming a geometry can be serialized. Writers reject unsupported
dimensions, malformed ring/line structure, infinity and excessive nesting.
After a streaming error, discard the partial output. Readers no longer project
extra coordinates to XY or accept Unicode whitespace and leading `+` mantissas.

### Serialize a user-defined polygon

Application types can implement the lightweight [`geometry_trait`] traits
directly; they do not need to be converted to a `geometry_model` polygon.

```rust
use geometry_cs::Cartesian;
use geometry_io_wkt::to_wkt_polygon;
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

assert_eq!(
    to_wkt_polygon(&parcel).unwrap(),
    "POLYGON((0 0,0 2,2 2,2 0,0 0))"
);
```

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
