//! M-IO2 — WKB round-trip: build each OGC kind, serialise, re-parse, and
//! assert structural equality on the [`DynGeometry`]. Exercised in both
//! byte orders. Mirrors the WKT round-trip milestone (M-IO1).
//!
//! Reference: OGC Simple Feature Access 06-103r4 §8.2.

use geometry_cs::Cartesian;
use geometry_io_wkb::{ByteOrder, WkbError, WriteWkb, from_wkb, to_wkb, to_wkb_polygon};
use geometry_model::{
    DynGeometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point2D, Polygon, Ring,
};
use geometry_tag::PointTag;
use geometry_trait::Geometry;

type Pt = Point2D<f64, Cartesian>;
type Dyn = DynGeometry<f64, Cartesian>;

/// Serialise `g` in `order`, re-parse, and assert it is unchanged.
fn assert_round_trip(g: &Dyn, order: ByteOrder) {
    let bytes = to_wkb(g, order);
    let back = from_wkb(&bytes).expect("re-parse must succeed");
    assert_eq!(&back, g, "round-trip mismatch in {order:?}");
}

/// Round-trip `g` in both byte orders.
fn assert_both_orders(g: &Dyn) {
    assert_round_trip(g, ByteOrder::LittleEndian);
    assert_round_trip(g, ByteOrder::BigEndian);
}

fn sample_ring() -> Ring<Pt> {
    Ring::from_vec(vec![
        Pt::new(0.0, 0.0),
        Pt::new(0.0, 10.0),
        Pt::new(10.0, 10.0),
        Pt::new(10.0, 0.0),
        Pt::new(0.0, 0.0),
    ])
}

fn sample_hole() -> Ring<Pt> {
    Ring::from_vec(vec![
        Pt::new(2.0, 2.0),
        Pt::new(2.0, 4.0),
        Pt::new(4.0, 4.0),
        Pt::new(4.0, 2.0),
        Pt::new(2.0, 2.0),
    ])
}

fn sample_polygon() -> Polygon<Pt> {
    Polygon::with_inners(sample_ring(), vec![sample_hole()])
}

#[test]
fn point_round_trips() {
    assert_both_orders(&DynGeometry::Point(Pt::new(1.5, -2.25)));
}

#[test]
fn linestring_round_trips() {
    let ls = Linestring(vec![
        Pt::new(10.0, 10.0),
        Pt::new(20.0, 20.0),
        Pt::new(30.0, 40.0),
    ]);
    assert_both_orders(&DynGeometry::LineString(ls));
}

#[test]
fn polygon_with_hole_round_trips() {
    assert_both_orders(&DynGeometry::Polygon(sample_polygon()));
}

#[test]
fn multipoint_round_trips() {
    let mp = MultiPoint(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)]);
    assert_both_orders(&DynGeometry::MultiPoint(mp));
}

#[test]
fn multilinestring_round_trips() {
    let mls = MultiLinestring(vec![
        Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)]),
        Linestring(vec![Pt::new(15.0, 15.0), Pt::new(30.0, 15.0)]),
    ]);
    assert_both_orders(&DynGeometry::MultiLineString(mls));
}

#[test]
fn multipolygon_round_trips() {
    let mpg = MultiPolygon(vec![sample_polygon(), Polygon::new(sample_ring())]);
    assert_both_orders(&DynGeometry::MultiPolygon(mpg));
}

#[test]
fn geometry_collection_round_trips() {
    let g = DynGeometry::GeometryCollection(vec![
        DynGeometry::Point(Pt::new(10.0, 10.0)),
        DynGeometry::LineString(Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)])),
        DynGeometry::Polygon(sample_polygon()),
        // Nested collection.
        DynGeometry::GeometryCollection(vec![DynGeometry::Point(Pt::new(3.0, 4.0))]),
    ]);
    assert_both_orders(&g);
}

#[test]
fn byte_for_byte_stable_across_reparse() {
    // A serialised buffer that survives parse → re-emit unchanged
    // (the PostGIS-parity property of M-IO2).
    let g = DynGeometry::Polygon(sample_polygon());
    for order in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
        let bytes = to_wkb(&g, order);
        let reparsed = from_wkb(&bytes).unwrap();
        let reemitted = to_wkb(&reparsed, order);
        assert_eq!(bytes, reemitted, "byte-for-byte parity failed in {order:?}");
    }
}

#[test]
fn bare_ring_and_public_polygon_writer_round_trip() {
    for order in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
        let ring = sample_ring();
        let parsed_ring = from_wkb(&to_wkb(&ring, order)).unwrap();
        assert_eq!(parsed_ring, Dyn::Polygon(Polygon::new(ring)));

        let polygon = sample_polygon();
        let parsed_polygon = from_wkb(&to_wkb_polygon(&polygon, order)).unwrap();
        assert_eq!(parsed_polygon, Dyn::Polygon(polygon));
    }
}

#[test]
fn collection_with_every_geometry_kind_round_trips() {
    let polygon = sample_polygon();
    let collection = Dyn::GeometryCollection(vec![
        Dyn::Point(Pt::new(1.0, 2.0)),
        Dyn::LineString(Linestring(vec![Pt::new(3.0, 4.0), Pt::new(5.0, 6.0)])),
        Dyn::Polygon(polygon.clone()),
        Dyn::MultiPoint(MultiPoint(vec![Pt::new(7.0, 8.0)])),
        Dyn::MultiLineString(MultiLinestring(vec![Linestring(vec![Pt::new(9.0, 10.0)])])),
        Dyn::MultiPolygon(MultiPolygon(vec![polygon])),
        Dyn::GeometryCollection(vec![Dyn::Point(Pt::new(11.0, 12.0))]),
    ]);
    assert_both_orders(&collection);
}

fn little_endian_container(type_code: u32, member: &[u8]) -> Vec<u8> {
    let mut bytes = vec![1];
    bytes.extend_from_slice(&type_code.to_le_bytes());
    bytes.extend_from_slice(&1_u32.to_le_bytes());
    bytes.extend_from_slice(member);
    bytes
}

#[test]
fn malformed_documents_cover_the_public_error_contract() {
    assert_eq!(from_wkb(&[]).unwrap_err(), WkbError::UnexpectedEof);
    assert_eq!(from_wkb(&[2]).unwrap_err(), WkbError::InvalidByteOrder(2));

    let mut unknown = vec![1];
    unknown.extend_from_slice(&8_u32.to_le_bytes());
    assert_eq!(
        from_wkb(&unknown).unwrap_err(),
        WkbError::UnknownGeometryType(8)
    );

    for type_code in [0x8000_0001_u32, 0x4000_0001, 0x2000_0001, 1001, 2001, 3001] {
        let mut dimensional = vec![1];
        dimensional.extend_from_slice(&type_code.to_le_bytes());
        assert_eq!(
            from_wkb(&dimensional).unwrap_err(),
            WkbError::UnsupportedDimension
        );
    }

    let mut trailing = to_wkb(&Pt::new(1.0, 2.0), ByteOrder::LittleEndian);
    trailing.push(0xff);
    assert_eq!(from_wkb(&trailing).unwrap_err(), WkbError::TrailingBytes);

    let wrong_members = [
        Dyn::LineString(Linestring(vec![])),
        Dyn::Polygon(Polygon::new(Ring::new())),
        Dyn::MultiPoint(MultiPoint(vec![])),
        Dyn::MultiLineString(MultiLinestring(vec![])),
        Dyn::MultiPolygon(MultiPolygon(vec![])),
        Dyn::GeometryCollection(vec![]),
    ];
    for (offset, member) in wrong_members.into_iter().enumerate() {
        let member_code = u32::try_from(offset + 2).unwrap();
        let member = to_wkb(&member, ByteOrder::LittleEndian);
        assert_eq!(
            from_wkb(&little_endian_container(4, &member)).unwrap_err(),
            WkbError::MismatchedMemberType {
                expected: 1,
                found: member_code,
            }
        );
    }

    for container_code in [2_u32, 3, 4, 5, 6, 7] {
        let mut truncated = vec![1];
        truncated.extend_from_slice(&container_code.to_le_bytes());
        truncated.extend_from_slice(&u32::MAX.to_le_bytes());
        assert_eq!(from_wkb(&truncated).unwrap_err(), WkbError::UnexpectedEof);
    }

    let mut deep = to_wkb(&Pt::new(0.0, 0.0), ByteOrder::LittleEndian);
    for _ in 0..129 {
        deep = little_endian_container(7, &deep);
    }
    assert_eq!(from_wkb(&deep).unwrap_err(), WkbError::NestingTooDeep);

    let errors = [
        WkbError::UnexpectedEof,
        WkbError::InvalidByteOrder(2),
        WkbError::UnknownGeometryType(8),
        WkbError::UnsupportedDimension,
        WkbError::TrailingBytes,
        WkbError::NestingTooDeep,
        WkbError::MismatchedMemberType {
            expected: 1,
            found: 2,
        },
    ];
    for error in errors {
        assert!(!error.to_string().is_empty());
    }
}

struct ExternalPointWriter;

impl Geometry for ExternalPointWriter {
    type Kind = PointTag;
    type Point = Pt;
}

impl WriteWkb for ExternalPointWriter {
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        let point = Pt::new(3.0, 4.0);
        out.extend(to_wkb(&point, order));
    }
}

#[test]
fn external_writer_uses_the_public_default_length_hint() {
    let bytes = to_wkb(&ExternalPointWriter, ByteOrder::LittleEndian);
    assert_eq!(from_wkb(&bytes), Ok(Dyn::Point(Pt::new(3.0, 4.0))));
}

// ---- Sweeps over every kind -------------------------------------------

const BOTH_ORDERS: [ByteOrder; 2] = [ByteOrder::LittleEndian, ByteOrder::BigEndian];

/// Every kind, with empty members, holes, and nesting — the corpus for
/// the truncation sweep and the length-hint check.
fn every_kind_corpus() -> Vec<Dyn> {
    let empty_polygon = Polygon::<Pt>::new(Ring::new());
    vec![
        Dyn::Point(Pt::new(1.5, -2.25)),
        Dyn::LineString(Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)])),
        Dyn::LineString(Linestring(vec![])),
        Dyn::Polygon(sample_polygon()),
        Dyn::Polygon(empty_polygon.clone()),
        Dyn::Polygon(Polygon::with_inners(Ring::new(), vec![sample_hole()])),
        Dyn::MultiPoint(MultiPoint(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)])),
        Dyn::MultiPoint(MultiPoint(vec![])),
        Dyn::MultiLineString(MultiLinestring(vec![
            Linestring(vec![]),
            Linestring(vec![Pt::new(10.0, 10.0), Pt::new(20.0, 20.0)]),
        ])),
        Dyn::MultiPolygon(MultiPolygon(vec![empty_polygon, sample_polygon()])),
        Dyn::GeometryCollection(vec![
            Dyn::Point(Pt::new(10.0, 10.0)),
            Dyn::LineString(Linestring(vec![])),
            Dyn::GeometryCollection(vec![
                Dyn::MultiPolygon(MultiPolygon(vec![sample_polygon()])),
                Dyn::GeometryCollection(vec![]),
            ]),
        ]),
        Dyn::GeometryCollection(vec![]),
    ]
}

/// Every proper prefix of a valid record is `Err` and never a panic, in
/// both byte orders; one extra byte is `TrailingBytes`.
#[test]
fn every_proper_prefix_is_an_error_and_never_a_panic() {
    for g in every_kind_corpus() {
        for order in BOTH_ORDERS {
            let bytes = to_wkb(&g, order);
            assert_eq!(from_wkb(&bytes), Ok(g.clone()), "{order:?} {g:?}");
            for end in 0..bytes.len() {
                assert!(
                    from_wkb(&bytes[..end]).is_err(),
                    "prefix of {end} bytes of {g:?} in {order:?} was accepted"
                );
            }
            let mut trailing = bytes.clone();
            trailing.push(0x00);
            assert_eq!(from_wkb(&trailing), Err(WkbError::TrailingBytes), "{g:?}");
        }
    }
}

/// OGC 06-103r4 §8.2: every nested record carries its own byte-order
/// flag, so a container may mix orders among its members. `to_wkb` only
/// ever writes one order, so these buffers are assembled by hand.
#[test]
fn nested_records_may_each_declare_their_own_byte_order() {
    let p1 = Pt::new(1.5, -2.25);
    let p2 = Pt::new(3.0, 4.0);

    // Big-endian MultiPoint header; members little- then big-endian.
    let mut mp = vec![0x00];
    mp.extend_from_slice(&4_u32.to_be_bytes());
    mp.extend_from_slice(&2_u32.to_be_bytes());
    mp.extend(to_wkb(&p1, ByteOrder::LittleEndian));
    mp.extend(to_wkb(&p2, ByteOrder::BigEndian));
    assert_eq!(from_wkb(&mp), Ok(Dyn::MultiPoint(MultiPoint(vec![p1, p2]))));

    // Little-endian MultiPolygon header with a big-endian member.
    let mut mpg = vec![0x01];
    mpg.extend_from_slice(&6_u32.to_le_bytes());
    mpg.extend_from_slice(&1_u32.to_le_bytes());
    mpg.extend(to_wkb(&sample_polygon(), ByteOrder::BigEndian));
    assert_eq!(
        from_wkb(&mpg),
        Ok(Dyn::MultiPolygon(MultiPolygon(vec![sample_polygon()])))
    );

    // Big-endian MultiLineString header with a little-endian member.
    let ls = Linestring(vec![p1, p2]);
    let mut mls = vec![0x00];
    mls.extend_from_slice(&5_u32.to_be_bytes());
    mls.extend_from_slice(&1_u32.to_be_bytes());
    mls.extend(to_wkb(&ls, ByteOrder::LittleEndian));
    assert_eq!(
        from_wkb(&mls),
        Ok(Dyn::MultiLineString(MultiLinestring(vec![ls.clone()])))
    );

    // Little-endian collection: big-endian linestring, little-endian
    // polygon, and a big-endian nested collection holding a
    // little-endian point.
    let mut gc = vec![0x01];
    gc.extend_from_slice(&7_u32.to_le_bytes());
    gc.extend_from_slice(&3_u32.to_le_bytes());
    gc.extend(to_wkb(&ls, ByteOrder::BigEndian));
    gc.extend(to_wkb(&sample_polygon(), ByteOrder::LittleEndian));
    gc.push(0x00);
    gc.extend_from_slice(&7_u32.to_be_bytes());
    gc.extend_from_slice(&1_u32.to_be_bytes());
    gc.extend(to_wkb(&p2, ByteOrder::LittleEndian));
    assert_eq!(
        from_wkb(&gc),
        Ok(Dyn::GeometryCollection(vec![
            Dyn::LineString(ls),
            Dyn::Polygon(sample_polygon()),
            Dyn::GeometryCollection(vec![Dyn::Point(p2)]),
        ]))
    );
}

/// `WriteWkb::wkb_len` promises the *exact* encoded length for the
/// built-in models; a wrong hint only changes allocation, so nothing but
/// this comparison would notice it.
#[test]
fn wkb_len_is_the_exact_encoded_length_for_every_kind() {
    for g in every_kind_corpus() {
        for order in BOTH_ORDERS {
            assert_eq!(g.wkb_len(), Some(to_wkb(&g, order).len()), "{g:?}");
        }
    }
    let ring = sample_ring();
    assert_eq!(
        ring.wkb_len(),
        Some(to_wkb(&ring, ByteOrder::LittleEndian).len())
    );
    let polygon = sample_polygon();
    assert_eq!(
        polygon.wkb_len(),
        Some(to_wkb_polygon(&polygon, ByteOrder::LittleEndian).len())
    );
    let mp = MultiPoint(vec![
        Pt::new(1.0, 2.0),
        Pt::new(3.0, 4.0),
        Pt::new(5.0, 6.0),
    ]);
    assert_eq!(mp.wkb_len(), Some(to_wkb(&mp, ByteOrder::BigEndian).len()));
    let mls = MultiLinestring(vec![
        Linestring(vec![Pt::new(1.0, 2.0)]),
        Linestring(vec![]),
    ]);
    assert_eq!(
        mls.wkb_len(),
        Some(to_wkb(&mls, ByteOrder::BigEndian).len())
    );
    let mpg = MultiPolygon(vec![sample_polygon(), Polygon::new(Ring::new())]);
    assert_eq!(
        mpg.wkb_len(),
        Some(to_wkb(&mpg, ByteOrder::BigEndian).len())
    );
}

/// A container header claiming `u32::MAX` members followed by a partial
/// member (16 bytes total) fails with `UnexpectedEof` — no reservation
/// proportional to the claimed count, in either order.
#[test]
fn hostile_member_counts_with_a_partial_body_fail_with_eof() {
    for code in [2_u32, 3, 4, 5, 6, 7] {
        for order in BOTH_ORDERS {
            let (flag, code_bytes, count_bytes) = match order {
                ByteOrder::LittleEndian => (0x01, code.to_le_bytes(), u32::MAX.to_le_bytes()),
                ByteOrder::BigEndian => (0x00, code.to_be_bytes(), u32::MAX.to_be_bytes()),
            };
            let mut bytes = vec![flag];
            bytes.extend_from_slice(&code_bytes);
            bytes.extend_from_slice(&count_bytes);
            bytes.extend_from_slice(&[0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
            assert_eq!(bytes.len(), 16);
            assert_eq!(
                from_wkb(&bytes),
                Err(WkbError::UnexpectedEof),
                "code {code} in {order:?}"
            );
        }
    }
    // A polygon whose ring count is hostile and whose first ring claims a
    // hostile point count too.
    let mut polygon = vec![0x01];
    polygon.extend_from_slice(&3_u32.to_le_bytes());
    polygon.extend_from_slice(&u32::MAX.to_le_bytes());
    polygon.extend_from_slice(&u32::MAX.to_le_bytes());
    assert_eq!(from_wkb(&polygon), Err(WkbError::UnexpectedEof));
}

/// Unknown and flagged type tags are rejected inside containers too, and
/// a zero tag is unknown rather than a dimension error.
#[test]
fn unknown_and_flagged_tags_inside_containers_are_rejected() {
    let mut zero = vec![0x01];
    zero.extend_from_slice(&0_u32.to_le_bytes());
    assert_eq!(from_wkb(&zero), Err(WkbError::UnknownGeometryType(0)));

    for (tag, want) in [
        (0x8000_0001_u32, WkbError::UnsupportedDimension),
        (0x2000_0001, WkbError::UnsupportedDimension),
        (1001, WkbError::UnsupportedDimension),
        (3007, WkbError::UnsupportedDimension),
        (8, WkbError::UnknownGeometryType(8)),
        (999, WkbError::UnknownGeometryType(999)),
    ] {
        let mut member = vec![0x01];
        member.extend_from_slice(&tag.to_le_bytes());
        member.extend_from_slice(&[0; 16]);
        assert_eq!(
            from_wkb(&little_endian_container(7, &member)),
            Err(want.clone()),
            "tag {tag:#x} inside a collection"
        );
        assert_eq!(
            from_wkb(&little_endian_container(4, &member)),
            Err(want),
            "tag {tag:#x} inside a multipoint"
        );
    }
}
