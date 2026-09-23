//! Header contracts for external writers and malformed input.

use geometry_cs::Cartesian;
use geometry_io_ewkb::{ByteOrder, EwkbError, Srid, WkbError, WriteWkb, from_ewkb, to_ewkb};
use geometry_model::{DynGeometry, Point2D};
use geometry_tag::PointTag;
use geometry_trait::Geometry;

type Pt = Point2D<f64, Cartesian>;

struct UnhintedPoint(Pt);

impl Geometry for UnhintedPoint {
    type Kind = PointTag;
    type Point = Pt;
}

impl WriteWkb for UnhintedPoint {
    fn write_wkb(&self, order: ByteOrder, out: &mut Vec<u8>) {
        self.0.write_wkb(order, out);
    }
}

#[test]
fn external_writer_needs_no_length_hint() {
    let point = Pt::new(1.5, -2.25);
    let unhinted = UnhintedPoint(point);
    assert_eq!(unhinted.wkb_len(), None);
    for order in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
        for srid in [None, Some(Srid::UNKNOWN), Some(Srid::new(4326))] {
            assert_eq!(
                to_ewkb(&unhinted, srid, order),
                to_ewkb(&point, srid, order)
            );
        }
    }
}

struct MissingRecord;

impl Geometry for MissingRecord {
    type Kind = PointTag;
    type Point = Pt;
}

impl WriteWkb for MissingRecord {
    fn write_wkb(&self, _: ByteOrder, _: &mut Vec<u8>) {}
}

#[test]
#[should_panic(expected = "write_ogc_record appends a complete WKB record")]
fn external_writer_must_append_a_header_for_srid_insertion() {
    let _ = to_ewkb(&MissingRecord, Some(Srid::UNKNOWN), ByteOrder::LittleEndian);
}

#[test]
fn every_order_flag_preserves_the_header_error() {
    for flag in 0..=u8::MAX {
        let expected = if flag <= 1 {
            WkbError::UnexpectedEof
        } else {
            WkbError::InvalidByteOrder(flag)
        };
        for len in 1..5 {
            let mut input = vec![0; len];
            input[0] = flag;
            assert_eq!(from_ewkb(&input), Err(EwkbError::Wkb(expected.clone())));
        }
    }
    assert_eq!(from_ewkb(&[]), Err(EwkbError::Wkb(WkbError::UnexpectedEof)));
}

#[test]
fn flag_precedence_and_srid_truncation_hold_in_both_orders() {
    for order in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
        for flags in 0_u32..16 {
            let word = (flags << 28) | 1;
            for tail_len in 0..4 {
                let mut input = vec![u8::from(order == ByteOrder::LittleEndian)];
                input.extend_from_slice(&order.to_bytes(word));
                input.extend_from_slice(&[0; 3][..tail_len]);
                let expected = if word & 0x1000_0000 != 0 {
                    EwkbError::BoundingBoxFlag { type_word: word }
                } else if word & 0xC000_0000 != 0 {
                    EwkbError::DimensionFlag { type_word: word }
                } else if word & 0x2000_0000 != 0 {
                    EwkbError::TruncatedSrid { type_word: word }
                } else {
                    EwkbError::Wkb(WkbError::UnexpectedEof)
                };
                assert_eq!(from_ewkb(&input), Err(expected));
            }
        }
    }
}

#[test]
fn mixed_endian_members_remain_readable_under_an_srid_header() {
    let point = Pt::new(1.5, -2.25);
    for (outer, inner) in [
        (ByteOrder::LittleEndian, ByteOrder::BigEndian),
        (ByteOrder::BigEndian, ByteOrder::LittleEndian),
    ] {
        let mut bytes = vec![u8::from(outer == ByteOrder::LittleEndian)];
        bytes.extend_from_slice(&outer.to_bytes(0x2000_0007));
        bytes.extend_from_slice(&outer.to_bytes(4326));
        bytes.extend_from_slice(&outer.to_bytes(1));
        bytes.extend(to_ewkb(&point, None, inner));
        let read = from_ewkb(&bytes).unwrap();
        assert_eq!(read.srid, Some(Srid::new(4326)));
        assert_eq!(read.byte_order, outer);
        assert_eq!(
            read.geometry,
            DynGeometry::GeometryCollection(vec![DynGeometry::Point(point)])
        );
        let normalized = to_ewkb(&read.geometry, read.srid, read.byte_order);
        assert_ne!(normalized, bytes);
        assert_eq!(from_ewkb(&normalized).unwrap(), read);
    }
}

#[test]
fn srid_header_does_not_reset_the_nesting_limit() {
    for order in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
        for collection_count in [127, 128] {
            let mut input = vec![u8::from(order == ByteOrder::LittleEndian)];
            input.extend_from_slice(&order.to_bytes(0x2000_0007));
            input.extend_from_slice(&order.to_bytes(4326));
            input.extend_from_slice(&order.to_bytes(1));
            for _ in 1..collection_count {
                input.push(u8::from(order == ByteOrder::LittleEndian));
                input.extend_from_slice(&order.to_bytes(7));
                input.extend_from_slice(&order.to_bytes(1));
            }
            input.extend(to_ewkb(&Pt::new(1.0, 2.0), None, order));
            let result = from_ewkb(&input);
            if collection_count == 127 {
                assert!(result.is_ok());
            } else {
                assert_eq!(result, Err(EwkbError::Wkb(WkbError::NestingTooDeep)));
            }
        }
    }
}
