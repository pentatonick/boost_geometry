//! Public-facade parity tests for pairs of segments.
//!
//! Boost decides whether two segments meet, and where, with one kernel —
//! `cartesian_segments`, the strategy behind `intersects` and the meeting
//! point of `closest_points` — and pairs segments that do not meet by the
//! first of their nearest endpoint projections. Each case is a pair a
//! rounding error from meeting, or with several equally near pairs, with
//! the answer Boost (`aed7bc3`) gives for it.

use boost_geometry::algorithm::{closest_points, intersects};
use boost_geometry::model::{Point2D, Segment};
use boost_geometry::prelude::Cartesian;
use boost_geometry::trait_::Point as _;

type P = Point2D<f64, Cartesian>;

fn segment(a: (f64, f64), b: (f64, f64)) -> Segment<P> {
    Segment::new(P::new(a.0, a.1), P::new(b.0, b.1))
}

fn xy(point: P) -> (f64, f64) {
    (point.get::<0>(), point.get::<1>())
}

/// Two segments a rounding error from touching meet where Boost's side
/// tests, coincident-point tests and parallel test say they do.
#[test]
fn segments_meet_where_boost_says_they_do() {
    let apart = (
        segment(
            (-30.129_271_173_086_195, -44.173_245_779_471_73),
            (-6.447_719_017_463_655, -70.193_533_668_137_18),
        ),
        segment(
            (3.442_316_699_809_396, -81.060_286_611_589_4),
            (-31.269_992_775_753_5, -42.919_869_129_699_386),
        ),
    );
    assert!(!intersects(&apart.0, &apart.1));

    let touching = (
        segment(
            (90.466_768_562_406_68, 94.100_413_118_952_3),
            (-57.988_536_117_917_256, -58.102_128_795_509_33),
        ),
        segment(
            (112.772_731_007_931_75, 116.969_411_915_378_66),
            (-80.862_608_436_162_83, -81.553_577_420_191_77),
        ),
    );
    assert!(intersects(&touching.0, &touching.1));
}

/// Where segments meet, both closest points are the meeting point Boost
/// reports first: the crossing it interpolates, the start of a collinear
/// overlap, or a degenerate segment lying on the other.
#[test]
fn meeting_segments_pair_at_boosts_meeting_point() {
    let crossing = closest_points(
        &segment((3.0, -1.9), (2.7, 5.649_473_707_416_458)),
        &segment(
            (5.349_995_802_851_444, 2.109_247_894_604_216_4),
            (-1.0, -1.777_201_313_683_082_9),
        ),
    );
    let meeting = (2.900_261_392_996_614, 0.609_913_303_954_693_6);
    assert_eq!((xy(crossing.0), xy(crossing.1)), (meeting, meeting));

    let overlap = closest_points(
        &segment(
            (-0.174_177_642_793_646_56, -0.425_446_141_667_369_4),
            (-1.0, 2.0),
        ),
        &segment(
            (0.238_733_535_809_530_16, -1.638_169_212_501_054),
            (-0.380_633_232_095_234_9, 0.180_915_393_749_472_96),
        ),
    );
    let start = (-0.174_177_642_793_646_56, -0.425_446_141_667_369_4);
    assert_eq!((xy(overlap.0), xy(overlap.1)), (start, start));

    let on_segment = (21_162.152_781_271_303, 39_904.121_003_390_74);
    let point = closest_points(
        &segment(
            (-3_275.199_547_756_274, -48_095.631_798_032_13),
            (22_080.316_140_716_59, 43_210.459_086_773_626),
        ),
        &segment(on_segment, on_segment),
    );
    assert_eq!((xy(point.0), xy(point.1)), (on_segment, on_segment));
}

/// Segments that do not meet pair at the first of their nearest endpoint
/// projections, `b`'s endpoints tried before `a`'s, and a projection past
/// the end of a segment is that endpoint exactly.
#[test]
fn apart_segments_pair_at_boosts_nearest_projection() {
    let parallel = closest_points(
        &segment((-1.0, 2.0), (-4.6, 4.0)),
        &segment((-4.5, 3.0), (-8.1, 5.0)),
    );
    assert_eq!(
        (xy(parallel.0), xy(parallel.1)),
        (
            (-4.099_056_603_773_585, 3.721_698_113_207_547_3),
            (-4.5, 3.0)
        )
    );

    let clamped = closest_points(
        &P::new(-3.0, -3.182_050_717_668_332),
        &segment((5.6, 4.0), (1.3, -2.702_163_921_182_137_3)),
    );
    assert_eq!(xy(clamped.1), (1.3, -2.702_163_921_182_137_3));
}
