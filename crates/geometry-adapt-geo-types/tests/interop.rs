//! M-AD1 — `geo-types` interop validation milestone.
//!
//! Constructs the *same* polygon two ways — once as a
//! `geo_types::Polygon` wrapped in [`GeoPolygon`], once as a
//! `geometry_model::Polygon` — and confirms every v1 algorithm named in
//! the plan (§AD1, M-AD1) agrees between the two representations.
//!
//! # All algorithms are now trait-generic
//!
//! Every v1 algorithm is **trait-generic**: its per-kind strategy is
//! selected by a tag-keyed picker, so it
//! runs *directly on the [`GeoPolygon`] wrapper* (and on `GeoRing`,
//! `GeoLineString`, `GeoMultiPolygon`, …) and is compared against the
//! equivalent `geometry_model` value.
//!
//! * `distance`, `length` / `perimeter`, `area` were open from the start
//!   (each per-kind strategy lives on a distinct strategy struct).
//! * `within`, `covered_by`, `intersects`, `equals`, `envelope`,
//!   `centroid`, `num_points` were converted to the same open,
//!   tag-dispatched form: a per-kind (or per-pair) strategy struct plus a
//!   tag-keyed picker on `G::Kind` (see the per-algorithm coherence notes
//!   in `geometry_strategy::{within, intersects, equals, envelope,
//!   centroid}` and `geometry_algorithm::num_points`). The `*_direct_on_foreign`
//!   tests below pass a foreign value **directly** and assert equality
//!   with the model result.
//!
//! The older interop path — read the coordinates *out through the
//! wrapper's concept interface*, rebuild a `geometry_model::Polygon`, then
//! run the algorithm (`model_from_wrapper`) — is kept in the `*_matches`
//! tests as an additional cross-check that the wrapper surfaces exactly
//! the same coordinates, in the same order, as the native model.

#![allow(
    clippy::float_cmp,
    reason = "every compared value is an exact integer-valued result (3-4-5 distance, integer areas / bounds), so `==` is exact; approximate values use an explicit epsilon"
)]

use geo_types::{LineString, MultiPolygon as GtMultiPolygon, Polygon as GtPolygon, coord};
use geometry_adapt_geo_types::{GeoCoord, GeoLineString, GeoMultiPolygon, GeoPolygon, GeoRing};
use geometry_algorithm::{
    area, centroid, covered_by, distance, envelope, equals, intersects, num_points, perimeter,
    ring_area, ring_perimeter, within,
};
use geometry_cs::Cartesian;
use geometry_model::{
    Linestring, MultiPolygon as ModelMultiPolygon, Point2D, Polygon as ModelPolygon,
    Ring as ModelRing, linestring,
};
use geometry_trait::{IndexedAccess as _, Point as _, Polygon as _};

type P = Point2D<f64, Cartesian>;

/// The exterior ring: a 4x4 square, closed, clockwise-declared.
const EXTERIOR: &[(f64, f64)] = &[(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0), (0.0, 0.0)];

/// One interior ring (hole): a unit square near the centre, wound
/// opposite the exterior so its signed area subtracts.
const HOLE: &[(f64, f64)] = &[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)];

/// Build the polygon as a wrapped `geo_types::Polygon`.
fn geo_polygon() -> GeoPolygon<f64> {
    let exterior = LineString::from(EXTERIOR.to_vec());
    let hole = LineString::from(HOLE.to_vec());
    GeoPolygon::new(GtPolygon::new(exterior, vec![hole]))
}

/// Build the identical polygon as a `geometry_model::Polygon`.
fn model_polygon() -> ModelPolygon<P> {
    let exterior = ModelRing::from_vec(EXTERIOR.iter().map(|&(x, y)| Point2D::new(x, y)).collect());
    let hole = ModelRing::from_vec(HOLE.iter().map(|&(x, y)| Point2D::new(x, y)).collect());
    ModelPolygon::with_inners(exterior, vec![hole])
}

/// Reconstruct a `geometry_model::Polygon` by reading coordinates back
/// *through the wrapper's concept interface* — the interop path used to
/// feed the model-bound algorithms. If this equals [`model_polygon`],
/// the wrapper surfaces the same coordinates the native model does.
fn model_from_wrapper(g: &GeoPolygon<f64>) -> ModelPolygon<P> {
    fn ring_of<R>(r: &R) -> ModelRing<P>
    where
        R: geometry_trait::Ring<Point = GeoCoord<f64>>,
    {
        ModelRing::from_vec(
            r.points()
                .map(|c| Point2D::new(c.get::<0>(), c.get::<1>()))
                .collect(),
        )
    }
    let exterior = ring_of(g.exterior());
    let inners = g.interiors().map(ring_of).collect();
    ModelPolygon::with_inners(exterior, inners)
}

const EPS: f64 = 1e-12;

// ---- Trait-generic algorithms: run directly on the wrapper ----------

#[test]
fn area_matches() {
    let got = area(&geo_polygon());
    let want = area(&model_polygon());
    assert!((got - want).abs() < EPS, "area: {got} vs {want}");
    // 4x4 exterior (16) minus the unit-square hole (1) = 15.
    assert!((got - 15.0).abs() < EPS, "area value: {got}");
}

#[test]
fn ring_area_matches() {
    // Exterior ring only, both representations.
    let geo = geo_polygon();
    let model = model_polygon();
    let got = ring_area(geo.exterior());
    let want = ring_area(&model.outer);
    assert!((got - want).abs() < EPS, "ring_area: {got} vs {want}");
    assert!((got - 16.0).abs() < EPS, "ring_area value: {got}");
}

#[test]
fn perimeter_matches() {
    let got = perimeter(&geo_polygon());
    let want = perimeter(&model_polygon());
    assert!((got - want).abs() < EPS, "perimeter: {got} vs {want}");
    // 4x4 exterior (16) + unit-square hole (4) = 20.
    assert!((got - 20.0).abs() < EPS, "perimeter value: {got}");
}

#[test]
fn ring_perimeter_matches() {
    let geo = geo_polygon();
    let model = model_polygon();
    let got = ring_perimeter(geo.exterior());
    let want = ring_perimeter(&model.outer);
    assert!((got - want).abs() < EPS, "ring_perimeter: {got} vs {want}");
    assert!((got - 16.0).abs() < EPS, "ring_perimeter value: {got}");
}

#[test]
fn distance_between_wrapper_vertices_matches_model() {
    // Point-to-point distance is trait-generic, so it runs on the
    // wrapper's point type directly.
    let a = GeoCoord::new(geo_types::coord! { x: 0.0_f64, y: 0.0 });
    let b = GeoCoord::new(geo_types::coord! { x: 3.0_f64, y: 4.0 });
    let ma = Point2D::<f64, Cartesian>::new(0.0, 0.0);
    let mb = Point2D::<f64, Cartesian>::new(3.0, 4.0);
    assert_eq!(distance(&a, &b), distance(&ma, &mb));
    assert_eq!(distance(&a, &b), 5.0);
}

// ---- Model-bound algorithms: run via the interop reconstruction -----

#[test]
fn reconstruction_equals_native_model() {
    // The interop reconstruction must be coordinate-identical to the
    // native model — this is what licenses every model-bound assertion
    // below.
    assert_eq!(model_from_wrapper(&geo_polygon()), model_polygon());
}

#[test]
fn envelope_matches() {
    let from_wrapper = envelope(&model_from_wrapper(&geo_polygon()));
    let native = envelope(&model_polygon());
    for i in 0..2 {
        for d in 0..2 {
            let a = match (i, d) {
                (0, 0) => from_wrapper.get_indexed::<0, 0>(),
                (0, 1) => from_wrapper.get_indexed::<0, 1>(),
                (1, 0) => from_wrapper.get_indexed::<1, 0>(),
                _ => from_wrapper.get_indexed::<1, 1>(),
            };
            let b = match (i, d) {
                (0, 0) => native.get_indexed::<0, 0>(),
                (0, 1) => native.get_indexed::<0, 1>(),
                (1, 0) => native.get_indexed::<1, 0>(),
                _ => native.get_indexed::<1, 1>(),
            };
            assert_eq!(a.to_bits(), b.to_bits(), "envelope corner ({i},{d})");
        }
    }
    // The 4x4 exterior bounds: (0,0)-(4,4).
    assert_eq!(native.get_indexed::<0, 0>(), 0.0);
    assert_eq!(native.get_indexed::<1, 0>(), 4.0);
}

/// Open tag-dispatch: `envelope` runs *directly* on the foreign
/// `GeoPolygon` (its box is a `Box<GeoCoord>`) and the corners match the
/// native model's `Box<Point2D>` coordinate-for-coordinate.
#[test]
fn envelope_direct_on_foreign() {
    let foreign = envelope(&geo_polygon());
    let native = envelope(&model_polygon());
    assert_eq!(
        foreign.get_indexed::<0, 0>().to_bits(),
        native.get_indexed::<0, 0>().to_bits(),
    );
    assert_eq!(
        foreign.get_indexed::<0, 1>().to_bits(),
        native.get_indexed::<0, 1>().to_bits(),
    );
    assert_eq!(
        foreign.get_indexed::<1, 0>().to_bits(),
        native.get_indexed::<1, 0>().to_bits(),
    );
    assert_eq!(
        foreign.get_indexed::<1, 1>().to_bits(),
        native.get_indexed::<1, 1>().to_bits(),
    );

    // A foreign multipolygon envelopes to the same bounds as the model
    // multipolygon (the extra member is a shifted copy, widening x by 10).
    let foreign_multi = GeoMultiPolygon::new(GtMultiPolygon::new(vec![
        GtPolygon::new(LineString::from(EXTERIOR.to_vec()), vec![]),
        GtPolygon::new(
            LineString::from(
                EXTERIOR
                    .iter()
                    .map(|&(x, y)| (x + 10.0, y))
                    .collect::<Vec<_>>(),
            ),
            vec![],
        ),
    ]));
    let model_multi: ModelMultiPolygon<ModelPolygon<P>> = ModelMultiPolygon(vec![
        ModelPolygon::with_inners(
            ModelRing::from_vec(EXTERIOR.iter().map(|&(x, y)| Point2D::new(x, y)).collect()),
            vec![],
        ),
        ModelPolygon::with_inners(
            ModelRing::from_vec(
                EXTERIOR
                    .iter()
                    .map(|&(x, y)| Point2D::new(x + 10.0, y))
                    .collect(),
            ),
            vec![],
        ),
    ]);
    let fm = envelope(&foreign_multi);
    let nm = envelope(&model_multi);
    assert_eq!(
        fm.get_indexed::<0, 0>().to_bits(),
        nm.get_indexed::<0, 0>().to_bits()
    );
    assert_eq!(
        fm.get_indexed::<1, 0>().to_bits(),
        nm.get_indexed::<1, 0>().to_bits()
    );
    assert_eq!(nm.get_indexed::<1, 0>(), 14.0); // max x = 4 + 10
}

#[test]
fn centroid_matches() {
    let from_wrapper = centroid(&model_from_wrapper(&geo_polygon()));
    let native = centroid(&model_polygon());
    assert_eq!(
        from_wrapper.get::<0>().to_bits(),
        native.get::<0>().to_bits()
    );
    assert_eq!(
        from_wrapper.get::<1>().to_bits(),
        native.get::<1>().to_bits()
    );
    // Area-weighted: exterior (area 16, centroid (2,2)) minus the hole
    // (area 1, centroid (1.5,1.5)) → (16*2 - 1*1.5)/(16 - 1) = 30.5/15.
    let expected = 30.5 / 15.0;
    assert!(
        (native.get::<0>() - expected).abs() < EPS,
        "cx {}",
        native.get::<0>()
    );
    assert!(
        (native.get::<1>() - expected).abs() < EPS,
        "cy {}",
        native.get::<1>()
    );
}

/// Open tag-dispatch: `centroid` runs
/// *directly* on the foreign `GeoPolygon` — no rebuilt model value — and
/// matches the native model's centroid coordinate-for-coordinate.
#[test]
fn centroid_direct_on_foreign() {
    let foreign = centroid(&geo_polygon());
    let native = centroid(&model_polygon());
    assert_eq!(foreign.get::<0>().to_bits(), native.get::<0>().to_bits());
    assert_eq!(foreign.get::<1>().to_bits(), native.get::<1>().to_bits());
}

/// Open tag-dispatch: `num_points` runs *directly* on the foreign
/// `GeoPolygon` (and on a foreign `GeoMultiPolygon`) and matches the
/// native model count. The polygon is a 5-point exterior + 5-point hole.
#[test]
fn num_points_direct_on_foreign() {
    assert_eq!(num_points(&geo_polygon()), num_points(&model_polygon()));
    assert_eq!(num_points(&geo_polygon()), 10);

    // A foreign multipolygon of two copies → 20; identical to the model.
    let foreign_multi = GeoMultiPolygon::new(GtMultiPolygon::new(vec![
        GtPolygon::new(
            LineString::from(EXTERIOR.to_vec()),
            vec![LineString::from(HOLE.to_vec())],
        ),
        GtPolygon::new(
            LineString::from(EXTERIOR.to_vec()),
            vec![LineString::from(HOLE.to_vec())],
        ),
    ]));
    let model_multi = ModelMultiPolygon(vec![model_polygon(), model_polygon()]);
    assert_eq!(num_points(&foreign_multi), num_points(&model_multi));
    assert_eq!(num_points(&foreign_multi), 20);
}

#[test]
fn within_matches() {
    let from_wrapper = model_from_wrapper(&geo_polygon());
    let native = model_polygon();
    // A point inside the ring of material (not in the hole).
    let inside = Point2D::<f64, Cartesian>::new(0.5, 0.5);
    // A point in the hole (outside the polygon's interior material).
    let in_hole = Point2D::<f64, Cartesian>::new(1.5, 1.5);
    // A point well outside the exterior.
    let outside = Point2D::<f64, Cartesian>::new(10.0, 10.0);

    assert_eq!(within(&inside, &from_wrapper), within(&inside, &native));
    assert_eq!(within(&in_hole, &from_wrapper), within(&in_hole, &native));
    assert_eq!(within(&outside, &from_wrapper), within(&outside, &native));

    assert!(within(&inside, &native));
    assert!(!within(&in_hole, &native));
    assert!(!within(&outside, &native));
}

/// Open tag-dispatch: `within`/`covered_by` run *directly* on a foreign
/// `GeoRing` and `GeoPolygon` (query point is a `GeoCoord`, matching the
/// container's point), covering the interior, boundary, and hole cases.
#[test]
fn within_covered_by_direct_on_foreign() {
    // Foreign 4x4 ring (the polygon's exterior).
    let foreign_ring = GeoRing::new(LineString::from(EXTERIOR.to_vec()));
    let inside = GeoCoord::new(coord! { x: 0.5_f64, y: 0.5 });
    let corner = GeoCoord::new(coord! { x: 0.0_f64, y: 0.0 });
    let outside = GeoCoord::new(coord! { x: 10.0_f64, y: 10.0 });

    // Interior: within and covered_by both true.
    assert!(within(&inside, &foreign_ring));
    assert!(covered_by(&inside, &foreign_ring));
    // Boundary corner: within false, covered_by true.
    assert!(!within(&corner, &foreign_ring));
    assert!(covered_by(&corner, &foreign_ring));
    // Exterior: both false.
    assert!(!within(&outside, &foreign_ring));
    assert!(!covered_by(&outside, &foreign_ring));

    // Foreign polygon-with-hole: a point in the hole is neither within
    // nor covered_by; a point in the material is both.
    let foreign_pg = geo_polygon();
    let in_hole = GeoCoord::new(coord! { x: 1.5_f64, y: 1.5 });
    assert!(within(&inside, &foreign_pg));
    assert!(covered_by(&inside, &foreign_pg));
    assert!(!within(&in_hole, &foreign_pg));
    assert!(!covered_by(&in_hole, &foreign_pg));

    // Cross-check equality with the native model result.
    let native = model_polygon();
    let m_inside = Point2D::<f64, Cartesian>::new(0.5, 0.5);
    let m_in_hole = Point2D::<f64, Cartesian>::new(1.5, 1.5);
    assert_eq!(within(&inside, &foreign_pg), within(&m_inside, &native));
    assert_eq!(
        covered_by(&in_hole, &foreign_pg),
        covered_by(&m_in_hole, &native)
    );
}

#[test]
fn intersects_matches() {
    let from_wrapper = model_from_wrapper(&geo_polygon());
    let native = model_polygon();
    let inside = Point2D::<f64, Cartesian>::new(0.5, 0.5);
    let outside = Point2D::<f64, Cartesian>::new(10.0, 10.0);

    assert_eq!(
        intersects(&inside, &from_wrapper),
        intersects(&inside, &native),
    );
    assert_eq!(
        intersects(&outside, &from_wrapper),
        intersects(&outside, &native),
    );
    assert!(intersects(&inside, &native));
    assert!(!intersects(&outside, &native));
}

/// Open tag-dispatch: `intersects` runs *directly* on foreign
/// `GeoLineString × GeoPolygon` and foreign `GeoPolygon × GeoPolygon`,
/// agreeing with the model result on identical coordinates.
#[test]
fn intersects_direct_on_foreign() {
    // Linestring crossing into the polygon material (from outside a
    // corner to a point inside the 4x4 exterior, avoiding the hole).
    let foreign_ls = GeoLineString::new(LineString::from(vec![(-1.0, -1.0), (0.5, 0.5)]));
    let foreign_pg = geo_polygon();
    let model_ls: Linestring<P> = linestring![(-1.0, -1.0), (0.5, 0.5)];
    let model_pg = model_polygon();
    assert_eq!(
        intersects(&foreign_ls, &foreign_pg),
        intersects(&model_ls, &model_pg),
    );
    assert!(intersects(&foreign_ls, &foreign_pg));

    // A linestring entirely outside does not intersect.
    let outside_ls = GeoLineString::new(LineString::from(vec![(20.0, 20.0), (30.0, 30.0)]));
    assert!(!intersects(&outside_ls, &foreign_pg));

    // Foreign polygon × foreign polygon: two overlapping squares.
    let a = GeoPolygon::new(GtPolygon::new(LineString::from(EXTERIOR.to_vec()), vec![]));
    let b = GeoPolygon::new(GtPolygon::new(
        LineString::from(vec![
            (2.0, 2.0),
            (6.0, 2.0),
            (6.0, 6.0),
            (2.0, 6.0),
            (2.0, 2.0),
        ]),
        vec![],
    ));
    let ma: ModelPolygon<P> = ModelPolygon::with_inners(
        ModelRing::from_vec(EXTERIOR.iter().map(|&(x, y)| Point2D::new(x, y)).collect()),
        vec![],
    );
    let mb: ModelPolygon<P> = ModelPolygon::with_inners(
        ModelRing::from_vec(
            [(2.0, 2.0), (6.0, 2.0), (6.0, 6.0), (2.0, 6.0), (2.0, 2.0)]
                .iter()
                .map(|&(x, y)| Point2D::new(x, y))
                .collect(),
        ),
        vec![],
    );
    assert_eq!(intersects(&a, &b), intersects(&ma, &mb));
    assert!(intersects(&a, &b));
}

#[test]
fn equals_matches() {
    let from_wrapper = model_from_wrapper(&geo_polygon());
    let native = model_polygon();
    // The reconstruction equals the native polygon geometrically.
    assert!(equals(&from_wrapper, &native));
    // And a translated copy does not.
    let shifted = ModelPolygon::with_inners(
        ModelRing::from_vec(
            EXTERIOR
                .iter()
                .map(|&(x, y)| Point2D::new(x + 100.0, y))
                .collect(),
        ),
        vec![],
    );
    assert!(!equals(&native, &shifted));
}

/// Open tag-dispatch: `equals` runs *directly* between two foreign
/// `GeoPolygon`s (no rebuilt model value) and agrees with the model
/// result computed from identical coordinates. One operand is the same
/// loop rotated to a different starting vertex — equal up to rotation.
#[test]
fn equals_direct_on_foreign() {
    let foreign = geo_polygon();
    // Same exterior loop, rotated start; drop the hole for a clean
    // rotation comparison (both foreign polygons hole-free).
    let ext_no_hole =
        |verts: Vec<(f64, f64)>| GeoPolygon::new(GtPolygon::new(LineString::from(verts), vec![]));
    let foreign_a = ext_no_hole(EXTERIOR.to_vec());
    // rotate EXTERIOR (a closed 5-vertex ring) by one vertex
    let rotated: Vec<(f64, f64)> = vec![
        EXTERIOR[1],
        EXTERIOR[2],
        EXTERIOR[3],
        EXTERIOR[0],
        EXTERIOR[1],
    ];
    let foreign_b = ext_no_hole(rotated);
    assert!(equals(&foreign_a, &foreign_b));

    // Cross-check with the model: same rotation is equal there too.
    let model_a: ModelPolygon<P> = ModelPolygon::with_inners(
        ModelRing::from_vec(EXTERIOR.iter().map(|&(x, y)| Point2D::new(x, y)).collect()),
        vec![],
    );
    let model_b: ModelPolygon<P> = ModelPolygon::with_inners(
        ModelRing::from_vec(
            [
                EXTERIOR[1],
                EXTERIOR[2],
                EXTERIOR[3],
                EXTERIOR[0],
                EXTERIOR[1],
            ]
            .iter()
            .map(|&(x, y)| Point2D::new(x, y))
            .collect(),
        ),
        vec![],
    );
    assert_eq!(equals(&foreign_a, &foreign_b), equals(&model_a, &model_b));

    // A shifted foreign polygon is not equal.
    let foreign_shifted = ext_no_hole(EXTERIOR.iter().map(|&(x, y)| (x + 100.0, y)).collect());
    assert!(!equals(&foreign_a, &foreign_shifted));

    // Silence unused: the full polygon-with-hole still resolves through
    // the picker directly on the foreign value.
    assert!(equals(&foreign, &foreign));
}
