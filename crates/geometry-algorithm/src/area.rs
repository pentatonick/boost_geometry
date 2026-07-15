//! `area(&g)` — see `boost/geometry/algorithms/area.hpp`.
//!
//! Cartesian-only in v1; spherical / geographic area strategies arrive
//! alongside the Haversine / Andoyer / Vincenty work in later tasks.
//!
//! # Why four entry points
//!
//! Boost overloads on the same name `area` and resolves the right
//! per-tag `dispatch::area` arm at the call site
//! (`algorithms/area.hpp:149-187`). Rust has no overloading, and the
//! Cartesian strategy in `geometry-strategy::area` is intentionally
//! split into four sibling unit-structs (one per geometry kind) to
//! sidestep coherence — see that module's docs. The split is
//! reflected here as four entry points keyed on the input geometry's
//! kind; the names follow the same `<kind>_area` shape that
//! `length` / `perimeter` already use for the same reason
//! (`crates/geometry-algorithm/src/length.rs`).

use geometry_cs::CoordinateSystem;
use geometry_strategy::{
    AreaStrategy, DefaultArea, DefaultAreaStrategy, ShoelaceArea, ShoelaceBoxArea,
    ShoelaceMultiPolygonArea,
};
use geometry_trait::{Box, Geometry, MultiPolygon, Point, Polygon, Ring};

/// Shorthand for the CS family of `G`'s point type. Keeps the `where`
/// clauses readable; matches the projection in
/// [`geometry_strategy::DefaultAreaStrategy`].
type Family<G> = <<<G as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family;

/// Signed area of a [`Ring`] via the Cartesian shoelace formula.
///
/// Mirrors `boost::geometry::area(ring)` from
/// `boost/geometry/algorithms/area.hpp` resolved through the
/// `dispatch::area<Ring, ring_tag>` arm at
/// `algorithms/area.hpp:154-157`.
///
/// # Sign convention
///
/// Follows Boost: rings traversed in their declared
/// [`geometry_trait::PointOrder`] produce a positive area,
/// rings traversed in the opposite direction produce a negative area
/// (`test/algorithms/area/area.cpp:63-64`).
#[inline]
#[must_use]
pub fn ring_area<R>(r: &R) -> <ShoelaceArea as AreaStrategy<R>>::Out
where
    R: Ring,
    ShoelaceArea: AreaStrategy<R>,
{
    ShoelaceArea.area(r)
}

/// Signed area of a [`Polygon`]: exterior-ring area plus the sum of
/// interior-ring areas (interior rings are conventionally wound
/// opposite the exterior, so their signed area already cancels).
///
/// Mirrors `boost::geometry::area(polygon)` from
/// `boost/geometry/algorithms/area.hpp` resolved through the
/// `dispatch::area<Polygon, polygon_tag>` arm at
/// `algorithms/area.hpp:160-172`.
///
/// The area is computed with the default strategy for the polygon's
/// coordinate-system family via
/// [`geometry_strategy::DefaultAreaStrategy`] — Cartesian input
/// resolves to the shoelace
/// [`geometry_strategy::ShoelacePolygonArea`] (identical to the v1
/// behaviour), spherical to the spherical-excess
/// [`geometry_strategy::SphericalPolygonArea`], geographic to the
/// authalic-sphere [`geometry_strategy::GeographicPolygonArea`]. For an
/// explicit strategy use [`area_with`].
///
/// # Behaviour on "wrong" kinds
///
/// This *static* entry point requires a `Polygon`; a `Point` or
/// `LineString` argument is a compile error. Boost's runtime "area of
/// a non-areal kind is 0" contract is honoured on the *dynamic* path:
/// [`crate::area_dyn`] returns `0` for `Point`,
/// `LineString`, `MultiPoint`, … (the static/dynamic split is the
/// same coherence workaround described on [`length`](fn@crate::length)).
#[inline]
#[must_use]
pub fn area<P>(p: &P) -> <DefaultAreaStrategy<P> as AreaStrategy<P>>::Out
where
    P: Polygon,
    Family<P>: DefaultArea<Family<P>>,
    DefaultAreaStrategy<P>: AreaStrategy<P> + Default,
{
    DefaultAreaStrategy::<P>::default().area(p)
}

/// Area of a polygon using an explicitly supplied strategy.
///
/// Mirrors the `area(g, strategy)` overload at
/// `boost/geometry/algorithms/area.hpp`. Taking the strategy by value
/// matches the by-value call shape of [`crate::distance_with`];
/// concrete strategies are zero-sized or small `Copy` configuration
/// objects, so this monomorphises into nothing.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "Strategies are ZST/small Copy configuration objects; by-value matches the user-facing call shape."
)]
pub fn area_with<G, S>(g: &G, s: S) -> S::Out
where
    G: Geometry,
    S: AreaStrategy<G>,
{
    s.area(g)
}

/// Area of an axis-aligned [`Box`]: `(xmax - xmin) * (ymax - ymin)`.
///
/// Mirrors `boost::geometry::area(box)` from
/// `boost/geometry/algorithms/area.hpp` resolved through the
/// `dispatch::area<Box, box_tag>` arm at
/// `algorithms/area.hpp:149-151`. Always non-negative — the Cartesian
/// box formula is sign-blind to corner ordering
/// (`test/algorithms/area/area.cpp:56-57`).
#[inline]
#[must_use]
pub fn box_area<B>(b: &B) -> <ShoelaceBoxArea as AreaStrategy<B>>::Out
where
    B: Box,
    ShoelaceBoxArea: AreaStrategy<B>,
{
    ShoelaceBoxArea.area(b)
}

/// Signed area of a [`MultiPolygon`]: sum of the signed areas of its
/// member polygons.
///
/// Mirrors `boost::geometry::area(multi_polygon)` from
/// `boost/geometry/algorithms/area.hpp` resolved through the
/// `dispatch::area<MultiGeometry, multi_polygon_tag>` arm at
/// `algorithms/area.hpp:175-187`.
#[inline]
#[must_use]
pub fn multi_polygon_area<MPg>(mpg: &MPg) -> <ShoelaceMultiPolygonArea as AreaStrategy<MPg>>::Out
where
    MPg: MultiPolygon,
    ShoelaceMultiPolygonArea: AreaStrategy<MPg>,
{
    ShoelaceMultiPolygonArea.area(mpg)
}

#[cfg(test)]
mod tests {
    //! Reference values from `geometry/test/algorithms/area/area.cpp`
    //! (lines 45-64) and `area_sph_geo.cpp` / `area_geo.cpp` for the
    //! spherical / geographic dispatch cases; see also the quickstart
    //! `Area: 3.015` example.
    #![allow(
        clippy::float_cmp,
        reason = "areas are compared with an explicit tolerance, not `==`"
    )]

    use super::{area, area_with, box_area, multi_polygon_area, ring_area};
    use geometry_cs::Cartesian;
    use geometry_model::{Box, MultiPolygon, Point2D, Polygon, Ring, polygon};

    type P = Point2D<f64, Cartesian>;

    /// `area.cpp:45` — rotated unit square diamond, area = 2.
    #[test]
    fn diamond_polygon_is_2() {
        let p: Polygon<P> = polygon![[(1.0, 1.0), (2.0, 2.0), (3.0, 1.0), (2.0, 0.0), (1.0, 1.0)]];
        assert!((area(&p) - 2.0).abs() < 1e-12);
    }

    /// `area.cpp:47` — pentagon, area = 16.
    #[test]
    fn pentagon_is_16() {
        let p: Polygon<P> = polygon![[(0.0, 0.0), (0.0, 7.0), (4.0, 2.0), (2.0, 0.0), (0.0, 0.0)]];
        assert!((area(&p) - 16.0).abs() < 1e-12);
    }

    /// `area.cpp:48` — unit-square CCW on default-CW polygon → -1.
    #[test]
    fn ccw_unit_square_is_minus_1() {
        let p: Polygon<P> = polygon![[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)]];
        assert!((area(&p) - -1.0).abs() < 1e-12);
    }

    /// `area.cpp:49` — pentagon (16) with a unit-square hole (-1) = 15.
    #[test]
    fn pentagon_with_hole_is_15() {
        let p: Polygon<P> = polygon![
            [(0.0, 0.0), (0.0, 7.0), (4.0, 2.0), (2.0, 0.0), (0.0, 0.0)],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)],
        ];
        assert!((area(&p) - 15.0).abs() < 1e-12);
    }

    /// `area.cpp:56-57` — a 2x2 box has area 4, regardless of corner
    /// order.
    #[test]
    fn box_2x2_is_4() {
        let b = Box::from_corners(
            Point2D::<f64, Cartesian>::new(0.0, 0.0),
            Point2D::<f64, Cartesian>::new(2.0, 2.0),
        );
        assert!((box_area(&b) - 4.0).abs() < 1e-12);
    }

    /// `area.cpp:63` — ring directly (no polygon wrapper), area = 16.
    #[test]
    fn ring_pentagon_is_16() {
        let r: Ring<P> = Ring::from_vec(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(0.0, 7.0),
            Point2D::new(4.0, 2.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(0.0, 0.0),
        ]);
        assert!((ring_area(&r) - 16.0).abs() < 1e-12);
    }

    /// `doc/quickstart.qbk` — the canonical "Area: 3.015" example.
    #[test]
    fn quickstart_polygon_area_is_3_015() {
        let p: Polygon<P> = polygon![[(2.0, 1.3), (4.1, 3.0), (5.3, 2.6), (2.9, 0.7), (2.0, 1.3)]];
        assert!((area(&p) - 3.015).abs() < 1e-3);
    }

    /// Multi-polygon: two disjoint CW unit squares → area = 2.
    #[test]
    fn multi_polygon_two_unit_squares_is_2() {
        let unit_at = |x: f64, y: f64| -> Polygon<P> {
            polygon![[
                (x, y),
                (x, y + 1.0),
                (x + 1.0, y + 1.0),
                (x + 1.0, y),
                (x, y)
            ]]
        };
        let mpg: MultiPolygon<Polygon<P>> =
            MultiPolygon::from_vec(vec![unit_at(0.0, 0.0), unit_at(5.0, 0.0)]);
        assert!((multi_polygon_area(&mpg) - 2.0).abs() < 1e-12);
    }

    /// `area_sph_geo.cpp:93-106` — strategy-less `area` on a spherical
    /// polygon resolves to `SphericalPolygonArea`; `POLYGON((0 0,0 90,
    /// 90 0,0 0))` on the default (Earth) sphere covers `1/8` of it,
    /// i.e. `4π/8 · R²`.
    #[cfg(feature = "std")]
    #[test]
    fn spherical_area_dispatches_to_spherical_excess() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Spherical};

        type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;
        let sp = |lon: f64, lat: f64| -> Sp { WithCs::new(Adapt([lon, lat])) };

        let pg: Polygon<Sp> = Polygon::new(Ring::from_vec(vec![
            sp(0., 0.),
            sp(0., 90.),
            sp(90., 0.),
            sp(0., 0.),
        ]));
        let got = area(&pg);
        // Default SphericalPolygonArea uses R = 6_371_000 m.
        let r = 6_371_000.0_f64;
        let expected = core::f64::consts::FRAC_PI_2 * r * r;
        assert!(
            (got - expected).abs() / expected < 1e-6,
            "got {got} expected {expected}"
        );
    }

    /// A spherical polygon with an oppositely-wound interior ring (hole)
    /// subtracts the hole's area: outer octant minus a smaller hole is
    /// strictly less than the whole octant but still positive.
    #[cfg(feature = "std")]
    #[test]
    fn spherical_polygon_with_hole_subtracts_inner_area() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Spherical};
        use geometry_strategy::SphericalPolygonArea;

        type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;
        let sp = |lon: f64, lat: f64| -> Sp { WithCs::new(Adapt([lon, lat])) };

        // Outer: the full octant (CW, positive). Hole: a small triangle
        // wound CCW (opposite), so its excess is negative.
        let outer = Ring::from_vec(vec![sp(0., 0.), sp(0., 90.), sp(90., 0.), sp(0., 0.)]);
        let hole = Ring::from_vec(vec![sp(10., 10.), sp(30., 10.), sp(20., 20.), sp(10., 10.)]);
        let pg: Polygon<Sp> = Polygon::with_inners(outer, vec![hole]);
        let with_hole = area_with(&pg, SphericalPolygonArea::UNIT);
        let whole = core::f64::consts::FRAC_PI_2;
        assert!(with_hole < whole, "hole must reduce area: {with_hole}");
        assert!(with_hole > 0.0, "still positive: {with_hole}");
    }

    /// An *open* spherical ring (its closing vertex omitted) is closed
    /// implicitly: the `last -> first` edge is added, so the area
    /// matches the closed form.
    #[cfg(feature = "std")]
    #[test]
    fn spherical_open_ring_closes_implicitly() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Spherical};
        use geometry_strategy::SphericalArea;

        type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;
        let sp = |lon: f64, lat: f64| -> Sp { WithCs::new(Adapt([lon, lat])) };

        let closed: Ring<Sp> =
            Ring::from_vec(vec![sp(0., 0.), sp(0., 90.), sp(90., 0.), sp(0., 0.)]);
        let open: Ring<Sp, true, false> =
            Ring::from_vec(vec![sp(0., 0.), sp(0., 90.), sp(90., 0.)]);
        let a_closed = area_with(&closed, SphericalArea::UNIT);
        let a_open = area_with(&open, SphericalArea::UNIT);
        assert!((a_closed - a_open).abs() < 1e-9, "{a_closed} vs {a_open}");
    }

    /// A spherical ring crossing the antimeridian drives Δlon
    /// normalisation (the `±2π` wrap): a thin sliver straddling ±180°
    /// must yield a small excess, not a ~2π artefact.
    #[cfg(feature = "std")]
    #[test]
    fn spherical_antimeridian_crossing_normalises_dlon() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Spherical};
        use geometry_strategy::SphericalArea;

        type Sp = WithCs<Adapt<[f64; 2]>, Spherical<Degree>>;
        let sp = |lon: f64, lat: f64| -> Sp { WithCs::new(Adapt([lon, lat])) };

        let r: Ring<Sp> = Ring::from_vec(vec![
            sp(170., 0.),
            sp(170., 10.),
            sp(-170., 10.),
            sp(-170., 0.),
            sp(170., 0.),
        ]);
        let got = area_with(&r, SphericalArea::UNIT).abs();
        // A 20°×10° sliver subtends far less than a hemisphere (2π).
        assert!(got < 0.2, "expected a small sliver area, got {got}");
    }

    /// `area_geo.cpp` — strategy-less `area` on a geographic polygon
    /// resolves to the authalic-sphere `GeographicPolygonArea`; a
    /// 1° × 1° box near the equator on WGS84 ≈ `12_309` km² (within 2 %).
    #[cfg(feature = "std")]
    #[test]
    fn geographic_area_dispatches_to_authalic_sphere() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Geographic};

        type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;
        let gg = |lon: f64, lat: f64| -> Gg { WithCs::new(Adapt([lon, lat])) };

        let pg: Polygon<Gg> = Polygon::new(Ring::from_vec(vec![
            gg(0., 0.),
            gg(1., 0.),
            gg(1., 1.),
            gg(0., 1.),
            gg(0., 0.),
        ]));
        let got = area(&pg).abs();
        let expected = 12_309e6;
        assert!((got - expected).abs() / expected < 0.02);
    }

    /// `area_geo.cpp` — a geographic polygon with a hole wound opposite
    /// the outer ring: `area(polygon)` = outer − hole (the interior-ring
    /// subtraction loop of `GeographicPolygonArea`).
    #[cfg(feature = "std")]
    #[test]
    fn geographic_polygon_with_hole_subtracts_hole_area() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Geographic};

        type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;
        let gg = |lon: f64, lat: f64| -> Gg { WithCs::new(Adapt([lon, lat])) };

        let outer = Ring::from_vec(vec![
            gg(0., 0.),
            gg(1., 0.),
            gg(1., 1.),
            gg(0., 1.),
            gg(0., 0.),
        ]);
        // Same winding sense reversed → opposite-signed excess: a hole.
        let hole: Ring<Gg> = Ring::from_vec(vec![
            gg(0.2, 0.2),
            gg(0.2, 0.8),
            gg(0.8, 0.8),
            gg(0.8, 0.2),
            gg(0.2, 0.2),
        ]);
        let outer_only = area(&Polygon::new(outer.clone())).abs();
        let hole_alone = area(&Polygon::new(hole.clone())).abs();
        let holed = Polygon::with_inners(outer, vec![hole]);
        let got = area(&holed).abs();
        let expected = outer_only - hole_alone;
        assert!(
            (got - expected).abs() / expected < 1e-9,
            "got {got}, expected outer − hole = {expected}"
        );
    }

    /// A geographic ring declared *open* closes implicitly: same area
    /// as the explicitly closed ring (the closing-edge branch of the
    /// excess accumulator).
    #[cfg(feature = "std")]
    #[test]
    fn geographic_open_ring_closes_implicitly() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Geographic};

        type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;
        let gg = |lon: f64, lat: f64| -> Gg { WithCs::new(Adapt([lon, lat])) };

        let open: Polygon<Gg, true, false> = Polygon::new(Ring::from_vec(vec![
            gg(0., 0.),
            gg(1., 0.),
            gg(1., 1.),
            gg(0., 1.),
        ]));
        let closed: Polygon<Gg> = Polygon::new(Ring::from_vec(vec![
            gg(0., 0.),
            gg(1., 0.),
            gg(1., 1.),
            gg(0., 1.),
            gg(0., 0.),
        ]));
        let got_open = area(&open).abs();
        let got_closed = area(&closed).abs();
        assert!(
            (got_open - got_closed).abs() / got_closed < 1e-12,
            "open {got_open} != closed {got_closed}"
        );
    }

    /// A counter-clockwise-declared geographic polygon negates the
    /// signed area (the `PointOrder::CounterClockwise` arm): same
    /// vertices, opposite declared order → opposite sign.
    #[cfg(feature = "std")]
    #[test]
    fn geographic_ccw_declared_polygon_negates_sign() {
        use geometry_adapt::{Adapt, WithCs};
        use geometry_cs::{Degree, Geographic};

        type Gg = WithCs<Adapt<[f64; 2]>, Geographic<Degree>>;
        let gg = |lon: f64, lat: f64| -> Gg { WithCs::new(Adapt([lon, lat])) };

        let verts = vec![gg(0., 0.), gg(1., 0.), gg(1., 1.), gg(0., 1.), gg(0., 0.)];
        let cw: Polygon<Gg, true> = Polygon::new(Ring::from_vec(verts.clone()));
        let ccw: Polygon<Gg, false> = Polygon::new(Ring::from_vec(verts));
        let sum = area(&cw) + area(&ccw);
        assert!(
            sum.abs() < 1e-3,
            "signed areas do not cancel: cw + ccw = {sum}"
        );
        assert!(area(&cw).abs() > 1e9, "area unexpectedly tiny");
    }
}
