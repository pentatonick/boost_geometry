//! Strategy for computing the area of a Cartesian geometry.
//!
//! Mirrors three pieces of Boost.Geometry that collaborate to make
//! `boost::geometry::area(g)` work for any ring / polygon / box /
//! multi-polygon in any coordinate system:
//!
//! * `boost/geometry/strategies/area/services.hpp` — the
//!   `services::default_strategy<G>` metafunction that picks the
//!   per-CS area strategy.
//! * `boost/geometry/strategies/area/cartesian.hpp` —
//!   `strategies::area::cartesian<>` plus its
//!   `services::default_strategy<Geometry, cartesian_tag>`
//!   specialisation; the umbrella strategy hands out
//!   `strategy::area::cartesian<>` for ring / polygon geometries and
//!   `strategy::area::cartesian_box<>` for boxes.
//! * `boost/geometry/strategy/cartesian/area.hpp:91-120` — the
//!   trapezoidal-rule accumulation `(x1 + x2) * (y1 - y2)` summed over
//!   consecutive segments and halved at the end. The Boost code wraps
//!   the ring in `closed_clockwise_view` first, closed and reversed
//!   when counter-clockwise, so a ring wound as declared has a positive
//!   area; the Rust port walks the same view, in the same order.
//!
//! The spherical and geographic strategies live beside their families'
//! other strategies, in [`crate::spherical`] and [`crate::geographic`].
//!
//! # Coherence note
//!
//! Rust's coherence rules cannot prove that no single type is both a
//! [`Ring`] and a [`Polygon`] (or a [`Box`] / [`MultiPolygon`]) at the
//! same time, so a single `ShoelaceArea` carrying four
//! `impl AreaStrategy<G>` blocks keyed off `G: Ring`, `G: Polygon`,
//! `G: Box`, `G: MultiPolygon` is rejected as overlapping. Boost
//! sidesteps this with tag dispatch (`strategy::area::cartesian` vs.
//! `strategy::area::cartesian_box`, plus the per-tag `dispatch::area`
//! arms in `algorithms/area.hpp:131-187`); we mirror that split with
//! four sibling unit-structs below, each implementing
//! [`AreaStrategy`] for exactly one geometry kind.

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem, GeographicFamily, SphericalFamily};
use geometry_tag::SameAs;
use geometry_trait::{Box, Geometry, MultiPolygon, Point, Polygon, Ring, corner};

use crate::clockwise_view::clockwise_points;

/// The scalar a Cartesian area of `P` coordinates is computed and returned
/// in — `f64` for integer coordinates, as Boost's `area_result` has it.
type Measure<P> = <<P as Point>::Scalar as CoordinateScalar>::Measure;

/// A strategy for computing the area of a geometry.
///
/// Mirrors the per-CS area-strategy concept declared in
/// `boost/geometry/strategies/area/services.hpp` and refined per
/// coordinate system in `strategies/area/{cartesian,spherical,
/// geographic}.hpp`. The Boost concept exposes a stateful `apply(p1,
/// p2, state)` accumulator plus a final `result(state)` reduction;
/// the Rust analogue collapses the two phases into a single method
/// [`AreaStrategy::area`] keyed on the geometry type, because the
/// per-segment walk shape is identical for every CS — only the
/// per-segment kernel changes.
///
/// # Associated items
///
/// * [`Self::Out`] — the scalar the area comes back as.
///   Equivalent to Boost's `area_result<Geometry, Strategies>::type`
///   (`algorithms/area_result.hpp`); typically the coordinate scalar
///   of `G`'s point type.
pub trait AreaStrategy<G: Geometry> {
    /// The output scalar type. Typically the geometry's coordinate
    /// scalar. Mirrors `area_result<G, Strategies>::type` from
    /// `algorithms/area_result.hpp`.
    type Out: CoordinateScalar;

    /// Compute the area of `g`.
    ///
    /// Mirrors the `result(strategy.apply(...))` pair from
    /// `algorithms/area.hpp:111-116` together with the CS-specific
    /// `strategy::area::cartesian::apply` walk at
    /// `strategy/cartesian/area.hpp:91-112`.
    fn area(&self, g: &G) -> Self::Out;
}

/// Cartesian shoelace area for a [`Ring`].
///
/// Mirrors `boost::geometry::strategy::area::cartesian<>` from
/// `strategy/cartesian/area.hpp:50-120` applied to a ring through the
/// `dispatch::area<Ring, ring_tag>` arm at `algorithms/area.hpp:154-157`.
///
/// Sign convention follows Boost: rings whose vertices match the
/// declared [`PointOrder`](geometry_trait::PointOrder) yield a positive
/// area, rings traversed in
/// the opposite direction yield a negative area
/// (`test/algorithms/area/area.cpp:63-64`).
#[derive(Debug, Default, Clone, Copy)]
pub struct ShoelaceArea;

/// Cartesian shoelace area for a [`Polygon`] — outer ring area minus
/// the sum of interior-ring areas.
///
/// Mirrors the `dispatch::area<Polygon, polygon_tag>` arm at
/// `algorithms/area.hpp:160-172`, which inherits from
/// `detail::calculate_polygon_sum` and delegates the per-ring work to
/// the same `ring_area` used by [`ShoelaceArea`]. The split into a
/// dedicated strategy type is a Rust coherence concession; see the
/// module-level documentation.
#[derive(Debug, Default, Clone, Copy)]
pub struct ShoelacePolygonArea;

/// Cartesian area for a [`Box`]: `(xmax - xmin) * (ymax - ymin)`.
///
/// Mirrors `boost::geometry::strategy::area::cartesian_box<>` from
/// `strategy/cartesian/area_box.hpp:28-48` applied through the
/// `dispatch::area<Box, box_tag>` arm at
/// `algorithms/area.hpp:149-151`.
#[derive(Debug, Default, Clone, Copy)]
pub struct ShoelaceBoxArea;

/// Cartesian shoelace area for a [`MultiPolygon`] — sum of the areas
/// of its member polygons.
///
/// Mirrors the `dispatch::area<MultiGeometry, multi_polygon_tag>` arm
/// at `algorithms/area.hpp:175-187`, which inherits from
/// `detail::multi_sum` and delegates the per-polygon work to the same
/// `polygon_area` used by [`ShoelacePolygonArea`].
#[derive(Debug, Default, Clone, Copy)]
pub struct ShoelaceMultiPolygonArea;

// ---- Ring ------------------------------------------------------------
//
// Mirrors the `dispatch::area<Ring, ring_tag>` arm at
// `algorithms/area.hpp:154-157`, which inherits from
// `detail::area::ring_area::apply` (`algorithms/area.hpp:82-118`).
// The Boost code wraps the ring in `closed_clockwise_view` first so
// that a counter-clockwise declared ring still feeds clockwise vertices
// to the strategy, and an open one its closing edge; the port walks the
// same view, so the terms are summed in Boost's order.

impl<R> AreaStrategy<R> for ShoelaceArea
where
    R: Ring,
    <R::Point as Point>::Cs: CoordinateSystem,
    <<R::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = Measure<R::Point>;

    #[inline]
    fn area(&self, r: &R) -> Self::Out {
        let acc = shoelace_accumulator::<R>(r);
        let two = <Self::Out as CoordinateScalar>::ONE + <Self::Out as CoordinateScalar>::ONE;
        acc / two
    }
}

// ---- Polygon ---------------------------------------------------------
//
// Mirrors the `dispatch::area<Polygon, polygon_tag>` arm at
// `algorithms/area.hpp:160-172`. Boost spells the recursion as
// `calculate_polygon_sum::apply<…, ring_area>(polygon, strategy)`,
// summing the (signed) area of every ring — exterior plus interiors.
// Boost's signed-area convention means the interior rings, which by
// convention are wound opposite the exterior, already arrive with the
// opposite sign, so a *sum* of ring areas gives the polygon area; the
// Rust mirror is exactly the same arithmetic.

impl<P> AreaStrategy<P> for ShoelacePolygonArea
where
    P: Polygon,
    ShoelaceArea: AreaStrategy<P::Ring, Out = Measure<P::Point>>,
{
    type Out = Measure<P::Point>;

    #[inline]
    fn area(&self, p: &P) -> Self::Out {
        // The interiors summed from zero, then added to the exterior:
        // `calculate_polygon_sum` (`algorithms/detail/calculate_sum.hpp:36-55`).
        let interiors = p
            .interiors()
            .fold(<Self::Out as CoordinateScalar>::ZERO, |sum, inner| {
                sum + ShoelaceArea.area(inner)
            });
        ShoelaceArea.area(p.exterior()) + interiors
    }
}

// ---- Box -------------------------------------------------------------
//
// Mirrors the `dispatch::area<Box, box_tag>` arm at
// `algorithms/area.hpp:149-151`, which inherits from
// `detail::area::box_area::apply`. Boost asserts a 2D box and computes
// `(xmax - xmin) * (ymax - ymin)` (`strategy/cartesian/area_box.hpp:41-47`).
// The Rust port enforces 2D via the const-generic bound on
// [`corner::MIN`] / [`corner::MAX`]; higher-dimensional boxes simply
// will not see this impl matched because the formula only reads
// dimensions 0 and 1.

impl<B> AreaStrategy<B> for ShoelaceBoxArea
where
    B: Box,
    <B::Point as Point>::Cs: CoordinateSystem,
    <<B::Point as Point>::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    type Out = Measure<B::Point>;

    #[inline]
    fn area(&self, b: &B) -> Self::Out {
        let xmin = b.get_indexed::<{ corner::MIN }, 0>().to_measure();
        let ymin = b.get_indexed::<{ corner::MIN }, 1>().to_measure();
        let xmax = b.get_indexed::<{ corner::MAX }, 0>().to_measure();
        let ymax = b.get_indexed::<{ corner::MAX }, 1>().to_measure();
        (xmax - xmin) * (ymax - ymin)
    }
}

// ---- MultiPolygon ----------------------------------------------------
//
// Mirrors the `dispatch::area<MultiGeometry, multi_polygon_tag>` arm
// at `algorithms/area.hpp:175-187`. Boost spells the recursion as
// `multi_sum::apply<…, area<polygon>>(multi, strategy)`, summing the
// signed area of every member polygon.

impl<MPg> AreaStrategy<MPg> for ShoelaceMultiPolygonArea
where
    MPg: MultiPolygon,
    ShoelacePolygonArea: AreaStrategy<MPg::ItemPolygon, Out = Measure<MPg::Point>>,
{
    type Out = Measure<MPg::Point>;

    #[inline]
    fn area(&self, mpg: &MPg) -> Self::Out {
        let mut total = <Self::Out as CoordinateScalar>::ZERO;
        for p in mpg.polygons() {
            total = total + ShoelacePolygonArea.area(p);
        }
        total
    }
}

/// Sum `(x_i + x_{i+1}) * (y_i - y_{i+1})` over the consecutive
/// vertex pairs of `r`, walked as `ring_area` walks
/// `closed_clockwise_view` (`algorithms/area.hpp:82-118`,
/// `strategy/cartesian/area.hpp:91-120`): closed, reversed when
/// counter-clockwise, and zero for a ring below its closure's minimum
/// size.
#[inline]
fn shoelace_accumulator<R>(r: &R) -> Measure<R::Point>
where
    R: Ring,
{
    let mut acc = <Measure<R::Point> as CoordinateScalar>::ZERO;
    for edge in clockwise_points(r).windows(2) {
        acc = acc + segment_term::<R::Point>(edge[0], edge[1]);
    }
    acc
}

/// One trapezoidal-rule term: `(x_a + x_b) * (y_a - y_b)`.
///
/// Mirrors the per-segment kernel at
/// `strategy/cartesian/area.hpp:110-111`. Boost notes that this
/// formulation loses less precision than the naive
/// `x_a * y_b - x_b * y_a` cross product at large coordinate values
/// (Boost trac #11928 cited in the same header).
#[inline]
fn segment_term<P>(a: &P, b: &P) -> Measure<P>
where
    P: Point,
{
    (a.get::<0>().to_measure() + b.get::<0>().to_measure())
        * (a.get::<1>().to_measure() - b.get::<1>().to_measure())
}

// ---- Default area strategy per CS family ----------------------------

/// "Which (polygon) area strategy do we pick by default for this CS
/// family?"
///
/// Mirrors v1's [`DefaultDistance`](crate::distance::DefaultDistance)
/// and [`DefaultLength`](crate::length::DefaultLength) — the Rust
/// analogue of Boost's `services::default_strategy<Geometry, cs_tag>`
/// in `strategies/area/services.hpp`, specialised per CS in
/// `strategies/area/{cartesian,spherical,geographic}.hpp`.
///
/// Keyed on the *polygon* area strategy, since the `area(&polygon)`
/// free function is the entry point that dispatches through it:
///
/// ```ignore
/// impl DefaultArea<CartesianFamily>  for CartesianFamily  { type Strategy = ShoelacePolygonArea;   }
/// impl DefaultArea<SphericalFamily>  for SphericalFamily  { type Strategy = SphericalPolygonArea;  }
/// impl DefaultArea<GeographicFamily> for GeographicFamily { type Strategy = GeographicPolygonArea; }
/// ```
pub trait DefaultArea<Family> {
    /// The area strategy chosen for this family. Must implement
    /// [`Default`] because the free-function `area(g)` builds it
    /// without arguments.
    type Strategy: Default;
}

/// Cartesian family defaults to [`ShoelacePolygonArea`].
impl DefaultArea<CartesianFamily> for CartesianFamily {
    type Strategy = ShoelacePolygonArea;
}

/// Spherical family defaults to [`SphericalPolygonArea`](crate::spherical::SphericalPolygonArea).
impl DefaultArea<SphericalFamily> for SphericalFamily {
    type Strategy = crate::spherical::SphericalPolygonArea;
}

/// Geographic family defaults to [`GeographicPolygonArea`](crate::geographic::GeographicPolygonArea)
/// — Boost's spheroidal series with Andoyer azimuths.
impl DefaultArea<GeographicFamily> for GeographicFamily {
    type Strategy = crate::geographic::GeographicPolygonArea;
}

/// Type alias resolving the default area strategy for geometry `G` by
/// walking `G -> G::Point -> Cs -> Family -> DefaultArea::Strategy`.
///
/// Mirrors [`DefaultDistanceStrategy`](crate::distance::DefaultDistanceStrategy)
/// for the area algorithm.
pub type DefaultAreaStrategy<G> =
    <<<<G as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family as DefaultArea<
        <<<G as Geometry>::Point as Point>::Cs as CoordinateSystem>::Family,
    >>::Strategy;

#[cfg(test)]
mod tests {
    //! Reference values from `geometry/test/algorithms/area/area.cpp`
    //! (lines 45-64). Each test cites the source it mirrors.

    use super::{
        AreaStrategy, ShoelaceArea, ShoelaceBoxArea, ShoelaceMultiPolygonArea, ShoelacePolygonArea,
    };
    use geometry_cs::Cartesian;
    use geometry_model::{Box, MultiPolygon, Point2D, Polygon, Ring, polygon};

    type P = Point2D<f64, Cartesian>;

    /// `area.cpp:45` — rotated unit square, area = 2.
    #[test]
    fn ring_diamond_is_2() {
        let r: Ring<P> = Ring::from_vec(vec![
            Point2D::new(1.0, 1.0),
            Point2D::new(2.0, 2.0),
            Point2D::new(3.0, 1.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(1.0, 1.0),
        ]);
        let got = accepts_readonly_point(&ShoelaceArea, &r);
        assert!((got - 2.0).abs() < 1e-12);
    }

    /// `area.cpp:47, 63` — pentagon, area = 16.
    #[test]
    fn ring_pentagon_is_16() {
        let r: Ring<P> = Ring::from_vec(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(0.0, 7.0),
            Point2D::new(4.0, 2.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(0.0, 0.0),
        ]);
        let got = ShoelaceArea.area(&r);
        assert!((got - 16.0).abs() < 1e-12);
    }

    /// `area.cpp:64` — same pentagon traversed in the opposite
    /// direction, declared as a default (CW) ring → area = -16.
    #[test]
    fn ring_wrongly_ordered_is_minus_16() {
        let r: Ring<P> = Ring::from_vec(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(4.0, 2.0),
            Point2D::new(0.0, 7.0),
            Point2D::new(0.0, 0.0),
        ]);
        let got = ShoelaceArea.area(&r);
        assert!((got - -16.0).abs() < 1e-12);
    }

    /// `area.cpp:48` — unit-square vertices in CCW order on a
    /// default-CW polygon → area = -1.
    #[test]
    fn polygon_ccw_unit_square_is_minus_1() {
        let p: Polygon<P> = polygon![[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)]];
        let got = ShoelacePolygonArea.area(&p);
        assert!((got - -1.0).abs() < 1e-12);
    }

    /// `area.cpp:49` — pentagon (area 16) minus a unit-square hole
    /// (signed area -1) = 15.
    #[test]
    fn polygon_pentagon_with_hole_is_15() {
        let outer: Ring<P> = Ring::from_vec(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(0.0, 7.0),
            Point2D::new(4.0, 2.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(0.0, 0.0),
        ]);
        let hole: Ring<P> = Ring::from_vec(vec![
            Point2D::new(1.0, 1.0),
            Point2D::new(2.0, 1.0),
            Point2D::new(2.0, 2.0),
            Point2D::new(1.0, 2.0),
            Point2D::new(1.0, 1.0),
        ]);
        let mut p: Polygon<P> = Polygon::new(outer);
        p.inners.push(hole);
        let got = ShoelacePolygonArea.area(&p);
        assert!((got - 15.0).abs() < 1e-12);
    }

    /// `area.cpp:56-57` — both orderings of the same box give the
    /// same area (4). The Cartesian box formula is sign-blind.
    #[test]
    fn box_2x2_is_4() {
        let b = Box::from_corners(
            Point2D::<f64, Cartesian>::new(0.0, 0.0),
            Point2D::new(2.0, 2.0),
        );
        let got = ShoelaceBoxArea.area(&b);
        assert!((got - 4.0).abs() < 1e-12);
    }

    /// Multi-polygon: two disjoint CW unit squares → area = 2.
    #[test]
    fn multipolygon_two_unit_squares_is_2() {
        // CW: (x,y) -> (x,y+1) -> (x+1,y+1) -> (x+1,y) -> (x,y).
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
        let got = ShoelaceMultiPolygonArea.area(&mpg);
        assert!((got - 2.0).abs() < 1e-12);
    }

    /// Open ring of a 2x2 square (no repeated closing vertex): the
    /// strategy must add the implicit last->first edge so the area
    /// still comes out to 4.
    #[test]
    fn open_ring_2x2_square_is_4() {
        let mut r = Ring::<P, true, false>::new();
        r.push(Point2D::new(0.0, 0.0));
        r.push(Point2D::new(0.0, 2.0));
        r.push(Point2D::new(2.0, 2.0));
        r.push(Point2D::new(2.0, 0.0));
        let got = ShoelaceArea.area(&r);
        assert!((got - 4.0).abs() < 1e-12);
    }

    /// Counter-clockwise declared ring traversed CCW (matching its
    /// declared order) → positive area. Mirrors
    /// `area.cpp:69-73` whose diamond on `polygon<P, false>` yields 2.
    #[test]
    fn ccw_declared_ccw_traversed_diamond_is_2() {
        let r: Ring<P, false> = Ring::from_vec(vec![
            Point2D::new(1.0, 0.0),
            Point2D::new(0.0, 1.0),
            Point2D::new(-1.0, 0.0),
            Point2D::new(0.0, -1.0),
            Point2D::new(1.0, 0.0),
        ]);
        let got = ShoelaceArea.area(&r);
        assert!((got - 2.0).abs() < 1e-12);
    }

    /// A ring below its closure's minimum size encloses nothing: a
    /// closed ring of three points measures `0` in Boost (`aed7bc3`),
    /// whatever the points.
    #[test]
    fn a_closed_ring_of_three_points_has_no_area() {
        let r: Ring<P> = Ring::from_vec(vec![
            Point2D::new(-376_534.732_048_763_54, -217_554.022_127_133_94),
            Point2D::new(-709_156.943_996_279_5, 106_487.395_991_661_45),
            Point2D::new(-376_534.732_048_763_54, -217_554.022_127_133_94),
        ]);
        assert_eq!(ShoelaceArea.area(&r), 0.0);
        let open: Ring<P, true, false> =
            Ring::from_vec(vec![Point2D::new(0.0, 0.0), Point2D::new(0.0, 1.0)]);
        assert_eq!(ShoelaceArea.area(&open), 0.0);
    }

    /// A counter-clockwise ring is summed backwards, as Boost walks its
    /// `closed_clockwise_view`: near `1e9` the trapezoids cancel to their
    /// last bits, and Boost (`aed7bc3`) gets `0.22786015272140503` where
    /// the forward sum negated gets `0.22786018252372742`.
    /// The interiors are summed from zero and then added to the exterior,
    /// as `calculate_polygon_sum` adds them
    /// (`algorithms/detail/calculate_sum.hpp:36-55`); added one by one, these
    /// two holes round to `99.14349899999999`. Boost (`aed7bc3`): `99.143499`.
    #[test]
    fn interiors_are_summed_before_the_exterior() {
        let pg: Polygon<P> = polygon![
            [
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (0.0, 0.0)
            ],
            [(3.759, 2.11), (2.758, 2.983), (2.047, 2.849), (3.759, 2.11)],
            [
                (7.813, 6.224),
                (7.157, 7.791),
                (7.194, 6.242),
                (7.813, 6.224)
            ]
        ];
        assert_eq!(ShoelacePolygonArea.area(&pg), 99.143_499);
    }

    #[test]
    fn a_counter_clockwise_ring_sums_in_boosts_order() {
        let r: Ring<P, false, false> = Ring::from_vec(vec![
            Point2D::new(1_000_000_000.841_793_4, 999_999_999.538_218_6),
            Point2D::new(999_999_999.912_683_4, 999_999_999.114_749_9),
            Point2D::new(1_000_000_000.825_902_5, 999_999_999.081_959_4),
            Point2D::new(1_000_000_000.932_290_7, 999_999_999.711_619),
        ]);
        assert_eq!(ShoelaceArea.area(&r), 0.227_860_152_721_405_03);
    }

    // KC1.T2 witness: proves this strategy accepts a geometry whose
    // `Point` is read-only (need not implement `PointMut`). If it
    // compiles, the read-only bound is locked.
    fn accepts_readonly_point<G, S>(s: &S, g: &G) -> S::Out
    where
        G: geometry_trait::Geometry,
        <G as geometry_trait::Geometry>::Point: geometry_trait::Point,
        S: AreaStrategy<G>,
    {
        s.area(g)
    }
}
