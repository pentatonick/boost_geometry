//! Declarative macros that register a user-owned container as a
//! [`Linestring`], [`Ring`], or [`Polygon`].
//!
//! Mirrors the role of `BOOST_GEOMETRY_REGISTER_LINESTRING`
//! (`boost/geometry/geometries/register/linestring.hpp`) and
//! `BOOST_GEOMETRY_REGISTER_RING`
//! (`boost/geometry/geometries/register/ring.hpp`). Boost has no
//! `register/polygon.hpp` — adapting a polygon there is done by
//! hand-specialising five traits (see
//! `doc/example_adapting_a_legacy_geometry_object_model.qbk`); the
//! [`register_polygon!`] macro consolidates that pattern into one
//! declaration on the Rust side.
//!
//! Why macros instead of a blanket impl: Rust's orphan rule forbids
//! `impl<P: Point, C: AsRef<[P]>> Linestring for C` from a downstream
//! crate, so we mint the impl per-user-type with a macro — exactly
//! the role the `BOOST_GEOMETRY_REGISTER_*` macros play in C++ for
//! the same coherence reason (see proposal §3.7, Option D).
//!
//! The macros expand to references inside [`__macros`], a hidden
//! re-export module, so the user's `use` graph never has to mention
//! `geometry_trait` or `geometry_tag` directly.

/// Hidden re-exports the macros expand to. Not part of the public
/// API — users never name this path.
#[doc(hidden)]
pub mod __macros {
    pub use geometry_tag::{
        LinestringTag, MultiLinestringTag, MultiPointTag, MultiPolygonTag, PolygonTag, RingTag,
    };
    pub use geometry_trait::{
        Geometry, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Polygon, Ring,
    };

    pub use geometry_trait::{Closure, PointOrder};
}

/// Register a user-owned multi-point container.
///
/// Emits the `Geometry<Kind = MultiPointTag>` and [`geometry_trait::MultiPoint`]
/// impls needed to expose the supplied point iterator. Mirrors
/// `BOOST_GEOMETRY_REGISTER_MULTI_POINT` from
/// `geometries/register/multi_point.hpp:36-40`; Rust additionally accepts the
/// iterator expression because native containers do not share a Boost.Range
/// base class.
#[macro_export]
macro_rules! register_multi_point {
    ($Ty:ty, $Point:ty, |$value:ident| $iter:expr) => {
        impl $crate::__macros::Geometry for $Ty {
            type Kind = $crate::__macros::MultiPointTag;
            type Point = $Point;
        }

        impl $crate::__macros::MultiPoint for $Ty {
            type ItemPoint = $Point;

            fn points(&self) -> impl ::core::iter::ExactSizeIterator<Item = &$Point> {
                let $value = self;
                $iter
            }
        }
    };
}

/// Register a user-owned multi-linestring container.
///
/// Mirrors `BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING` from
/// `geometries/register/multi_linestring.hpp:36-40`. The item type and
/// iterator are explicit on Rust's side because there is no common
/// Boost.Range-style container base.
#[macro_export]
macro_rules! register_multi_linestring {
    ($Ty:ty, $Point:ty, item = $Item:ty, |$value:ident| $iter:expr) => {
        impl $crate::__macros::Geometry for $Ty {
            type Kind = $crate::__macros::MultiLinestringTag;
            type Point = $Point;
        }

        impl $crate::__macros::MultiLinestring for $Ty {
            type ItemLinestring = $Item;

            fn linestrings(&self) -> impl ::core::iter::ExactSizeIterator<Item = &$Item> {
                let $value = self;
                $iter
            }
        }
    };
}

/// Register a user-owned multi-polygon container.
///
/// Mirrors `BOOST_GEOMETRY_REGISTER_MULTI_POLYGON` from
/// `geometries/register/multi_polygon.hpp:36-40`. The item type and iterator
/// make the collection shape explicit in the generated Rust concept impl.
#[macro_export]
macro_rules! register_multi_polygon {
    ($Ty:ty, $Point:ty, item = $Item:ty, |$value:ident| $iter:expr) => {
        impl $crate::__macros::Geometry for $Ty {
            type Kind = $crate::__macros::MultiPolygonTag;
            type Point = $Point;
        }

        impl $crate::__macros::MultiPolygon for $Ty {
            type ItemPolygon = $Item;

            fn polygons(&self) -> impl ::core::iter::ExactSizeIterator<Item = &$Item> {
                let $value = self;
                $iter
            }
        }
    };
}

/// Register a user-owned linestring type whose storage is iterable as
/// `&Point` via a user-supplied expression.
///
/// Emits `impl Geometry for $Ty` with `Kind = LinestringTag` and
/// `Point = $Point`, plus `impl Linestring for $Ty` whose `points()`
/// returns the iterator the closure-shaped argument produces.
///
/// Mirrors `BOOST_GEOMETRY_REGISTER_LINESTRING` from
/// `boost/geometry/geometries/register/linestring.hpp`. The Boost
/// macro only has to specialise `traits::tag<G>` because the rest of
/// the linestring concept rides on `boost::range` for free; the Rust
/// counterpart has to spell the iterator out, which is why the
/// macro takes a closure-shaped expression for the iteration.
///
/// The iterator expression must yield an
/// `ExactSizeIterator<Item = &$Point> + Clone` — the same bound
/// [`geometry_trait::Linestring::points`] requires. For the common
/// case of a `Vec<$Point>` field, `|s| s.field.iter()` is the right
/// shape.
///
/// # Example
///
/// ```
/// use geometry_adapt::register_linestring;
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_trait::Linestring;
///
/// struct MyLineString { points: Vec<Point2D<f64, Cartesian>> }
///
/// register_linestring!(MyLineString, Point2D<f64, Cartesian>, |s| s.points.iter());
///
/// let ls = MyLineString {
///     points: vec![Point2D::new(0.0, 0.0), Point2D::new(1.0, 1.0)],
/// };
/// assert_eq!(ls.points().count(), 2);
/// ```
#[macro_export]
macro_rules! register_linestring {
    ($Ty:ty, $Point:ty, |$s:ident| $iter:expr) => {
        impl $crate::__macros::Geometry for $Ty {
            type Kind = $crate::__macros::LinestringTag;
            type Point = $Point;
        }
        impl $crate::__macros::Linestring for $Ty {
            fn points(
                &self,
            ) -> impl ::core::iter::ExactSizeIterator<Item = &$Point> + ::core::clone::Clone {
                let $s = self;
                $iter
            }
        }
    };
}

/// Register a user-owned ring type whose storage is iterable as
/// `&Point` via a user-supplied expression.
///
/// Emits `impl Geometry for $Ty` with `Kind = RingTag` and
/// `Point = $Point`, plus `impl Ring for $Ty` whose `points()` returns
/// the iterator the closure-shaped argument produces. Optional
/// trailing `closure = …` and `point_order = …` clauses override the
/// defaults [`geometry_trait::Ring`] inherits from
/// `boost/geometry/core/{closure,point_order}.hpp` (closed,
/// clockwise).
///
/// Mirrors `BOOST_GEOMETRY_REGISTER_RING` from
/// `boost/geometry/geometries/register/ring.hpp`, plus the
/// per-ring `traits::closure<R>` / `traits::point_order<R>`
/// specialisations from `boost/geometry/core/closure.hpp` and
/// `boost/geometry/core/point_order.hpp`.
///
/// # Examples
///
/// Default-orientation ring (`closed`, `clockwise`):
///
/// ```
/// use geometry_adapt::register_ring;
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_trait::Ring;
///
/// struct MyRing { points: Vec<Point2D<f64, Cartesian>> }
///
/// register_ring!(MyRing, Point2D<f64, Cartesian>, |s| s.points.iter());
///
/// let r = MyRing {
///     points: vec![
///         Point2D::new(0.0, 0.0),
///         Point2D::new(1.0, 0.0),
///         Point2D::new(1.0, 1.0),
///         Point2D::new(0.0, 0.0),
///     ],
/// };
/// assert_eq!(r.points().count(), 4);
/// ```
///
/// Override `closure` and `point_order`:
///
/// ```
/// use geometry_adapt::register_ring;
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_trait::{Closure, PointOrder, Ring};
///
/// struct OpenCcwRing { points: Vec<Point2D<f64, Cartesian>> }
///
/// register_ring!(
///     OpenCcwRing,
///     Point2D<f64, Cartesian>,
///     |s| s.points.iter(),
///     closure = Closure::Open,
///     point_order = PointOrder::CounterClockwise
/// );
///
/// let r = OpenCcwRing { points: vec![] };
/// assert_eq!(r.closure(), Closure::Open);
/// assert_eq!(r.point_order(), PointOrder::CounterClockwise);
/// ```
#[macro_export]
macro_rules! register_ring {
    ($Ty:ty, $Point:ty, |$s:ident| $iter:expr) => {
        impl $crate::__macros::Geometry for $Ty {
            type Kind = $crate::__macros::RingTag;
            type Point = $Point;
        }
        impl $crate::__macros::Ring for $Ty {
            fn points(
                &self,
            ) -> impl ::core::iter::ExactSizeIterator<Item = &$Point> + ::core::clone::Clone {
                let $s = self;
                $iter
            }
        }
    };
    (
        $Ty:ty,
        $Point:ty,
        |$s:ident| $iter:expr,
        closure = $closure:expr,
        point_order = $order:expr $(,)?
    ) => {
        impl $crate::__macros::Geometry for $Ty {
            type Kind = $crate::__macros::RingTag;
            type Point = $Point;
        }
        impl $crate::__macros::Ring for $Ty {
            fn points(
                &self,
            ) -> impl ::core::iter::ExactSizeIterator<Item = &$Point> + ::core::clone::Clone {
                let $s = self;
                $iter
            }
            fn closure(&self) -> $crate::__macros::Closure {
                $closure
            }
            fn point_order(&self) -> $crate::__macros::PointOrder {
                $order
            }
        }
    };
}

/// Register a user-owned polygon type whose interior decomposes as
/// `{ outer: $Ring, inners: <iterable of $Ring> }`.
///
/// Emits `impl Geometry for $Ty` with `Kind = PolygonTag` and
/// `Point = $Point`, plus `impl Polygon for $Ty` with the supplied
/// `$Ring` as the associated `Ring` type. The `outer = …` expression
/// must produce `&$Ring`; the `inners = …` expression must produce
/// `impl ExactSizeIterator<Item = &$Ring>` — the bound
/// [`geometry_trait::Polygon::interiors`] requires.
///
/// Mirrors the five-trait pattern in
/// `doc/example_adapting_a_legacy_geometry_object_model.qbk`
/// (`tag`, `ring_const_type`, `interior_const_type`, `exterior_ring`,
/// `interior_rings`) — Boost.Geometry has no `register/polygon.hpp`,
/// so this macro is the equivalent shortcut on the Rust side.
///
/// # Example
///
/// ```
/// use geometry_adapt::{register_polygon, register_ring};
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_trait::{Polygon, Ring};
///
/// type P = Point2D<f64, Cartesian>;
///
/// struct MyRing { points: Vec<P> }
/// register_ring!(MyRing, P, |s| s.points.iter());
///
/// struct MyPoly { outer: MyRing, inners: Vec<MyRing> }
/// register_polygon!(
///     MyPoly,
///     P,
///     ring = MyRing,
///     |s| outer = &s.outer, inners = s.inners.iter()
/// );
///
/// let poly = MyPoly {
///     outer: MyRing { points: vec![] },
///     inners: vec![],
/// };
/// assert_eq!(poly.exterior().points().count(), 0);
/// assert_eq!(poly.interiors().count(), 0);
/// ```
#[macro_export]
macro_rules! register_polygon {
    (
        $Ty:ty,
        $Point:ty,
        ring = $Ring:ty,
        |$s:ident| outer = $outer:expr, inners = $inners:expr $(,)?
    ) => {
        impl $crate::__macros::Geometry for $Ty {
            type Kind = $crate::__macros::PolygonTag;
            type Point = $Point;
        }
        impl $crate::__macros::Polygon for $Ty {
            type Ring = $Ring;

            fn exterior(&self) -> &Self::Ring {
                let $s = self;
                $outer
            }

            fn interiors(&self) -> impl ::core::iter::ExactSizeIterator<Item = &Self::Ring> {
                let $s = self;
                $inners
            }
        }
    };
}
