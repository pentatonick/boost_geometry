//! Declarative literal macros for `model::Point`, `model::Linestring`,
//! and `model::Polygon`.
//!
//! These mirror the literal-style polygon construction shown in
//! `specs/rust-port-proposal.md` §5 — they give callers a compact way
//! to write geometry values from nested tuple syntax, without standing
//! up a builder type.
//!
//! Boost.Geometry has no direct equivalent (C++ initialiser-list syntax
//! covers that role), so the surface is Rust-specific ergonomics over
//! the existing model types.
//!
//! All three macros are `#[macro_export]`ed at the crate root.

/// Construct a [`crate::Point2D`] or [`crate::Point3D`]
/// from a tuple literal.
///
/// The `(x, y)` form expands to `Point2D::new(x, y)`; the `(x, y, z)`
/// form expands to `Point3D::new(x, y, z)`. The coordinate type and
/// coordinate system are inferred from context, so a binding annotated
/// `Point2D<f64, Cartesian>` will pick up `f64` for the expressions.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{point, Point2D, Point3D};
/// use geometry_trait::Point as _;
///
/// let p: Point2D<f64, Cartesian> = point!((1.0, 2.0));
/// assert_eq!(p.get::<0>(), 1.0);
///
/// let q: Point3D<f64, Cartesian> = point!((1.0, 2.0, 3.0));
/// assert_eq!(q.get::<2>(), 3.0);
/// ```
#[macro_export]
macro_rules! point {
    (($x:expr, $y:expr)) => {
        $crate::Point2D::new($x, $y)
    };
    (($x:expr, $y:expr, $z:expr)) => {
        $crate::Point3D::new($x, $y, $z)
    };
}

/// Construct a [`crate::Linestring`] from a comma-separated
/// list of `(x, y)` tuples.
///
/// Each tuple is expanded through [`point!`] and the resulting points
/// are collected into a `Vec`, then wrapped via
/// [`Linestring::from_vec`](crate::Linestring::from_vec).
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{linestring, Linestring, Point2D};
/// use geometry_trait::Linestring as _;
///
/// let ls: Linestring<Point2D<f64, Cartesian>> =
///     linestring![(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)];
/// assert_eq!(ls.points().count(), 3);
/// ```
#[macro_export]
macro_rules! linestring {
    [ $( ( $($pt:expr),+ $(,)? ) ),* $(,)? ] => {{
        let mut __ls = $crate::Linestring::new();
        $( __ls.push($crate::point!(( $($pt),+ ))); )*
        __ls
    }};
}

/// Construct a [`crate::Polygon`] from a bracketed list whose
/// first element is the outer ring and the rest are holes (interior
/// rings).
///
/// Each ring is written as a bracketed, comma-separated list of `(x, y)`
/// tuples; tuples expand through [`point!`].
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_trait::{Polygon as _, Ring as _};
///
/// let p: Polygon<Point2D<f64, Cartesian>> = polygon![
///     [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)]
/// ];
/// assert_eq!(p.exterior().points().count(), 5);
/// assert_eq!(p.interiors().count(), 0);
/// ```
///
/// ## Polygons with holes
///
/// The outer ring comes first, then zero or more inner rings (holes)
/// each as their own bracketed list:
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_trait::{Polygon as _, Ring as _};
///
/// let p: Polygon<Point2D<f64, Cartesian>> = polygon![
///     // Outer ring: a 5×5 square.
///     [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)],
///     // Hole: a 1×1 square inside it.
///     [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)],
/// ];
/// assert_eq!(p.exterior().points().count(), 5);
/// assert_eq!(p.interiors().count(), 1);
/// assert_eq!(p.interiors().next().unwrap().points().count(), 5);
/// ```
///
/// Mirrors the C++ "polygon with hole" example in
/// `boost/geometry/doc/src/examples/quick_start.cpp` lines 121-129,
/// rebuilt against the Rust [`crate::Polygon`] constructor.
#[macro_export]
macro_rules! polygon {
    [ [ $( ( $($outer_pt:expr),+ $(,)? ) ),* $(,)? ]
      $(, [ $( ( $($inner_pt:expr),+ $(,)? ) ),* $(,)? ] )*
      $(,)?
    ] => {{
        let mut __outer = $crate::Ring::new();
        $( __outer.push($crate::point!(( $($outer_pt),+ ))); )*
        let mut __poly = $crate::Polygon::new(__outer);
        $(
            {
                let mut __hole = $crate::Ring::new();
                $( __hole.push($crate::point!(( $($inner_pt),+ ))); )*
                __poly.inners.push(__hole);
            }
        )*
        __poly
    }};
}
