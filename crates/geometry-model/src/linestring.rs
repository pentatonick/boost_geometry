//! Default `model::Linestring<P>` — an ordered sequence of >= 2 points
//! stored in a `Vec<P>`.
//!
//! Mirrors `boost::geometry::model::linestring<Point, Container,
//! Allocator>` declared in `boost/geometry/geometries/linestring.hpp:54-95`,
//! together with the trait specialisation the same header provides
//! under `namespace traits` (`tag`) at
//! `boost/geometry/geometries/linestring.hpp:103-113`. The Rust port
//! collapses that specialisation into the two trait impls below
//! ([`Geometry`], [`LinestringTrait`]), keyed off the generic point
//! parameter `P`.
//!
//! Boost derives `model::linestring` from `std::vector<Point>` so any
//! `boost::range` algorithm sees the underlying storage directly; the
//! Rust port stores a `Vec<P>` and surfaces it through the `points()`
//! iterator the [`LinestringTrait`] concept requires.

use alloc::vec::Vec;

use geometry_tag::LinestringTag;
use geometry_trait::{Geometry, Linestring as LinestringTrait, Point as PointTrait};

/// Default Linestring — a `Vec<P>` newtype.
///
/// Mirrors `boost::geometry::model::linestring<Point>` from
/// `boost/geometry/geometries/linestring.hpp:54-95`. The
/// `#[repr(transparent)]` annotation guarantees the in-memory layout
/// matches the underlying `Vec<P>`, so callers can safely transmute
/// between the two when interfacing with code that already operates
/// on `Vec<P>` directly — the same invariant Boost relies on by
/// inheriting publicly from `std::vector<Point>`.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(transparent)]
pub struct Linestring<P: PointTrait>(pub Vec<P>);

impl<P: PointTrait> Linestring<P> {
    /// Construct an empty linestring.
    ///
    /// Mirrors the default `linestring()` constructor at
    /// `boost/geometry/geometries/linestring.hpp:68-70`.
    #[must_use]
    pub const fn new() -> Self {
        Self(Vec::new())
    }

    /// Wrap an existing `Vec<P>` as a linestring without copying.
    ///
    /// Rust analogue of constructing `boost::geometry::model::linestring`
    /// from an iterator range
    /// (`boost/geometry/geometries/linestring.hpp:73-76`); the `Vec`
    /// is moved in rather than copied element-by-element.
    #[must_use]
    pub const fn from_vec(v: Vec<P>) -> Self {
        Self(v)
    }

    /// Append a point to the back of the linestring.
    ///
    /// Plays the role of `std::vector::push_back` on the inherited
    /// `base_type` in `boost/geometry/geometries/linestring.hpp:60-64`.
    pub fn push(&mut self, p: P) {
        self.0.push(p);
    }
}

impl<P: PointTrait> Default for Linestring<P> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Tag this concrete type as a Linestring. Mirrors the
/// `traits::tag<model::linestring<...>>` specialisation at
/// `boost/geometry/geometries/linestring.hpp:103-113`.
impl<P: PointTrait> Geometry for Linestring<P> {
    type Kind = LinestringTag;
    type Point = P;
}

/// Wire the Linestring concept up. The `points()` iterator hands back
/// `self.0.iter()` — `slice::Iter` is already
/// `ExactSizeIterator + Clone`, so no adapter is needed. Plays the
/// role of `boost::begin(ls)` / `boost::end(ls)` on
/// `model::linestring` (`boost/geometry/geometries/linestring.hpp:60-64`).
impl<P: PointTrait> LinestringTrait for Linestring<P> {
    fn points(&self) -> impl ExactSizeIterator<Item = &P> + Clone {
        self.0.iter()
    }
}

#[cfg(test)]
mod tests {
    //! Iteration + tag-resolution + `check_linestring` witness for
    //! [`Linestring`]. Mirrors
    //! `boost/geometry/test/core/tag.cpp` (the linestring-tag arm)
    //! and the iteration smoke tests in
    //! `boost/geometry/test/geometries/linestring.cpp`.

    use super::Linestring;
    use crate::point::Point2D;
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_tag::LinestringTag;
    use geometry_trait::{Geometry, Linestring as LinestringTrait, Point as _, check_linestring};

    #[test]
    fn linestring_iterates_in_declared_order() {
        let mut ls = Linestring::<Point2D<f64, Cartesian>>::new();
        ls.push(Point2D::new(0.0, 0.0));
        ls.push(Point2D::new(1.0, 1.0));
        ls.push(Point2D::new(2.0, 0.0));
        assert_eq!(ls.points().count(), 3);
        let xs: Vec<u64> = ls.points().map(|p| p.get::<0>().to_bits()).collect();
        assert_eq!(
            xs,
            vec![0.0_f64.to_bits(), 1.0_f64.to_bits(), 2.0_f64.to_bits()]
        );
    }

    #[test]
    fn linestring_iterator_reports_exact_size() {
        let ls = Linestring::<Point2D<f64, Cartesian>>::from_vec(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 1.0),
        ]);
        let it = ls.points();
        assert_eq!(it.len(), 2);
    }

    #[test]
    fn linestring_kind_is_linestring_tag() {
        fn k<T: Geometry<Kind = LinestringTag>>() {}
        k::<Linestring<Point2D<f64, Cartesian>>>();
    }

    #[test]
    fn linestring_satisfies_concept() {
        check_linestring::<Linestring<Point2D<f64, Cartesian>>>();
    }
}
