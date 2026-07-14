//! The [`Linestring`] concept: an ordered sequence of points
//! (>= 2) with no implicit closing edge.
//!
//! Mirrors `doc/concept/linestring.qbk` and the model in
//! `boost/geometry/geometries/linestring.hpp`. Where the C++ side
//! exposes the underlying container through `boost::range`
//! (`begin`/`end` + `range_value`), the Rust port surfaces the
//! sequence as a `points()` iterator returned via return-position
//! `impl Trait` in trait (RPITIT) — see proposal §3.6.

use crate::geometry::Geometry;
use geometry_tag::LinestringTag;

/// A linestring — an ordered sequence of >= 2 points.
///
/// Mirrors the Linestring concept (`doc/concept/linestring.qbk`); the
/// canonical model is `boost::geometry::model::linestring` in
/// `boost/geometry/geometries/linestring.hpp`, which derives from
/// `std::vector<Point>` and is consumed through `boost::range`.
///
/// The C++ side leaves the iterator type up to each model and reads
/// it via `boost::range_iterator<G>::type`. The Rust counterpart uses
/// return-position `impl Trait` in trait so an impl can hand back
/// whatever iterator its storage produces (`slice::Iter`,
/// `VecDeque::Iter`, an adapter, ...) without naming it. The
/// `ExactSizeIterator + Clone` bound mirrors the random-access shape
/// `boost::range` relies on: callers can ask for `.len()` and
/// re-iterate without consuming the source.
///
/// # Examples
///
/// ```
/// use geometry_trait::Linestring;
/// fn point_count<L: Linestring>(ls: &L) -> usize { ls.points().len() }
/// ```
pub trait Linestring: Geometry<Kind = LinestringTag> {
    /// The points of this linestring, in declared order.
    ///
    /// Plays the role of `boost::begin(ls)` / `boost::end(ls)` from
    /// `boost/geometry/geometries/linestring.hpp` when read together.
    fn points(&self) -> impl ExactSizeIterator<Item = &Self::Point> + Clone;
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::point::Point;
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_tag::PointTag;

    struct Xy(f64, f64);

    impl Geometry for Xy {
        type Kind = PointTag;
        type Point = Self;
    }

    impl Point for Xy {
        type Scalar = f64;
        type Cs = Cartesian;
        const DIM: usize = 2;

        fn get<const D: usize>(&self) -> f64 {
            if D == 0 { self.0 } else { self.1 }
        }
    }

    struct VLs(Vec<Xy>);

    impl Geometry for VLs {
        type Kind = LinestringTag;
        type Point = Xy;
    }

    impl Linestring for VLs {
        fn points(&self) -> impl ExactSizeIterator<Item = &Xy> + Clone {
            self.0.iter()
        }
    }

    #[test]
    fn linestring_iterates_in_declared_order() {
        let ls = VLs(vec![Xy(0.0, 0.0), Xy(3.0, 4.0), Xy(4.0, 3.0)]);
        assert_eq!(ls.points().count(), 3);
        let xs: Vec<f64> = ls.points().map(Xy::get::<0>).collect();
        assert_eq!(xs, vec![0.0, 3.0, 4.0]);
        let ys: Vec<f64> = ls.points().map(Xy::get::<1>).collect();
        assert_eq!(ys, vec![0.0, 4.0, 3.0]);
    }

    #[test]
    fn linestring_iterator_reports_exact_size_and_clones() {
        let ls = VLs(vec![Xy(0.0, 0.0), Xy(1.0, 1.0)]);
        let it = ls.points();
        assert_eq!(it.len(), 2);
        // Cloning the iterator must not consume the source.
        let it2 = it.clone();
        assert_eq!(it.count(), 2);
        assert_eq!(it2.count(), 2);
    }
}
