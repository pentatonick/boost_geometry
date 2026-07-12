//! The [`GeometryCollection`] concept — a sequence of geometries.
//!
//! Mirrors `boost::geometry::model::geometry_collection` in
//! `boost/geometry/geometries/geometry_collection.hpp` and the
//! `geometry_collection_tag` marker in
//! `boost/geometry/core/tags.hpp` (`struct geometry_collection_tag :
//! multi_tag {};`). The C++ model derives from
//! `Container<DynamicGeometry, Allocator<DynamicGeometry>>` and is
//! consumed through `boost::range`; this Rust port surfaces the
//! element sequence via an RPITIT `items()` iterator, matching the
//! pattern already used by the multi-* traits.
//!
//! **Scope:** v1 only supplies the trait surface. Heterogeneous
//! dynamic dispatch (a `dyn`/`enum`-based story that would let one
//! collection mix Points, Linestrings, Polygons, …) is intentionally
//! deferred — see `specs/FUTURE_ITERATIONS.md` §1.3 (`DynGeometry`
//! variant) and §1.4 (heterogeneous `GeometryCollection`). The
//! [`GeometryCollection::Item`] associated type is therefore bound on
//! [`Geometry`] alone, so today's impls are expected to be
//! homogeneous in practice (e.g. `Vec<Polygon<P>>`).

use crate::geometry::Geometry;
use geometry_tag::GeometryCollectionTag;

/// A collection of geometries belonging to each other.
///
/// Mirrors the `GeometryCollection` concept; the canonical C++ model
/// is `boost::geometry::model::geometry_collection` in
/// `boost/geometry/geometries/geometry_collection.hpp`, tagged with
/// `geometry_collection_tag` (`boost/geometry/core/tags.hpp`).
///
/// In v1 the [`Item`](Self::Item) associated type is bound only on
/// [`Geometry`] — every concrete impl is homogeneous (a single
/// element type for the whole collection). Heterogeneous dispatch is
/// intentionally deferred to a later iteration; see
/// `specs/FUTURE_ITERATIONS.md` §1.3 (`DynGeometry` variant) and
/// §1.4 (heterogeneous `GeometryCollection`).
///
/// # Examples
///
/// ```
/// use geometry_trait::GeometryCollection;
/// fn count<C: GeometryCollection>(c: &C) -> usize { c.items().len() }
/// ```
pub trait GeometryCollection: Geometry<Kind = GeometryCollectionTag> {
    /// The element geometry type.
    ///
    /// Mirrors the `DynamicGeometry` template parameter on
    /// `boost::geometry::model::geometry_collection<DynamicGeometry, …>`
    /// (`boost/geometry/geometries/geometry_collection.hpp`). In v1
    /// the bound is just [`Geometry`]; once §1.4 lands this will
    /// tighten to a dynamic-geometry concept.
    type Item: Geometry;

    /// The items of this collection, in declared order.
    ///
    /// Plays the role of `boost::begin(gc)` / `boost::end(gc)` from
    /// `boost/geometry/geometries/geometry_collection.hpp` when read
    /// together.
    fn items(&self) -> impl ExactSizeIterator<Item = &Self::Item>;
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::point::{Point, PointMut};
    use alloc::vec;
    use alloc::vec::Vec;
    use geometry_cs::Cartesian;
    use geometry_tag::PointTag;

    // Witness: any T satisfying the trait is accepted. Static,
    // compile-time check; never runs.
    fn accepts_gc<G: GeometryCollection>() {}

    // Shared 2D Cartesian point used by the test impl below.
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

    impl PointMut for Xy {
        fn set<const D: usize>(&mut self, v: f64) {
            if D == 0 {
                self.0 = v;
            } else {
                self.1 = v;
            }
        }
    }

    // Homogeneous Vec-backed collection of points.
    struct ManyPoints(Vec<Xy>);

    impl Geometry for ManyPoints {
        type Kind = GeometryCollectionTag;
        type Point = Xy;
    }

    impl GeometryCollection for ManyPoints {
        type Item = Xy;

        fn items(&self) -> impl ExactSizeIterator<Item = &Xy> {
            self.0.iter()
        }
    }

    #[test]
    fn collection_iterates_all_items() {
        let m = ManyPoints(vec![Xy(0.0, 0.0), Xy(1.0, 1.0), Xy(2.0, 2.0)]);
        accepts_gc::<ManyPoints>();
        assert_eq!(m.items().len(), 3);
        let xs: Vec<f64> = m.items().map(Xy::get::<0>).collect();
        assert_eq!(xs, vec![0.0, 1.0, 2.0]);
    }
}
