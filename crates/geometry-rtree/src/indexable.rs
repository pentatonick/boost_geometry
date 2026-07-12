//! The [`Indexable`] trait — what can be stored in an [`Rtree`].
//!
//! Mirrors `boost/geometry/index/indexable.hpp`: an indexable value is
//! anything from which the index can read an axis-aligned bounding box.
//! Boost's `indexable` function object maps points, boxes, and segments
//! to their bounds; the port makes that a trait with impls for the same
//! kinds.
//!
//! [`Rtree`]: crate::Rtree

use geometry_cs::CoordinateSystem;
use geometry_model::Point2D;
use geometry_trait::Point;

use crate::bounds::Bounds;

/// A value the [`Rtree`](crate::Rtree) can index — one that exposes an
/// axis-aligned bounding box.
///
/// Mirrors `boost::geometry::index::indexable<Value>`
/// (`index/indexable.hpp`). Implemented for the kernel's point type and
/// for [`Bounds`] itself; wrap any other payload in a `(Bounds, T)`
/// pair, which is also `Indexable`, to index it by a precomputed box.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_rtree::indexable::Indexable;
///
/// let p = Point2D::<f64, Cartesian>::new(3.0, 4.0);
/// let b = p.bounds();
/// assert_eq!(b.min, [3.0, 4.0]);
/// assert_eq!(b.max, [3.0, 4.0]);
/// ```
pub trait Indexable {
    /// The value's axis-aligned bounding box.
    fn bounds(&self) -> Bounds;
}

impl<Cs: CoordinateSystem> Indexable for Point2D<f64, Cs> {
    fn bounds(&self) -> Bounds {
        Bounds::point([self.get::<0>(), self.get::<1>()])
    }
}

impl Indexable for Bounds {
    fn bounds(&self) -> Bounds {
        *self
    }
}

/// Index an arbitrary payload `T` by a precomputed box. The tree stores
/// the pair and reads the box from `.0`.
impl<T> Indexable for (Bounds, T) {
    fn bounds(&self) -> Bounds {
        self.0
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact integer-valued coordinate literals")]
mod tests {
    use super::Indexable;
    use crate::bounds::Bounds;
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    #[test]
    fn point_bounds_is_degenerate() {
        let p = Point2D::<f64, Cartesian>::new(1.0, 2.0);
        let b = p.bounds();
        assert_eq!(b.min, [1.0, 2.0]);
        assert_eq!(b.max, [1.0, 2.0]);
    }

    #[test]
    fn box_is_its_own_bounds() {
        let b = Bounds::new([0.0, 0.0], [5.0, 5.0]);
        assert_eq!(b.bounds(), b);
    }

    #[test]
    fn payload_pair_indexes_by_box() {
        let value = (Bounds::new([1.0, 1.0], [2.0, 2.0]), "label");
        assert_eq!(value.bounds(), Bounds::new([1.0, 1.0], [2.0, 2.0]));
    }
}
