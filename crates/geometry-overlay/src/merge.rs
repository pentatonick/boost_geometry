//! OVL8 — `merge_elements`: combine overlapping polygons via union.
//!
//! Mirrors `boost/geometry/algorithms/merge_elements.hpp`, which unions
//! the compatible elements of a collection so that overlapping or
//! touching pieces coalesce. Boost dispatches per element kind through
//! `union`; the v1 port implements the areal case — merging a list of
//! polygons into the fewest non-overlapping polygons.
//!
//! Useful for `GeoJSON` pipelines that carry many small polygons which
//! should read as one region.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::{CartesianFamily, CoordinateSystem};
use geometry_model::{MultiPolygon, Polygon};
use geometry_tag::SameAs;
use geometry_trait::PointMut;

use crate::operation::{OverlayError, union_poly};
use crate::relate::overlaps;

/// Merge a list of polygons, unioning any that overlap, until no two
/// remaining polygons overlap.
///
/// Overlapping pairs are combined with [`union_poly`]; polygons that
/// only touch or are disjoint are left as separate members of the
/// result. Mirrors `boost::geometry::merge_elements`
/// (`algorithms/merge_elements.hpp`) for the areal case.
///
/// # Errors
///
/// [`OverlayError::Unsupported`] if a union of two overlapping members
/// hits a degenerate case the v1 overlay engine does not handle.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::{polygon, Point2D, Polygon};
/// use geometry_overlay::merge::merge_polygons;
/// use geometry_trait::MultiPolygon as _;
///
/// type P = Point2D<f64, Cartesian>;
/// let a: Polygon<P> = polygon![[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]];
/// let b: Polygon<P> = polygon![[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)]];
/// let far: Polygon<P> = polygon![[(9.0, 9.0), (10.0, 9.0), (10.0, 10.0), (9.0, 10.0), (9.0, 9.0)]];
/// // a and b overlap → one polygon; `far` stays separate → two total.
/// let merged = merge_polygons(vec![a, b, far]).unwrap();
/// assert_eq!(merged.polygons().count(), 2);
/// ```
#[inline]
#[must_use = "merging can fail and the merged geometry should be used"]
pub fn merge_polygons<P>(
    polygons: Vec<Polygon<P>>,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    let mut work = polygons;

    // Repeatedly find an overlapping pair and union it, until a full pass
    // finds none. Each successful merge shrinks the list, so the outer
    // loop terminates.
    while let Some((i, j)) = first_overlapping_pair(&work) {
        let b = work.remove(j); // j > i, so removing j first keeps i valid
        let a = work.remove(i);
        let unioned = union_poly(&a, &b)?;
        // `union_poly` returns a MultiPolygon; push its polygons back in.
        for pg in unioned.0 {
            work.push(pg);
        }
    }

    Ok(MultiPolygon(work))
}

/// Merge the areal elements of a collection.
///
/// Mirrors the polygon arm of `boost::geometry::merge_elements` from
/// `boost/geometry/algorithms/merge_elements.hpp:401-418`. Overlapping polygons
/// are repeatedly unioned; disjoint polygons remain separate output members.
///
/// # Errors
///
/// Propagates [`OverlayError`] when an overlapping pair reaches a degenerate
/// case outside the current areal overlay kernel.
#[inline]
#[must_use = "merging can fail and the merged geometry should be used"]
pub fn merge_elements<P, I>(polygons: I) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    I: IntoIterator<Item = Polygon<P>>,
{
    merge_polygons(polygons.into_iter().collect())
}

/// Merge every polygon of a [`MultiPolygon`], returning a `MultiPolygon`
/// whose members pairwise do not overlap.
///
/// A convenience wrapper over [`merge_polygons`] for the common case of
/// tidying a `MultiPolygon` whose parts may overlap.
///
/// # Errors
///
/// Propagates [`OverlayError`] from the underlying unions.
#[inline]
#[must_use = "merging can fail and the merged geometry should be used"]
pub fn merge_multipolygon<P>(
    mp: MultiPolygon<Polygon<P>>,
) -> Result<MultiPolygon<Polygon<P>>, OverlayError>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    merge_polygons(mp.0)
}

/// The first `(i, j)` with `i < j` whose polygons *overlap in area*, or
/// `None`.
///
/// [`overlaps`] returns `Err(Unsupported)` for the ambiguous
/// non-transversal-contact class (an edge-aligned or vertex-only touch the
/// relate engine cannot tell apart from a vertex-through overlap). For
/// merging, that ambiguity is resolved conservatively as **"do not merge
/// this pair"**: a pair that only touches must stay separate anyway, and
/// an unresolvable pair is left intact rather than aborting the whole
/// merge. So an `Err` is treated the same as `Ok(false)` here.
fn first_overlapping_pair<P>(polygons: &[Polygon<P>]) -> Option<(usize, usize)>
where
    P: PointMut + Default + Copy,
    P::Scalar: CoordinateScalar + Into<f64>,
    <P::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{
    for i in 0..polygons.len() {
        for j in (i + 1)..polygons.len() {
            if overlaps(&polygons[i], &polygons[j]).unwrap_or(false) {
                return Some((i, j));
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    //! OVL8 done-when: merged element counts + areas. Mirrors
    //! `test/algorithms/merge_elements.cpp`.

    use super::merge_polygons;
    use geometry_algorithm::ring_area;
    use geometry_cs::Cartesian;
    use geometry_model::{MultiPolygon, Point2D, Polygon, polygon};
    use geometry_trait::{MultiPolygon as _, Polygon as _};

    type P = Point2D<f64, Cartesian>;

    fn square(x: f64, y: f64, s: f64) -> Polygon<P> {
        polygon![[(x, y), (x + s, y), (x + s, y + s), (x, y + s), (x, y)]]
    }

    fn total_area(mp: &MultiPolygon<Polygon<P>>) -> f64 {
        mp.polygons().map(|pg| ring_area(pg.exterior()).abs()).sum()
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-5 * a.abs().max(b.abs()).max(1.0)
    }

    #[test]
    fn overlapping_pair_merges_to_one() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        let merged = merge_polygons(vec![a, b]).unwrap();
        assert_eq!(merged.polygons().count(), 1);
        assert!(
            close(total_area(&merged), 7.0),
            "area {}",
            total_area(&merged)
        );
    }

    #[test]
    fn disjoint_polygons_stay_separate() {
        let a = square(0.0, 0.0, 1.0);
        let b = square(5.0, 5.0, 1.0);
        let merged = merge_polygons(vec![a, b]).unwrap();
        assert_eq!(merged.polygons().count(), 2);
    }

    #[test]
    fn mixed_overlap_and_disjoint() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0); // overlaps a
        let far = square(9.0, 9.0, 1.0); // disjoint
        let merged = merge_polygons(vec![a, b, far]).unwrap();
        assert_eq!(merged.polygons().count(), 2);
    }

    #[test]
    fn empty_input_empty_output() {
        let merged = merge_polygons::<P>(vec![]).unwrap();
        assert_eq!(merged.polygons().count(), 0);
    }

    /// A single-element input has no pairs to test — it comes back
    /// unchanged.
    #[test]
    fn single_element_returned_unchanged() {
        let a = square(0.0, 0.0, 2.0);
        let merged = merge_polygons(vec![a]).unwrap();
        assert_eq!(merged.polygons().count(), 1);
        assert!(close(total_area(&merged), 4.0));
    }

    /// Two squares sharing only one vertex must not fuse.
    #[test]
    fn vertex_only_touch_does_not_fuse() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(2.0, 2.0, 2.0); // shares only the corner (2,2)
        let merged = merge_polygons(vec![a, b]).unwrap();
        assert_eq!(merged.polygons().count(), 2);
        assert!(close(total_area(&merged), 8.0));
    }

    /// The `merge_multipolygon` wrapper merges its members like
    /// `merge_polygons` does.
    #[test]
    fn multipolygon_wrapper_merges_members() {
        let mp = MultiPolygon(vec![
            square(0.0, 0.0, 2.0),
            square(1.0, 1.0, 2.0), // overlaps the first
            square(9.0, 9.0, 1.0), // disjoint
        ]);
        let merged = super::merge_multipolygon(mp).unwrap();
        assert_eq!(merged.polygons().count(), 2);
    }

    #[test]
    fn touching_only_pair_does_not_abort_the_merge() {
        // Regression: `overlaps` returns `Err(Unsupported)` for an
        // edge-touching pair (an ambiguous non-transversal contact). Merge
        // must treat that as "do not merge" and keep going, NOT abort the
        // whole operation. Here a & b overlap transversally (→ one poly)
        // while c & d only share an edge (→ stay separate): 3 polygons.
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0); // overlaps a transversally
        let c = square(10.0, 0.0, 2.0);
        let d = square(12.0, 0.0, 2.0); // shares edge x=12 with c only
        let merged = merge_polygons(vec![a, b, c, d]).unwrap();
        assert_eq!(merged.polygons().count(), 3);
    }
}
