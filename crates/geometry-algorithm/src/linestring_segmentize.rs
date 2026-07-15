//! Split a linestring into equal-length pieces.
//!
//! The default is Cartesian. The explicit-strategy entry also supports
//! spherical Haversine measurement and great-circle interpolation.

use geometry_strategy::{CartesianSegmentize, SegmentizeStrategy};

/// Split a linestring into equal Cartesian arc-length pieces.
#[inline]
#[must_use]
pub fn linestring_segmentize<L>(
    line: &L,
    count: usize,
) -> <CartesianSegmentize as SegmentizeStrategy<L>>::Output
where
    CartesianSegmentize: SegmentizeStrategy<L>,
{
    CartesianSegmentize.segmentize(line, count)
}

/// Split a linestring using an explicitly supplied measurement and
/// interpolation strategy.
#[inline]
#[must_use]
#[allow(
    clippy::needless_pass_by_value,
    reason = "segmentization strategies are zero-sized or small Copy values, matching other _with entries"
)]
pub fn linestring_segmentize_with<L, S>(line: &L, count: usize, strategy: S) -> S::Output
where
    S: SegmentizeStrategy<L>,
{
    strategy.segmentize(line, count)
}

#[cfg(test)]
mod tests {
    use super::linestring_segmentize;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D};

    type P = Point2D<f64, Cartesian>;

    #[test]
    fn equal_pieces_keep_original_corner_vertices() {
        let line = Linestring::from_vec(vec![P::new(0.0, 0.0), P::new(2.0, 0.0), P::new(2.0, 2.0)]);
        let pieces = linestring_segmentize(&line, 2);
        assert_eq!(pieces.0.len(), 2);
        assert_eq!(pieces.0[0].0.last(), Some(&P::new(2.0, 0.0)));
        assert_eq!(pieces.0[1].0.first(), Some(&P::new(2.0, 0.0)));
        assert!(linestring_segmentize(&line, 0).0.is_empty());
    }
}
