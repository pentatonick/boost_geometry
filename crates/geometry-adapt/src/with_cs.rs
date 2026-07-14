//! The coordinate-system re-tagging wrapper.
//!
//! Mirrors no single Boost header — C++ would simply specialise a
//! different `traits::coordinate_system` for a different adapter
//! type. In Rust, with the orphan rule and no specialisation, we
//! lift the choice of coordinate system into its own zero-cost
//! newtype that composes on top of any [`Point`].
//!
//! See proposal §3.7 ("Adapting foreign types: the coherence story")
//! for the design rationale: `Adapt<T>` answers *"how do I read
//! coordinates out of this foreign data layout?"* while
//! [`WithCs`] answers *"what does that coordinate pair mean?"*.
//! The two decisions are orthogonal, so they get independent
//! wrappers.

use core::marker::PhantomData;

use geometry_cs::CoordinateSystem;
use geometry_tag::PointTag;
use geometry_trait::{Geometry, Point, PointMut};

/// Re-tag any [`Point`] with a different coordinate system, without
/// changing its storage layout.
///
/// Zero-cost: `#[repr(transparent)]` plus pure forwarding impls
/// means `&WithCs<T, Cs>` is layout-compatible with `&T` and
/// monomorphises to identical code. Only the `Cs` associated type
/// of the [`Point`] impl changes — `Scalar`, `DIM`, `get`, and
/// `set` all forward to the inner point.
///
/// See proposal §3.7 for the design rationale (orthogonality of
/// shape vs. coordinate system, and the silent-Cartesian risk
/// mitigation that lives in T31).
///
/// # Example
///
/// ```
/// use geometry_adapt::{Adapt, WithCs};
/// use geometry_cs::{Degree, Geographic};
/// use geometry_trait::Point;
///
/// // An [f64; 2] adapted as a Cartesian point, then re-tagged as
/// // a geographic lat/lon pair in degrees.
/// let p: WithCs<Adapt<[f64; 2]>, Geographic<Degree>> =
///     WithCs::new(Adapt([4.9, 52.4]));
/// assert_eq!(p.get::<0>(), 4.9);
/// assert_eq!(p.get::<1>(), 52.4);
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct WithCs<T, Cs: CoordinateSystem> {
    /// The wrapped inner point. Public so call sites can read or
    /// mutate the foreign storage directly when convenient.
    pub inner: T,
    _cs: PhantomData<Cs>,
}

impl<T, Cs: CoordinateSystem> WithCs<T, Cs> {
    /// Wrap an inner point and tag it with coordinate system `Cs`.
    #[inline]
    #[must_use]
    pub const fn new(inner: T) -> Self {
        Self {
            inner,
            _cs: PhantomData,
        }
    }

    /// Unwrap, returning the inner point.
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> T {
        self.inner
    }
}

impl<P: Point, Cs: CoordinateSystem> Geometry for WithCs<P, Cs> {
    type Kind = PointTag;
    type Point = Self;
}

impl<P: Point, Cs: CoordinateSystem> Point for WithCs<P, Cs> {
    type Scalar = P::Scalar;
    type Cs = Cs;
    const DIM: usize = P::DIM;

    #[inline]
    fn get<const D: usize>(&self) -> P::Scalar {
        self.inner.get::<D>()
    }
}

impl<P: PointMut, Cs: CoordinateSystem> PointMut for WithCs<P, Cs> {
    #[inline]
    fn set<const D: usize>(&mut self, value: P::Scalar) {
        self.inner.set::<D>(value);
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "Every assertion here reads back a value just written, \
              with no arithmetic in between, so strict equality is exact."
)]
mod tests {
    use super::*;
    use crate::Adapt;
    use geometry_cs::{Cartesian, CoordinateSystem, Degree, Geographic, Spherical};
    use geometry_trait::{Point, PointMut};

    // Coordinates are passed through unchanged.
    #[test]
    fn re_tag_array_as_geographic_preserves_values() {
        let p: WithCs<Adapt<[f64; 2]>, Geographic<Degree>> = WithCs::new(Adapt([4.9, 52.4]));
        assert_eq!(p.get::<0>(), 4.9);
        assert_eq!(p.get::<1>(), 52.4);
        assert_eq!(
            <WithCs<Adapt<[f64; 2]>, Geographic<Degree>> as Point>::DIM,
            2
        );
    }

    // The new Cs *is* in the type — the family witness changes.
    #[test]
    fn re_tag_changes_family() {
        fn family<P: Point>() -> &'static str
        where
            <P::Cs as CoordinateSystem>::Family: 'static,
        {
            core::any::type_name::<<P::Cs as CoordinateSystem>::Family>()
        }
        // Before: Cartesian.
        assert!(family::<Adapt<[f64; 2]>>().contains("CartesianFamily"));
        // After WithCs<Geographic>: Geographic.
        assert!(
            family::<WithCs<Adapt<[f64; 2]>, Geographic<Degree>>>().contains("GeographicFamily")
        );
        // And Spherical works just as well.
        assert!(family::<WithCs<Adapt<[f64; 2]>, Spherical<Degree>>>().contains("SphericalFamily"));
    }

    // Stacking: re-tag a user-owned Cartesian point as Spherical.
    struct MyXy(f64, f64);

    impl Geometry for MyXy {
        type Kind = PointTag;
        type Point = Self;
    }

    impl Point for MyXy {
        type Scalar = f64;
        type Cs = Cartesian;
        const DIM: usize = 2;

        fn get<const D: usize>(&self) -> f64 {
            if D == 0 { self.0 } else { self.1 }
        }
    }

    impl PointMut for MyXy {
        fn set<const D: usize>(&mut self, v: f64) {
            if D == 0 {
                self.0 = v;
            } else {
                self.1 = v;
            }
        }
    }

    #[test]
    fn re_tag_user_point() {
        let p: WithCs<MyXy, Spherical<Degree>> = WithCs::new(MyXy(1.0, 2.0));
        assert_eq!(p.get::<0>(), 1.0);
        assert_eq!(p.get::<1>(), 2.0);
    }

    /// `PointMut::set` on a `WithCs` writes through to the inner point's
    /// storage; re-reading through `get` returns the written value.
    #[test]
    fn set_writes_through_to_inner() {
        let mut p: WithCs<MyXy, Spherical<Degree>> = WithCs::new(MyXy(0.0, 0.0));
        p.set::<0>(7.0);
        p.set::<1>(9.0);
        assert_eq!(p.get::<0>(), 7.0);
        assert_eq!(p.get::<1>(), 9.0);
        // The change landed in the wrapped MyXy, visible after unwrap.
        let inner = p.into_inner();
        assert_eq!(inner.0, 7.0);
        assert_eq!(inner.1, 9.0);
    }

    // `#[repr(transparent)]` guarantees layout-compat; this is a
    // sanity check rather than a real test, but it pins the
    // assumption that adapter code depends on.
    #[test]
    fn size_is_inner_size() {
        assert_eq!(
            core::mem::size_of::<WithCs<Adapt<[f64; 2]>, Geographic<Degree>>>(),
            core::mem::size_of::<[f64; 2]>(),
        );
    }
}
