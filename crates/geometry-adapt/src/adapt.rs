//! The shape-only adapter newtype.
//!
//! Mirrors the role of the Boost `BOOST_GEOMETRY_REGISTER_*` macros
//! in `boost/geometry/geometries/adapted/c_array.hpp`,
//! `std_array.hpp`, and `boost_tuple.hpp`: those macros specialise
//! `traits::tag`, `traits::dimension`, `traits::coordinate_type`,
//! `traits::coordinate_system`, and `traits::access<P, D>` for a
//! foreign storage layout. Rust has no specialisation, so we wrap
//! the foreign value in [`Adapt`] and hang the trait impls off the
//! wrapper instead — coherence-safe and layout-free thanks to
//! `#[repr(transparent)]`.

/// Shape-only adapter wrapper.
///
/// `#[repr(transparent)]` so `&Adapt<T>` is layout-compatible with
/// `&T` — no boxing, no allocation, the wrapper is purely a place to
/// hang trait impls that coherence wouldn't allow on `T` directly.
///
/// Defaults to `Cs = Cartesian` for every shape this crate adapts.
/// Compose with [`WithCs<T, Cs>`](crate::WithCs) to re-tag with a
/// different coordinate system.
///
/// Mirrors `boost/geometry/geometries/adapted/{c_array,
/// std_array, boost_tuple}.hpp` collectively.
///
/// # Coordinate system default — silent-Cartesian warning
///
/// **`Adapt<T>` defaults to [`Cartesian`](geometry_cs::Cartesian).**
/// This matters for adapted containers like `[lat, lon]` arrays or
/// `(lat, lon)` tuples — by default the library will compute
/// *Cartesian Pythagorean* distances over them, **not** great-circle
/// distances. If your coordinates are spherical or geographic, wrap
/// them with [`WithCs`](crate::WithCs):
///
/// ```ignore
/// use geometry_adapt::{Adapt, WithCs};
/// use geometry_cs::{Degree, Geographic};
///
/// // WRONG — treats lon/lat as Cartesian, returns nonsense for distance.
/// let p = Adapt([4.9_f64, 52.4]);
///
/// // RIGHT — the Cs is in the type; distance() picks the right strategy
/// // (haversine / andoyer / vincenty) and Pythagoras refuses to compile.
/// let p = WithCs::<_, Geographic<Degree>>::new(Adapt([4.9_f64, 52.4]));
/// ```
///
/// The matching compile-time diagnostic that catches a bare
/// `Adapt<[lat, lon]>` from reaching `Pythagoras` lives on
/// [`SameAs`](geometry_tag::SameAs) — see proposal §3.7 and §8 for
/// the rationale and the mitigations.
///
/// # Borrowed arrays — read only
///
/// `Adapt<&[T; N]>` works just like `Adapt<[T; N]>` for the
/// read-only algorithm surface (`distance`, `length`, `area`, …),
/// without copying the storage. The borrow does not implement
/// `PointMut`, so any algorithm that needs to *output* a Point
/// (e.g. `envelope`) is a compile error — own your array if you
/// need mutation.
///
/// ```
/// use geometry_adapt::Adapt;
/// use geometry_algorithm::distance;
///
/// let a_storage = [0.0_f64, 0.0];
/// let b_storage = [3.0_f64, 4.0];
/// assert_eq!(distance(&Adapt(&a_storage), &Adapt(&b_storage)), 5.0);
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct Adapt<T>(pub T);

impl<T> Adapt<T> {
    /// Wrap a foreign value so the geometry concepts apply to it.
    #[inline]
    #[must_use]
    pub const fn new(value: T) -> Self {
        Self(value)
    }

    /// Unwrap, returning the foreign value.
    #[inline]
    #[must_use]
    pub fn into_inner(self) -> T {
        self.0
    }
}
