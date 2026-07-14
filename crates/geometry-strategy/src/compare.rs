//! Point-ordering policies.
//!
//! Mirrors `boost/geometry/policies/compare.hpp` and the default Cartesian,
//! spherical, and geographic strategies selected from
//! `boost/geometry/strategies/{compare,spherical/compare}.hpp`.
//!
//! The default dimension (`-1`) compares lexicographically across the
//! dimensions shared by both points. A non-negative const parameter compares
//! only that dimension. Spherical and geographic longitude comparison treats
//! `-180°` and `180°` as the same meridian, orders the antimeridian after
//! ordinary longitudes, and ignores longitude differences at a shared pole.

use core::cmp::Ordering;

use geometry_coords::{CoordinateScalar, Rational, RationalInteger};
use geometry_cs::{
    AngleUnit, CartesianFamily, CoordinateSystem, Geographic, GeographicFamily, Spherical,
    SphericalFamily,
};
use geometry_tag::SameAs;
use geometry_trait::Point;

/// Sentinel used by the comparison policies to inspect all shared dimensions.
pub const ALL_DIMENSIONS: i8 = -1;

/// Sort points in lexicographically ascending order using epsilon equality.
///
/// Mirrors `boost::geometry::less` from `policies/compare.hpp:75-265`.
#[derive(Debug, Default, Clone, Copy)]
pub struct Less<const DIMENSION: i8 = ALL_DIMENSIONS>;

/// Sort points in lexicographically ascending order using exact equality.
///
/// Mirrors `boost::geometry::less_exact` from
/// `policies/compare.hpp:35-73`.
#[derive(Debug, Default, Clone, Copy)]
pub struct LessExact<const DIMENSION: i8 = ALL_DIMENSIONS>;

/// Sort points in lexicographically descending order using epsilon equality.
///
/// Mirrors `boost::geometry::greater` from
/// `policies/compare.hpp:268-367`.
#[derive(Debug, Default, Clone, Copy)]
pub struct Greater<const DIMENSION: i8 = ALL_DIMENSIONS>;

/// Test points for coordinate-wise epsilon equality.
///
/// Mirrors `boost::geometry::equal_to` from
/// `policies/compare.hpp:370-470`.
#[derive(Debug, Default, Clone, Copy)]
pub struct EqualTo<const DIMENSION: i8 = ALL_DIMENSIONS>;

impl<const DIMENSION: i8> Less<DIMENSION> {
    /// Return whether `left` precedes `right` under this policy.
    ///
    /// # Panics
    ///
    /// Panics when `DIMENSION` is neither [`ALL_DIMENSIONS`] nor a dimension
    /// present in both points.
    #[inline]
    #[must_use]
    #[allow(
        clippy::unused_self,
        reason = "value-method policy objects are directly usable as sorting functors"
    )]
    pub fn apply<P1, P2>(self, left: &P1, right: &P2) -> bool
    where
        P1: Point,
        P2: Point,
        <P1::Cs as CoordinateSystem>::Family: ComparisonFamily<P1, P2>,
    {
        <<P1::Cs as CoordinateSystem>::Family as ComparisonFamily<P1, P2>>::less(
            left, right, DIMENSION, false,
        )
    }
}

impl<const DIMENSION: i8> LessExact<DIMENSION> {
    /// Return whether `left` exactly precedes `right` under this policy.
    ///
    /// # Panics
    ///
    /// Panics when `DIMENSION` is neither [`ALL_DIMENSIONS`] nor a dimension
    /// present in both points.
    #[inline]
    #[must_use]
    #[allow(
        clippy::unused_self,
        reason = "value-method policy objects are directly usable as sorting functors"
    )]
    pub fn apply<P1, P2>(self, left: &P1, right: &P2) -> bool
    where
        P1: Point,
        P2: Point,
        <P1::Cs as CoordinateSystem>::Family: ComparisonFamily<P1, P2>,
    {
        <<P1::Cs as CoordinateSystem>::Family as ComparisonFamily<P1, P2>>::less(
            left, right, DIMENSION, true,
        )
    }
}

impl<const DIMENSION: i8> Greater<DIMENSION> {
    /// Return whether `left` follows `right` under this policy.
    ///
    /// # Panics
    ///
    /// Panics when `DIMENSION` is neither [`ALL_DIMENSIONS`] nor a dimension
    /// present in both points.
    #[inline]
    #[must_use]
    #[allow(
        clippy::unused_self,
        reason = "value-method policy objects are directly usable as sorting functors"
    )]
    pub fn apply<P1, P2>(self, left: &P1, right: &P2) -> bool
    where
        P1: Point,
        P2: Point,
        <P1::Cs as CoordinateSystem>::Family: ComparisonFamily<P1, P2>,
    {
        <<P1::Cs as CoordinateSystem>::Family as ComparisonFamily<P1, P2>>::greater(
            left, right, DIMENSION,
        )
    }
}

impl<const DIMENSION: i8> EqualTo<DIMENSION> {
    /// Return whether the selected coordinates compare equal within epsilon.
    ///
    /// # Panics
    ///
    /// Panics when `DIMENSION` is neither [`ALL_DIMENSIONS`] nor a dimension
    /// present in both points.
    #[inline]
    #[must_use]
    #[allow(
        clippy::unused_self,
        reason = "value-method policy objects are directly usable as sorting functors"
    )]
    pub fn apply<P1, P2>(self, left: &P1, right: &P2) -> bool
    where
        P1: Point,
        P2: Point,
        <P1::Cs as CoordinateSystem>::Family: ComparisonFamily<P1, P2>,
    {
        <<P1::Cs as CoordinateSystem>::Family as ComparisonFamily<P1, P2>>::equal(
            left, right, DIMENSION,
        )
    }
}

/// Coordinate-family dispatch used by [`Less`], [`Greater`], and [`EqualTo`].
///
/// This is public only because it appears in the generic policy method bounds;
/// downstream implementations are sealed.
#[doc(hidden)]
pub trait ComparisonFamily<P1: Point, P2: Point>: sealed::ComparisonFamily {
    #[doc(hidden)]
    fn less(left: &P1, right: &P2, dimension: i8, exact: bool) -> bool;

    #[doc(hidden)]
    fn greater(left: &P1, right: &P2, dimension: i8) -> bool;

    #[doc(hidden)]
    fn equal(left: &P1, right: &P2, dimension: i8) -> bool;
}

mod sealed {
    pub trait ComparisonFamily {}

    impl ComparisonFamily for geometry_cs::CartesianFamily {}
    impl ComparisonFamily for geometry_cs::SphericalFamily {}
    impl ComparisonFamily for geometry_cs::GeographicFamily {}
}

impl<P1, P2> ComparisonFamily<P1, P2> for CartesianFamily
where
    P1: Point,
    P2: Point,
    <P1::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    P1::Scalar: CoordinateComparison<P2::Scalar>,
{
    #[inline]
    fn less(left: &P1, right: &P2, dimension: i8, exact: bool) -> bool {
        cartesian_compare(left, right, dimension, Relation::Less, exact)
    }

    #[inline]
    fn greater(left: &P1, right: &P2, dimension: i8) -> bool {
        cartesian_compare(left, right, dimension, Relation::Greater, false)
    }

    #[inline]
    fn equal(left: &P1, right: &P2, dimension: i8) -> bool {
        cartesian_compare(left, right, dimension, Relation::Equal, false)
    }
}

impl<P1, P2> ComparisonFamily<P1, P2> for SphericalFamily
where
    P1: Point,
    P2: Point,
    P1::Cs: AngularCoordinateSystem,
    P2::Cs: AngularCoordinateSystem,
    <P1::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<SphericalFamily>,
    P1::Scalar: AngularScalar + CoordinateComparison<P2::Scalar>,
    P2::Scalar: AngularScalar,
{
    #[inline]
    fn less(left: &P1, right: &P2, dimension: i8, exact: bool) -> bool {
        angular_compare(left, right, dimension, Relation::Less, exact)
    }

    #[inline]
    fn greater(left: &P1, right: &P2, dimension: i8) -> bool {
        angular_compare(left, right, dimension, Relation::Greater, false)
    }

    #[inline]
    fn equal(left: &P1, right: &P2, dimension: i8) -> bool {
        angular_compare(left, right, dimension, Relation::Equal, false)
    }
}

impl<P1, P2> ComparisonFamily<P1, P2> for GeographicFamily
where
    P1: Point,
    P2: Point,
    P1::Cs: AngularCoordinateSystem,
    P2::Cs: AngularCoordinateSystem,
    <P1::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<GeographicFamily>,
    P1::Scalar: AngularScalar + CoordinateComparison<P2::Scalar>,
    P2::Scalar: AngularScalar,
{
    #[inline]
    fn less(left: &P1, right: &P2, dimension: i8, exact: bool) -> bool {
        angular_compare(left, right, dimension, Relation::Less, exact)
    }

    #[inline]
    fn greater(left: &P1, right: &P2, dimension: i8) -> bool {
        angular_compare(left, right, dimension, Relation::Greater, false)
    }

    #[inline]
    fn equal(left: &P1, right: &P2, dimension: i8) -> bool {
        angular_compare(left, right, dimension, Relation::Equal, false)
    }
}

#[derive(Clone, Copy)]
enum Relation {
    Less,
    Greater,
    Equal,
}

fn cartesian_compare<P1, P2>(
    left: &P1,
    right: &P2,
    dimension: i8,
    relation: Relation,
    exact: bool,
) -> bool
where
    P1: Point,
    P2: Point,
    P1::Scalar: CoordinateComparison<P2::Scalar>,
{
    let shared_dimensions = P1::DIM.min(P2::DIM);
    validate_dimension(dimension, shared_dimensions);

    if dimension == ALL_DIMENSIONS {
        for index in 0..shared_dimensions {
            let result = compare_ordinate(left, right, index, relation, exact);
            if let Some(result) = result {
                return result;
            }
        }
        matches!(relation, Relation::Equal)
    } else {
        compare_ordinate(left, right, dimension_index(dimension), relation, exact)
            .unwrap_or(matches!(relation, Relation::Equal))
    }
}

fn angular_compare<P1, P2>(
    left: &P1,
    right: &P2,
    dimension: i8,
    relation: Relation,
    exact: bool,
) -> bool
where
    P1: Point,
    P2: Point,
    P1::Cs: AngularCoordinateSystem,
    P2::Cs: AngularCoordinateSystem,
    P1::Scalar: AngularScalar + CoordinateComparison<P2::Scalar>,
    P2::Scalar: AngularScalar,
{
    let shared_dimensions = P1::DIM.min(P2::DIM);
    validate_dimension(dimension, shared_dimensions);

    if dimension >= 2 {
        return compare_ordinate(left, right, dimension_index(dimension), relation, exact)
            .unwrap_or(matches!(relation, Relation::Equal));
    }

    if dimension == 1 {
        return angular_latitude(left, right, relation, exact);
    }

    let longitude = angular_longitude(left, right, relation, exact);
    if let Some(result) = longitude {
        return result;
    }
    if dimension == 0 {
        return matches!(relation, Relation::Equal);
    }

    let latitude = angular_latitude_result(left, right, relation, exact);
    if let Some(result) = latitude {
        return result;
    }
    for index in 2..shared_dimensions {
        if let Some(result) = compare_ordinate(left, right, index, relation, exact) {
            return result;
        }
    }
    matches!(relation, Relation::Equal)
}

fn angular_longitude<P1, P2>(left: &P1, right: &P2, relation: Relation, exact: bool) -> Option<bool>
where
    P1: Point,
    P2: Point,
    P1::Cs: AngularCoordinateSystem,
    P2::Cs: AngularCoordinateSystem,
    P1::Scalar: AngularScalar,
    P2::Scalar: AngularScalar,
{
    let left_longitude = P1::Cs::to_radians(left.get::<0>());
    let right_longitude = P2::Cs::to_radians(right.get::<0>());
    let epsilon = angular_epsilon::<P1::Scalar, P2::Scalar>();
    let coordinates_equal = values_equal(left_longitude, right_longitude, epsilon, exact);
    let left_antimeridian = is_antimeridian(left_longitude, epsilon, exact);
    let right_antimeridian = is_antimeridian(right_longitude, epsilon, exact);

    let left_latitude = P1::Cs::to_radians(left.get::<1>());
    let right_latitude = P2::Cs::to_radians(right.get::<1>());
    let same_latitude = values_equal(left_latitude, right_latitude, epsilon, exact);
    let shared_pole = same_latitude && is_pole(left_latitude, epsilon, exact);

    if coordinates_equal || (left_antimeridian && right_antimeridian) || shared_pole {
        None
    } else if left_antimeridian {
        Some(apply_ordering(Ordering::Greater, relation))
    } else if right_antimeridian {
        Some(apply_ordering(Ordering::Less, relation))
    } else {
        Some(apply_partial_order(
            left_longitude.partial_cmp(&right_longitude),
            relation,
        ))
    }
}

fn angular_latitude<P1, P2>(left: &P1, right: &P2, relation: Relation, exact: bool) -> bool
where
    P1: Point,
    P2: Point,
    P1::Cs: AngularCoordinateSystem,
    P2::Cs: AngularCoordinateSystem,
    P1::Scalar: AngularScalar,
    P2::Scalar: AngularScalar,
{
    angular_latitude_result(left, right, relation, exact)
        .unwrap_or(matches!(relation, Relation::Equal))
}

fn angular_latitude_result<P1, P2>(
    left: &P1,
    right: &P2,
    relation: Relation,
    exact: bool,
) -> Option<bool>
where
    P1: Point,
    P2: Point,
    P1::Cs: AngularCoordinateSystem,
    P2::Cs: AngularCoordinateSystem,
    P1::Scalar: AngularScalar,
    P2::Scalar: AngularScalar,
{
    let left_latitude = P1::Cs::to_radians(left.get::<1>());
    let right_latitude = P2::Cs::to_radians(right.get::<1>());
    let epsilon = angular_epsilon::<P1::Scalar, P2::Scalar>();
    if values_equal(left_latitude, right_latitude, epsilon, exact) {
        None
    } else {
        Some(apply_partial_order(
            left_latitude.partial_cmp(&right_latitude),
            relation,
        ))
    }
}

fn compare_ordinate<P1, P2>(
    left: &P1,
    right: &P2,
    dimension: usize,
    relation: Relation,
    exact: bool,
) -> Option<bool>
where
    P1: Point,
    P2: Point,
    P1::Scalar: CoordinateComparison<P2::Scalar>,
{
    let (ordering, epsilon_equal) = match dimension {
        0 => left.get::<0>().compare_coordinate(right.get::<0>()),
        1 => left.get::<1>().compare_coordinate(right.get::<1>()),
        2 => left.get::<2>().compare_coordinate(right.get::<2>()),
        3 => left.get::<3>().compare_coordinate(right.get::<3>()),
        _ => unreachable!("Point dimensions above four are not supported"),
    };
    let equal = if exact {
        ordering == Some(Ordering::Equal)
    } else {
        epsilon_equal
    };
    if equal {
        None
    } else {
        Some(apply_partial_order(ordering, relation))
    }
}

fn apply_partial_order(ordering: Option<Ordering>, relation: Relation) -> bool {
    ordering.is_some_and(|ordering| apply_ordering(ordering, relation))
}

fn apply_ordering(ordering: Ordering, relation: Relation) -> bool {
    match relation {
        Relation::Less => ordering == Ordering::Less,
        Relation::Greater => ordering == Ordering::Greater,
        Relation::Equal => false,
    }
}

fn validate_dimension(dimension: i8, shared_dimensions: usize) {
    assert!(
        shared_dimensions <= 4,
        "Point dimensions above four are not supported"
    );
    assert!(
        dimension == ALL_DIMENSIONS
            || (dimension >= 0 && dimension_index(dimension) < shared_dimensions),
        "comparison dimension must be present in both points"
    );
}

fn dimension_index(dimension: i8) -> usize {
    match usize::try_from(dimension) {
        Ok(index) => index,
        Err(_) => unreachable!("validated comparison dimensions are non-negative"),
    }
}

#[allow(clippy::float_cmp, reason = "exact comparison is a selectable policy")]
fn values_equal(left: f64, right: f64, epsilon: f64, exact: bool) -> bool {
    if exact {
        left == right
    } else {
        scaled_equal(left, right, epsilon)
    }
}

#[allow(
    clippy::float_cmp,
    reason = "exact equality is the required fast path before epsilon scaling"
)]
fn scaled_equal(left: f64, right: f64, epsilon: f64) -> bool {
    left == right
        || (left.is_finite()
            && right.is_finite()
            && (left - right).abs() <= epsilon * left.abs().max(right.abs()).max(1.0))
}

fn is_antimeridian(value: f64, epsilon: f64, exact: bool) -> bool {
    values_equal(value.abs(), core::f64::consts::PI, epsilon, exact)
}

fn is_pole(value: f64, epsilon: f64, exact: bool) -> bool {
    values_equal(value.abs(), core::f64::consts::FRAC_PI_2, epsilon, exact)
}

fn angular_epsilon<L: AngularScalar, R: AngularScalar>() -> f64 {
    match (L::EPSILON, R::EPSILON) {
        (0.0, right) => right,
        (left, 0.0) => left,
        (left, right) => left.min(right),
    }
}

/// Scalar conversion used only after angular coordinates are normalized to
/// radians.
#[doc(hidden)]
pub trait AngularScalar: CoordinateScalar {
    #[doc(hidden)]
    const EPSILON: f64;

    #[doc(hidden)]
    fn to_f64(self) -> f64;
}

impl AngularScalar for f32 {
    const EPSILON: f64 = f32::EPSILON as f64;

    #[inline]
    fn to_f64(self) -> f64 {
        f64::from(self)
    }
}

impl AngularScalar for f64 {
    const EPSILON: f64 = f64::EPSILON;

    #[inline]
    fn to_f64(self) -> f64 {
        self
    }
}

impl AngularScalar for i32 {
    const EPSILON: f64 = 0.0;

    #[inline]
    fn to_f64(self) -> f64 {
        f64::from(self)
    }
}

impl AngularScalar for i64 {
    const EPSILON: f64 = 0.0;

    #[inline]
    #[allow(
        clippy::cast_precision_loss,
        reason = "angular normalization uses f64 just like Boost calculation promotion"
    )]
    fn to_f64(self) -> f64 {
        self as f64
    }
}

impl<I: RationalInteger> AngularScalar for Rational<I> {
    const EPSILON: f64 = 0.0;

    #[inline]
    fn to_f64(self) -> f64 {
        self.to_f64()
    }
}

/// Angle-unit extraction for spherical and geographic coordinate systems.
#[doc(hidden)]
pub trait AngularCoordinateSystem: CoordinateSystem {
    #[doc(hidden)]
    fn to_radians<T: AngularScalar>(value: T) -> f64;
}

impl<U: AngleUnit> AngularCoordinateSystem for Spherical<U> {
    #[inline]
    fn to_radians<T: AngularScalar>(value: T) -> f64 {
        U::to_radians(value.to_f64())
    }
}

impl<U: AngleUnit> AngularCoordinateSystem for Geographic<U> {
    #[inline]
    fn to_radians<T: AngularScalar>(value: T) -> f64 {
        U::to_radians(value.to_f64())
    }
}

/// Cross-scalar ordering and epsilon comparison for coordinate values.
#[doc(hidden)]
pub trait CoordinateComparison<Rhs: CoordinateScalar>: CoordinateScalar {
    #[doc(hidden)]
    fn compare_coordinate(self, right: Rhs) -> (Option<Ordering>, bool);
}

macro_rules! impl_same_float_comparison {
    ($type:ty) => {
        impl CoordinateComparison<$type> for $type {
            #[inline]
            fn compare_coordinate(self, right: $type) -> (Option<Ordering>, bool) {
                (
                    self.partial_cmp(&right),
                    scaled_equal(
                        f64::from(self),
                        f64::from(right),
                        f64::from(<$type>::EPSILON),
                    ),
                )
            }
        }
    };
}

impl_same_float_comparison!(f32);
impl_same_float_comparison!(f64);

macro_rules! impl_same_integer_comparison {
    ($type:ty) => {
        impl CoordinateComparison<$type> for $type {
            #[inline]
            fn compare_coordinate(self, right: $type) -> (Option<Ordering>, bool) {
                (Some(self.cmp(&right)), self == right)
            }
        }
    };
}

impl_same_integer_comparison!(i32);
impl_same_integer_comparison!(i64);

macro_rules! impl_mixed_float_comparison {
    ($left:ty, $right:ty, $epsilon:expr) => {
        impl CoordinateComparison<$right> for $left {
            #[inline]
            fn compare_coordinate(self, right: $right) -> (Option<Ordering>, bool) {
                let left = <$left as AngularScalar>::to_f64(self);
                let right = <$right as AngularScalar>::to_f64(right);
                (
                    left.partial_cmp(&right),
                    scaled_equal(left, right, $epsilon),
                )
            }
        }
    };
}

impl_mixed_float_comparison!(f32, f64, f64::EPSILON);
impl_mixed_float_comparison!(f64, f32, f64::EPSILON);
impl_mixed_float_comparison!(i32, f32, f64::EPSILON);
impl_mixed_float_comparison!(f32, i32, f64::EPSILON);
impl_mixed_float_comparison!(i32, f64, f64::EPSILON);
impl_mixed_float_comparison!(f64, i32, f64::EPSILON);
impl_mixed_float_comparison!(i64, f32, f64::EPSILON);
impl_mixed_float_comparison!(f32, i64, f64::EPSILON);
impl_mixed_float_comparison!(i64, f64, f64::EPSILON);
impl_mixed_float_comparison!(f64, i64, f64::EPSILON);

impl CoordinateComparison<i64> for i32 {
    #[inline]
    fn compare_coordinate(self, right: i64) -> (Option<Ordering>, bool) {
        let left = i64::from(self);
        (Some(left.cmp(&right)), left == right)
    }
}

impl CoordinateComparison<i32> for i64 {
    #[inline]
    fn compare_coordinate(self, right: i32) -> (Option<Ordering>, bool) {
        let right = i64::from(right);
        (Some(self.cmp(&right)), self == right)
    }
}

impl<I, J> CoordinateComparison<Rational<J>> for Rational<I>
where
    I: RationalInteger,
    J: RationalInteger,
{
    #[inline]
    fn compare_coordinate(self, right: Rational<J>) -> (Option<Ordering>, bool) {
        let left_cross = self.numerator().to_i128() * right.denominator().to_i128();
        let right_cross = right.numerator().to_i128() * self.denominator().to_i128();
        let ordering = left_cross.cmp(&right_cross);
        (Some(ordering), ordering == Ordering::Equal)
    }
}

macro_rules! impl_rational_integer_comparison {
    ($integer:ty) => {
        impl<I: RationalInteger> CoordinateComparison<$integer> for Rational<I> {
            #[inline]
            fn compare_coordinate(self, right: $integer) -> (Option<Ordering>, bool) {
                let left = self.numerator().to_i128();
                let right = i128::from(right) * self.denominator().to_i128();
                let ordering = left.cmp(&right);
                (Some(ordering), ordering == Ordering::Equal)
            }
        }

        impl<I: RationalInteger> CoordinateComparison<Rational<I>> for $integer {
            #[inline]
            fn compare_coordinate(self, right: Rational<I>) -> (Option<Ordering>, bool) {
                let left = i128::from(self) * right.denominator().to_i128();
                let right = right.numerator().to_i128();
                let ordering = left.cmp(&right);
                (Some(ordering), ordering == Ordering::Equal)
            }
        }
    };
}

impl_rational_integer_comparison!(i32);
impl_rational_integer_comparison!(i64);

macro_rules! impl_rational_float_comparison {
    ($float:ty) => {
        impl<I: RationalInteger> CoordinateComparison<$float> for Rational<I> {
            #[inline]
            fn compare_coordinate(self, right: $float) -> (Option<Ordering>, bool) {
                let left = self.to_f64();
                let right = f64::from(right);
                (
                    left.partial_cmp(&right),
                    scaled_equal(left, right, f64::from(<$float>::EPSILON)),
                )
            }
        }

        impl<I: RationalInteger> CoordinateComparison<Rational<I>> for $float {
            #[inline]
            fn compare_coordinate(self, right: Rational<I>) -> (Option<Ordering>, bool) {
                let left = f64::from(self);
                let right = right.to_f64();
                (
                    left.partial_cmp(&right),
                    scaled_equal(left, right, f64::from(<$float>::EPSILON)),
                )
            }
        }
    };
}

impl_rational_float_comparison!(f32);
impl_rational_float_comparison!(f64);
