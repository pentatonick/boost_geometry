//! Dimension-agnostic geometry values with explicit empty points.

use alloc::vec::Vec;

use geometry_coords::CoordinateScalar;
use geometry_cs::CoordinateSystem;
use geometry_tag::DynamicGeometryTag;
use geometry_trait::{Geometry, Point as PointTrait};

use crate::{
    DynGeometry, DynKind, Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon,
};

/// A geometry that preserves empty points and empty multipoint members.
///
/// `None` is an empty point. A populated point containing NaNs stays `Some`.
/// Linear payloads reuse the existing models. The point type determines the
/// dimension, scalar, and coordinate system. Codec and algorithm support is
/// independent of storage: convert to [`DynGeometry`] for its existing algorithms.
///
/// ```
/// use geometry_model::{DynGeometry, EmptyPointError, GeometryValue};
///
/// let empty = GeometryValue::<geometry_model::Point2D<f64>>::Point(None);
/// assert_eq!(empty.kind(), geometry_model::DynKind::Point);
/// assert_eq!(DynGeometry::try_from(empty), Err(EmptyPointError));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(bound(
        serialize = "P: serde::Serialize",
        deserialize = "P: serde::Deserialize<'de>"
    ))
)]
pub enum GeometryValue<P: PointTrait> {
    /// A populated or empty point.
    Point(Option<P>),
    /// A line string, including its empty form.
    LineString(Linestring<P>),
    /// A polygon, including its empty form.
    Polygon(Polygon<P>),
    /// Point members in order, retaining empty members.
    MultiPoint(Vec<Option<P>>),
    /// Line-string members in order.
    MultiLineString(MultiLinestring<Linestring<P>>),
    /// Polygon members in order.
    MultiPolygon(MultiPolygon<Polygon<P>>),
    /// Heterogeneous members, including nested collections.
    GeometryCollection(Vec<Self>),
}

impl<P: PointTrait> Geometry for GeometryValue<P> {
    type Kind = DynamicGeometryTag;
    type Point = P;
}

impl<P: PointTrait> GeometryValue<P> {
    /// The geometry kind, independent of whether it is empty.
    ///
    /// ```
    /// use geometry_model::{DynKind, GeometryValue};
    /// assert_eq!(GeometryValue::<geometry_model::Point2D<f64>>::Point(None).kind(), DynKind::Point);
    /// ```
    #[must_use]
    pub fn kind(&self) -> DynKind {
        match self {
            Self::Point(_) => DynKind::Point,
            Self::LineString(_) => DynKind::LineString,
            Self::Polygon(_) => DynKind::Polygon,
            Self::MultiPoint(_) => DynKind::MultiPoint,
            Self::MultiLineString(_) => DynKind::MultiLineString,
            Self::MultiPolygon(_) => DynKind::MultiPolygon,
            Self::GeometryCollection(_) => DynKind::GeometryCollection,
        }
    }
}

/// Conversion would lose an empty point or empty multipoint member.
///
/// ```
/// use geometry_model::{DynGeometry, EmptyPointError, GeometryValue};
/// let points = GeometryValue::<geometry_model::Point2D<f64>>::MultiPoint(vec![None]);
/// assert_eq!(DynGeometry::try_from(points), Err(EmptyPointError));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmptyPointError;

impl core::fmt::Display for EmptyPointError {
    fn fmt(&self, out: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        out.write_str("the destination geometry cannot represent an empty point")
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EmptyPointError {}

/// Post-order collection construction keeps conversion off the call stack.
enum CollectionPart<G> {
    Geometry(G),
    Members(usize),
}

impl<S: CoordinateScalar, Cs: CoordinateSystem> From<DynGeometry<S, Cs>>
    for GeometryValue<Point<S, 2, Cs>>
{
    fn from(value: DynGeometry<S, Cs>) -> Self {
        let mut pending = alloc::vec![CollectionPart::Geometry(value)];
        let mut values = Vec::new();
        while let Some(part) = pending.pop() {
            let value = match part {
                CollectionPart::Members(count) => {
                    let children = values.split_off(values.len() - count);
                    values.push(Self::GeometryCollection(children));
                    continue;
                }
                CollectionPart::Geometry(value) => value,
            };
            values.push(match value {
                DynGeometry::Point(p) => Self::Point(Some(p)),
                DynGeometry::LineString(g) => Self::LineString(g),
                DynGeometry::Polygon(g) => Self::Polygon(g),
                DynGeometry::MultiPoint(g) => Self::MultiPoint(g.0.into_iter().map(Some).collect()),
                DynGeometry::MultiLineString(g) => Self::MultiLineString(g),
                DynGeometry::MultiPolygon(g) => Self::MultiPolygon(g),
                DynGeometry::GeometryCollection(children) => {
                    pending.push(CollectionPart::Members(children.len()));
                    pending.extend(children.into_iter().rev().map(CollectionPart::Geometry));
                    continue;
                }
            });
        }
        values.pop().expect("a geometry produces one root value")
    }
}

impl<S: CoordinateScalar, Cs: CoordinateSystem> TryFrom<GeometryValue<Point<S, 2, Cs>>>
    for DynGeometry<S, Cs>
{
    type Error = EmptyPointError;

    fn try_from(value: GeometryValue<Point<S, 2, Cs>>) -> Result<Self, Self::Error> {
        let mut pending = alloc::vec![CollectionPart::Geometry(value)];
        let mut values = Vec::new();
        while let Some(part) = pending.pop() {
            let value = match part {
                CollectionPart::Members(count) => {
                    let children = values.split_off(values.len() - count);
                    values.push(Self::GeometryCollection(children));
                    continue;
                }
                CollectionPart::Geometry(value) => value,
            };
            values.push(match value {
                GeometryValue::Point(p) => Self::Point(p.ok_or(EmptyPointError)?),
                GeometryValue::LineString(g) => Self::LineString(g),
                GeometryValue::Polygon(g) => Self::Polygon(g),
                GeometryValue::MultiPoint(points) => Self::MultiPoint(MultiPoint(
                    points
                        .into_iter()
                        .map(|p| p.ok_or(EmptyPointError))
                        .collect::<Result<_, _>>()?,
                )),
                GeometryValue::MultiLineString(g) => Self::MultiLineString(g),
                GeometryValue::MultiPolygon(g) => Self::MultiPolygon(g),
                GeometryValue::GeometryCollection(children) => {
                    pending.push(CollectionPart::Members(children.len()));
                    pending.extend(children.into_iter().rev().map(CollectionPart::Geometry));
                    continue;
                }
            });
        }
        Ok(values.pop().expect("a geometry produces one root value"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Point2D, Point3D, Ring};
    use alloc::vec;

    type Pt = Point2D<f64>;

    #[test]
    fn all_legacy_kinds_convert_without_changing_members() {
        let point = Pt::new(1.0, 2.0);
        let line = Linestring(vec![point, Pt::new(3.0, 4.0)]);
        let polygon = Polygon::new(Ring::<Pt>::new());
        let members = vec![
            DynGeometry::Point(point),
            DynGeometry::LineString(line.clone()),
            DynGeometry::Polygon(polygon.clone()),
            DynGeometry::MultiPoint(MultiPoint(vec![point])),
            DynGeometry::MultiLineString(MultiLinestring(vec![line])),
            DynGeometry::MultiPolygon(MultiPolygon(vec![polygon])),
            DynGeometry::GeometryCollection(vec![DynGeometry::GeometryCollection(vec![])]),
        ];
        assert_eq!(
            members
                .iter()
                .cloned()
                .map(GeometryValue::from)
                .map(|g| g.kind())
                .collect::<Vec<_>>(),
            vec![
                DynKind::Point,
                DynKind::LineString,
                DynKind::Polygon,
                DynKind::MultiPoint,
                DynKind::MultiLineString,
                DynKind::MultiPolygon,
                DynKind::GeometryCollection
            ]
        );
        let original = DynGeometry::GeometryCollection(members);
        let value = GeometryValue::from(original.clone());
        assert_eq!(value.kind(), DynKind::GeometryCollection);
        assert_eq!(DynGeometry::try_from(value), Ok(original));
    }

    #[test]
    fn conversion_rejects_empty_points_at_any_position() {
        for empty in [
            GeometryValue::<Pt>::Point(None),
            GeometryValue::MultiPoint(vec![Some(Pt::new(1.0, 2.0)), None]),
        ] {
            assert_eq!(DynGeometry::try_from(empty.clone()), Err(EmptyPointError));
            assert_eq!(
                DynGeometry::try_from(GeometryValue::GeometryCollection(vec![
                    GeometryValue::GeometryCollection(vec![empty])
                ])),
                Err(EmptyPointError)
            );
        }
    }

    #[test]
    fn storage_and_metadata_follow_the_point_dimension() {
        fn dimensions<G: Geometry>(_: &G) -> usize {
            G::Point::DIM
        }
        let point = Point3D::<f64>::new(1.0, 2.0, 3.0);
        let value = GeometryValue::GeometryCollection(vec![GeometryValue::Point(Some(point))]);
        assert_eq!(dimensions(&value), 3);
        assert_eq!(value.kind(), DynKind::GeometryCollection);
    }
}
