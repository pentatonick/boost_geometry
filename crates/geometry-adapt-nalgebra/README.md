# `geometry-adapt-nalgebra`

**Ecosystem adapter, peer of `geometry-adapt`.** Depends on `geometry-trait`.

Adapts [`nalgebra`](https://nalgebra.org) points and vectors to the
`geometry-trait` concept surface. Same orphan-rule-driven wrapper pattern as
`geometry-adapt-geo-types`.

## Wrapper inventory

| Wrapper | Wraps |
|---|---|
| `NaPoint2` | `nalgebra::Point2` |
| `NaPoint3` | `nalgebra::Point3` |
| `NaVector2` | `nalgebra::Vector2` |
| `NaVector3` | `nalgebra::Vector3` |

Every wrapper is `#[repr(transparent)]`, pins `Cs = Cartesian`, and is
read-write (implements `PointMut`) since `nalgebra` points/vectors are
mutable by nature.
