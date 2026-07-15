//! Reproject a geometry from one [`Crs`] to another, in place.
//!
//! Bridges the geometry kernel's mutable point access to
//! [`proj4rs::transform::transform`]. `proj4rs` drives the conversion
//! through its [`proj4rs::transform::Transform`] trait, which
//! hands each coordinate to a closure; this module implements that trait
//! over any geometry that can enumerate its mutable points
//! ([`ReprojectPoints`]).
//!
//! # Units
//!
//! `proj4rs` carries **geographic coordinates in radians**. When the
//! source or destination CRS is geographic, callers work in radians on
//! that side — matching `proj4rs`'s own convention. Use
//! [`f64::to_radians`] / [`f64::to_degrees`] at the boundary if your
//! data is in degrees.

use proj4rs::transform::{Transform, TransformClosure};

use geometry_cs::CoordinateSystem;
use geometry_model::{Linestring, MultiPolygon, Point2D, Polygon, Ring};
use geometry_trait::{Point, PointMut};

use crate::crs::{Crs, CrsError};

/// A geometry whose points can be enumerated and rewritten — the hook
/// reprojection needs.
///
/// Implemented for the kernel's mutable areal / linear / point model
/// types. The single method visits every point, letting the caller
/// replace its `(x, y)`.
pub trait ReprojectPoints {
    /// Visit each point, replacing it with `f(x, y)`. `f` may fail; the
    /// first failure stops the walk and is propagated.
    ///
    /// # Errors
    ///
    /// Whatever error `f` returns for the first point it rejects.
    fn map_points<F>(&mut self, f: &mut F) -> Result<(), CrsError>
    where
        F: FnMut(f64, f64) -> Result<(f64, f64), CrsError>;
}

/// Reproject `geometry` in place from `from` to `to`.
///
/// # Errors
///
/// [`CrsError::Proj`] if `proj4rs` cannot transform a coordinate (out
/// of the projection's valid domain, missing inverse, …). On error the
/// geometry is **unchanged** — coordinates are transformed into a
/// scratch buffer first and written back only after every point has
/// succeeded.
///
/// # Panics
///
/// Panics if the write-back walk visits a different number of points
/// than the collection walk did — unreachable in practice, since both
/// walks traverse the same exclusively-borrowed geometry via the same
/// [`ReprojectPoints::map_points`] impl.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_proj::{reproject, Crs};
/// use geometry_trait::Point as _;
///
/// // WGS84 lon/lat (radians) → Web Mercator metres.
/// let wgs84 = Crs::from_epsg(4326).unwrap();
/// let mercator = Crs::from_epsg(3857).unwrap();
///
/// // 0°,0° is the origin in both systems.
/// let mut p = Point2D::<f64, Cartesian>::new(0.0, 0.0);
/// reproject(&mut p, &wgs84, &mercator).unwrap();
/// assert!(p.get::<0>().abs() < 1e-6);
/// assert!(p.get::<1>().abs() < 1e-6);
/// ```
pub fn reproject<G: ReprojectPoints>(
    geometry: &mut G,
    from: &Crs,
    to: &Crs,
) -> Result<(), CrsError> {
    // Phase 1: collect every coordinate without changing anything —
    // the closure writes back the values it read.
    let mut buf = CoordBuffer(alloc::vec::Vec::new());
    geometry.map_points(&mut |x, y| {
        buf.0.push((x, y));
        Ok((x, y))
    })?;

    // Phase 2: transform the scratch buffer. On error the geometry has
    // not been modified — the all-or-nothing guarantee.
    proj4rs::transform::transform(from.proj(), to.proj(), &mut buf).map_err(CrsError::Proj)?;

    // Phase 3: write the transformed pairs back in the same visit
    // order. `map_points` walks a `&mut` geometry the same way both
    // times, so the counts match by construction.
    let mut it = buf.0.into_iter();
    geometry.map_points(&mut |_x, _y| {
        let (nx, ny) = it.next().expect(
            "reproject: point count changed between walks over an exclusively borrowed geometry",
        );
        Ok((nx, ny))
    })
}

/// Scratch coordinate buffer bridging `proj4rs`'s [`Transform`] trait.
/// Phase 2 of the transactional walk: `proj4rs` transforms these pairs
/// in place; the source geometry is not touched until every pair has
/// succeeded.
struct CoordBuffer(alloc::vec::Vec<(f64, f64)>);

impl Transform for CoordBuffer {
    fn transform_coordinates<F: TransformClosure>(
        &mut self,
        f: &mut F,
    ) -> proj4rs::errors::Result<()> {
        for pair in &mut self.0 {
            let (nx, ny, _nz) = f(pair.0, pair.1, 0.0)?;
            *pair = (nx, ny);
        }
        Ok(())
    }
}

// ---- ReprojectPoints impls for the model geometries -----------------

impl<Cs: CoordinateSystem> ReprojectPoints for Point2D<f64, Cs> {
    fn map_points<F>(&mut self, f: &mut F) -> Result<(), CrsError>
    where
        F: FnMut(f64, f64) -> Result<(f64, f64), CrsError>,
    {
        let (nx, ny) = f(self.get::<0>(), self.get::<1>())?;
        self.set::<0>(nx);
        self.set::<1>(ny);
        Ok(())
    }
}

impl<Cs: CoordinateSystem> ReprojectPoints for Linestring<Point2D<f64, Cs>> {
    fn map_points<F>(&mut self, f: &mut F) -> Result<(), CrsError>
    where
        F: FnMut(f64, f64) -> Result<(f64, f64), CrsError>,
    {
        for p in &mut self.0 {
            p.map_points(f)?;
        }
        Ok(())
    }
}

impl<Cs: CoordinateSystem> ReprojectPoints for Ring<Point2D<f64, Cs>> {
    fn map_points<F>(&mut self, f: &mut F) -> Result<(), CrsError>
    where
        F: FnMut(f64, f64) -> Result<(f64, f64), CrsError>,
    {
        for p in &mut self.0 {
            p.map_points(f)?;
        }
        Ok(())
    }
}

impl<Cs: CoordinateSystem> ReprojectPoints for Polygon<Point2D<f64, Cs>> {
    fn map_points<F>(&mut self, f: &mut F) -> Result<(), CrsError>
    where
        F: FnMut(f64, f64) -> Result<(f64, f64), CrsError>,
    {
        self.outer.map_points(f)?;
        for hole in &mut self.inners {
            hole.map_points(f)?;
        }
        Ok(())
    }
}

impl<Cs: CoordinateSystem> ReprojectPoints for MultiPolygon<Polygon<Point2D<f64, Cs>>> {
    fn map_points<F>(&mut self, f: &mut F) -> Result<(), CrsError>
    where
        F: FnMut(f64, f64) -> Result<(f64, f64), CrsError>,
    {
        for pg in &mut self.0 {
            pg.map_points(f)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::reproject;
    use crate::crs::Crs;
    use geometry_cs::Cartesian;
    use geometry_model::{Linestring, Point2D};
    use geometry_trait::Point as _;

    type P = Point2D<f64, Cartesian>;

    fn wgs84() -> Crs {
        Crs::from_epsg(4326).unwrap()
    }
    fn mercator() -> Crs {
        Crs::from_epsg(3857).unwrap()
    }

    #[test]
    fn origin_maps_to_origin() {
        let mut p = P::new(0.0, 0.0);
        reproject(&mut p, &wgs84(), &mercator()).unwrap();
        assert!(p.get::<0>().abs() < 1e-6);
        assert!(p.get::<1>().abs() < 1e-6);
    }

    #[test]
    fn known_point_web_mercator() {
        // 45° E, 0° N in radians → Web Mercator x = R · λ.
        // R = 6378137, λ = π/4 → x ≈ 5_009_377.
        let mut p = P::new(45.0_f64.to_radians(), 0.0);
        reproject(&mut p, &wgs84(), &mercator()).unwrap();
        let expected_x = 6_378_137.0 * core::f64::consts::FRAC_PI_4;
        assert!((p.get::<0>() - expected_x).abs() < 1.0);
        assert!(p.get::<1>().abs() < 1e-3, "y = {}", p.get::<1>());
    }

    #[test]
    fn round_trip_returns_original() {
        let mut p = P::new(0.3_f64, 0.8);
        let orig = (p.get::<0>(), p.get::<1>());
        reproject(&mut p, &wgs84(), &mercator()).unwrap();
        reproject(&mut p, &mercator(), &wgs84()).unwrap();
        assert!((p.get::<0>() - orig.0).abs() < 1e-9);
        assert!((p.get::<1>() - orig.1).abs() < 1e-9);
    }

    #[test]
    fn reproject_a_linestring() {
        let mut ls: Linestring<P> =
            Linestring(vec![P::new(0.0, 0.0), P::new(0.1, 0.1), P::new(0.2, 0.05)]);
        reproject(&mut ls, &wgs84(), &mercator()).unwrap();
        // Origin vertex stays at the origin; the others move outward.
        assert!(ls.0[0].get::<0>().abs() < 1e-6);
        assert!(ls.0[1].get::<0>() > 1000.0);
    }

    /// A `Ring`'s every vertex is reprojected; the origin stays put and
    /// the rest move outward under Web Mercator.
    #[test]
    fn reproject_a_ring() {
        use geometry_model::Ring;
        let mut ring: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.1, 0.0),
            P::new(0.1, 0.1),
            P::new(0.0, 0.0),
        ]);
        reproject(&mut ring, &wgs84(), &mercator()).unwrap();
        assert!(ring.0[0].get::<0>().abs() < 1e-6); // origin fixed
        assert!(ring.0[1].get::<0>() > 1000.0); // moved outward
    }

    /// A `Polygon`'s exterior *and* interior rings are all reprojected.
    #[test]
    fn reproject_a_polygon_with_hole() {
        use geometry_model::{Polygon, Ring};
        let outer: Ring<P> = Ring::from_vec(vec![
            P::new(0.0, 0.0),
            P::new(0.2, 0.0),
            P::new(0.2, 0.2),
            P::new(0.0, 0.0),
        ]);
        let hole: Ring<P> = Ring::from_vec(vec![
            P::new(0.05, 0.05),
            P::new(0.1, 0.05),
            P::new(0.1, 0.1),
            P::new(0.05, 0.05),
        ]);
        let mut pg = Polygon::with_inners(outer, vec![hole]);
        reproject(&mut pg, &wgs84(), &mercator()).unwrap();
        // Exterior origin vertex fixed at (0,0); the hole's first vertex
        // moved outward (was 0.05 rad in each axis).
        assert!(pg.outer.0[0].get::<0>().abs() < 1e-6);
        assert!(pg.inners[0].0[0].get::<0>() > 1000.0);
    }

    /// A `MultiPolygon` reprojects each member polygon.
    #[test]
    fn reproject_a_multipolygon() {
        use geometry_model::{MultiPolygon, Polygon, Ring};
        let member = Polygon::with_inners(
            Ring::from_vec(vec![
                P::new(0.1, 0.1),
                P::new(0.2, 0.1),
                P::new(0.2, 0.2),
                P::new(0.1, 0.1),
            ]),
            vec![],
        );
        let mut mpg: MultiPolygon<Polygon<P>> = MultiPolygon(vec![member.clone(), member]);
        reproject(&mut mpg, &wgs84(), &mercator()).unwrap();
        // Both members' first vertices moved to the same reprojected x
        // (they were identical before), well away from the origin.
        let x0 = mpg.0[0].outer.0[0].get::<0>();
        let x1 = mpg.0[1].outer.0[0].get::<0>();
        assert!(x0 > 1000.0);
        assert!((x0 - x1).abs() < 1e-9);
    }

    #[test]
    fn failed_reproject_leaves_geometry_unchanged() {
        // lat = π/2 rad is the Mercator singularity: proj4rs refuses
        // it. The FIRST point is transformable, so the old
        // point-at-a-time walk would have rewritten it before hitting
        // the error — the transactional walk must leave BOTH points
        // bitwise-untouched.
        let mut ls: Linestring<P> = Linestring(vec![
            P::new(0.1, 0.1),
            P::new(0.0, core::f64::consts::FRAC_PI_2),
        ]);
        let before: Vec<(u64, u64)> =
            ls.0.iter()
                .map(|p| (p.get::<0>().to_bits(), p.get::<1>().to_bits()))
                .collect();
        let result = reproject(&mut ls, &wgs84(), &mercator());
        assert!(result.is_err(), "π/2 latitude must be rejected by proj4rs");
        let after: Vec<(u64, u64)> =
            ls.0.iter()
                .map(|p| (p.get::<0>().to_bits(), p.get::<1>().to_bits()))
                .collect();
        assert_eq!(before, after, "geometry must be unchanged on Err");
    }
}
