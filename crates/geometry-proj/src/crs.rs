//! A coordinate reference system, and construction of one from the
//! usual textual sources.
//!
//! Wraps [`proj4rs::Proj`]. A CRS is built from a proj4 string, an EPSG
//! code, or a WKT definition (parsed to a proj string by
//! [`proj4wkt`]). There is no Boost.Geometry counterpart — Boost defers
//! projections to its unsupported `extensions/gis/projections/`; the
//! Rust port fills the gap with the pure-Rust `proj4rs` engine.

use proj4rs::Proj;

/// A coordinate reference system the reprojection functions transform
/// between.
///
/// Thin newtype over [`proj4rs::Proj`]; construct it from a proj4
/// string ([`Crs::from_proj_string`]), an EPSG code
/// ([`Crs::from_epsg`]), or a WKT definition ([`Crs::from_wkt`]).
pub struct Crs {
    proj: Proj,
}

/// Failure to build a [`Crs`] from a textual definition.
#[derive(Debug)]
pub enum CrsError {
    /// The proj4 / EPSG definition was rejected by `proj4rs`.
    Proj(proj4rs::errors::Error),
    /// The WKT string could not be parsed to a proj definition.
    Wkt(alloc::string::String),
}

impl core::fmt::Display for CrsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            CrsError::Proj(e) => write!(f, "invalid CRS definition: {e}"),
            CrsError::Wkt(m) => write!(f, "invalid WKT: {m}"),
        }
    }
}

impl std::error::Error for CrsError {}

impl Crs {
    /// Build a CRS from a proj4 definition string, e.g.
    /// `"+proj=utm +zone=32 +datum=WGS84"`.
    ///
    /// # Errors
    ///
    /// [`CrsError::Proj`] if `proj4rs` rejects the string.
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_proj::Crs;
    /// let wgs84 = Crs::from_proj_string("+proj=longlat +datum=WGS84 +no_defs").unwrap();
    /// # let _ = wgs84;
    /// ```
    pub fn from_proj_string(s: &str) -> Result<Self, CrsError> {
        Proj::from_proj_string(s)
            .map(|proj| Self { proj })
            .map_err(CrsError::Proj)
    }

    /// Build a CRS from an EPSG code, e.g. `4326` (WGS84 lon/lat) or
    /// `3857` (Web Mercator).
    ///
    /// # Errors
    ///
    /// [`CrsError::Proj`] if the code is unknown to `proj4rs`.
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_proj::Crs;
    /// let web_mercator = Crs::from_epsg(3857).unwrap();
    /// # let _ = web_mercator;
    /// ```
    pub fn from_epsg(code: u16) -> Result<Self, CrsError> {
        Proj::from_epsg_code(code)
            .map(|proj| Self { proj })
            .map_err(CrsError::Proj)
    }

    /// Build a CRS from a WKT definition, parsed to a proj string by
    /// [`proj4wkt`].
    ///
    /// # Errors
    ///
    /// [`CrsError::Wkt`] if the WKT cannot be parsed, or
    /// [`CrsError::Proj`] if the resulting proj string is invalid.
    pub fn from_wkt(wkt: &str) -> Result<Self, CrsError> {
        let projstr =
            proj4wkt::wkt_to_projstring(wkt).map_err(|e| CrsError::Wkt(alloc::format!("{e:?}")))?;
        Self::from_proj_string(&projstr)
    }

    /// Borrow the underlying `proj4rs` projection.
    #[must_use]
    pub(crate) fn proj(&self) -> &Proj {
        &self.proj
    }

    /// Whether this CRS is geographic (lon/lat), whose coordinates are
    /// carried in **radians** by `proj4rs`.
    #[must_use]
    pub fn is_geographic(&self) -> bool {
        self.proj.is_latlong()
    }
}

#[cfg(test)]
mod tests {
    use super::Crs;

    #[test]
    fn build_from_epsg() {
        assert!(Crs::from_epsg(4326).is_ok());
        assert!(Crs::from_epsg(3857).is_ok());
    }

    #[test]
    fn build_from_proj_string() {
        assert!(Crs::from_proj_string("+proj=longlat +datum=WGS84 +no_defs").is_ok());
    }

    #[test]
    fn wgs84_is_geographic() {
        let wgs84 = Crs::from_epsg(4326).unwrap();
        assert!(wgs84.is_geographic());
        let mercator = Crs::from_epsg(3857).unwrap();
        assert!(!mercator.is_geographic());
    }

    #[test]
    fn bad_epsg_errors() {
        assert!(Crs::from_epsg(1).is_err());
    }
}
