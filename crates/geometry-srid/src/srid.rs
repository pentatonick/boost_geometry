//! The `PostGIS` spatial-reference identifier.
//!
//! Reference: the `PostGIS` manual §4.1.3 and §4.2.1 ("`PostGIS` EWKB
//! and EWKT") and `clamp_srid` in `liblwgeom/lwutil.c`.

/// A `PostGIS` spatial-reference identifier: the code naming the
/// coordinate reference system a geometry's ordinates are expressed in.
///
/// Dialect-neutral. Each dialect crate documents its own wire form —
/// `geometry-io-ewkt` the decimal run after `SRID=`, `geometry-io-ewkb`
/// a 32-bit field in the record header — and both carry the same value.
/// The storage is `u32`.
///
/// # What `PostGIS` stores, and what this type does
///
/// `PostGIS` keeps an SRID in `0..=999999`. Its `clamp_srid` rewrites
/// every value outside that range before it can be stored or emitted:
/// anything `<= 0` becomes 0, and anything above the maximum is folded
/// into `999000..=999998`. **This type carries whatever it is given,
/// unchanged, and never rewrites it** — so a value outside
/// `0..=999999` round-trips faithfully here and would be rewritten by
/// `PostGIS` on ingest.
///
/// # Examples
///
/// ```
/// use geometry_srid::Srid;
///
/// let srid = Srid::new(4326);
/// assert_eq!(srid.get(), 4326);
/// assert_eq!(srid.to_string(), "4326");
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Srid(u32);

impl Srid {
    /// SRID 0, which `PostGIS` treats as "unknown": its `clamp_srid`
    /// maps every value `<= 0` to 0. It is an ordinary value here —
    /// distinguishable from "no SRID at all", which each dialect crate
    /// represents as `None` rather than as this constant.
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_srid::Srid;
    ///
    /// assert_eq!(Srid::UNKNOWN.get(), 0);
    /// ```
    pub const UNKNOWN: Srid = Srid(0);

    /// Wrap a spatial-reference code.
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_srid::Srid;
    ///
    /// assert_eq!(Srid::new(0), Srid::UNKNOWN);
    /// ```
    #[must_use]
    pub const fn new(code: u32) -> Self {
        Self(code)
    }

    /// The wrapped spatial-reference code.
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_srid::Srid;
    ///
    /// assert_eq!(Srid::new(3857).get(), 3857);
    /// ```
    #[must_use]
    pub const fn get(self) -> u32 {
        self.0
    }
}

impl core::fmt::Display for Srid {
    /// Writes the bare decimal integer, with no prefix or punctuation.
    /// Each dialect crate adds whatever its own wire form requires.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use alloc::format;

    use super::Srid;

    #[test]
    fn zero_is_unknown() {
        assert_eq!(Srid::new(0), Srid::UNKNOWN);
        assert_eq!(Srid::UNKNOWN.get(), 0);
    }

    #[test]
    fn upper_bound_round_trips() {
        assert_eq!(Srid::new(u32::MAX).get(), u32::MAX);
    }

    #[test]
    fn display_is_the_bare_decimal() {
        assert_eq!(format!("{}", Srid::new(4326)), "4326");
    }
}
