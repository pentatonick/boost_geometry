//! The spatial-reference id carried by the `SRID=` prefix.
//!
//! Reference: the `PostGIS` manual §4.2.1 ("`PostGIS` EWKB and EWKT") and
//! `clamp_srid` in `liblwgeom/lwutil.c`.

/// A `PostGIS` spatial-reference id: the integer after `SRID=`.
///
/// The wire form is a run of unsigned decimal digits, so the storage is
/// `u32`; the prefix grammar admits no sign.
///
/// # Examples
///
/// ```
/// use geometry_io_ewkt::Srid;
///
/// let srid = Srid::new(4326);
/// assert_eq!(srid.get(), 4326);
/// assert_eq!(srid.to_string(), "4326");
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Srid(u32);

impl Srid {
    /// SRID 0, which `PostGIS` treats as "unknown": its `clamp_srid`
    /// maps every value `<= 0` to 0. This crate reads `SRID=0;` into it
    /// and writes it back as `SRID=0;` when passed as `Some`; `PostGIS`
    /// itself omits the prefix for 0.
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_io_ewkt::Srid;
    ///
    /// assert_eq!(Srid::UNKNOWN.get(), 0);
    /// ```
    pub const UNKNOWN: Srid = Srid(0);

    /// Wrap a spatial-reference code.
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_io_ewkt::Srid;
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
    /// use geometry_io_ewkt::Srid;
    ///
    /// assert_eq!(Srid::new(3857).get(), 3857);
    /// ```
    #[must_use]
    pub const fn get(self) -> u32 {
        self.0
    }
}

impl core::fmt::Display for Srid {
    /// Writes the bare decimal integer, as the `SRID=` prefix spells it.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[cfg(test)]
mod tests {
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
