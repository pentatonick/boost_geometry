# `geometry-srid`

**Layer 0 — foundation.** No dependencies. `#![no_std]`, no `alloc`.

Not part of Boost.Geometry — Boost has no notion of a spatial-reference id.
Reference: the PostGIS manual §4.1.3 and §4.2.1 ("PostGIS EWKB and EWKT"),
plus `clamp_srid` in `liblwgeom/lwutil.c`.

## Purpose

The PostGIS spatial-reference identifier (SRID) — the code naming the
coordinate reference system a geometry's ordinates are expressed in. It
is a foundation noun: both PostGIS dialect crates carry one,
`geometry-io-ewkt` as the decimal integer after `SRID=` and
`geometry-io-ewkb` as a 32-bit field in the record header. Neither owns
it, so it lives here, below both, and each dialect crate re-exports it
rather than defining its own copy.

## Files

| File | Contents |
|---|---|
| `src/srid.rs` | `Srid` — the newtype over the id, and `Srid::UNKNOWN` |
| `src/lib.rs` | Re-exports only (manifest) |

## Public surface

`Srid(u32)` — `Srid::new(code)` wraps a code, `Srid::get(self)` unwraps
it, and `Srid::UNKNOWN` is `Srid::new(0)`, the value PostGIS treats as
"unknown" (distinct from "no SRID at all", which each dialect crate
represents as `None`). `Display` writes the bare decimal integer with no
prefix or punctuation; each dialect crate adds whatever its own wire form
requires. This crate interprets nothing: it does not resolve an id to a
coordinate system, validate one against a registry, or rewrite a value
the way PostGIS's `clamp_srid` does on ingest — a `Srid` carries whatever
it is given, unchanged.

## Who depends on this

`geometry-io-ewkt` and `geometry-io-ewkb`, the two PostGIS dialect crates.
