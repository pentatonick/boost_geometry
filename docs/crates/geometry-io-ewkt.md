# `geometry-io-ewkt`

**I/O peer, consumes `geometry-io-wkt` and `geometry-model`.**
`#![no_std]` + `alloc`.

Not part of Boost.Geometry — Boost has no notion of an SRID prefix.
Reference: the PostGIS manual, §4.1.3 "WKT and WKB" and §4.2.1 "PostGIS
EWKB and EWKT", plus the reader in `liblwgeom/lwin_wkt_lex.l`.

## Purpose

PostGIS Extended Well-Known Text reader and writer — OGC WKT with an
optional `SRID=<digits>;` prefix and the glued dimension-suffix spellings
`POINTM`/`POINTZ`/`POINTZM` that `ST_AsEWKT` and `ST_GeomFromEWKT` use.
The geometry body itself is delegated to `geometry-io-wkt`; this crate owns
only the prefix and the spelling.

## Files

| File | Contents |
|---|---|
| `src/srid.rs` | `Srid` — the newtype over the prefix's integer, and `Srid::UNKNOWN` |
| `src/srid_prefix.rs` | `scan` — the `SRID=<digits>;` prefix scanner, and where the body starts |
| `src/dimension_suffix.rs` | `normalise` — blanks a glued `Z`/`M`/`ZM` suffix in place, preserving every byte offset |
| `src/ewkt_error.rs` | `EwktError` — `InvalidSrid { reason, pos }` or a wrapped `WktError` |
| `src/ewkt.rs` | `Ewkt<G>`, `from_ewkt` + the typed `parse_*`, `to_ewkt`, `write_ewkt`, `to_ewkt_polygon` |

## Public surface

`from_ewkt` returns an `Ewkt<DynGeometry>` — the geometry plus
`Option<Srid>`, which is `None` exactly when the input carried no prefix.
The typed `parse_*` functions mirror the WKT crate's, wrapping the same
return types in `Ewkt`. On the write side `to_ewkt` takes the SRID as an
`Option<Srid>` argument rather than reading it off the geometry, because
the model types carry no SRID; passing `None` produces output byte-identical
to `geometry-io-wkt`'s. Every error position indexes the caller's original
string, prefix included.

## Who depends on this

Nothing. It is the top of the WKT branch of the spine, and the facade does
not re-export the I/O crates.
