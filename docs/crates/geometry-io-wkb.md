# `geometry-io-wkb`

**I/O peer, consumes `geometry-model`.** `#![no_std]` + `alloc`.

Follows OGC Simple Feature Access 06-103r4 §8. **Not part of Boost.Geometry**
— Boost ships WKT but not WKB; this crate fills that gap for the Rust port.

## Purpose

Well-Known Binary reader and writer, endianness-aware.

## Files

| File | Contents |
|---|---|
| `src/header.rs` | `ByteOrder`, `WkbHeader`, `WkbError`, `split_header` |
| `src/parse.rs` | `from_wkb`, `from_wkb_parts` |
| `src/write.rs` | `to_wkb`, `to_wkb_polygon`, `write_wkb_polygon`, `polygon_wkb_len` |

## Public surface

`from_wkb` parses bytes into a `DynGeometry` (same rationale as WKT — WKB is
heterogeneous). `to_wkb` serialises any concrete model geometry to a byte
vector in a caller-chosen `ByteOrder`.

`split_header`/`from_wkb_parts` and `write_wkb_polygon`/`polygon_wkb_len`
exist so a dialect crate (`geometry-io-ewkb`) can strip or splice a header
without re-deriving the record layout: `split_header` returns
`Result<WkbHeader<'_>, WkbError>` containing the declared byte order, raw
type word, and borrowed body slice; `from_wkb_parts`
parses a body whose header a caller already consumed; `write_wkb_polygon`
appends a complete polygon record to a caller's own buffer, sized in
advance by `polygon_wkb_len`. `ByteOrder::{from_flag, read_u32, to_bytes}`
are the codec primitives those two lean on. `WkbError::UnsupportedDimension`
is now three variants — `HigherDimension`, `UnexpectedSridFlag`,
`UnrecognisedTypeWord` — because a dialect header can be wrong in three
distinct ways a strictly-2D OGC reader now tells apart.

Empty polygons and empty bare rings encode as zero-ring polygons. The same
rule applies inside multi-geometries and collections, and length calculations
match the canonical output. Other ring structure is preserved without
validation: empty holes or holes without an exterior are not silently removed.

## Who depends on this

`geometry-io-ewkb`, which delegates every record body to this crate and
only owns the `PostGIS` header dialect on top of it.
