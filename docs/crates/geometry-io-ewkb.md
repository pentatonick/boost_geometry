# `geometry-io-ewkb`

**I/O peer, consumes `geometry-io-wkb`, `geometry-srid`, and `geometry-model`.**
`#![no_std]` + `alloc`.

Not part of Boost.Geometry — Boost ships no WKB, EWKB doubly so.
Reference: the PostGIS manual §4.2.1 "PostGIS EWKB and EWKT", plus the
writer in `liblwgeom/lwout_wkb.c` and the reader in `liblwgeom/lwin_wkb.c`.

## Purpose

PostGIS Extended Well-Known Binary reader and writer — OGC WKB with a
PostGIS header dialect: four flag bits in the 32-bit type word, and an
optional 32-bit spatial-reference id between the type word and the body.
Everything after that header is byte-identical OGC WKB, so this crate is
a header codec: it delegates every record body to `geometry-io-wkb`
rather than carrying a second parser. Only the **outermost** record ever
carries an SRID, which is what PostGIS writes; this is a strictly-2D
reader, and the `Z`, `M`, and bounding-box flags are refused by name
rather than silently dropped. Hex EWKB — the text form PostGIS gives a
`geometry` column — is also in scope: `from_ewkb_hex`/`to_ewkb_hex` wrap
the same binary core.

## Files

| File | Contents |
|---|---|
| `src/ewkb.rs` | `Ewkb<G>`, `from_ewkb`, `to_ewkb`, `to_ewkb_polygon` |
| `src/ewkb_error.rs` | `EwkbError` |
| `src/hex.rs` | `from_ewkb_hex`, `to_ewkb_hex` — the hex codec, hand-rolled with no dependency |
| `src/record_header.rs` | The PostGIS header codec: adds or strips the flag bits and the SRID field. Crate-internal — no byte offset is computed outside it |

## Public surface

`from_ewkb` returns an `Ewkb<DynGeometry>` — the geometry, the `Option<Srid>`
its record carried (`None` exactly when the type word set no SRID flag),
and the `ByteOrder` the outermost record declared. `to_ewkb` and
`to_ewkb_polygon` write a geometry or a caller's own polygon type to EWKB;
passing `srid: None` produces output byte-identical to `geometry-io-wkb`'s
plain OGC WKB. `from_ewkb_hex`/`to_ewkb_hex` are the same read/write pair
over the hex form: output is uppercase with no `SRID=` prefix (what
PostGIS emits), input accepts either case. `EwkbError` covers a refused
flag (`BoundingBoxFlag`, `DimensionFlag`), a truncated SRID
(`TruncatedSrid`), a bad hex string (`InvalidHex`), and a wrapped
`WkbError` for anything wrong in the record body underneath. `Srid` and
`ByteOrder`/`WkbError` are re-exports of `geometry-srid` and
`geometry-io-wkb`, not defined here.

Empty polygons encode as zero rings, including inside multipolygons and
collections. Other ring structure is preserved without validation, so callers
must supply well-formed geometries when the consumer requires them. The reader
accepts the historical one-empty-ring encoding and the writer normalizes it to
zero rings.

## Who depends on this

Nothing. The facade does not re-export the I/O crates.
