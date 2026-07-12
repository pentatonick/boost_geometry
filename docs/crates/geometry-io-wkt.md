# `geometry-io-wkt`

**I/O peer, consumes `geometry-model`.** `#![no_std]` + `alloc`.

Mirrors `boost/geometry/io/wkt/{read,write,wkt}.hpp`. Reference grammar:
OGC Simple Feature Access Part 1 (SFA-1) §7.

## Purpose

Well-Known Text reader and writer — the geo-ecosystem lingua franca.

## Files

| File | Contents |
|---|---|
| `src/lexer.rs` | `Token`, `WktError` — tokenizer |
| `src/parse.rs` | `from_wkt` + typed `parse_point`/`parse_linestring`/`parse_polygon`/`parse_multi_point`/`parse_multi_linestring`/`parse_multi_polygon` |
| `src/write.rs` | `to_wkt`, `write_wkt`, `WriteWkt` trait |

## Public surface

`from_wkt` parses into a `DynGeometry` (WKT is heterogeneous by
construction — a `GEOMETRYCOLLECTION` mixes kinds). The typed
`parse_*` functions are a convenience for callers who already know the
expected kind and don't want to match on `DynGeometry`. `to_wkt`/`write_wkt`
serialise any concrete geometry implementing the model traits — no
`DynGeometry` required on the output side.
