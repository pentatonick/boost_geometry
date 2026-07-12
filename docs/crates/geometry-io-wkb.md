# `geometry-io-wkb`

**I/O peer, consumes `geometry-model`.** `#![no_std]` + `alloc`.

Follows OGC Simple Feature Access 06-103r4 §8. **Not part of Boost.Geometry**
— Boost ships WKT but not WKB; this crate fills that gap for the Rust port.

## Purpose

Well-Known Binary reader and writer, endianness-aware.

## Files

| File | Contents |
|---|---|
| `src/header.rs` | `ByteOrder`, `WkbError` |
| `src/parse.rs` | `from_wkb` |
| `src/write.rs` | `to_wkb` |

## Public surface

`from_wkb` parses bytes into a `DynGeometry` (same rationale as WKT — WKB is
heterogeneous). `to_wkb` serialises any concrete model geometry to a byte
vector in a caller-chosen `ByteOrder`.
