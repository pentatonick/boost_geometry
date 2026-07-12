# `geometry-io-svg`

**I/O peer, consumes `geometry-model`.** `#![no_std]` + `alloc`.

Mirrors `boost/geometry/io/svg/svg_mapper.hpp`. Cartesian only —
a debugging convenience, not a production output format.

## Purpose

Emit a self-contained `<svg>` document from accumulated geometries — useful
for visually inspecting test fixtures and intermediate overlay results
while developing.

## Files

| File | Contents |
|---|---|
| `src/mapper.rs` | `SvgMapper` |

## Public surface

`SvgMapper` accumulates geometries, tracks their combined bounding box, and
maps world coordinates to a fixed pixel canvas (y-flipped, since SVG's y
axis grows downward while Cartesian geometry's grows upward).
