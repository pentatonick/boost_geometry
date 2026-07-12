# geometry-io-wkt

OGC Well-Known Text (WKT) reader and writer for the geometry model.

Mirrors `boost/geometry/io/wkt`. `from_wkt` parses a WKT string into a
`DynGeometry`; `to_wkt` / `write_wkt` serialise any concrete model
geometry back to WKT. Typed-parse convenience functions
(`parse_point`, `parse_polygon`, …) are provided for callers who know
the expected kind.
