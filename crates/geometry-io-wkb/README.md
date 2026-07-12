# geometry-io-wkb

OGC Well-Known Binary (WKB) reader and writer. Follows OGC 06-103r4 §8.
`from_wkb` parses bytes into a `DynGeometry`; `to_wkb` serialises a model
geometry to a byte vector in a chosen `ByteOrder`.
