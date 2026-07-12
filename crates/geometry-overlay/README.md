# geometry-overlay

The boolean-overlay engine for the geometry port: the segment
intersection kernel, the turn graph, traversal, and the
`intersection` / `union` / `difference` / `sym_difference`
operations built on top of them.

Mirrors `boost/geometry/algorithms/detail/overlay/`. This is the
largest single subsystem in the port; it is built in strict phases
(see `specs/phase_03-rust-port-implementation-plan-overlay.md`):

* **OVL1 — predicate kernel** (`predicate/`): orientation, in-circle,
  segment-segment intersection, and the coordinate-range robustness
  gate. Robustness policy: exact input arithmetic, no rescale — see
  `specs/overlay-robustness-decision.md`.
* OVL2 — turn graph, OVL3 — traversal, OVL4 — output assembly,
  OVL5 — the four overlay free functions, then OVL6+ fan out
  (relate/is_valid/buffer/merge_elements).

Cartesian only for v1.
