# kernel-facade

This change (Phase 4 #6 curved blends, the off-the-circle breadth batch) adds ONE ADDITIVE
plain-C entry to the facade — `cc_chamfer_edges_asym`, a two-distance chamfer — alongside
the unchanged `cc_chamfer_edges`. No existing signature or POD layout changes.

## ADDED Requirements

### Requirement: Additive asymmetric two-distance chamfer entry (`cc_chamfer_edges_asym`)

The plain-C facade SHALL expose `CCShapeId cc_chamfer_edges_asym(CCShapeId body, const int*
edgeIds, int edgeCount, double distance1, double distance2)` as an ADDITIVE entry — a chamfer
that sets the two adjacent faces back by DIFFERENT distances (`distance1` on the first face,
`distance2` on the second) — WITHOUT changing the existing `cc_chamfer_edges(body, edgeIds,
edgeCount, distance)` (a single symmetric distance) or any other `cc_*` signature or POD
layout. `cc_chamfer_edges_asym` SHALL resolve `body` through the same shape registry (the
same 1-based edge ids `cc_subshape_ids` / `cc_chamfer_edges` use), dispatch to the active
engine's `chamfer_edges_asym`, register the result, and return its handle, and SHALL be
wrapped by the SAME exception-to-status guard model as every other `cc_*` entry (any thrown
engine exception is converted to the invalid/zero shape handle, never propagated across the C
boundary). When `distance1 == distance2` the call SHALL be equivalent to the symmetric
`cc_chamfer_edges` with that distance. The default engine SHALL remain OCCT; the native path
is opt-in via `cc_set_engine(1)`, and any input the active engine cannot build natively SHALL
be served by the OCCT fallback with no facade behaviour change.

#### Scenario: The additive two-distance chamfer entry resolves and dispatches like cc_chamfer_edges
- GIVEN a registered `body` shape handle and a valid 1-based `edgeIds` selection, with `distance1 > 0` and `distance2 > 0`
- WHEN `cc_chamfer_edges_asym(body, edgeIds, edgeCount, distance1, distance2)` is called
- THEN the facade SHALL resolve `body` through the shape registry, dispatch to the active engine's `chamfer_edges_asym`, register the result, and return a valid `CCShapeId`, exactly as `cc_chamfer_edges` does for a single distance — and the existing `cc_chamfer_edges` signature and behaviour SHALL be unchanged

#### Scenario: A failing asymmetric chamfer is converted to a status, never propagated
- GIVEN a `cc_chamfer_edges_asym` call whose active engine throws (a degenerate selection, an unbuildable native slice that cannot fall back, or an invalid distance)
- WHEN the call is made across the plain-C boundary
- THEN the facade's exception-to-status guard SHALL convert the failure to the invalid/zero shape handle and SHALL NOT propagate a C++ exception across the ABI, identical to every other `cc_*` entry
