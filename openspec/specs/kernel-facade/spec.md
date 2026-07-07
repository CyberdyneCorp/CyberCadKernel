# kernel-facade Specification

## Purpose
TBD - synced from change add-kernel-foundation (not archived; deferred tasks remain). Update Purpose when the change is archived.
## Requirements
### Requirement: Stable plain-C ABI
The library SHALL expose its functionality only through a plain-C ABI (`cc_*`
functions, POD structs, integer handles) that is binary-compatible with
CyberCad's existing `KernelBridgeAPI.h`. No C++ or engine (OCCT) type SHALL
appear in a public header.

#### Scenario: App links the library without source change
- GIVEN a host app built against the existing `cc_*` facade
- WHEN it links CyberCadKernel instead of the in-app OCCT bridge
- THEN it SHALL compile and run against the same `cc_*` signatures with no source
  changes

#### Scenario: ABI evolves additively
- GIVEN a released version of the facade
- WHEN new functionality is added
- THEN existing `cc_*` signatures SHALL remain unchanged (additive-only), so
  previously built callers keep working

### Requirement: Shape registry with opaque handles
The library SHALL manage geometry via integer handles (`CCShapeId`), where 0 is
the invalid/not-built sentinel, and SHALL provide explicit release of a handle's
resources. Engine objects behind a handle SHALL NOT be exposed to the caller.

#### Scenario: Build returns a handle
- WHEN a construction call (e.g. `cc_solid_extrude`) succeeds
- THEN it SHALL return a non-zero `CCShapeId` referring to the built shape

#### Scenario: Release frees resources
- GIVEN a valid `CCShapeId`
- WHEN `cc_shape_release` is called on it
- THEN the underlying resources SHALL be freed and the id SHALL no longer resolve

### Requirement: Exception-to-status guard model
Every `cc_*` entry point SHALL be wrapped so that an engine failure (OCCT
`Standard_Failure` or any C++ `std::exception`) is caught and converted to a
0/nil result rather than propagating across the C ABI, and a human-readable
message SHALL be retrievable for the current thread.

#### Scenario: Engine throws on degenerate input
- GIVEN an operation that raises an exception internally
- WHEN it is invoked through its `cc_*` function
- THEN the function SHALL return 0/nil
- AND `cc_last_error()` SHALL return the failure message for that thread

### Requirement: Engine availability probe
The library SHALL report whether a real B-rep engine is linked, so the host can
route to its fallback (preview) kernel when it is not.

#### Scenario: Query engine availability
- WHEN `cc_brep_available()` is called
- THEN it SHALL return non-zero if a B-rep engine is linked, zero otherwise

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

