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
