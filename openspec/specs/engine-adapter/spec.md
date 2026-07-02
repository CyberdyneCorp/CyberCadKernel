# engine-adapter Specification

## Purpose
TBD - synced from change add-kernel-foundation (not archived; deferred tasks remain). Update Purpose when the change is archived.
## Requirements

### Requirement: Pluggable geometry engine
The library SHALL define an internal engine interface for geometry operations
(construction, boolean, fillet, tessellation, query, transform, data exchange)
so that multiple implementations can be selected behind the same `cc_*` call.

#### Scenario: Facade delegates to the active engine
- GIVEN an engine implementation registered as active
- WHEN a `cc_*` operation is invoked
- THEN the facade SHALL delegate to that engine and return its result via the
  handle/status model

### Requirement: OCCT adapter as first implementation
The library SHALL provide an OCCT-backed engine implementing the full operation
set currently used by CyberCad (per `cybercad/openspec/specs/occt-usage`),
preserving today's behaviour including `IsDone()`/`BRepCheck_Analyzer::IsValid()`
result validation.

#### Scenario: Parity with the current bridge
- GIVEN an operation supported by the current in-app OCCT bridge
- WHEN the same operation runs through the OCCT adapter
- THEN it SHALL produce an equivalent shape and the same success/failure outcome

### Requirement: Coexisting implementations for migration
The interface SHALL allow an OCCT-backed and a native implementation of the same
capability to coexist and be selectable, so a native implementation can be
validated against OCCT behind one facade call before OCCT is retired for that
capability.

#### Scenario: Compare native vs OCCT for one capability
- GIVEN a native implementation of a capability and the OCCT implementation
- WHEN the same inputs are run through both
- THEN the library SHALL be able to execute either and expose results for
  correctness/performance comparison

### Requirement: No engine type in the public surface
Engine-specific types SHALL remain internal to the adapter; the public facade
SHALL depend only on the C ABI.

#### Scenario: Public header is engine-agnostic
- WHEN a public header is inspected
- THEN it SHALL not reference OCCT (or any engine) types
