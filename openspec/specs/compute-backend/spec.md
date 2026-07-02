# compute-backend Specification

## Purpose
TBD - synced from change add-kernel-foundation (not archived; deferred tasks remain). Update Purpose when the change is archived.
## Requirements

### Requirement: Compute-backend abstraction
The library SHALL define a compute-backend interface for data-parallel work
(e.g. batch surface evaluation, BVH build/traversal, picking, mesh
post-processing) so that CPU and GPU backends are interchangeable without
changing callers.

#### Scenario: Backend is selected without caller changes
- GIVEN a data-parallel task expressed against the compute-backend interface
- WHEN the active backend is CPU versus a GPU backend
- THEN the task SHALL run on either without changes to the calling code

### Requirement: Default CPU backend
The library SHALL provide a default CPU/multi-core backend so that no capability
depends on a GPU being present, and correctness SHALL not depend on which backend
is active.

#### Scenario: Runs with no GPU backend available
- GIVEN a build with no GPU backend registered
- WHEN a data-parallel task is executed
- THEN it SHALL run on the CPU backend and produce the same result a GPU backend
  would (within documented fp32 tolerance)

### Requirement: Platform-specific GPU backends are addable
The interface SHALL permit platform-specific GPU backends — **Metal** for Apple
targets first, and **CUDA / OpenCL / Vulkan** for desktop/Android later —
selected at build or runtime, without altering the facade or engine adapter.

#### Scenario: Add a Metal backend
- GIVEN the CPU backend is the default
- WHEN a Metal backend is registered on an Apple target
- THEN eligible data-parallel tasks SHALL run on Metal while the facade and
  engine adapter are unchanged

### Requirement: Precision boundary
GPU backends SHALL be used only for fp32-tolerant work; exact double-precision
geometry SHALL remain on the CPU. The library SHALL NOT route exact modeling
computations to a backend that cannot represent fp64.

#### Scenario: Exact modeling stays on CPU
- GIVEN an exact double-precision modeling computation
- WHEN backends are considered
- THEN it SHALL execute on the CPU and SHALL NOT be dispatched to an fp32-only GPU
  backend
