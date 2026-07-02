# thread-boolean Specification

## Purpose
TBD - created by archiving change add-robust-thread-boolean. Update Purpose after archive.
## Requirements
### Requirement: Additive thread-apply C ABI
The library SHALL expose one additive C-ABI entry point `CCShapeId
cc_thread_apply(CCShapeId shaft, CCShapeId thread, int op)` that applies a helical
thread body to a shaft body: `op = 0` fuses the thread onto the shaft (external
thread), `op = 1` cuts the thread out of the shaft (internal thread), and any
other `op` (including `2`/common) SHALL return `0`. On success it SHALL return a
new body id; on failure it SHALL return `0`. This is the ONLY C ABI addition; no
existing `cc_*` signature or POD struct layout SHALL change, and `cc_boolean` and
the thread builders SHALL be unchanged.

#### Scenario: ABI addition is source-compatible
- GIVEN the host app previously linked the kernel
- WHEN it links the version with `cc_thread_apply`
- THEN every existing `cc_*` signature SHALL be unchanged AND the ABI contract
  test (`tests/test_abi.cpp`) SHALL still pass

#### Scenario: Host stub is a safe no-op
- GIVEN a build with no B-rep engine (the host stub)
- WHEN `cc_thread_apply` is called
- THEN it SHALL return `0` without crashing

#### Scenario: Unsupported op returns zero
- GIVEN any two valid bodies
- WHEN `cc_thread_apply(shaft, thread, 2)` is called
- THEN it SHALL return `0`

### Requirement: Fine multi-turn thread apply completes within a time budget
`cc_thread_apply` SHALL apply a fine, multi-turn helical thread to a shaft by a
segmented / feature-based boolean (per-turn or per-turn-group accumulation with a
tuned fuzzy value under the operation scheduler), completing within a strict
wall-clock budget where a single brute-force `BRepAlgoAPI` boolean on the full
helix would hang. The check SHALL assert completion within the budget and SHALL
NOT execute the naive single-shot boolean on the same operands.

#### Scenario: Apply of a fine thread finishes under the budget
- GIVEN a fine, multi-turn helical thread body (many turns, high sample count)
  and a shaft body, on a booted iOS simulator
- WHEN `cc_thread_apply(shaft, thread, op)` is called inside a wall-clock
  stopwatch
- THEN the call SHALL return a non-zero body AND the measured elapsed time SHALL
  be under the documented budget (e.g. 8 seconds)
- AND the naive `cc_boolean(shaft, thread, op)` SHALL NOT be executed in the test
  (it is known to hang; completion of `cc_thread_apply` within budget is the
  asserted property)

### Requirement: Threaded shaft is a valid watertight solid with correct volume sign
The body returned by `cc_thread_apply` SHALL be `BRepCheck_Analyzer::IsValid` and
watertight (a closed shell with no free boundary), and its exact B-rep volume
SHALL change in the correct direction: an external thread (`op = 0`, fuse) SHALL
make the result volume GREATER than the shaft volume, and an internal thread
(`op = 1`, cut) SHALL make it LESS than the shaft volume, by a magnitude
consistent with the thread's own volume.

#### Scenario: External thread apply is valid and adds volume
- GIVEN a shaft with volume `V_shaft` from `cc_mass_properties`, on a booted iOS
  simulator
- WHEN `cc_thread_apply(shaft, thread, 0)` produces a body with volume `V_after`
- THEN the result SHALL be `BRepCheck_Analyzer::IsValid` and watertight AND
  `V_after` SHALL be strictly greater than `V_shaft`, with the delta in the
  documented plausible range of the thread volume

#### Scenario: Internal thread apply is valid and removes volume
- GIVEN a shaft with volume `V_shaft` from `cc_mass_properties`, on a booted iOS
  simulator
- WHEN `cc_thread_apply(shaft, thread, 1)` produces a body with volume `V_after`
- THEN the result SHALL be `BRepCheck_Analyzer::IsValid` and watertight AND
  `V_after` SHALL be strictly less than `V_shaft`, with the delta in the
  documented plausible range of the thread volume

### Requirement: Deterministic, cancellable, honest fallback
The segmented apply SHALL use a fixed segment order (increasing axial position)
and a fixed fuzzy value so repeated runs on the same input are reproducible, and
SHALL run under the operation scheduler's stop-token / wall-clock budget so a
pathological input is cancelled rather than hung. If the apply cannot produce a
valid solid within the budget, it SHALL return `0` (leaving thread and shaft as
separate bodies) and the case SHALL be recorded as deferred with the measured
time/validity — it SHALL NOT be faked as passing.

#### Scenario: Repeated apply is reproducible
- GIVEN the same shaft, thread, and op on a booted iOS simulator
- WHEN `cc_thread_apply` runs twice
- THEN both results SHALL have the same exact volume and bounding box within a
  tight tolerance

#### Scenario: Budget-exceeding input is cancelled, not hung
- GIVEN an input for which the segmented apply cannot complete a valid solid
  within the budget
- WHEN `cc_thread_apply` runs under the scheduler budget
- THEN it SHALL be cancelled and return `0` (thread and shaft remain separate)
  within a bounded time, AND the case SHALL be recorded as deferred with the
  measured time — NOT reported as a pass

