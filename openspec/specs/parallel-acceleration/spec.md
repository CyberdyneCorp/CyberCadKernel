# parallel-acceleration Specification

## Purpose
TBD - synced from change accelerate-multicore-occt (not archived; deferred tasks remain). Update Purpose when the change is archived.
## Requirements

### Requirement: Parallel boolean execution
The library SHALL run the OCCT-backed boolean path (fuse / cut / common behind
`cc_boolean`) with multi-core execution enabled (`BOPAlgo_Options::SetRunParallel`),
so a boolean scales across available CPU cores without any change to the `cc_*`
signature or result.

#### Scenario: A boolean uses multiple cores
- GIVEN two solids and a device with multiple CPU cores
- WHEN `cc_boolean` runs the OCCT boolean with parallel execution enabled
- THEN the operation SHALL use more than one core and return a result that passes
  the same `IsValid()` gate as the serial path

### Requirement: Tuned boolean tolerance for fine geometry
The library SHALL apply a tuned fuzzy tolerance (`SetFuzzyValue`) to the boolean
path so fine geometry (e.g. a fine multi-turn thread fused/cut into a shaft) is
robust and does not stall, and SHALL allow a per-operation override of that value.

#### Scenario: Fine-thread boolean is robustified
- GIVEN a fine-pitch thread and a shaft
- WHEN `cc_boolean` runs with the tuned fuzzy value
- THEN the boolean SHALL complete with a valid result rather than degenerating or
  stalling on the fine geometry

### Requirement: Parallel meshing
The library SHALL triangulate B-rep shapes with parallel per-face meshing enabled
(`BRepMesh_IncrementalMesh` `isInParallel`) behind `cc_tessellate` /
`cc_face_meshes`, producing the same triangulation as the serial mesher.

#### Scenario: Meshing uses multiple cores with identical output
- GIVEN a valid multi-face solid and a deflection
- WHEN `cc_tessellate` runs the mesher with parallel meshing enabled
- THEN it SHALL use multiple cores AND return the same vertex/triangle result as
  the serial mesher for the same deflection

### Requirement: Deterministic parallel results
Parallel execution SHALL preserve reproducible results: for a path enabled by
default, repeated runs of the same operation on the same input SHALL produce
identical output. A path that cannot be made reproducible SHALL remain serial.

#### Scenario: Repeated parallel runs are identical
- GIVEN the same input run twice through a default-parallel operation
- WHEN both runs complete
- THEN their outputs SHALL be bit-reproducible

### Requirement: Bounded worker pool
The library SHALL bound the number of worker threads via a host-settable policy
(over `OSD_ThreadPool`), with a sensible default derived from hardware
concurrency, so parallel work does not oversubscribe a mobile device.

#### Scenario: Host caps worker threads
- GIVEN the host sets a maximum worker-thread count
- WHEN a parallel boolean or mesh runs
- THEN it SHALL use no more than the configured number of worker threads

### Requirement: Cancellable accelerated operations
Long accelerated operations (boolean, meshing) SHALL run through the operation
scheduler off the UI thread and honour cooperative cancellation; because the
underlying OCCT `Build` is not interruptible, a cancelled operation SHALL return
a cancelled status and discard its result without leaking resources.

#### Scenario: Cancel a running parallel boolean
- GIVEN a long boolean running under the scheduler with parallel execution
- WHEN the host requests cancellation
- THEN the operation SHALL report a cancelled outcome and reclaim its resources,
  even though the OCCT call runs to completion internally

### Requirement: Fine-thread boolean gate
The library SHALL gate the fine-thread boolean until it completes within an
acceptable time budget. When the fuse/cut of a high-turn fine-pitch thread into a
shaft would exceed that budget, the library SHALL refuse it (keeping the thread
and shaft as separate bodies) and surface the gate decision to the host, rather
than hanging the application.

#### Scenario: A fine thread is too expensive to boolean
- GIVEN a high-turn fine-pitch thread and a shaft
- WHEN the host requests fusing the thread into the shaft and the operation
  exceeds the acceptable budget for fine threads
- THEN the library SHALL gate the operation (keep them as separate bodies) and
  report the gate decision, rather than hanging
