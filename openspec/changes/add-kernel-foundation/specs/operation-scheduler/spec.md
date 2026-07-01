# operation-scheduler

## ADDED Requirements

### Requirement: Off-thread operation execution
The library SHALL execute kernel operations off the host UI thread so that
long-running operations never block the caller's main thread.

#### Scenario: Long operation does not block the UI
- GIVEN a long-running operation (e.g. a fine-thread boolean)
- WHEN it is submitted through the scheduler
- THEN the calling UI thread SHALL remain responsive while it runs

### Requirement: Cancellable operations
The library SHALL allow an in-flight operation to be cancelled cooperatively via
a cancellation token, and a cancelled operation SHALL return a cancelled status
rather than a result.

#### Scenario: Cancel an in-flight build
- GIVEN an operation running under the scheduler
- WHEN the host requests cancellation
- THEN the operation SHALL stop at the next cooperative checkpoint and report a
  cancelled outcome

### Requirement: Progress reporting
The library SHALL let an operation report progress (0..1 or staged) to the host
during execution.

#### Scenario: Report progress during meshing
- GIVEN a tessellation operation submitted to the scheduler
- WHEN it runs
- THEN it SHALL emit progress updates the host can observe

### Requirement: Cancellation-safe engine boundary
Cancellation SHALL degrade safely when the underlying engine operation is not
itself interruptible: the library SHALL surface a cancelled result to the caller
and reclaim resources, even if the engine call must run to completion internally.

#### Scenario: Non-interruptible engine call is cancelled by the host
- GIVEN an engine operation that cannot be interrupted mid-computation
- WHEN the host cancels it
- THEN the library SHALL report a cancelled outcome to the host and discard the
  computed result without leaking resources
