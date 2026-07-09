# benchmark-harness

## ADDED Requirements

### Requirement: Native-vs-OCCT runtime-memory benchmark harness

The project SHALL provide a NON-SHIPPING benchmark that measures the RUNTIME MEMORY footprint
of representative `cc_*` operations under BOTH geometry engines, so the native-vs-OCCT runtime
memory payoff of dropping OCCT is measured rather than asserted. The harness SHALL exercise
each operation once under the OCCT engine (`cc_set_engine(0)`) and once under the NativeEngine
(`cc_set_engine(1)`) via the same public `cc_*` entry point the app calls, on the SAME fixed
deterministic inputs as the latency benchmark, and SHALL NOT reach any internal API.

The harness SHALL measure memory with process-level signals that count `mmap`-backed
allocations (process peak resident set via `getrusage(RUSAGE_SELF).ru_maxrss` and/or
`task_info(TASK_VM_INFO).phys_footprint`), NOT solely a malloc-zone statistic (which
under-counts large allocations on macOS). To avoid high-water-mark contamination, the per-op
peak SHALL be obtained with per-op isolation (one operation per process invocation, or an
equivalent per-op reset), and the harness SHALL be deterministic (no `rand()`, no wall-clock
seeding). It SHALL record the engine that actually SERVED each operation honestly — a forwarded
operation SHALL be given NO native number, and an operation the OCCT adapter does not implement
SHALL be labelled OCCT-declined / native-only — so no forwarded or unavailable operation is
presented as a native memory win.

#### Scenario: Both engines are driven through the facade with a per-op peak measured in isolation

- GIVEN the memory harness built against a build where both the NativeEngine and the OCCT
  adapter are linked
- WHEN it measures a native-domain operation (e.g. a planar boolean, tessellate, or
  mass_properties on a native body) under each engine
- THEN it SHALL report, for that operation under each engine, a peak resident / footprint
  figure obtained with per-op isolation so the figure is a clean high-water mark attributable
  to that engine performing that operation, over the same fixed input

#### Scenario: A whole-process peak captures each engine's working set plus static footprint

- GIVEN the memory harness running a fixed representative script (construct + booleans +
  tessellate + mass + section over the model-size spread) once per engine in a fresh process
- WHEN it reports the process-level result
- THEN it SHALL report the process peak resident set for each engine, so the comparison
  captures each engine's full working set and (for OCCT) its static/global footprint

#### Scenario: A forwarded or OCCT-declined operation is not reported as a native memory win

- GIVEN the memory harness measures an operation the NativeEngine forwards to OCCT (e.g. a
  curved-face edge fillet) or one the OCCT adapter does not implement (e.g. a planar section)
- WHEN it emits the result row for that operation
- THEN it SHALL label the row forwarded (no native number) or native-only (OCCT declined)
  respectively AND SHALL NOT present a native memory saving for it

### Requirement: Runtime-memory measurement track does not modify product code

The runtime-memory benchmark track SHALL NOT modify any product code: `src/native/**`,
`src/engine/**`, `include/**`, and the `cc_*` ABI SHALL remain byte-unchanged. The track SHALL
add only a benchmark harness, its runner, a findings document, and roadmap/OpenSpec
documentation.

#### Scenario: The committed diff touches only test, script, and documentation paths

- GIVEN the committed runtime-memory benchmark track
- WHEN its `git diff` is inspected
- THEN it SHALL touch only paths under `tests/`, `scripts/`, `docs/`, and `openspec/` AND SHALL
  NOT touch `src/native`, `src/engine`, `include`, or any `cc_*` ABI declaration
