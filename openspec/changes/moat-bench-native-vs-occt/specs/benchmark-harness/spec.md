# benchmark-harness

## ADDED Requirements

### Requirement: Native-vs-OCCT latency benchmark harness

The project SHALL provide a NON-SHIPPING latency benchmark that drives representative
`cc_*` operations through the public facade under BOTH geometry engines and reports the
per-operation median latency of each, so the native-vs-OCCT performance payoff of dropping
OCCT is measured rather than asserted. The harness SHALL exercise each operation once under
the OCCT engine (`cc_set_engine(0)`) and once under the NativeEngine (`cc_set_engine(1)`)
via the same public `cc_*` entry point the app calls, and SHALL NOT reach any internal API.

The harness SHALL be deterministic: it SHALL use fixed inputs (no `rand()` / no wall-clock
seeding), SHALL report the MEDIAN of a fixed iteration count after discarding warm-up
iterations, and SHALL report run-to-run variance (min/max). It SHALL record the engine that
actually SERVED each operation honestly — an operation the NativeEngine forwards to OCCT
SHALL be labelled forwarded and given NO native time, and an operation the OCCT engine
adapter does not implement SHALL be labelled OCCT-declined / native-only — so the reported
ratio never presents a forwarded or unavailable operation as a native speedup.

#### Scenario: Both engines are driven through the facade with a median-of-N timing

- GIVEN the latency harness built against a build where both the NativeEngine and the OCCT
  adapter are linked
- WHEN it times a native-domain operation (e.g. a planar boolean, tessellate, or
  mass_properties on a native body) under each engine
- THEN it SHALL report a median latency for the OCCT engine and a median latency for the
  NativeEngine over the same fixed input, each a median of the fixed measured-iteration
  count with warm-up iterations discarded, plus the min/max of that sample

#### Scenario: A forwarded or OCCT-declined operation is not reported as a native win

- GIVEN the latency harness times an operation the NativeEngine forwards to OCCT (e.g. a
  curved-face edge fillet) or one the OCCT adapter does not implement (e.g. a planar section)
- WHEN it emits the result row for that operation
- THEN it SHALL label the row forwarded (no native time) or native-only (OCCT declined)
  respectively AND SHALL NOT print a native/OCCT speedup ratio for it

### Requirement: OCCT-linked vs native-only binary-size measurement

The project SHALL provide a NON-SHIPPING runner that measures the shipped static-library and
link footprint of the OCCT-linked product build versus the native-only (post-`drop-occt`)
build for the iOS-simulator arm64 slice, so the binary-size payoff of dropping OCCT is
quantified. It SHALL build `libcybercadkernel.a` two ways that differ ONLY by the OCCT
adapter — one with the OCCT translation units compiled and linked, one with the
`CYBERCAD_M8_REHEARSAL` native-only configuration — and SHALL report the kernel archive size
delta, the OCCT static-toolkit footprint that would be dropped, and the count of OCCT
translation units and symbols eliminated.

The OCCT footprint SHALL be reported honestly in a way that distinguishes the on-disk
static-archive size (which over-counts, since a static archive carries unreachable objects)
from the dead-stripped final-link delta (the code that actually ships in the linked binary).

#### Scenario: The size runner reports both kernel delta and dropped OCCT footprint

- GIVEN the size runner with the trimmed iOS-simulator OCCT install and the numsci substrate
  available
- WHEN it builds the OCCT and native-only kernel archives and measures
- THEN it SHALL report the `libcybercadkernel.a` size for each build and their delta, the
  OCCT static-toolkit footprint (the linked-toolkit subset and the full install), a
  dead-stripped final-link delta for a representative reachable set, and the number of OCCT
  adapter translation units and OCCT-side symbols eliminated

### Requirement: Measurement track does not modify product code

The benchmark track SHALL NOT modify any product code: `src/native/**`, `src/engine/**`,
`include/**`, and the `cc_*` ABI SHALL remain byte-unchanged. The track SHALL add only a
benchmark harness, its runners, a findings document, and roadmap/OpenSpec documentation.

#### Scenario: The committed diff touches only test, script, and documentation paths

- GIVEN the committed benchmark track
- WHEN its `git diff` is inspected
- THEN it SHALL touch only paths under `tests/`, `scripts/`, `docs/`, and `openspec/` AND
  SHALL NOT touch `src/native`, `src/engine`, `include`, or any `cc_*` ABI declaration
