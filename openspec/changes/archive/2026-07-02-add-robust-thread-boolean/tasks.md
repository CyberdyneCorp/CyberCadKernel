# Tasks — add-robust-thread-boolean

Verification levels: **host** = the stub no-op + ABI contract run in the no-OCCT
host CTest; **ios-sim-build** = the segmented apply compiles for
`arm64-apple-ios16.0-simulator` with OCCT; **ios-sim-run** = `cc_thread_apply`
runs on the booted simulator and the completion-within-budget + IsValid +
watertight + volume-sign checks pass — this is the acceptance bar for every
performance/robustness requirement.

## 1. Entry point + routing
- [x] 1.1 Add `IEngine::thread_apply(EngineShape shaft, EngineShape thread, int
  op)` returning `ShapeResult`, default `engine_unsupported`; stub inherits it. (**host**)
- [x] 1.2 Add `CCShapeId cc_thread_apply(CCShapeId shaft, CCShapeId thread, int
  op)` to `include/cybercadkernel/cc_kernel.h` + `src/facade/cc_kernel.cpp`,
  routed through `active_engine()`; `op = 2` returns `0`. Additive only. (**host**)
- [x] 1.3 `tests/test_abi.cpp` still matches `KernelBridgeAPI.h`. (**host**)

## 2. Segmented boolean (OCCT adapter)
- [x] 2.1 Axial segmentation: slice the thread into per-turn (or per-turn-group)
  sub-solids using its `ThreadTag` (turns/pitch) + axis-aligned extent; fixed
  order by increasing axial position. (**ios-sim-build**)
- [x] 2.2 Accumulate: `acc = shaft`; for each segment `Fuse` (op 0) / `Cut`
  (op 1) with tuned `SetFuzzyValue` + `SetRunParallel(true)`; gate each `acc` on
  `BRepCheck_Analyzer::IsValid`. (**ios-sim-build**)
- [x] 2.3 Slight segment overlap + fuzzy to heal turn-boundary seams; final
  `addIfValid`. (**ios-sim-build**)

## 3. Scheduler budget + cancellation
- [x] 3.1 Run the accumulation as a scheduler task with a stop-token and a
  wall-clock budget; on budget-exceed, cancel and return `0` (deferred), never
  hang. (**ios-sim-build**)

## 4. Verification (REAL properties — completion asserted, naive path NOT run)
- [x] 4.1 Build a FINE multi-turn thread (`cc_helical_thread`, many turns / high
  sample count) + a shaft; call `cc_thread_apply(shaft, thread, 0)` inside a
  wall-clock stopwatch; assert elapsed < 8 s (completion within budget). (**ios-sim-run**)
- [x] 4.2 The result is `BRepCheck_Analyzer::IsValid` AND watertight (closed
  shell, no free boundary). (**ios-sim-run**)
- [x] 4.3 Volume sign correct: external thread (`op = 0`, fuse) →
  `V_after > V_shaft`; internal thread (`op = 1`, cut) → `V_after < V_shaft`, with
  the delta in the plausible range of the thread's own volume. (**ios-sim-run**)
- [x] 4.4 The naive `cc_boolean(shaft, thread, op)` is NOT executed in the test
  (it is known to hang); the asserted signal is that `cc_thread_apply` finished
  within budget. (**ios-sim-run**)
- [x] 4.5 Determinism: repeating the apply yields the same volume + bbox (fixed
  segment order + fixed fuzzy). (**ios-sim-run**)
  <!-- NOTE (measured): reproducible within tolerance, NOT bit-exact — repeat FUSE
  |ΔV|=0.2004 mm³ (rel 0.0002). Expected for parallel BOPAlgo (SetRunParallel(true)).
  Volume/bbox stable to ~2e-4 rel; not a bit-reproducible guarantee. -->
- [x] 4.6 Honest deferral: if a case exceeds the budget or cannot be made valid,
  `cc_thread_apply` returns `0` (thread + shaft stay separate) and the case is
  recorded deferred with the measured time/validity — NOT faked. (**ios-sim-run**)
  <!-- No case deferred: both tested cases finished within budget and valid —
  FUSE 4.3778 s (<8 s), CUT 4.4817 s (<8 s), both BRepCheck-valid + watertight. -->

## 5. Validation
- [x] 5.1 Host CTest green (stub `0` + ABI); `scripts/run-sim-suite.sh` unchanged
  (additive). (**host** + **ios-sim-run**)
- [x] 5.2 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 3 +
  change index for `thread-boolean`, recording the measured wall-clock + any
  deferred case honestly.
