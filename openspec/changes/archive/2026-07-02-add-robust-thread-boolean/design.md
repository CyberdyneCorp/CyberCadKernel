## Context

`cc_helical_thread` / `cc_tapered_thread` build a fine multi-turn helical solid
(`buildHelicalThread` in `occt_construct.cpp`: a thread profile swept along a
sampled helix spine via `BRepOffsetAPI_MakePipeShell`, with many turns and a high
per-turn sample count). Fusing/cutting that whole helix against a shaft in one
`BRepAlgoAPI_Fuse`/`_Cut` is the pathological OCCT case: the two operands share a
long, spiralling, near-tangent intersection with a huge number of section curves,
and the boolean's cost blows up super-linearly — minutes of wall-clock, which the
app experiences as a hang. Phase 1 added a **fine-thread gate**
(`checkFineThreadGate` + `ThreadTag`) that detects this from the thread's
turns/pitch provenance and refuses the single-shot boolean, leaving thread and
shaft as separate bodies. That prevents the hang but does not deliver the fused
feature.

The robust idea: the intersection of the shaft with any ONE turn of the thread is
a small, local, bounded problem. So segment the thread along the shaft axis into
per-turn (or small per-turn-group) sub-solids and accumulate the boolean one
segment at a time. Each sub-boolean intersects only a local axial band of the
current accumulator, so its cost is bounded and roughly constant per turn — total
time ≈ linear in turns, well inside a budget, instead of the single-shot blow-up.

Constraints:
- **Real completion bar**: the test must assert the apply COMPLETES within a
  strict wall-clock budget (e.g. < 8 s) AND yields a `BRepCheck_Analyzer::IsValid`,
  watertight solid with the correct volume-change sign. It must NOT run the naive
  hanging path (that would hang the suite); completion-within-budget is itself the
  asserted property.
- **ABI**: one additive entry point `cc_thread_apply`; no existing signature
  changes.
- **Reuse**: build on OCCT `BRepAlgoAPI` + the Phase-1 fuzzy/parallel tuning and
  the Phase-0 scheduler budget/cancellation — Phase 3 permits building on OCCT
  booleans.
- **Honesty**: if a case cannot be made valid within the budget, return `0`
  (separate bodies) and mark deferred with the measured time/validity — no fake.

## Goals / Non-Goals

Goals:
- `cc_thread_apply(shaft, thread, op)` producing the fused (external) or cut
  (internal) threaded shaft as one valid watertight solid.
- Segmented per-turn accumulation with tuned fuzzy + parallel, under a scheduler
  time budget with cancellation.
- A sim check asserting completion-within-budget + IsValid + watertight + correct
  volume sign, WITHOUT running the naive full boolean.
- Honest deferral when a case exceeds the budget or cannot be made valid.

Non-Goals:
- A native (non-OCCT) boolean kernel (Phase 4 `add-native-booleans`).
- Changing `cc_boolean` or the thread builders, or removing the Phase-1 gate.
- Threads on non-cylindrical/non-conical shafts beyond what the thread builder
  already supports.

## Decisions

- **New additive entry point `cc_thread_apply(shaft, thread, op)`.** `op = 0`
  fuse (external thread), `op = 1` cut (internal thread), `op = 2` returns `0`
  (common is not a thread apply). Routed through a new `IEngine::thread_apply`
  virtual (default `engine_unsupported`), overridden by the OCCT adapter; stub →
  `0`.
- **Axial segmentation from thread provenance.** Use the thread's `ThreadTag`
  (turns, pitch) and its axis-aligned extent to slice the thread into `N`
  segments, one per turn (or a small group of turns), each a bounded axial band.
  Segmentation is by intersecting the thread with axial half-spaces / a bounding
  slab per band (or by rebuilding per-turn thread sub-solids from the same
  parameters) — whichever yields valid segments; documented.
- **Accumulate with tuned fuzzy + parallel.** Start `acc = shaft`; for each
  segment `s_k`, `acc = Fuse/Cut(acc, s_k)` with `SetFuzzyValue` (the Phase-1
  feature-size-tied fuzz) and `SetRunParallel(true)`, gating each `acc` on
  `BRepCheck_Analyzer::IsValid`. Fixed segment order (increasing axial position)
  for determinism.
- **Scheduler budget + cancellation.** Run the accumulation as a scheduler task
  with a stop-token and a wall-clock budget; if the budget is exceeded, cancel and
  return `0` (deferred), never hang. The budget is the test's asserted property.
- **Correctness by volume sign, not a proxy.** External thread (fuse) →
  `V_after > V_shaft` (added crest material); internal thread (cut) →
  `V_after < V_shaft` (removed valley material); magnitude in the plausible range
  of the thread's own volume. Asserted from `cc_mass_properties`.
- **Do NOT run the naive path in the test.** The single-shot boolean is known to
  hang; the test only runs `cc_thread_apply` and asserts it finished within budget
  (a real, non-trivial property: a stopwatch around the call), IsValid,
  watertight, and correct sign.

## Risks / Trade-offs

- **Segment boundary artefacts.** Splitting the helix at turn boundaries can leave
  seams if a segment's cut face does not align with the next; mitigated by
  overlapping segments slightly and letting the fuzzy heal the seam, gating each
  accumulation on IsValid. If seams persist, the case is deferred.
- **Fuzzy tuning.** Too small → the near-tangent thread/shaft contact fails to
  resolve; too large → merges real crest/valley features. Tied to feature size
  (thread depth / pitch) and validated by IsValid + volume sign; documented.
- **Budget is machine-dependent.** 8 s is chosen for the simulator; on-device may
  differ. The asserted property is "completes within the stated budget on the sim"
  — an honest, real bar; on-device timing is deferred (Phase 0/1 acceptance).
- **Some inputs may still not complete valid-in-budget.** Those return `0`
  (separate bodies, as the gate already allows) and are marked deferred with the
  measured time — not faked.
- **Naive path not exercised.** By design (it hangs); we assert the segmented path
  finished, which is the meaningful signal that the hang is avoided.

## Migration Plan

1. Add `IEngine::thread_apply(shaft, thread, op)` (default `engine_unsupported`);
   stub inherits `0`.
2. OCCT adapter: axial segmentation of the thread (from `ThreadTag` + extent),
   segment order fixed by axial position.
3. Accumulate `Fuse`/`Cut` per segment with tuned fuzzy + `SetRunParallel`, each
   gated on `BRepCheck_Analyzer::IsValid`.
4. Wrap the accumulation in a scheduler task with a stop-token + wall-clock budget;
   exceed → cancel → `0` (deferred).
5. Add `cc_thread_apply` to `include/cybercadkernel/cc_kernel.h` +
   `src/facade/cc_kernel.cpp` (additive); `test_abi` unchanged.
6. iOS-sim check: build a fine multi-turn thread + shaft, call `cc_thread_apply`
   inside a stopwatch; assert elapsed < 8 s, result IsValid + watertight, and
   volume sign correct. Do NOT run the naive `cc_boolean` on the same operands.
7. Host stub `0`; `run-sim-suite.sh` unchanged (additive).
8. `openspec validate --all --strict`; update `ROADMAP.md` Phase 3 status.

## Open Questions

- Segment granularity (one turn vs a group of turns) that best balances per-boolean
  cost against segment count — pin from the sim timing on the fine-thread fixture.
- The exact fuzzy value + segment overlap that heals turn-boundary seams without
  merging features — tuned empirically, gated by IsValid + volume sign.
- Whether to also expose a build-and-apply convenience (`cc_thread_apply_params`)
  — deferred; the two-body entry point is the composable primitive and is enough
  to satisfy the checks.
- The final wall-clock budget constant (8 s target) — confirmed from the sim
  measurement; recorded with the result.
