## Context

Phase 0 leaves the OCCT adapter behaviourally identical to the app's current
bridge — including running booleans and meshing single-threaded. OCCT has
first-class parallel infrastructure (`OSD_Parallel`, `OSD_ThreadPool`, optional
TBB) and both the boolean pave-filler (`BOPAlgo_Options::SetRunParallel`) and the
mesher (`BRepMesh_IncrementalMesh` `isInParallel`) already ship a parallel path
that defaults **off for determinism** (`OCCT/openspec/acceleration.md`). This
change turns those paths on behind the facade and makes the long ops cancellable.

Constraints:
- **ABI stability**: no `cc_*` signature change; parallelism is internal.
- **Determinism**: parallelism must preserve reproducible results
  (`project.md` convention) — audited before it becomes the default.
- **Mobile**: bounded worker threads; unified-memory devices, but this phase is
  CPU-only (GPU is Phase 2).
- **Non-interruptible engine**: OCCT `Build` cannot be cancelled mid-computation;
  cancellation is cooperative-at-boundary, inherited from Phase 0's scheduler.

## Goals / Non-Goals

Goals:
- Enable OCCT parallel booleans (+ tuned fuzzy value) and parallel meshing behind
  `cc_boolean` / `cc_tessellate`.
- Make those long ops run off the UI thread and cancel cooperatively.
- Gate the fine-thread boolean until it is fast, instead of hanging the app.
- Prove determinism (parallel == serial) before defaulting to parallel.

Non-Goals (later phases):
- Any GPU offload (Phase 2 — Metal tessellation, BVH, picking).
- New task-parallelism for fillets/offsets/features that OCCT runs serially
  (candidate, but new work — later change).
- A thread-specific / feature-based boolean that replaces the general boolean
  (Phase 3 `add-robust-thread-boolean`, GitHub #286). This phase only accelerates
  and gates the *existing* boolean.

## Decisions

- **Reuse OCCT's parallel paths, don't write new ones.** `SetRunParallel(true)`
  on `BRepAlgoAPI_*` (via `BOPAlgo_Options`) and `isInParallel` on
  `BRepMesh_IncrementalMesh`. Lowest risk, immediate multi-core scaling.
- **Tuned `SetFuzzyValue`** on the boolean for fine geometry (the fine-thread
  fuse/cut). The value is chosen empirically and documented; too large distorts
  geometry, too small keeps it slow/fragile.
- **Worker cap via `OSD_ThreadPool`** (`SetNbDefaultThreadsToLaunch`) so mobile
  can bound CPU use; host-settable, sensible default from hardware concurrency.
- **Parallel is default-on but opt-out**, and only for paths that pass the
  determinism audit. Any path that cannot be made reproducible stays serial and
  is documented.
- **Cancellation reuses Phase-0 `operation-scheduler`.** Because OCCT `Build` is
  non-interruptible, cancel is honoured at the task boundary: the worker runs to
  completion internally but the result is discarded and resources reclaimed — the
  scheduler's "cancellation-safe engine boundary" requirement.
- **Fine-thread boolean gate.** Until the fused/cut fine thread completes within
  an acceptable budget, high-turn fine-pitch threads are refused / kept as
  separate bodies (mirrors today's app workaround, now owned by the kernel and
  removable once Phase 3's thread boolean lands).

## Risks / Trade-offs

- **Determinism under parallelism.** Parallel pave-filler / mesher could reorder
  results. Mitigation: the §5 audit gates default-on per path; serial fallback
  stays available.
- **Fuzzy value is a blunt instrument.** A single tuned default won't suit every
  model; expose it so specific ops can override. Document the trade-off.
- **Cancellation is cooperative-at-boundary only.** A running fine-thread boolean
  still burns CPU until it returns; the gate limits exposure, and Phase 3's
  thread boolean is the real fix. Documented as a known limitation.
- **Thread oversubscription on mobile** if the host also parallelises. Mitigation:
  the shared `OSD_ThreadPool` cap.

## Migration Plan

1. Add the parallelism policy (thread-pool cap + global/per-op toggle) to the
   OCCT adapter.
2. Flip `SetRunParallel(true)` + tuned `SetFuzzyValue` on the boolean path; keep
   the `IsValid()` gate.
3. Flip `isInParallel` on the mesher path; confirm identical triangulation.
4. Route both through the Phase-0 scheduler; add the fine-thread gate.
5. Run the determinism audit; enable parallel-by-default only where it passes.
6. Benchmark on device; update `ROADMAP.md` Phase 1 status.

## Open Questions

- The empirical `SetFuzzyValue` default and the fine-thread gate threshold (turns
  / pitch / wall-clock budget) — set from device benchmarks in §6.
- Whether meshing determinism holds across worker counts, or must be pinned to a
  fixed logical decomposition for reproducibility.
