## Why

Applying a helical thread to a shaft is the boolean OCCT is worst at
(`ROADMAP.md` Phase 3; GitHub #286; `occt-usage` §Performance). `cc_helical_thread`
/ `cc_tapered_thread` build a fine multi-turn helical solid (a swept thread
profile with many turns and a high sample count), and fusing/cutting that against
a shaft with a single `BRepAlgoAPI_Fuse` / `_Cut` pegs OCCT for **minutes** on the
intersection of two densely-faceted, near-tangent, spiralling surfaces — often
long enough that the app treats it as a hang. Phase 1 mitigated this by gating the
fine-thread boolean (keeping thread and shaft as separate bodies) rather than
letting it run, which is a correctness/UX stopgap, not a real feature apply.

This change adds a **robust, feature-based thread apply** that produces the fused
/ cut solid WITHOUT the brute-force hang: segment the boolean so each sub-operation
is between the shaft (or the running accumulator) and a SMALL piece of the thread
— one turn, or a small group of turns — with a tuned fuzzy value, and accumulate.
Each sub-boolean touches only a bounded, local region of the shaft, so total time
is roughly linear in turns and stays inside a strict budget, instead of the
super-linear blow-up of intersecting the whole multi-turn helix at once.

## What Changes

- Add an additive `cc_*` feature-apply entry point:
  - `CCShapeId cc_thread_apply(CCShapeId shaft, CCShapeId thread, int op)` — fuse
    (`op = 0`, external thread) or cut (`op = 1`, internal thread) the thread into
    the shaft using the robust segmented path, returning a new body (`0` on
    failure). `op = 2` (common) is not meaningful for a thread apply and returns
    `0`.
- Implement the robust apply in the OCCT adapter:
  - **Segment the thread** into per-turn (or small per-turn-group) sub-solids,
    using the thread's provenance (`ThreadTag` turns/pitch already tracked for the
    Phase-1 gate) plus its bounding box along the shaft axis, so each segment is a
    bounded axial slice of the helix.
  - **Accumulate** the boolean: start from the shaft, fuse/cut one segment at a
    time (`BRepAlgoAPI_Fuse`/`_Cut` with a tuned `SetFuzzyValue` and
    `SetRunParallel`), gating each sub-result on `BRepCheck_Analyzer::IsValid`.
    Because each segment intersects only a local band of the current accumulator,
    each sub-boolean is small and fast.
  - Run under the operation scheduler with a **time budget / cancellation**, so a
    pathological input is cancelled rather than hanging.
- Optionally add a **build-and-apply** convenience later (build the thread from
  params and apply in one call); the primary entry point takes two existing
  bodies to keep it composable and testable.
- The naive single-shot `cc_boolean(shaft, thread, op)` remains as-is (and remains
  gated by the Phase-1 fine-thread gate); `cc_thread_apply` is the path that
  actually completes the feature.

C ABI change is **ADDITIVE only**: one new entry point `cc_thread_apply`; no
existing `cc_*` signature or POD layout changes. OCCT-only; the host stub returns
`0`.

## Capabilities

### New Capabilities
- `thread-boolean`: a feature-based, segmented thread-to-shaft boolean
  (`cc_thread_apply`) that fuses/cuts a fine multi-turn helical thread into a
  shaft by accumulating per-turn (or per-turn-group) sub-booleans with a tuned
  fuzzy value under the operation scheduler's time budget — completing within a
  strict wall-clock bound where a single brute-force `BRepAlgoAPI` on the full
  helix would hang for minutes — and yielding a `BRepCheck_Analyzer::IsValid`,
  watertight solid with the correct volume-change sign. Builds on `engine-adapter`,
  the Phase-1 `parallel-acceleration` fuzzy/parallel tuning, and the Phase-0
  `operation-scheduler` cancellation/budget.

## Impact

- **Contract**: delivers the feature `occt-usage` §Performance / #286 calls for —
  a thread↔shaft boolean that completes — without changing `cc_boolean` or the
  thread builders; complements the Phase-1 gate (which prevents the hang) by
  providing the path that actually produces the fused/cut body.
- **App**: gains `cc_thread_apply` to realize a threaded shaft as one solid
  instead of two gated bodies.
- **Build**: OCCT-only (segmented `BRepAlgoAPI`); guarded by `CYBERCAD_HAS_OCCT`;
  host stub returns `0`.
- **Performance / determinism**: bounded per-segment work, fixed segment order
  (axial), tuned fuzzy; the result is reproducible and completes within the
  budget. The test asserts completion-within-budget and does NOT run the naive
  hanging path.
- **Risk / honesty**: if segmentation cannot produce a valid solid within the
  budget for a case, the op returns `0` (thread + shaft stay separate, as the
  gate already allows) and the case is marked deferred with the measured
  time/validity — never a faked pass.
