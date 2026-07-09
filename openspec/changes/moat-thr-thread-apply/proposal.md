# Proposal — moat-thr-thread-apply (native `cc_thread_apply`: thread ↔ shaft boolean)

## Why

`cc_thread_apply(shaft, thread, op)` applies a helical thread body to a shaft — `op 0`
FUSES the crest (external thread), `op 1` CUTS the groove (internal thread). It is a
Class-B drop-OCCT op and one of the app's **#1 acceleration wall**: the app's OCCT-usage
catalogue records that `BRepAlgoAPI_Fuse/Cut` of a fine multi-turn helix into a shaft
"pegs OCCT for minutes and its `Build` is non-cancellable", so the app keeps thread +
shaft as separate bodies (GitHub #286). The OCCT adapter (`occt_thread_boolean.cpp`) only
makes it survivable by REBUILDING the thread one turn at a time and accumulating bounded
per-turn booleans under a wall-clock budget — an approximation, not a single-shot boolean.

The native engine hard-declined `thread_apply` on any native operand
(`native_engine.cpp` `CC_NATIVE_BODY_UNSUPPORTED`). With the M2 freeform-boolean substrate
landed (two-operand welds, curved-wall cut, freeform↔freeform closed-seam cut) and the
native planar-polyhedron BSP boolean (`boolean_solid`) proven to weld faceted curved
solids watertight (a faceted cylinder CUT by a box welds `bnd=0`, `sd=0`), thread_apply is
the next tractable-input Class-B op to attempt natively: it is fundamentally the boolean
of a shaft cylinder with a swept helical thread ridge.

## What

One ADDITIVE OCCT-free header-only verb `threadApply`
(`src/native/boolean/thread_apply.h`, namespace `cybercad::native::boolean`) plus its
engine wiring (`NativeEngine::thread_apply`) that:

1. **recognises the tractable input family** — the shaft is an axis-parallel finite
   cylinder (reuse `curved::recogniseCylinder`) and the thread is a native helical-ridge
   solid coaxial with it (measured crest radius > shaft radius for FUSE / groove inside
   the shaft for CUT), declining `ShaftNotCylinder` / `ThreadDegenerate` / `CrestBelowShaft`
   otherwise;
2. **facets both operands** into consistently-oriented planar-triangle B-rep solids at a
   controlled faceting deflection (the shaft to a many-sided prism, the thread to its
   triangulated ridge), so the exact planar BSP set-algebra applies;
3. **runs the native BSP boolean** `boolean_solid(shaftFacets, threadFacets, op)` — FUSE
   (`op 0`) unions the ridge onto the shaft, CUT (`op 1`) removes the groove;
4. **self-verifies the result under the full gauntlet** — WATERTIGHT + Euler χ = 2 +
   consistently-oriented (`tess::isConsistentlyOriented`, the DIRECTED-edge invariant) +
   a TWO-SIDED enclosed-volume band against the closed-form threaded-shaft volume
   (`V_fuse = V_shaft + V_ridge_outside`, `V_cut = V_shaft − V_groove_inside`, both
   deflection-bounded), rejecting any leaky / T-junction-cracked / orientation-inconsistent
   / wrong-volume candidate;
5. **honest-declines to OCCT** with a typed `ThreadApplyDecline` on ANY failure — the
   native path NEVER emits a leaky / partial / orientation-inconsistent / wrong-volume
   solid, and NEVER forwards a native void to OCCT.

## Outcome (measured, honest)

The recognition, closed-form analytic oracle, faceted-BSP attempt, and the full
self-verify gauntlet are landed, tested (host gate) and grounded native-vs-OCCT (sim
gate). The measured result is a **sharpened honest decline**: a multi-turn helical thread
does NOT weld to a watertight, consistently-oriented native solid today, for a precisely
identified reason — **the native helical-thread solid (`build_thread`) is watertight but
NOT consistently oriented (measured `sameDirectionEdgeCount == 6`, a latent cap/band
winding defect), so it cannot serve as a BSP boolean operand, and the near-tangent helical
root ↔ shaft-wall contact additionally fragments the dense-soup BSP into T-junction cracks
(`boundaryEdgeCount` 15–140 across every tested pitch/turns/inset/deflection).** Both are
verified defects, not tolerance issues: the self-verify catches them and declines to OCCT,
so no wrong solid is ever returned.

This lands the FULL native machinery + a razor-sharp next blocker (orientation-coherent
thread builder + robust dense-soup CSG with T-junction repair — M7b), with zero `cc_*` ABI
change; any future robustness win flips cases to native automatically through the same
self-verify. This is the disciplined "land the tractable machinery + a sharpened measured
decline" outcome — OCCT stays the oracle and the verified fallthrough.

## Impact

- Affected specs: `native-booleans` (ADDED requirement — the native `threadApply` verb +
  its self-verify/decline contract).
- Affected code (ADDITIVE): new `src/native/boolean/thread_apply.h`; the
  `NativeEngine::thread_apply` body (was an unconditional decline) now routes native
  operands through the verb with the self-verify + OCCT fallthrough. New host test
  (`tests/native/test_native_thread_apply.cpp`) and sim native-vs-OCCT parity harness
  (`tests/sim/native_thread_apply_parity.mm` + runner + `run-sim-suite.sh` SKIP entry).
- `src/native/**` stays OCCT-FREE; the tessellator and `construct/thread.h` are UNTOUCHED
  (the orientation defect is measured, not patched here); `cc_thread_apply` ABI unchanged.
