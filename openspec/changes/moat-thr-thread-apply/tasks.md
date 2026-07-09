# Tasks — moat-thr-thread-apply (native `cc_thread_apply`: thread ↔ shaft boolean)

Order: explore/diagnose the substrate → tractable-input recognizer + closed-form oracle →
the composed verb (facet → BSP → self-verify → decline) → engine wiring → host analytic
gate → sim native-vs-OCCT gate → structural discipline + docs. All new native code stays
OCCT-free and host-buildable (`clang++ -std=c++20`), namespace `cybercad::native::boolean`.
No `cc_*` ABI change. Strictly ADDITIVE: `construct/thread.h`, the M0 tessellator,
`boolean_solid`, `curved.h` recognizers and every landed boolean header stay
BYTE-IDENTICAL. No tolerance weakened; a correct decline is first-class.

## STATUS

LANDED (BOTH gates), as a SHARPENED HONEST DECLINE. The verb `threadApply`
(`src/native/boolean/thread_apply.h`) recognises the tractable input (axis cylinder shaft
+ coaxial native helical-ridge thread via `curved::recogniseCylinder` + a measured
crest/root/z-extent), facets both operands into consistently-oriented planar-triangle
solids, runs the native planar BSP `boolean_solid`, and self-verifies WATERTIGHT + χ=2 +
consistently-oriented + a TWO-SIDED volume band vs the closed-form threaded-shaft volume,
returning NULL (→ OCCT) with a typed `ThreadApplyDecline` on any failure.

MEASURED OUTCOME: a multi-turn helical thread declines to OCCT for two verified reasons —
(1) the native `build_thread` solid is watertight but NOT consistently oriented (measured
`sameDirectionEdgeCount == 6`), so it is an invalid BSP operand; (2) the near-tangent
helical root ↔ shaft-wall contact fragments the dense-soup BSP into T-junction cracks
(`boundaryEdgeCount` 15–140 across single-turn to 4-turn, insets 0.6–1.5, deflections
0.05–0.15). Both are caught by the self-verify → the native path NEVER emits a wrong solid.
Baseline probe that ISOLATES the blocker: a faceted cylinder CUT by a box welds `bnd=0,
sd=0` through the SAME verb, so the BSP substrate is sound — the defect is specific to the
orientation-inconsistent thread operand + helical near-tangency (the sharpened next blocker
for M7b: an orientation-coherent thread builder + robust dense-soup CSG with T-junction
repair). Host gate + sim native-vs-OCCT gate both green. No tessellator change; no
`construct/thread.h` change; `cc_thread_apply` ABI unchanged.

## 1. Explore + diagnose

- [x] Read `cc_thread_apply` signature (`include/cybercadkernel/cc_kernel.h`) + the engine
      dispatch (`native_engine.cpp` `thread_apply`, was an unconditional decline).
- [x] Study the landed native thread constructors (`construct/thread.h`
      `build_helical_thread` / `build_thread`) and the M2 freeform-boolean substrate
      (`boolean/native_boolean.h` `boolean_solid`, `curved.h` recognizers,
      `freeform_freeform_cut.h` weld+self-verify pattern, `freeform_membership.h`).
- [x] Read the app's OCCT-usage catalogue for `thread_apply` (fine-thread boolean is the
      #1 wall; app keeps thread + shaft separate) + the OCCT oracle
      (`occt_thread_boolean.cpp` per-turn rebuild + accumulate).
- [x] Diagnose the tractability of a faceted-BSP thread boolean (prototypes): BSP welds
      faceted curved solids in general (cyl−box `bnd=0`), but the thread operand is
      orientation-inconsistent (`sd=6`) and the helical near-tangency fragments the BSP.

## 2. Recognizer + closed-form oracle

- [x] Reuse `curved::recogniseCylinder` for the shaft; measure the thread crest/root/z
      from its mesh (crest = max radius, root = min radius, z-extent = mesh bbox), mirroring
      the OCCT oracle's `measureThread` / `measureRootRadius`.
- [x] Closed-form threaded-shaft volume for the two-sided verify: `V_fuse = V_shaft +
      V_ridge_outside_shaft`, `V_cut = V_shaft − V_groove_inside_shaft`, with the ridge/
      groove volume measured from the thread mesh (exact for the tessellated ridge) and the
      shaft volume analytic (`π r² L`).

## 3. The composed verb (`thread_apply.h`)

- [x] `facetSolid(solid, deflection)` — mesh → consistently-oriented planar-triangle B-rep
      solid (each triangle a `construct::detail::planarFace` with the winding normal).
- [x] `threadApply(shaft, thread, op, deflection, why, analyticVolume)` — recognise → facet
      → `boolean_solid` → self-verify gauntlet → typed decline. Cognitive complexity within
      the backend band (delegates facet/verify to helpers).
- [x] Typed `ThreadApplyDecline` enum + `threadApplyDeclineName` (ShaftNotCylinder,
      ThreadDegenerate, CrestBelowShaft, BooleanEmpty, NotWatertight, NotOriented,
      VolumeInconsistent, Ok).

## 4. Engine wiring

- [x] `NativeEngine::thread_apply` routes two native operands through `threadApply`; a
      verified non-null result is kept native, else the SAME arguments fall through to
      OCCT. Mixed native/OCCT or OCCT-only operands forward to OCCT (never a native void).

## 5. Gate (a) — HOST analytic (no OCCT)

- [x] `tests/native/test_native_thread_apply.cpp`: the recognizer admits a cylinder shaft
      + declines a box; the closed-form oracle tiles (`V_fuse − V_shaft ∈ (0, V_ridge]`,
      `V_shaft − V_cut ∈ (0, V_groove]`); the baseline cyl−box CUT through the verb WELDS
      watertight + χ=2 + consistently-oriented at the analytic volume (proves the machinery);
      and a multi-turn thread FUSE/CUT honest-declines with the measured typed reason (the
      self-verify never returns a leaky/misoriented solid).

## 6. Gate (b) — SIM native-vs-OCCT (booted simulator)

- [x] `tests/sim/native_thread_apply_parity.mm` (own `main()`), runner
      `scripts/run-sim-native-thread-apply.sh`, and `run-sim-suite.sh` SKIP entry: drive
      the SAME thread+shaft through the native verb and OCCT `BRepAlgoAPI_{Fuse,Cut}` +
      `BRepGProp`; assert the native baseline (cyl−box) matches OCCT within the deflection
      band, and that the native thread path DECLINES while OCCT's per-turn accumulate
      produces the reference volume — the correct honest fallthrough.

## 7. Structural discipline + docs

- [x] `git diff src/native` OCCT-free + additive; tessellator + `construct/thread.h`
      byte-identical; `cc_*` unchanged. `openspec validate moat-thr-thread-apply --strict`.
- [x] Update `openspec/MOAT-ROADMAP.md` (M2/M7b thread_apply native) + the `thread_apply`
      row in `openspec/DROP-OCCT-READINESS.md`.
