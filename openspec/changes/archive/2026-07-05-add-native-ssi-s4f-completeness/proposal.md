## Why

`SSI-ROADMAP.md` S4 is **the moat** — tangent / degeneracy robustness. The prior slices hardened
the DEGENERACY paths: S4-a/S4-b CLASSIFY (typed `CoincidentRegion` + `TangentContact`); S4-c
MARCHES THROUGH a `NearTangentTransversal` graze; S4-d LOCALIZES a branch point (the Steinmetz
self-crossing) and routes the arms; S4-e STEPS ACROSS a single-surface chart singularity (sphere
pole / cone apex). **S4-f is a DIFFERENT, less-glamorous, more-production-critical concern:
COMPLETENESS + LOOP ROBUSTNESS.** It does NOT add a new geometry capability — it HARDENS the
CORRECTNESS of curves we already trace. Two orthogonal holes in the current tracer, both silent:

1. **LOOP CLOSURE IS PURE PROXIMITY.** In `marching.cpp`, `DirEnd::LoopClosed` fires when the
   march returns within `loopClose · h` of the seed (`loopCloseFrac`, default 2). Proximity is a
   NECESSARY but NOT SUFFICIENT condition for a closed loop:
   - It can **FALSE-CLOSE**: a curve that merely PASSES NEAR its seed (or near an earlier node)
     while heading the OTHER way is declared closed and the march stops EARLY, truncating a real
     open/longer curve. (`closeAligned()` added a direction gate, but ONLY once a graze was
     crossed — a pure S3 curve that passes near its seed has NO such guard.)
   - It can **SKIP A SELF-CROSSING**: a single-arm curve that genuinely self-intersects (crosses
     an EARLIER non-seed node) is a SELF-INTERSECTION, not a loop close — but the proximity test
     has no concept of "earlier node", only "the seed", so a self-crossing near the seed is
     mis-closed and a self-crossing elsewhere is invisible.

2. **THE FIXED-RESOLUTION SEEDER SILENTLY MISSES SMALL LOOPS.** S2 recall is gated by
   `SeedOptions.minPatchFrac` (default 1/32 — the resolution/recall knob) + `initialGridU/V` +
   `maxDepth`. A small intersection loop that lies ENTIRELY INSIDE one default leaf cell produces
   NO candidate region, so it is NEVER seeded and NEVER traced — the acknowledged completeness
   hole. `seeding.h` documents this ("too-shallow subdivision can miss a small loop"), but nothing
   RECOVERS it: recall is silently `< 1` and the caller cannot tell which loops were missed.

Diagnosed on the CURRENT tracer (host, `CYBERCAD_HAS_NUMSCI` ON):

| Fixture | S3 / S4-e result today | Verdict |
|---|---|---|
| **(A) SMALL LOOP MISSED** — a pair whose intersection has one small loop entirely inside a default (1/32) leaf cell | `SeedSet` produces **0 seeds for the small loop** (no candidate region survives the AABB prune at the default resolution); `RecallReport.recall() < 1` | **MISSED** — the loop is real + on both surfaces but below the fixed floor; nothing recovers it |
| **(B) FALSE-CLOSE** — an OPEN curve (or a longer loop) whose trace passes within `loopClose·h` of the seed while heading the other way | one WLine, `status = Closed`, **truncated at the near-pass** (arc length short of the true curve), `closeAligned` inert (no graze crossed) | **FALSE-CLOSED** — the proximity test stopped a real curve early |
| **(C) SELF-INTERSECTION** — a single-arm curve that genuinely self-crosses (distinct from an S4-d branch: ONE arm crossing ITSELF, not the locus branching) | the crossing near an earlier node is either mis-read as a loop close or stepped past unrecorded | **SKIPPED / MIS-CLOSED** — a self-crossing is neither detected nor traced through |
| **(D) MANY SMALL LOOPS** — an adversarial pair with several disjoint small loops (the wrap-emboss / blend seam pattern) | most loops below the floor produce no seeds; `recall() ≪ 1` | **MOSTLY MISSED** — the fixed resolution catches only the loops that straddle a leaf boundary |
| 5 transversal S3 pairs (`nt = 0`) | bit-identical | **must stay UNTOUCHED** |
| S4-c graze / S4-d Steinmetz / S4-e pole+apex | crossed / traced / crossed | **must stay UNCHANGED** |

**HONEST FRAMING (baked into this spec, the tests, and the docs — no overclaim):**
- **Completeness is ASYMPTOTIC, NEVER a proof.** Below ANY fixed subdivision resolution there is
  always a SMALLER loop that can be missed. This change claims **MEASURED recall wins on concrete
  fixtures vs OCCT `GeomAPI_IntSS`** and **RAISES the completeness floor**; it does NOT claim a
  completeness guarantee. Recall is reported as a measured figure with the **residual floor
  explicitly acknowledged** (a loop smaller than the finest re-seed round can still exist).
- **S4-f DE-RISKS (does NOT unblock / complete) curved blends (#6) and wrap-emboss (#7).** Their
  intersection seams are exactly the small-loop / self-intersecting / many-loop patterns this
  slice hardens, so reliable SSI-curve completeness they depend on becomes measurably more
  trustworthy — but #6/#7 still need their wider S5/S6/S7 assemblers; this change delivers none of
  those.

Both holes are honestly fixable WITHOUT weakening any tolerance: (1) make closure a TRUE-RETURN +
tangent-continuous test (return to the seed AND heading the way it left) and add a
self-intersection guard (crossing an EARLIER non-seed node) — a proximity near-pass no longer
false-closes; (2) after the initial seed + trace, an ADAPTIVE COMPLETENESS-CRITIC re-seeds at
finer resolution in the regions NOT covered by a traced curve, loop-until-dry (K consecutive
rounds find nothing new), bounded by a cost cap — recovering small loops the fixed floor missed
and REPORTING the recall floor reached (plus the honest caveat that a smaller loop can still
exist). Both are behind default-off gates so the controls stay byte-identical.

## What Changes

- **(S4-f-1) TRUE-RETURN + TANGENT-CONTINUOUS LOOP CLOSURE.** Replace the pure-proximity
  `DirEnd::LoopClosed` test with a TRUE-RETURN test: a loop closes ONLY when the march returns
  within the (existing) proximity radius of the SEED **AND** the current forward tangent is
  CONTINUOUS with the seed's outgoing tangent (heading the way it left — the direction gate
  currently applied only after a graze crossing is generalized to ALL closures). A near-pass that
  fails the tangent-continuity test does NOT close; the march continues. On the 5 transversal
  pairs (which return to the seed tangent-continuously — they DO close) and the S4-c/S4-d/S4-e
  controls, this reduces to the CURRENT result exactly (no false-close there, so byte-identical).
- **(S4-f-2) SELF-INTERSECTION GUARD.** Track the march's own accepted nodes; when the curve
  crosses an EARLIER NON-SEED node (within a tolerance-scaled radius, tangent NOT continuous with
  that earlier node — i.e. a genuine transverse crossing of the arm with itself, not a retrace),
  flag a SELF-INTERSECTION. A self-crossing is distinct from a loop close (the crossed node is not
  the seed) and from an S4-d branch (ONE arm crossing ITSELF, not the intersection LOCUS
  branching). The guard records the self-intersection point (typed) so the curve is NOT
  false-closed there and NOT silently stepped past; the crossing is REPORTED and the arm continues
  to its true termination. This is a NEW, independent witness — it does not reuse the S4-c sine or
  the S4-d locus flip (both surfaces are transversal at a curve self-crossing).
- **(S4-f-3) ADAPTIVE COMPLETENESS-CRITIC RE-SEED (bounded, loop-until-dry).** After the initial
  S2 seed + S3/S4 trace, engage a CRITIC pass (default off): compute the regions of the two param
  domains NOT covered by any traced curve (a coarse coverage grid keyed on the traced polylines'
  footprints), RE-SUBDIVIDE those uncovered regions at a FINER resolution (`minPatchFrac` refined
  by a bounded factor per round), refine + dedup the new candidates against ALL kept curves, trace
  any genuinely NEW seed, and REPEAT. Stop after `criticDryRounds` (K) consecutive rounds that
  find NO new branch (loop-until-dry) OR when the cost cap (`criticMaxRounds` /
  `criticMaxCandidates`) is hit — whichever first. Every recovered loop is a VERIFIED on-both-
  surfaces seed that refines to a real curve; a candidate that does not refine to a real
  on-both-surfaces seed/curve is DISCARDED (never a fabricated branch).
- **(S4-f-4) HONEST RECALL FLOOR REPORTING.** The critic reports the FLOOR it reached: the finest
  `minPatchFrac` it re-seeded at, the number of rounds run, whether it stopped dry or on the cost
  cap, and the measured recall — with the EXPLICIT residual that a loop smaller than the finest
  round can STILL exist (completeness is asymptotic, not proven). `RecallReport` (seed.h) and
  `TraceSet` (marching.h) gain additive fields carrying this floor; the recall figure is never
  asserted to be a blind 1.0.
- **Result carries the completeness + closure outcome (additive).** `marching.h`: `TraceSet` gains
  `selfIntersections` (count) + `criticRounds` / `criticRecoveredLoops` / `criticFloorFrac` /
  `criticStoppedDry`; `WLine` gains `selfIntersectionCount` + a typed self-intersection list.
  `seed.h`: `RecallReport` gains `criticFloorFrac` + a `residualAcknowledged` flag. No `cc_*` ABI
  change.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-ssi capability with the S4-f completeness +
loop-robustness slice (true-return + tangent-continuous closure, a self-intersection guard, and a
bounded adaptive completeness-critic re-seed with an honestly reported recall floor). It adds no
new capability spec and no cc_* ABI. -->

### Modified Capabilities
- `native-ssi`: add (S4-f) a native, OCCT-free **completeness + loop-robustness** capability with
  two independent parts. **Part 1 (robust closure)** replaces the pure-PROXIMITY loop-closure test
  with a TRUE-RETURN + TANGENT-CONTINUOUS test (a loop closes only on a verified return to the seed
  heading the way it left) and adds a SELF-INTERSECTION guard (the arm crossing an EARLIER non-seed
  node is a typed self-intersection, not a loop close and not an S4-d branch) — so a curve that
  merely passes near its seed is NOT false-closed and a genuine self-crossing is detected + traced
  through, not skipped. **Part 2 (completeness critic)** adds a BOUNDED ADAPTIVE re-seed that, after
  the initial seed + trace, re-subdivides the param regions NOT covered by a traced curve at finer
  resolution, LOOP-UNTIL-DRY (K consecutive rounds with no new branch), to RECOVER small loops the
  fixed 1/32 floor silently missed — bounded by a cost cap and HONEST about the floor it reached.
  **Completeness is MEASURED + ASYMPTOTIC, never a proof:** the change claims measured recall wins
  vs OCCT and raises the completeness floor, and REPORTS the residual (a smaller loop can still
  exist); it does NOT claim a completeness guarantee. **S4-f DE-RISKS (does not unblock) curved
  blends (#6) and wrap-emboss (#7)** — their seams are exactly these small-loop / self-intersecting
  / many-loop patterns, so the SSI-curve completeness they depend on becomes measurably more
  reliable, but #6/#7 still need their wider S5/S6/S7 assemblers. The tracer NEVER fabricates a
  loop, a closure, or a seed: a loop closes only on a verified true return; a re-seeded branch is
  kept only if it refines to a real on-both-surfaces seed/curve; anything unresolved is reported
  honestly (measured recall `< 1`), never a faked branch. The 5 transversal S3 pairs (`nt = 0`,
  bit-identical), the S4-c crossable graze, the S4-d Steinmetz branch trace, and the S4-e pole/apex
  crossings are UNCHANGED — the new closure reduces to the current proximity result on curves that
  truly close, and the critic + self-intersection guard are behind default-off gates. No `cc_*` ABI
  change; `src/native/**` stays OCCT-free; the S4-f machinery is compiled under `CYBERCAD_HAS_NUMSCI`
  like S2/S3/S4-c/S4-d/S4-e.

## Impact

- **ABI**: none. SSI is INTERNAL — no `cc_*` entry point, signature, or POD struct change.
  Additive only; the tessellator (`src/native/tessellate`) and the CyberCad app are untouched.
- **Build**: extends `src/native/ssi/marching.{h,cpp}` (a true-return + tangent-continuous closure
  predicate generalizing `closeAligned`; a self-intersection guard over the march's own node
  history; the additive `TraceSet` / `WLine` completeness + self-intersection fields) and
  `src/native/ssi/seeding.{h,cpp}` + `seed.h` (the `SeedOptions` critic knobs, the additive
  `RecallReport` floor fields), plus a new OCCT-free `src/native/ssi/completeness_critic.h`
  isolating the uncovered-region coverage math + the loop-until-dry re-seed driver (like
  `chart_singularity.h` / `branch_point.h` isolate their math). The critic re-uses `seed_intersection`
  at a finer `minPatchFrac` and the existing dedup unchanged. Under `CYBERCAD_HAS_NUMSCI` like the
  S2/S3/S4-c/S4-d/S4-e stack. No change to `src/native/tessellate`; no new tolerance beyond the
  critic/closure discriminators (`criticRefineFactor`, `criticDryRounds`, `criticMaxRounds`,
  `criticMaxCandidates`, the closure tangent-continuity cosine, the self-intersection radius —
  documented, sentinel-resolved, NEVER weakening `onSurfTol` / `tangentSinTol` / `minStep` /
  `maxDeflection` / `loopCloseFrac`).
- **Verification**: two gates. **Host (no OCCT)** — extend `tests/native/test_native_ssi_marching.cpp`
  and `tests/native/test_native_ssi_seeding.cpp` (or a new
  `tests/native/test_native_ssi_s4f_completeness.cpp`), all under `CYBERCAD_HAS_NUMSCI`:
  - (A) SMALL LOOP MISSED — confirm recall `< 1` at the DEFAULT resolution (the small loop is
    unseeded), then the completeness critic recovers it (recall `→ 1` on THIS fixture, floor
    reported).
  - (B) FALSE-CLOSE — the open/longer curve that the proximity test truncated at a near-pass is
    now traced to its TRUE termination (the true-return + tangent-continuous test does NOT close
    it early); arc length matches the analytic ground truth.
  - (C) SELF-INTERSECTION — the single-arm self-crossing curve is DETECTED (typed self-intersection,
    `selfIntersections ≥ 1`) and traced THROUGH the crossing (not false-closed, not skipped),
    distinct from an S4-d branch (`branchPoints == 0` there).
  - (D) MANY SMALL LOOPS — the critic MEASURABLY raises recall vs the default (report the floor +
    the residual); recall improves toward the OCCT branch count without claiming a proof.
  - Controls: the 5 transversal pairs trace bit-identically (`nt = 0`, closure unchanged); the
    S4-c graze STILL crosses; the S4-d Steinmetz STILL traces (`branchPoints = 2`,
    `selfIntersections = 0`); the S4-e pole + apex STILL cross. With the critic + self-intersection
    guard OFF the whole trace is byte-identical. Full CTest green NUMSCI ON and OFF (the S4-f
    assertions absent with NUMSCI off, like S2/S3/S4-c/S4-d/S4-e). No OCCT linked; no tolerance
    weakened. **Sim native-vs-OCCT (booted sim)** — model on
    `scripts/run-sim-native-ssi-seeding.sh` (recall vs OCCT `GeomAPI_IntSS` branch count) +
    `scripts/run-sim-native-ssi-marching.sh` (loop-closure / self-intersection): add fixtures A–D
    and assert the MEASURED recall win vs OCCT (recall at default < recall after critic ≤ 1 on the
    small-loop fixtures; the many-loop fixture's recall rises toward OCCT's branch count), the
    false-close curve now matches the OCCT curve end-to-end, and the self-crossing is detected on
    the OCCT locus — every recovered/traced native node on the OCCT locus ≤ `onCurveTol`
    (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces ≤ `onSurfTol`. Report per-fixture the
    recall floor + the residual acknowledgement (a smaller loop can still exist), and confirm the
    transversal + S4-c + S4-d + S4-e controls are UNCHANGED. `xcrun simctl list devices booted`.
- **Roadmap**: advances `SSI-ROADMAP.md` S4-f from ✗ PENDING to a FIRST completeness + loop-
  robustness slice — MEASURED recall wins on fixtures A–D vs OCCT + robust true-return closure +
  a self-intersection guard. **Explicitly a measured, asymptotic result, NOT a completeness proof:**
  the recall floor is reported with the residual that a smaller loop can still be missed. S4-f
  DE-RISKS (does not unblock) #6/#7.
- **Risk (honest)**: (a) the true-return + tangent-continuous closure could FAIL TO CLOSE a genuine
  loop whose return tangent is noisy — mitigated by using the SAME proximity radius as today plus a
  GENEROUS tangent-continuity cosine (only a near-ANTIPARALLEL heading — a clear pass-through the
  other way — blocks the close), and pinned by the 5 transversal pairs + every existing loop-closing
  control staying byte-identical; if the generalized gate ever changed a control result the gate is
  wrong and the control catches it. (b) The self-intersection guard could FALSE-FIRE on a curve that
  legitimately runs close to an earlier node without crossing (a tight but non-self-intersecting
  bend) — mitigated by requiring both proximity AND a transverse (non-continuous, non-antiparallel-
  retrace) tangent at the crossing, and by reporting it as data (it does not stop the march, so a
  false positive costs a spurious count, not a wrong curve); the count is verified against the
  analytic self-crossing fixtures. (c) The critic could COST-EXPLODE chasing ever-smaller phantom
  loops — mitigated by the loop-until-dry stop (K dry rounds) + the hard cost cap
  (`criticMaxRounds` / `criticMaxCandidates`) and by DISCARDING any candidate that does not refine
  to a real on-both-surfaces seed; the floor reached is reported honestly. (d) The critic could
  claim a completeness it does not have — mitigated BY CONSTRUCTION: recall is a MEASURED figure with
  the residual acknowledged, never asserted 1.0 blindly; a fixture's recall `→ 1` is a claim about
  THAT fixture at THAT floor, not a general guarantee. (e) Any change near the marcher / seeder risks
  perturbing the transversal / S4-c / S4-d / S4-e results — mitigated by the closure reducing to the
  current proximity result on truly-closing curves and by the critic + self-intersection guard being
  default-off, pinned by the 5 transversal pairs + the S4-c graze + the S4-d Steinmetz + the S4-e
  pole/apex staying green. Whatever the critic cannot recover is reported as a measured recall `< 1`
  with the floor; no loop, closure, or seed is faked, hand-tuned, or weakened to pass.
