# Tasks — add-native-ssi-s4f-completeness (SSI Stage S4-f, first slice)

Verification levels: **host** = OCCT-free host CTest
(`tests/native/test_native_ssi_marching.cpp` + `tests/native/test_native_ssi_seeding.cpp`, or a new
`tests/native/test_native_ssi_s4f_completeness.cpp`) — small-loop recovery (A) + false-close
prevention (B) + self-intersection detect-and-trace-through (C) + many-small-loops measured recall
win (D) + transversal / S4-c graze / S4-d Steinmetz / S4-e pole+apex regression, all under
`CYBERCAD_HAS_NUMSCI`; **sim** = native-vs-OCCT parity (model on
`scripts/run-sim-native-ssi-seeding.sh` for recall vs `GeomAPI_IntSS` branch count +
`scripts/run-sim-native-ssi-marching.sh` for loop-closure / self-intersection, or a new
`scripts/run-sim-native-ssi-s4f.sh` + `tests/sim/native_ssi_completeness_parity.mm`). SSI is
INTERNAL — **no `cc_*` entry point is added or changed**. The S4-f parts are compiled under
`CYBERCAD_HAS_NUMSCI` (like S2/S3/S4-c/S4-d/S4-e). `src/native/**` stays OCCT-free. **No change to
`src/native/tessellate`**; **no weakened tolerance**; **no fabricated loop, closure, or seed**.
**Completeness is MEASURED + ASYMPTOTIC, never a proof — the recall residual is always reported.**
**S4-f DE-RISKS (does not unblock) curved blends (#6) + wrap-emboss (#7).**

## 0. Diagnose (confirm the "before")
- [x] 0.1 Confirm the SMALL-LOOP MISS (A): a pair whose intersection has one small loop entirely
  inside a default (1/32) leaf cell produces 0 seeds for that loop at the default `minPatchFrac`
  (`RecallReport.recall() < 1`); the loop is real + on both surfaces (record its analytic
  location). (**host** — record in design.md)
- [x] 0.2 Confirm the FALSE-CLOSE (B): an open / longer curve whose S3 trace passes within
  `loopClose·h` of the seed heading the OTHER way is declared `Closed` and truncated at the
  near-pass (arc length short of the analytic ground truth); `closeAligned` is inert
  (`crossedAny == false`). (**host** — record in design.md)
- [x] 0.3 Confirm the SELF-INTERSECTION (C): a single-arm curve that genuinely self-crosses (ONE
  arm crossing itself, both surfaces transversal) is mis-closed or stepped past unrecorded today,
  and is DISTINCT from an S4-d branch (no locus flip, no new arms). Record the analytic crossing
  point. (**host** — record in design.md)
- [x] 0.4 Confirm the CONTROLS: the 5 transversal pairs trace `nt == 0` and each loop CLOSES
  tangent-continuously; the S4-c graze crosses (`nearTangentCrossed ≥ 1`); the S4-d Steinmetz
  traces (`branchPoints == 2`); the S4-e pole + apex cross (`singularitiesCrossed ≥ 1`). Record the
  analytic ground truth for A–D (loop counts, loop sizes, the false-close curve's true arc, the
  self-crossing point) so host + OCCT parity are well-defined. (**host**)

## 1. Result + options plumbing (additive)  [host]
- [x] 1.1 Add to `WLine` (`marching.h`): `int selfIntersectionCount = 0;` and
  `std::vector<SelfIntersection> selfIntersections{};`, plus the `struct SelfIntersection` (point,
  u1,v1,u2,v2, two node indices). Add to `TraceSet`: `int selfIntersections = 0;`,
  `int criticRounds = 0;`, `int criticRecoveredLoops = 0;`, `double criticFloorFrac = 0.0;`,
  `bool criticStoppedDry = false;`, `bool completenessResidual = true;`. Keep all existing fields.
  Document the completeness + self-intersection contract (and the honest asymptotic framing) in the
  header. (**host**)
- [x] 1.2 Extend `MarchOptions` (`marching.h`): add `bool enableSelfIntersection = false;` (off →
  the current S3 behaviour, byte-identical), `double selfIntersectRadiusFrac = -1.0;`,
  `double closureTangentCos = -1.0;` (≤0 → `-0.5`, the generous block threshold). Note in the
  header that the TRUE-RETURN closure is always on and reduces to today's result on truly-closing
  curves; `closureTangentCos` only sets how antiparallel a heading must be to BLOCK a false-close.
  Sentinel-resolved in `tune()`. Keep all existing fields. (**host**)
- [x] 1.3 Extend `SeedOptions` (`seeding.h`): add `bool completenessCritic = false;` (off → the
  current fixed-resolution seed), `double criticRefineFactor = 0.5;`, `int criticDryRounds = 2;`
  (K), `int criticMaxRounds = 6;`, `int criticMaxCandidates = 4096;`. Extend `RecallReport`
  (`seed.h`): add `double criticFloorFrac = 0.0;` and `bool residualAcknowledged = true;` (ALWAYS
  true — a loop below the floor can still exist). Document the honest floor + residual. (**host**)

## 2. Robust true-return + tangent-continuous closure (S4-f-1)  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 Generalize `closeAligned` (`marching.cpp`) so the tangent-continuity gate applies to ALL
  closures (not just `crossedAny`): a close requires `distance(cur, seed) ≤ closeRadius` AND
  `dot(fwdNow, seedFwd) ≥ closureTangentCos` (using the already-captured `seedFwd`/`haveSeedFwd`).
  Confirm it is a NECESSARY-condition tightening — it can only REFUSE a close, never make one — so a
  truly-closing curve is unaffected and a near-antiparallel pass-through does NOT close. Wire it
  into the three closure checks in `marchDir` / `tryBandEntry` / `tryChartBand`. (**host**)
- [x] 2.2 Confirm on the 5 transversal pairs + every existing loop-closing control that the
  generalized gate is BYTE-IDENTICAL (they return tangent-continuously → still `Closed`), and on
  fixture (B) that the false-close near-pass is now BLOCKED and the curve traces to its true
  termination. NEVER fabricate a closure. (**host** regression)

## 3. Self-intersection guard (S4-f-2)  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Add a self-intersection guard in `marchDir` (`marching.cpp`, `enableSelfIntersection`
  on): over the arm's own node history, detect a crossing of an EARLIER non-seed, non-adjacent node
  (`distance ≤ selfIntersectRadiusFrac·scale` AND the local tangents TRANSVERSE — `|dot| < 1 − εcont`,
  not a continuation and not a retrace) → record a typed `SelfIntersection`, `++selfIntersectionCount`,
  and DO NOT close + DO NOT stop (continue the arm). Confirm it is INDEPENDENT of the S4-c pair sine
  and the S4-d locus flip (both surfaces transversal at a curve self-crossing; the locus does not
  branch). Off → no-op (byte-identical S3). (**host**)
- [x] 3.2 Confirm the retrace-vs-crossing distinction: an arm running back over itself
  (`dot ≈ ±1`, a dedup / periodic-seam artifact) is NOT reported; only a transverse self-crossing
  is. Confirm a self-intersection is distinct from an S4-d branch (`branchPoints == 0` on the C
  fixture) and is reported as DATA (a false positive is a spurious count, not a wrong curve).
  (**host**)

## 4. Bounded adaptive completeness-critic re-seed (S4-f-3)  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 Add a new OCCT-free header `src/native/ssi/completeness_critic.h` (`namespace critic`):
  `coverageOf(tracedLines, A.domain, B.domain, gridN)` → a coarse param-grid occupancy from the
  traced polylines' footprints on BOTH surfaces; `uncoveredBoxes(Coverage)` → the `ParamBox`es
  with NO traced curve. Document that this is the RE-SEED targeting, not a completeness proof.
  (**host**)
- [x] 4.2 Add the critic re-seed loop (`seeding.cpp` / `marching.cpp`, `CYBERCAD_HAS_NUMSCI`,
  `completenessCritic` on): after the initial `seed_intersection` (fixed floor) + trace,
  loop-until-dry — re-seed the uncovered boxes at `minPatchFrac *= criticRefineFactor`, refine
  (SAME `onSurfTol`, DISCARD any candidate that does not refine on both surfaces), dedup NEW seeds
  vs ALL kept curves, trace each genuinely new seed (dedup retraces). Stop after `criticDryRounds`
  (K) consecutive rounds with NO new branch OR at the cost cap (`criticMaxRounds` /
  `criticMaxCandidates`). Tally `criticRounds`, `criticRecoveredLoops`, `criticFloorFrac`,
  `criticStoppedDry`. NEVER fabricate a seed. Off → byte-identical to today. (**host**)

## 5. Honest recall floor + residual (S4-f-4)
- [x] 5.1 Confirm the critic reports the FLOOR reached (`criticFloorFrac`, `criticRounds`,
  `criticStoppedDry` — dry vs cap) and the MEASURED recall (`RecallReport.recall()`, never a blind
  1.0), with `completenessResidual` / `residualAcknowledged` ALWAYS true (a loop below the floor
  can still exist). Confirm fixture (A) reports recall `< 1` at the default and `== 1` after the
  critic WITH the residual still acknowledged; fixture (D) reports a measured RISE toward OCCT's
  branch count WITH the residual — never a claim of totality. Document the asymptotic framing in
  the headers. (**host**)

## 6. Honesty invariants (no fabrication, no weakened tolerance)
- [x] 6.1 Confirm `src/native/**` never links OCCT; a loop the critic cannot recover is a reported
  measured recall `< 1` + `completenessResidual`, never a fabricated branch; a self-intersection is
  a recorded typed crossing, never a faked closure; a false-close is prevented, never a faked
  continuation. Document the detect → report → (critic) recover contract + the honest scope table
  in `marching.h` / `seed.h` / `completeness_critic.h`. (**host**)
- [x] 6.2 Confirm `onSurfTol`, `tangentSinTol`, `minStep`, `maxDeflection`, `loopCloseFrac` are
  UNCHANGED and the S4-f discriminators (`closureTangentCos`, `selfIntersectRadiusFrac`,
  `criticRefineFactor`, `criticDryRounds`, `criticMaxRounds`, `criticMaxCandidates`) introduce no
  weakened tolerance (the critic refine uses the SAME `onSurfTol`). Confirm `completenessCritic` +
  `enableSelfIntersection` OFF is byte-identical to the current fixed-resolution seed + S3 trace,
  and the TRUE-RETURN closure is byte-identical on truly-closing curves. (**host** regression)

## 7. Verification (two gates)
- [x] 7.1 Host suite (NUMSCI): (A) the small loop is MISSED at the default (`recall() < 1`) and
  RECOVERED by the critic (`recall() == 1` on this fixture, `criticRecoveredLoops ≥ 1`,
  `criticFloorFrac` finer than 1/32, `completenessResidual == true`); (B) the false-close curve is
  traced to its TRUE termination (arc matches the analytic ground truth, no early `Closed`); (C)
  the self-crossing is DETECTED (`selfIntersections ≥ 1`, typed crossing on both surfaces ≤
  `onSurfTol`) and traced THROUGH (distinct from S4-d, `branchPoints == 0`); (D) the critic
  MEASURABLY raises recall (`recall_after > recall_before`), reports the floor + `criticStoppedDry`
  + the residual. Regression: the 5 transversal pairs bit-identical (`nt == 0`, still `Closed`);
  the S4-c graze STILL crosses; the S4-d Steinmetz STILL traces (`branchPoints == 2`,
  `selfIntersections == 0`); the S4-e pole + apex STILL cross (`singularitiesCrossed ≥ 1`). Full
  CTest green NUMSCI ON and OFF (S4-f assertions absent with NUMSCI off). No OCCT; no tolerance
  weakened. (**host**)
- [x] 7.2 Sim parity (model on `scripts/run-sim-native-ssi-seeding.sh` +
  `scripts/run-sim-native-ssi-marching.sh`, or a new `scripts/run-sim-native-ssi-s4f.sh` +
  `tests/sim/native_ssi_completeness_parity.mm`): add fixtures A–D; assert the MEASURED recall win
  vs OCCT `GeomAPI_IntSS` (`recall_default < recall_critic ≤ 1` on the small-loop fixture; the
  many-loop fixture's recall RISES toward OCCT's branch count); every recovered / traced native
  node on the OCCT locus ≤ `onCurveTol` (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces ≤
  `onSurfTol`; the false-close curve matches the OCCT curve END-TO-END; the self-crossing is
  detected at the OCCT self-intersection point ≤ tol. Report per-fixture the recall FLOOR + the
  RESIDUAL acknowledgement (NOT a completeness proof). Confirm the transversal + S4-c + S4-d + S4-e
  controls are UNCHANGED. Run via `xcrun simctl spawn <booted udid>`
  (`xcrun simctl list devices booted`). (**sim**)
- [x] 7.3 `openspec validate add-native-ssi-s4f-completeness --strict` green; update
  `SSI-ROADMAP.md` S4-f (from ✗ PENDING to a FIRST completeness + loop-robustness slice — MEASURED
  recall wins on A–D vs OCCT + robust true-return closure + a self-intersection guard, with the
  recall floor + residual reported; S4-f DE-RISKS, does not unblock, #6/#7; general topology repair
  / self-intersection resolution stay the tail), and `ROADMAP.md` / `NATIVE-REWRITE.md` /
  `README.md` where they cite S4.

## Deferred to S4-f-wider / S5–S7 (NOT in this change — honest)

- [x] **A completeness GUARANTEE / proof** — the critic RAISES the floor; it never proves no loop
  is missed. The residual is ALWAYS reported (a loop below `criticFloorFrac` can still exist).
- [x] **Global topology repair / watertight self-intersection resolution** — splitting a
  self-crossing arm into sub-arcs, healing a self-intersecting shell → S5/S6 assemblers. S4-f
  DETECTS + REPORTS + traces-through, it does not repair topology.
- [x] **Completing curved blends (#6) / wrap-emboss (#7)** — S4-f DE-RISKS them (reliable
  small-loop / self-intersecting / many-loop SSI-curve completeness they depend on); their
  assemblers stay S5/S6/S7.
- [x] **Any loop below the finest re-seed round** → reported as a measured recall `< 1` +
  `completenessResidual`, never faked.
