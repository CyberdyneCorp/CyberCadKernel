# Tasks — add-native-ssi-s4c-near-tangent-marching (SSI Stage S4-c, first slice)

Verification levels: **host** = OCCT-free host CTest
(`tests/native/test_native_ssi_marching.cpp` or a new
`tests/native/test_native_ssi_s4c_near_tangent.cpp`) — full trace of the crossable
near-tangent fixture + genuine-tangency defer + transversal regression, all under
`CYBERCAD_HAS_NUMSCI`; **sim** = native-vs-OCCT parity
(`tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-marching.sh`) vs
`IntPatch` / `GeomAPI_IntSS` (on-curve + arc length + on-surface). SSI is INTERNAL — **no
`cc_*` entry point is added or changed**. The S4-c parts are compiled under
`CYBERCAD_HAS_NUMSCI` (like S2/S3). `src/native/**` stays OCCT-free. **No change to
`src/native/tessellate`**; **no weakened tolerance**; **no fabricated point past a
degeneracy**.

## 0. Diagnose (confirm the "before" — DONE in the proposal/design)
- [x] 0.1 Confirm on the CURRENT marcher that a near-tangent transversal pair TRUNCATES:
  equal crossing cylinders (R=1, axes crossing) trace ~167 pts and stop with
  `TraceStatus::NearTangent`, `stopReason == NearTangentTransversal` (sine ≈ 1e-4…1e-3),
  `nearTangentGaps == 1`; a tiny axis offset traces the full closed loop (control — the
  curve genuinely continues). (**host** ✓ recorded in design.md)
- [x] 0.2 Confirm a GENUINE tangency defers cleanly on the current stack: two spheres at
  `d = R₁+R₂` and a sphere tangent to a cylinder → `deferredTangent == 1`, no curve. (**host** ✓)
- [x] 0.3 Pick the S4-c crossable fixture whose stall is a SINGLE-BRANCH graze (not a
  branch crossing), enumerated from the diagnose sweep (equal 90° saddle vs tilted/offset
  variants); document the choice + why its crossable gate genuinely fires. (**host**)

## 1. Result + options plumbing (additive)  [host]
- [x] 1.1 Extend `MarchOptions` (`marching.h`): add `double bandEnterSin = -1.0;`
  (≤0 → `≈ 3·tangentSinTol`), `double bandExitSin = -1.0;` (≤0 → `≈ 5·tangentSinTol`),
  `int crossMaxSteps = 64;`. Sentinel-resolved in `tune()`. Keep all existing fields.
- [x] 1.2 Extend `WLine` (`marching.h`): add `int nearTangentCrossed = 0;` and
  `double crossMaxResidual = 0.0;`. Extend `TraceSet`: add `int nearTangentCrossed = 0;`.
  `nearTangentGaps` now counts ONLY regions that could NOT be crossed. Keep all existing
  fields + `stopReason`. (**host**)

## 2. Robust corrector — fixed-plane cut (S4-c-1)  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 Add `fixedPlaneCorrect(A,B, prev, tStar, h, guess, onSurfTol)` in `marching.cpp`:
  a 4-residual `nn::least_squares` — `r₀..₂ = A.point − B.point`,
  `r₃ = dot(A.point − Pprev, tStar) − h` — with `tStar` the LAST-GOOD (pre-band) tangent, so
  the cut plane is well-posed as `sine → 0`. Fall back to `nn::minimize` of
  `‖A.point − B.point‖` with the cut as an equality penalty when the LM solve is
  ill-conditioned. Returns the corrected `State`, residual, nfev, ok. (**host**)
- [x] 2.2 Confirm the S3 along-local-`t` `correct()` is UNCHANGED and still used outside the
  low-sine band (bit-identical transversal trace). (**host** ✓ regression)

## 3. Curvature-aware predictor (S4-c-2)  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Add `curvaturePredict(prevPrev, prev, h, tStar)` in `marching.cpp`: discrete
  curvature `κ̂·N̂ ≈ (t_k − t_{k-1})/Δs_k`; bent guess `P + h·tStar + ½·h²·(κ̂·N̂)`; advance
  params via the existing `advanceParams` on the bent world step. Fall back to the
  first-order guess when < 2 prior nodes / degenerate `Δs` / non-finite `κ̂`. (**host**)

## 4. Step control through the low-sine band (S4-c-3)  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 In `marchDir`, replace the immediate `sine < tangentSinTol` truncation with a
  BAND ENTER at `sine < bandEnterSin`: shrink `h ← max(minStep, 0.5·h)` per step and drive
  the crossing (task 5). BAND EXIT once a corrected node's `sine ≥ bandExitSin`: refresh
  `tStar` to the local tangent, resume normal growth. Preserve the `minStep` floor + the
  `crossMaxSteps` budget → defer if exhausted. (**host**)

## 5. Crossable gate + crossing loop (S4-c-4 — the honesty core)  [CYBERCAD_HAS_NUMSCI]
- [x] 5.1 Add `crossNearTangent(A,B, cur, sign, tStar, band, out)` in `marching.cpp`: at
  the stall, call `classify_tangent_contact_seeded(...)` (S4-b, `tangent_seeded.h`). Attempt
  a crossing ONLY on `NearTangentTransversal`; on `TangentPoint`/`TangentCurve`/`Undecided`
  STOP + record `stopReason` + defer (unchanged from today). (**host**)
- [x] 5.2 Reject a BRANCH CROSSING (S4-d): if the stall has multiple valid intersection-
  tangent directions (rank-deficient `nₐ × n_b` admitting two branches) or multiple distinct
  near-curve seeds cluster at it, STOP + defer (do NOT cross). (**host**)
- [x] 5.3 Drive the crossing: shrink-`h` loop, `curvaturePredict` → `fixedPlaneCorrect` →
  VERIFY (on both surfaces ≤ `onSurfTol` AND monotone along `tStar`) until `sine ≥
  bandExitSin` on the far side with `dot(t_local, tStar) > 0`. On success splice the arc,
  `++nearTangentCrossed`, update `crossMaxResidual`, refresh `tStar`, CONTINUE the walk. On
  ANY verification failure or non-convergence at `minStep`: DISCARD the arc, STOP + defer
  (count in `nearTangentGaps`). NEVER emit a partial/fabricated arc. (**host**)
- [x] 5.4 Wire `crossNearTangent` into `marchDir` (forward + backward) and `march_branch`
  so a crossed branch stays ONE WLine and is classified `Closed`/`BoundaryExit` normally;
  update `trace_from_seeds` tallies (`nearTangentCrossed` up, `nearTangentGaps` only for
  uncrossed). (**host**)

## 6. Honesty invariants (no fabrication, no weakened tolerance)
- [x] 6.1 Confirm `src/native/**` never links OCCT; a region not robustly crossable returns
  a truncated WLine + typed `stopReason` + `nearTangentGaps` increment (deferred → OCCT),
  never a fabricated arc. Document the fixed-plane-cut rationale + the crossable-vs-defer
  contract in the `marching.h` header. (**host**)
- [x] 6.2 Confirm `tangentSinTol`, `onSurfTol`, `minStep`, `maxDeflection` are UNCHANGED and
  the band thresholds are `tangentSinTol`-derived (documented) — no tolerance weakened to
  pass. (**host**)

## 7. Verification (two gates)
- [x] 7.1 Host suite (NUMSCI): the crossable near-tangent fixture now FULLY traced
  (`nearTangentGaps == 0`, `nearTangentCrossed ≥ 1`, curve complete, every node on both
  surfaces ≤ `onSurfTol`, trace length ≈ analytic length); genuine-tangency pairs (spheres
  `d=R₁+R₂`; sphere∩cylinder tangent) STILL stop + classify (`TangentPoint`/`TangentCurve`,
  no curve fabricated, `nearTangentCrossed == 0`); a branch crossing STILL deferred; every
  passing S3 transversal fixture traces bit-identically. Full CTest green NUMSCI ON and OFF
  (S4-c assertions absent with NUMSCI off). No OCCT; no tolerance weakened. (**host**)
- [x] 7.2 Sim parity (`scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm`): add the crossable near-tangent fixture; assert
  native `nearTangentGaps → 0` and the full native curve matches OCCT `IntPatch` /
  `GeomAPI_IntSS` — every WLine point on the OCCT curve ≤ `onCurveTol`
  (`GeomAPI_ProjectPointOnCurve`), on BOTH surfaces ≤ `onSurfTol`, native arc length ≈ OCCT
  arc length within the deflection/step tol (`GCPnts_AbscissaPoint`), closed/branch counts
  agree; AND a genuine-tangency pair STILL stops (reported, not crossed). Report per-pair
  crossed vs still-deferred; run via `xcrun simctl spawn <booted udid>`
  (`xcrun simctl list devices booted`). (**sim**)
- [x] 7.3 `openspec validate add-native-ssi-s4c-near-tangent-marching --strict` green;
  update `SSI-ROADMAP.md` S4 (S4-c first slice landed — a truncating grazing-but-continuous
  curve now fully traced vs OCCT; S4-d branch points / S4-e singularities / S4-f
  self-intersection stay the tail), and `ROADMAP.md` / `NATIVE-REWRITE.md` / `README.md`
  where they cite S4.

## Deferred to S4-d / later (NOT in this change — honest)

- [ ] **S4-d: branch points / self-intersections of the intersection locus** — where two
  or more curve branches meet (the exact equal-cylinder saddle crossing). Detected and
  DEFERRED by the crossable gate; NEVER crossed by this slice.
- [ ] **S4-e: singular points** — a surface's own degeneracy (cone apex, sphere pole).
- [ ] **S4-f: self-intersection resolution / global intersection-topology repair.**
- [ ] **Coincident-region / tangent-seam marching** — a whole `TangentCurve` /
  `CoincidentRegion` seam (surfaces coincide over a stretch), handed on, not marched.
- [ ] **Any near-tangent region not robustly crossable** (deep near-coincident band,
  unverifiable crossing, ambiguous jet) → truncate + defer → OCCT (engine self-verify),
  reported with the measured gap, never faked.
