# Tasks — add-native-ssi-s4e-general (SSI Stage S4-e, second slice: FREEFORM pole + curve-cusp assessment)

Verification levels: **host** = OCCT-free host CTest
(`tests/native/test_native_ssi_s4e_singularities.cpp`) — a degenerate-pole B-spline ∩ plane through
the tip CROSSED, an asymmetric freeform pole still DEFERS, and the analytic sphere-pole /
cone-apex / genuine-boundary / transversal / S4-c / S4-d cases UNCHANGED, all under
`CYBERCAD_HAS_NUMSCI`; **sim** = native-vs-OCCT parity
(`tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-marching.sh`) vs
`GeomAPI_IntSS` / `IntPatch` (on-locus + on-both-surfaces + tip-point match + far-side reached).
SSI is INTERNAL — **no `cc_*` entry point is added or changed**. The S4-e-g parts are compiled
under `CYBERCAD_HAS_NUMSCI` (like S2/S3/S4-c/S4-d/S4-e). `src/native/**` stays OCCT-free. **No change
to `src/native/tessellate`**; **no weakened tolerance**; **no fabricated point across a
singularity**; **no unreachable dead code**.

## 0. Diagnose (confirm reachability + the one gap)
- [x] 0.1 Confirm the point-based corrector is NORMAL-FREE: `branchpt::reproject` residual is
  `{A.point − B.point, dot(A.point − anchor, t★) − d}`, calling only `A.point` / `B.point` / `t★`
  — never a surface normal. So it is well-posed at a freeform pole where only the normal
  degenerates. (**host** — recorded in design.md §"What is TRUE")
- [x] 0.2 Confirm the chart witness FIRES at a freeform degenerate pole: `chartConditionAt` on a
  collapsed-row B-spline reads `‖dU‖ ≈ 0`, `‖dV‖` finite, and `normalFinite == true` (the
  degenerate freeform normal `normalize(Sᵤ×Sᵥ)` is a FINITE near-zero `Dir3`, `valid_ = false`,
  never NaN — `vec.h::normalizeFrom`). (**host**)
- [x] 0.3 Confirm freeform pairs are MARCHED (`trace_from_seeds` / `trace_intersection` on
  `makeBSplineAdapter` adapters) and that freeform adapters carry `uPeriod == 0` (so the analytic
  `poleContinuationU` meridian jump does NOT apply — the ONE gap the freeform-pole re-seed closes).
  (**host** — recorded in design.md)
- [x] 0.4 Confirm the CURVE-CUSP DECLINE by the IFT argument: a cusp of the intersection curve
  (velocity → 0) requires `‖n_A × n_B‖ → 0`; with regular charts and healthy pair sine a cusp is
  impossible, so a curve cusp coincides with the pair-tangency S4-c/S4-d regime and a standalone
  S4-e cusp witness would be UNREACHABLE DEAD CODE. Record the blocker. (**host** — design.md §Slice B)

## 1. Period-free far-side pole re-seed (S4-e-g-2)  [CYBERCAD_HAS_NUMSCI]
- [x] 1.1 Add `freeformChartInvert(S, target, vFix) → uOut` to `chart_singularity.h` (OCCT-free,
  header-only, alongside `poleContinuationU`): a POINT-ONLY 1-D search for the far LONGITUDE — the
  `u` at the FIXED near-pole latitude `vFix` minimizing `‖S.point(u, vFix) − target‖` (coarse
  `u`-scan + shrinking refine, using `S.point` ONLY — no `dU`, no normal), clamped to `S.domain`.
  Fixed-latitude (mirroring `poleContinuationU`, which keeps `v` and jumps `u`) so it never
  collapses onto the degenerate tip — the empirical refinement over the initial 2-D nearest-`(u,v)`
  sketch (a full 2-D search sticks on the flat pole-tip minimum). Re-seeds the FAR side of a
  freeform pole from the continued 3D tangent when the surface carries no analytic `uPeriod`. (**host**)
- [x] 1.2 Generalize `chartFarUV`'s POLE branch in `marching.cpp` (now taking the continued
  `target`): when `S.uPeriod > 0` keep the existing `uOut = poleContinuationU(u, uPeriod)` (analytic
  sphere pole — UNCHANGED); when `S.uPeriod == 0` (freeform) set
  `uOut = freeformChartInvert(S, target = anchor + t★·h, vFix = v)`; `vOut = v` in BOTH pole cases
  (KEEP the latitude). Keep the APEX branch (`uOut = u`, `vOut = −v`) UNCHANGED. (**host**)
- [x] 1.3 Confirm `crossChartSingularity` / `chartCondition` / `isPoleEdge` / `tryChartBand` are
  REUSED UNCHANGED — the freeform-pole crossing differs from the analytic one ONLY in the far-side
  re-seed. No new corrector, no new witness, no new result field. (**host**)

## 2. Curve-cusp DECLINE (S4-e-g-4)  — no dead code
- [x] 2.1 Add NO cusp detector / corrector / result field. Document the DECLINE (the IFT blocker,
  and the routing to S4-c graze / S4-d branch / OCCT deferral) in `marching.h` /
  `chart_singularity.h`. Confirm a curve cusp reaches the existing S4-c/S4-d/OCCT path — a genuine
  tangential endpoint / tacnode DEFERS honestly (truncated + `nearTangentGaps`), never fabricated.
  (**host**)

## 3. Honest guard + boundary discrimination (S4-e-g-5)  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Confirm a freeform-pole crossing is EMITTED only if every node verifies on both surfaces
  `≤ onSurfTol`, the far side makes real progress off the tip, and the continued-tangent-re-seeded
  far node verifies; otherwise DISCARD the arc + STOP + defer (truncated `NearTangent` /
  `BoundaryExit` + `nearTangentGaps` increment → OCCT), reporting the measured gap. NEVER fabricate
  a freeform-pole-crossing point. (**host**)
- [x] 3.2 Confirm the freeform-pole path does NOT perturb the analytic sphere-pole / cone-apex
  crossings (gated on `uPeriod == 0`; `uPeriod > 0` keeps the exact `u_in + π` jump), the
  genuine-boundary exit, the transversal S3 trace, the S4-c graze, or the S4-d branch trace.
  (**host** regression)

## 4. Honesty invariants (no fabrication, no weakened tolerance, no dead code)
- [x] 4.1 Confirm `src/native/**` never links OCCT; a freeform pole not robustly crossed returns a
  truncated WLine + typed stop reason + `nearTangentGaps` increment (deferred → OCCT), never a
  fabricated point; the curve cusp adds NO unreachable code. (**host**)
- [x] 4.2 Confirm `onSurfTol`, `tangentSinTol`, `minStep`, `maxDeflection` are UNCHANGED and the
  freeform-pole path introduces no weakened tolerance (the re-seed is verified ≤ `onSurfTol` like
  every other node). Confirm `enableChartSingularities` OFF is byte-identical to the current
  S3/S4-c/S4-e behaviour. (**host** regression)

## 5. Verification (two gates)
- [x] 5.1 Host suite (NUMSCI): DONE — `tests/native/test_native_ssi_s4e_singularities.cpp` grows two
  cases and the full file is 7/7 green (whole CTest 36/36). The LAND fixture is a NURBS unit sphere
  (a collapsed-control-ROW surface of revolution, `uPeriod == 0`, so NO analytic meridian map — the
  freeform analog of the analytic sphere pole) ∩ plane x=0: chart OFF truncates at the first freeform
  pole (`BoundaryExit`, arc ≈ π, `singularitiesCrossed == 0`); chart ON crosses BOTH freeform poles
  and closes the full great circle (`singularitiesCrossed == 2`, `chartSingularCrossed ≥ 2`,
  `status == Closed`, `nearTangentGaps == 0`, arc ≈ 2π, every node on both surfaces `≤ 1e-6`, actual
  `≈ 4e-16`). The must-still-DEFER control is a collapsed-row Bézier cone-tip whose pole is on the
  v=1 DOMAIN BOUNDARY (a genuine surface ENDPOINT, no far side): the witness fires but the far-side
  re-seed cannot verify past a nonexistent surface, so `singularitiesCrossed == 0`,
  `chartSingularCrossed == 0`, `status == NearTangent` (honest truncation → OCCT). The analytic
  sphere-pole great circle still `singularitiesCrossed ≥ 2`/`Closed`; the cone-apex still
  `singularitiesCrossed ≥ 1`; the genuine-boundary cylinder cap still `BoundaryExit`
  (`singularitiesCrossed == 0`); the S4-c graze still crosses; the S4-d Steinmetz still
  `branchPoints == 2`. (**host**)
  > NOTE — fixture refinement vs the original sketch: the collapsed-row cap on a v-EDGE is a genuine
  > ENDPOINT (no far side → correctly DEFERS), so it is the DEFER control; the continuable THROUGH
  > pole needed for the LAND fixture is the NURBS sphere (the surface wraps past the pole). This is
  > exactly the diagnosis's finding (collapsed-edge cap defers / u-closed sphere pole crosses).
- [x] 5.2 Sim parity (`scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm`): DONE — a `pairFreeformPoleS4e()` fixture builds the
  native NURBS unit sphere (freeform pole) ∩ plane x=0 vs the OCCT ORACLE `Geom_SphericalSurface`
  (geometrically identical — OCCT has no parametric pole there) ∩ `Geom_Plane`, and asserts the
  native freeform-pole crossing reproduces the OCCT `GeomAPI_IntSS` great circle: every sampled
  native node on the OCCT locus AND on both OCCT surfaces, closed, arc ≈ the OCCT circle. Measured
  `singX=2 NTgaps=0 closed=1 onCurve=1.51e-07 onSurf=1.51e-07 lenDelta=5.04e-05` (chart OFF
  before=3.1415 half circle → chart ON nat=6.2829 ≈ occt=6.2832). Whole harness 11/11 PASS; the
  analytic sphere-pole / cone-apex, genuine-boundary, transversal, S4-c, S4-d cases all UNCHANGED.
  Ran via `xcrun simctl spawn` on a booted simulator. (**sim**)
- [x] 5.3 `openspec validate add-native-ssi-s4e-general --strict` green; update `SSI-ROADMAP.md`
  S4-e (the FREEFORM parametric-pole slice landed — a degenerate-pole B-spline ∩ plane now crossed
  vs OCCT; the CURVE CUSP declined with the IFT blocker and routed to S4-c/S4-d/OCCT; asymmetric
  freeform poles, higher-order / edge / seam singularities, and S4-f self-intersection completeness
  stay the tail), and `ROADMAP.md` / `NATIVE-REWRITE.md` / `README.md` where they cite S4.

## Deferred to S4-e-tail / S4-f (NOT in this change — honest)

- [ ] **Curve cusps as a standalone S4-e mechanism** — declined by the IFT argument (a cusp
  coincides with the pair-tangency S4-c/S4-d regime); routed to S4-c/S4-d/OCCT, never faked.
- [ ] **Asymmetric freeform poles** whose continued-tangent inversion does not verify on both
  surfaces → DEFER → OCCT.
- [ ] **Higher-order / edge / seam singularities**, ridge singularities, and a full brep
  degenerate-pole B-spline SOLID through the boolean pipeline.
- [ ] **S4-f: self-intersection completeness / global topology repair.**
- [ ] **Any singularity not robustly crossable / verifiable** → truncate + defer → OCCT, reported
  with the measured gap, never faked.
